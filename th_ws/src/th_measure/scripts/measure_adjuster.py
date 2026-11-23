#!/usr/bin/env python3
"""measure_adjuster.py — 盤前の旋回・前後の並進（ADJUST）の実体（8-g P3）。

正本は docs/plan/detailed/DetailedDesign-measure.md §6.4 と
docs/plan/spec/Spec-modes.md SM-3.1.2-127・-128・-136・-137。

  - effect `measure_adjust`（{command_id, kind: turn｜translate, angle_rad, side,
    distance_m}）を受ける。
  - `/system/state` が ADJUST の間だけ `/cmd_vel_behavior` に出す
    （Nav2 を使わない直接制御。twist_mux → obstacle_limiter を通る）。
  - 帰還は TF `odom→base_link`。旋回は始点からのヨーの変化を巻き戻さずに
    積み上げて比べる。並進は始点からの進んだ距離で、間は向きを保つ。
  - ADJUST から出たら 0 を 1 回出して黙る。終わったら `/system/event` に
    `evt.adjust_done`（command_id を添える）。
  - `/cmd_vel*` へは `/cmd_vel_behavior` 以外に出さない。
"""
import json
import math

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from th_system_msgs.msg import StateEffect, StateEvent, SystemState

from th_measure.adjust_core import (accumulate_yaw, heading_hold,
                                    translate_done, turn_done, turn_wzr,
                                    wrap_pi)

_TIMER_PERIOD_S = 0.05  # 20 Hz


def _yaw_of(tf) -> float:
    q = tf.transform.rotation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


class MeasureAdjuster(Node):

    def __init__(self, **kwargs):
        super().__init__('measure_adjuster', **kwargs)

        # --- R2: 裸の数値を書かない。registry 由来は生成 yaml が渡す ---
        self.declare_parameter('align_kp', 1.2)
        self.declare_parameter('align_w_max_rps', 0.6)
        self.declare_parameter('turn_in_place_w_min_rps', 0.40)
        self.declare_parameter('align_tolerance_rad', 0.09)
        self.declare_parameter('adjust_translate_v_mps', 0.10)
        self.declare_parameter('adjust_translate_tolerance_m', 0.02)
        self._kp = float(self.get_parameter('align_kp').value)
        self._w_max = float(self.get_parameter('align_w_max_rps').value)
        self._w_min = float(self.get_parameter('turn_in_place_w_min_rps').value)
        self._tol_rad = float(self.get_parameter('align_tolerance_rad').value)
        self._v_trans = float(self.get_parameter('adjust_translate_v_mps').value)
        self._tol_m = float(
            self.get_parameter('adjust_translate_tolerance_m').value)

        self._mode = ''
        self._state = ''
        self._active = None        # dict（kind・目標など）
        self._start_x = None       # 並進の始点（odom→base_link の x, y, yaw）
        self._start_yaw = None
        self._prev_yaw = None
        self._accum = 0.0          # 旋回の積み上げ
        self._zeroed = False       # 退出で 0 を出たか

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._setup_io()
        self.create_timer(_TIMER_PERIOD_S, self._on_timer)

    # ------------------------------------------------------------
    def _setup_io(self):
        self.create_subscription(
            SystemState, '/system/state', self._on_state, 20)
        self.create_subscription(
            StateEffect, '/system/effect', self._on_effect, 10)
        cmd_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_cmd = self.create_publisher(
            Twist, '/cmd_vel_behavior', cmd_qos)
        event_qos = QoSProfile(
            depth=20, reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_event = self.create_publisher(
            StateEvent, '/system/event', event_qos)

    # ------------------------------------------------------------
    def _on_state(self, msg):
        was_adjust = self._state == 'ADJUST'
        self._mode, self._state = str(msg.mode), str(msg.state)
        now_adjust = self._state == 'ADJUST'
        if was_adjust and not now_adjust and self._active is not None:
            # ADJUST から出た（どの理由でも）→ 0 を 1 回出して黙る。
            if not self._zeroed:
                self._pub_cmd.publish(Twist())
                self._zeroed = True
            self._active = None
        self._zeroed = False if now_adjust else self._zeroed

    def _on_effect(self, msg):
        if str(msg.name) != 'measure_adjust':
            return
        try:
            args = json.loads(msg.args_json or '{}')
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(args, dict):
            return
        kind = str(args.get('kind') or '')
        # effect は状態が変わる前に届く（state_manager は effects を適用してから
        # /system/state を出す）。ここでは受け取って保持し、/system/state が
        # ADJUST になってから動き出す（venue_navigator の measure_goal と同じ作り）。
        if kind not in ('turn', 'translate'):
            return
        self._active = {
            'command_id': str(args.get('command_id') or ''),
            'kind': kind,
            'angle_rad': float(args.get('angle_rad') or 0.0),
            'side': str(args.get('side') or ''),
            'distance_m': float(args.get('distance_m') or 0.0),
        }
        self._accum = 0.0
        self._start_x = self._start_y = self._start_yaw = None
        self._prev_yaw = None
        self._zeroed = False

    # ------------------------------------------------------------
    def _odom_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException,
                tf2_ros.ConnectivityException):
            return None
        t = tf.transform.translation
        return float(t.x), float(t.y), _yaw_of(tf)

    def _emit_done(self):
        assert self._active is not None
        msg = StateEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.event = 'evt.adjust_done'
        msg.source_node = 'measure_adjuster'
        msg.arg_json = json.dumps(
            {'command_id': self._active['command_id']})
        self._pub_event.publish(msg)

    # ------------------------------------------------------------
    def _on_timer(self):
        if self._state != 'ADJUST' or self._active is None:
            return
        pose = self._odom_pose()
        if pose is None:
            return
        x, y, yaw = pose
        # 並進の始点を今の pose で揃える（最初の一回）。
        if self._active['kind'] == 'translate' and self._start_yaw is None:
            self._start_x, self._start_y, self._start_yaw = x, y, yaw
        # 旋回の積み上げ（巻き戻さない。3.5 rad が逆向きの近道を回らない）。
        if self._prev_yaw is None:
            self._prev_yaw = yaw
        else:
            self._accum = accumulate_yaw(yaw, self._prev_yaw, self._accum)
            self._prev_yaw = yaw
        out = Twist()
        if self._active['kind'] == 'turn':
            target = self._active['angle_rad']
            if turn_done(self._accum, target, self._tol_rad):
                self._pub_cmd.publish(Twist())
                self._emit_done()
                self._active = None
                return
            out.angular.z = float(
                turn_wzr(self._accum, target, self._kp, self._w_max,
                         self._w_min))
        else:
            target = self._active['distance_m']
            # 進んだ距離は odom の始点からの直線距離（向きを保つ動き）。
            dist = math.hypot(x - self._start_x, y - self._start_y)
            # BACKWARD は後ろへ（-x）。向きは始点の yaw を保つ。
            if self._active['side'] == 'BACKWARD':
                out.linear.x = -self._v_trans
            else:
                out.linear.x = self._v_trans
            out.angular.z = float(
                heading_hold(yaw, self._start_yaw, self._kp, self._w_max))
            if translate_done(dist, target, self._tol_m):
                self._pub_cmd.publish(Twist())
                self._emit_done()
                self._active = None
                return
        self._pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MeasureAdjuster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
