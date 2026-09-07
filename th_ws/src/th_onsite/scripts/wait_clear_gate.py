#!/usr/bin/env python3
# ============================================================
# wait_clear_gate — 退避待ちゲート (WP-ONSITE-03)
# ============================================================
# SUMMON/WAIT_CLEAR で、試験員がゴールから clear_distance_m 以上離れ
# clear_hold_ms 継続したら evt.clear_ok、clear_timeout_ms 超で
# evt.clear_timeout を /system/event へ publish する。
# 見失い中は hold カウンタを 0 に戻す。
#
# /onsite/wait_clear (WaitClearStatus, 5Hz) で退避状況を配信。
import math
import signal
import sys
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)

import tf2_ros

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from th_system_msgs.msg import PersonStatus, StateEvent, SystemState, WaitClearStatus
from th_onsite.wait_clear_core import (
    WaitClearParams, WaitClearState, remaining_sec, reset, step,
)


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaitClearGate(Node):
    def __init__(self):
        super().__init__('wait_clear_gate')

        # ── パラメータ ──────────────────────────────────────
        # WAIVER(demo): W-13 — 数値はノード内リテラル既定値
        self.declare_parameter('clear_distance_m', 1.0)
        self.declare_parameter('clear_hold_ms', 1500)
        self.declare_parameter('clear_timeout_ms', 30000)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tick_hz', 20)

        self._params = WaitClearParams(
            clear_distance_m=float(self.get_parameter('clear_distance_m').value),
            clear_hold_ms=int(self.get_parameter('clear_hold_ms').value),
            clear_timeout_ms=int(self.get_parameter('clear_timeout_ms').value),
        )
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        tick_hz = max(1, int(self.get_parameter('tick_hz').value))
        self._dt_ms = int(round(1000.0 / tick_hz))

        # ── 状態 ────────────────────────────────────────────
        self._state = reset()
        self._in_wait_clear = False   # 直前ティックが SUMMON/WAIT_CLEAR だったか
        self._mode = ""
        self._sub_mode_state = ""     # "MODE/STATE" 形式
        self._person = None           # 直近 PersonStatus
        self._goal = None             # 直近 PoseStamped (map フレーム)
        self._last_distance = 0.0
        self._last_verdict = 'NOT_CLEAR'   # _tick が step() から受け取った verdict

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ─────────────────────────────────────────────
        state_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               history=QoSHistoryPolicy.KEEP_LAST)
        event_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                               history=QoSHistoryPolicy.KEEP_LAST)
        goal_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              history=QoSHistoryPolicy.KEEP_LAST)
        wait_clear_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                    history=QoSHistoryPolicy.KEEP_LAST)

        sub_cbg = ReentrantCallbackGroup()

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(SystemState, '/system/state', self._on_state,
                                 state_qos, callback_group=sub_cbg)
        self.create_subscription(PersonStatus, '/person/status', self._on_person,
                                 10, callback_group=sub_cbg)
        self.create_subscription(PoseStamped, '/onsite/summon_goal', self._on_goal,
                                 goal_qos, callback_group=sub_cbg)

        # ── Publishers ──────────────────────────────────────
        self._pub_event = self.create_publisher(
            StateEvent, '/system/event', event_qos)
        self._pub_wait_clear = self.create_publisher(
            WaitClearStatus, '/onsite/wait_clear', wait_clear_qos)

        # ── タイマー (20Hz) ────────────────────────────────
        self.create_timer(1.0 / tick_hz, self._tick)

        # ── 5Hz publish タイマー ────────────────────────────
        self._wc_timer = self.create_timer(0.2, self._publish_wait_clear)

        self.get_logger().info(
            f'wait_clear_gate 起動 '
            f'(clear_distance={self._params.clear_distance_m}m, '
            f'hold={self._params.clear_hold_ms}ms, '
            f'timeout={self._params.clear_timeout_ms}ms)')

    # ── /system/state 受信 ─────────────────────────────────
    def _on_state(self, msg: SystemState):
        self._mode = msg.mode
        self._sub_mode_state = f'{msg.mode}/{msg.state}'

    # ── /person/status 受信 ────────────────────────────────
    def _on_person(self, msg: PersonStatus):
        self._person = msg

    # ── /onsite/summon_goal 受信 ───────────────────────────
    def _on_goal(self, msg: PoseStamped):
        self._goal = msg

    # ── 座標変換 ──────────────────────────────────────────
    def _map_pose(self):
        """map→base_link TF から (x, y, yaw) を返す。取れなければ None。"""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y, _yaw_from_quat(tf.transform.rotation))

    def _person_map_xy(self) -> tuple[float, float] | None:
        """直近の対象 (base_link 相対) を map 座標に変換して (x, y) を返す。"""
        if self._person is None or self._person.is_lost:
            return None
        mp = self._map_pose()
        if mp is None:
            return None
        robot_x, robot_y, robot_yaw = mp
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        bl_x = float(self._person.position.x)
        bl_y = float(self._person.position.y)
        map_x = robot_x + cos_yaw * bl_x - sin_yaw * bl_y
        map_y = robot_y + sin_yaw * bl_x + cos_yaw * bl_y
        return (map_x, map_y)

    # ── /system/event publish ─────────────────────────────
    def _emit_event(self, event: str):
        ev = StateEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.event = event
        ev.source_node = 'wait_clear_gate'
        ev.arg_json = '{}'
        self._pub_event.publish(ev)
        self.get_logger().info(f'/system/event 発行: {event}')

    # ── 20Hz タイマ ──────────────────────────────────────
    def _tick(self):
        is_wc = (self._mode == 'SUMMON' and self._sub_mode_state == 'SUMMON/WAIT_CLEAR')

        # WAIT_CLEAR に入った瞬間 → reset
        if is_wc and not self._in_wait_clear:
            self._state = reset()
        self._in_wait_clear = is_wc

        if not is_wc:
            return

        # 対象 map 位置とゴールの距離
        person_xy = self._person_map_xy()
        if self._goal is None or person_xy is None:
            distance_m = None
            target_visible = False
        else:
            gx = self._goal.pose.position.x
            gy = self._goal.pose.position.y
            distance_m = math.hypot(person_xy[0] - gx, person_xy[1] - gy)
            target_visible = True
            self._last_distance = distance_m

        self._state, verdict, events = step(
            self._state, self._dt_ms, distance_m, target_visible, self._params)
        self._last_verdict = verdict

        for evt_name in events:
            self._emit_event(evt_name)

    # ── 5Hz publish ───────────────────────────────────────
    def _publish_wait_clear(self):
        msg = WaitClearStatus()
        msg.header.stamp = self.get_clock().now().to_msg()

        if not self._in_wait_clear:
            return

        msg.distance_m = self._last_distance
        msg.remaining_sec = remaining_sec(self._state, self._params)
        msg.satisfied = bool(self._state.fired_ok)
        # verdict は _tick の step() が返した値をそのまま配信する（判定を 2 か所に
        # 持たない。core と node で食い違わないため）。
        msg.verdict = self._last_verdict
        self._pub_wait_clear.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaitClearGate()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # SIGTERM 二重 shutdown ガード (ONSITE-A と同じ)
    def _sigterm_handler(_signum, _frame):
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
