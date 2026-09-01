#!/usr/bin/env python3
# ============================================================
# replay_runner — 教示再生の走行（demo-teach-replay / P4）
# ============================================================
# /system/effect の load_route / rotate_to_start_yaw / resume_path を受け、
# route_replay_core の pure-pursuit で /cmd_vel_behavior (priority 20) に (v, w) を出す。
# 逆再生は点列反転。到着で evt.arrived を /system/event に返す。
#
# 特例（EXCEPTION-LEDGER W-01 / W-02）:
#   - 初期姿勢推定は省略し、load_route 受信時に即 evt.localize_done を発行（W-01）
#   - 走行中の自己位置補正なし（W-02）
# 速度指令の宛先は必ず /cmd_vel_behavior。/cmd_vel には直接 publish しない
# （obstacle_limiter だけが /cmd_vel の publisher。不変ルール WP-SAFE-03）。
# 純コア (route_replay_core / route_record_core) は import して使う（コピペしない）。
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from th_system_msgs.msg import (RouteInfo, RouteStatus, StateEffect,
                                StateEvent, SystemState)


def _points_to_path(points, frame_id='odom', stamp=None):
    """[(x,y,yaw),...] を nav_msgs/Path（frame_id）にする。"""
    path = Path()
    path.header.frame_id = frame_id
    if stamp is not None:
        path.header.stamp = stamp
    for (x, y, _yaw) in points:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path

from th_planning.route_record_core import polyline_length, route_from_dict
from th_planning.route_replay_core import (
    ReplayParams, advance_index, align_path_to_current, pure_pursuit,
    ramp_toward, reverse_points, rotate_toward,
)


def _yaw_from_quat(q) -> float:
    """クォータニオン (w,x,y,z フィールドを持つ) から yaw [rad] を返す。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ReplayRunner(Node):
    def __init__(self):
        super().__init__('replay_runner')

        # ── パラメータ ──────────────────────────────────────
        # WAIVER(demo): W-03 — 数値は registry.yaml 経由でなくノード内リテラル既定値
        self.declare_parameter('routes_dir', '/root/th_data/routes')
        self.declare_parameter('control_period_ms', 50)   # 20Hz
        self.declare_parameter('status_period_ms', 500)
        self.declare_parameter('lookahead_m', 0.40)
        # 2026-09-01: 実機で DRIVE_RUNAWAY を踏んだため巡航・旋回を下げてランプ化した（#2）。
        self.declare_parameter('cruise_speed_mps', 0.18)
        self.declare_parameter('max_yaw_rate_rps', 0.5)
        self.declare_parameter('arrive_dist_m', 0.20)
        self.declare_parameter('yaw_tol_rad', 0.10)
        self.declare_parameter('linear_accel_mps2', 0.5)
        self.declare_parameter('angular_accel_rps2', 1.5)
        self._routes_dir = self.get_parameter('routes_dir').value
        control_ms = self.get_parameter('control_period_ms').value
        status_ms = self.get_parameter('status_period_ms').value
        dt = control_ms / 1000.0
        self._max_dv = self.get_parameter('linear_accel_mps2').value * dt
        self._max_dw = self.get_parameter('angular_accel_rps2').value * dt

        # ── 純コアのパラメータ束（指示の値をそのまま使う）──
        self._params = ReplayParams(
            lookahead_m=self.get_parameter('lookahead_m').value,
            cruise_speed_mps=self.get_parameter('cruise_speed_mps').value,
            max_yaw_rate_rps=self.get_parameter('max_yaw_rate_rps').value,
            arrive_dist_m=self.get_parameter('arrive_dist_m').value,
            yaw_tol_rad=self.get_parameter('yaw_tol_rad').value)

        # ── 状態 ────────────────────────────────────────────
        self._route = None
        self._points = []
        self._start_yaw = 0.0
        self._from_index = 0
        self._need_rotate = False
        self._rotated = False
        self._arrived_sent = False
        self._was_moving = False
        self._target_index = -1
        self._cur_v = 0.0   # ランプ後の実際の publish 値
        self._cur_w = 0.0
        self._pose = None
        self._mode = ""
        self._state = ""

        # ── Subscribers ────────────────────────────────────
        effect_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(StateEffect, '/system/effect', self._on_effect, effect_qos)

        state_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(SystemState, '/system/state', self._on_state, state_qos)

        odom_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                              history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/odom', self._on_odom, odom_qos)

        # ── Publishers（速度は /cmd_vel_behavior のみ）──────
        cmd_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel_behavior', cmd_qos)
        self._pub_event = self.create_publisher(StateEvent, '/system/event', cmd_qos)
        self._pub_status = self.create_publisher(RouteStatus, '/route/status', cmd_qos)
        self._pub_preview = self.create_publisher(Path, '/route/preview', cmd_qos)

        # ── Timers ─────────────────────────────────────────
        self.create_timer(control_ms / 1000.0, self._control_timer)
        self.create_timer(status_ms / 1000.0, self._status_timer)

        self.get_logger().info('replay_runner 起動')

    # ── 購読コールバック ──────────────────────────────────
    def _on_effect(self, msg: StateEffect):
        name = msg.name
        if name == 'load_route':
            args = json.loads(msg.args_json or '{}')
            route_id = args.get('route_id')
            rev_arg = args.get('reverse')
            if isinstance(rev_arg, str):
                reverse = rev_arg.lower() in ('true', '1', 'yes')
            else:
                reverse = bool(rev_arg)
            path = os.path.join(self._routes_dir, f'{route_id}.json')
            try:
                with open(path, encoding='utf-8') as f:
                    self._route = route_from_dict(json.load(f))
            except Exception as e:
                self.get_logger().error(f'経路を読み込めない: {path}: {e}')
                return
            pts = list(self._route.points)
            if reverse:
                pts = reverse_points(pts)
            rec_start_yaw = pts[0][2] if reverse else self._route.start_yaw
            # WAIVER(demo): W-01 — LiDAR による初期姿勢推定を省略する代わりに、
            # 「今いる場所を記録の始点とみなす」。記録経路を現在 odom へ 2D 剛体変換で
            # 移し、その場から経路の形を辿れるようにする（align_path_to_current）。
            aligned = False
            if self._pose is not None and pts:
                recorded_start = (pts[0][0], pts[0][1], rec_start_yaw)
                pts = align_path_to_current(pts, recorded_start, self._pose)
                aligned = True
            self._points = pts
            self._start_yaw = pts[0][2] if pts else rec_start_yaw
            self._from_index = 0
            self._target_index = -1
            self._need_rotate = False
            self._rotated = False
            self._arrived_sent = False
            self.get_logger().info(
                f'load_route: id={route_id} reverse={reverse} 点={len(pts)} '
                f'現在地合わせ={"あり" if aligned else "なし(odom 未受信)"} '
                '(WAIVER(demo): W-01 により即 evt.localize_done を発行)')
            self._emit_event('evt.localize_done')
        elif name == 'rotate_to_start_yaw':
            self._need_rotate = True
            self._rotated = False
        elif name == 'resume_path':
            # _from_index から追従を続ける（特に何もしない）
            self.get_logger().info('resume_path: 追従を再開')
        elif name == 'widen_search':
            # WAIVER(demo): W-01 — デモでは来ない想定。ログのみ。
            self.get_logger().info('widen_search: 受信したが探索省略（WAIVER(demo): W-01）')
        elif name == 'global_localize':
            # WAIVER(demo): W-01 — no-op。念のため READY に進める。
            self.get_logger().info('global_localize: no-op（WAIVER(demo): W-01）')
            self._emit_event('evt.localize_done')
        else:
            self.get_logger().debug(f'無視する effect: {name}')

    def _on_state(self, msg: SystemState):
        self._mode = msg.mode
        self._state = msg.state

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, _yaw_from_quat(msg.pose.pose.orientation))

    def _publish_ramped(self, target_v: float, target_w: float):
        """目標 (v, w) へランプで近づけてから publish（#2: DRIVE_RUNAWAY 対策）。"""
        self._cur_v = ramp_toward(self._cur_v, target_v, self._max_dv)
        self._cur_w = ramp_toward(self._cur_w, target_w, self._max_dw)
        t = Twist()
        t.linear.x = self._cur_v
        t.angular.z = self._cur_w
        self._pub_cmd.publish(t)
        self._was_moving = abs(self._cur_v) > 1e-6 or abs(self._cur_w) > 1e-6

    # ── 制御ループ（20Hz）────────────────────────────────
    def _control_timer(self):
        # 走行条件が揃っていない → ランプで 0 へ戻す（急な 0 もステップになる）
        if self._mode != 'REPLAY' or self._state != 'RUN' or \
                self._route is None or self._pose is None:
            if self._was_moving:
                self._publish_ramped(0.0, 0.0)
            return

        if self._need_rotate and not self._rotated:
            w, done = rotate_toward(self._pose[2], self._start_yaw, self._params)
            self._publish_ramped(0.0, w)
            if done:
                self._rotated = True
            return

        # 追従フェーズ
        # WAIVER(demo): W-02 走行中の自己位置補正なし（/odom だけで辿る）
        cmd = pure_pursuit(self._pose, self._points, self._params, self._from_index)
        self._from_index = advance_index(self._pose, self._points, self._from_index, self._params)
        self._target_index = cmd.target_index
        if cmd.arrived:
            self._publish_ramped(0.0, 0.0)
            if not self._arrived_sent:
                self._emit_event('evt.arrived')
                self._arrived_sent = True
            return
        self._publish_ramped(cmd.v, cmd.w)
        self._was_moving = True

    # ── イベント発行 ─────────────────────────────────────
    def _emit_event(self, name: str):
        ev = StateEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.event = name
        ev.source_node = 'replay_runner'
        ev.arg_json = '{}'
        self._pub_event.publish(ev)
        self.get_logger().info(f'/system/event 発行: {name}')

    # ── ステータス publish ───────────────────────────────
    def _status_timer(self):
        stamp = self.get_clock().now().to_msg()
        msg = RouteStatus()
        msg.header.stamp = stamp
        msg.state = self._state or 'NONE'
        msg.points = len(self._points)
        msg.recorded_m = float(polyline_length(self._points))
        msg.elapsed_sec = 0.0  # 再生では未使用
        msg.target_index = int(self._target_index)
        if self._route is not None:
            info = RouteInfo()
            info.id = self._route.id
            info.name = self._route.name
            info.point_count = len(self._points)
            msg.current = info
        self._pub_status.publish(msg)
        # 現在地合わせ済みの点列を odom フレームの Path でプレビュー配信する。
        # #4: 経路を読んでいるときだけ出す（空 Path を出すと、記録側と交互配信になり
        # WebUI のプレビューが点滅する）。
        if self._route is not None and self._points:
            self._pub_preview.publish(_points_to_path(self._points, stamp=stamp))


def main(args=None):
    rclpy.init(args=args)
    node = ReplayRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
