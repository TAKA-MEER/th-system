#!/usr/bin/env python3
# ============================================================
# route_recorder — 教示経路の記録（demo-teach-replay / P3）
# ============================================================
# /system/effect の start_record / resume_record / finalize_route を受け、
# /odom のロボット姿勢を route_record_core で間引いて th_data/routes/<id>.json に
# 保存する。/route/catalog を latched で、/route/status を定期発行する。
# 姿勢サンプルは /system/state.state=="REC" の間だけ行い、PAUSE 中は積まない。
#
# 純コア (route_record_core) は P1 で完成済み。ここでは中身をコピペせず
# import して使う（追従ロジック follow_planner_core と同じ二層構造）。
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from th_system_msgs.msg import (RouteInfo, RouteList, RouteStatus, StateEffect,
                                SystemState)

from th_planning.route_record_core import (
    RouteRecorderCore, RouteRecordParams, polyline_length,
    route_from_dict, route_to_dict,
)


def _yaw_from_quat(q) -> float:
    """クォータニオン (w,x,y,z フィールドを持つ) から yaw [rad] を返す。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RouteRecorder(Node):
    def __init__(self):
        super().__init__('route_recorder')

        # ── パラメータ ──────────────────────────────────────
        # WAIVER(demo): W-03 — 数値は registry.yaml 経由でなくノード内リテラル既定値
        self.declare_parameter('routes_dir', '/root/th_data/routes')
        self.declare_parameter('sample_period_ms', 100)
        self.declare_parameter('sample_min_dist_m', 0.10)
        self.declare_parameter('sample_min_yaw_rad', 0.20)
        self.declare_parameter('status_period_ms', 500)
        self._routes_dir = self.get_parameter('routes_dir').value
        sample_ms = self.get_parameter('sample_period_ms').value
        status_ms = self.get_parameter('status_period_ms').value

        # ── 状態 ────────────────────────────────────────────
        self._recorder: RouteRecorderCore | None = None
        self._route_id: str | None = None
        self._name: str = ""
        self._rec_started_ms: int = 0
        self._pose: tuple[float, float, float] | None = None
        self._mode: str = ""
        self._state: str = ""

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

        # ── Publishers ─────────────────────────────────────
        routes_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_routes = self.create_publisher(RouteList, '/route/catalog', routes_qos)

        status_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_status = self.create_publisher(RouteStatus, '/route/status', status_qos)

        # ── Timers ─────────────────────────────────────────
        self.create_timer(sample_ms / 1000.0, self._sample_timer)
        self.create_timer(status_ms / 1000.0, self._status_timer)

        # 起動時に既存経路を latched publish する
        self._publish_routes_list()

        self.get_logger().info(
            f'route_recorder 起動 (routes_dir={self._routes_dir})')

    # ── 購読コールバック ──────────────────────────────────
    def _on_effect(self, msg: StateEffect):
        name = msg.name
        if name == 'start_record':
            args = json.loads(msg.args_json or '{}')
            route_id = args.get('route_id') or ''
            if not route_id:
                route_id = time.strftime('route_%Y%m%d_%H%M%S')
            self._route_id = route_id
            self._name = args.get('name') or route_id
            params = RouteRecordParams(
                sample_min_dist_m=self.get_parameter('sample_min_dist_m').value,
                sample_min_yaw_rad=self.get_parameter('sample_min_yaw_rad').value)
            self._recorder = RouteRecorderCore(params)
            if self._pose is None:
                self.get_logger().warn('odom 未受信のため (0,0,0) で記録開始')
                self._recorder.start(0.0, 0.0, 0.0)
            else:
                self._recorder.start(*self._pose)
            self._rec_started_ms = self._now_ms()
            self.get_logger().info(f'記録開始: route_id={self._route_id}')
        elif name == 'resume_record':
            if self._recorder is None:
                self.get_logger().warn('resume_record を受けたが記録中でない（無視）')
                return
            self._recorder.resume()
        elif name == 'finalize_route':
            if self._recorder is None:
                self.get_logger().warn('finalize_route を受けたが記録中でない（無視）')
                return
            route = self._recorder.finalize(
                self._route_id, self._name, self._now_ms(), frame_id='odom')
            # WAIVER(demo): W-04 — 同じ id は上書き。新版として旧版を残す世代管理はしない
            path = os.path.join(self._routes_dir, f'{self._route_id}.json')
            os.makedirs(self._routes_dir, exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(route_to_dict(route), f, ensure_ascii=False, indent=2)
            # RouteData は point_count / length_m を持たない（route_to_dict の出力だけが持つ）
            self.get_logger().info(
                f'保存: {path} ({len(route.points)} 点, '
                f'{polyline_length(route.points):.2f} m)')
            self._publish_routes_list()
        else:
            self.get_logger().debug(f"無視する effect: {name}")

    def _on_state(self, msg: SystemState):
        self._mode = msg.mode
        self._state = msg.state

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, _yaw_from_quat(msg.pose.pose.orientation))

    # ── タイマ ────────────────────────────────────────────
    def _sample_timer(self):
        # PAUSE 中は積まない。REC の間だけ姿勢を間引いて記録する。
        if self._recorder is None or self._state != 'REC':
            return
        if self._pose is None:
            return
        self._recorder.add_pose(*self._pose)

    def _status_timer(self):
        msg = RouteStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._recorder is None:
            msg.state = self._state or 'NONE'
            msg.recorded_m = 0.0
            msg.points = 0
            msg.elapsed_sec = 0.0
        else:
            msg.state = self._state or 'NONE'
            msg.recorded_m = float(self._recorder.recorded_m)
            msg.points = self._recorder.point_count
            msg.elapsed_sec = (self._now_ms() - self._rec_started_ms) / 1000.0
        self._pub_status.publish(msg)

    # ── 経路一覧 publish ──────────────────────────────────
    def _publish_routes_list(self):
        if not os.path.isdir(self._routes_dir):
            self.get_logger().debug(
                f'routes_dir が存在しないため /route/catalog は空: {self._routes_dir}')
            self._pub_routes.publish(RouteList())
            return
        infos = []
        for fname in sorted(os.listdir(self._routes_dir)):
            if not fname.endswith('.json'):
                continue
            path = os.path.join(self._routes_dir, fname)
            try:
                with open(path, encoding='utf-8') as f:
                    route = route_from_dict(json.load(f))
            except Exception as e:
                self.get_logger().warn(f'経路 JSON を読み込めないため skip: {path}: {e}')
                continue
            info = RouteInfo()
            info.id = route.id
            info.name = route.name
            info.generation = route.generation
            info.length_m = float(polyline_length(route.points))
            info.point_count = len(route.points)
            info.start_yaw = float(route.start_yaw)
            info.recorded_at = _ms_to_time(route.recorded_at_ms)
            infos.append(info)
        self._pub_routes.publish(RouteList(routes=infos))
        self.get_logger().debug(f'/route/catalog を publish: {len(infos)} 件')

    # ── ユーティリティ ────────────────────────────────────
    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000


def _ms_to_time(ms: int) -> TimeMsg:
    """エポックからのミリ秒を builtin_interfaces/Time に変換する。"""
    t = TimeMsg()
    t.sec = int(ms // 1000)
    t.nanosec = int((ms % 1000) * 1_000_000)
    return t


def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
