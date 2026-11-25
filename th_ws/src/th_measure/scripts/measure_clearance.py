#!/usr/bin/env python3
"""measure_clearance.py — 角が描く円の中の障害物の確認（8-g P5b）と
腕が振れる範囲の確認（8-g P4）。

正本は docs/plan/detailed/DetailedDesign-measure.md §6.2 P5b・§6.5 と
docs/plan/spec/Spec-measure.md §4.4・§5.2。

  - `/scan_filtered`（センサ QoS）を `base_link` へ変換した点と、局所コスト
    マップ（`/local_costmap/costmap`。届いていなければスキャンだけ）の占有
    セルから、`base_link` 中心・半径 `corner_clear_radius_m`（0.689 m。§6.8）の円の中に障害物があるかを
    **5 Hz** で `/measure/corner_status`（`std_msgs/String` の JSON
    `{"clear": bool, "reason": str}`）へ出す。
  - スキャンが `arm_clearance_scan_max_age_ms` より古い・TF が取れないときは
    clear=false（理由つき）。
  - サービス `/measure/check_arm_clearance`（P4。`CheckArmClearance`）を持つ。
    `/system/state` を購読して `AUTO_MEASURE`/`WAIT` と
    `AT_PANEL`/`WORKING` だけで受け（ほかは `state_not_accepting`。未受信も）、
    `region` を向きごとに回して腕が振れる範囲に物が無いかを確かめる。
    **状態は変えない**ので `state_manager` を通さない。
  - 受け手は `th_state` の `state_manager`（1 秒届かなければ clear=false とみなす）。

`/cmd_vel*` へは一切出さない（本ノードは状態の発行だけ）。
"""
import json
import math

import rclpy
import tf2_ros
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from th_system_msgs.msg import SystemState
from th_system_msgs.srv import CheckArmClearance

from th_measure.clearance_core import (CORNER_LETHAL_MIN, SIDE_ROTATION_RAD,
                                       any_point_in_region,
                                       circle_blocked_by_costmap,
                                       combine_free, decide_clear,
                                       decide_side, free_distance,
                                       free_distance_from_grid,
                                       points_in_circle, polygon_blocked_by_costmap,
                                       region_overlaps_blind, rotate_polygon,
                                       validate_arm_request)

_STATUS_PERIOD_S = 0.2  # 5 Hz（§6.2 P5b）

# 腕の範囲の確認（P4。§6.5）で受ける状態。ADJUST を含むほかは拒否する。
_ACCEPT_STATES = frozenset({
    ('AUTO_MEASURE', 'WAIT'),
    ('AT_PANEL', 'WORKING'),
})


class _GridView:
    """OccupancyGrid を純コア（circle_blocked_by_costmap）が読む形にする薄いラッパ。"""

    def __init__(self, msg):
        info = msg.info
        self.width = int(info.width)
        self.height = int(info.height)
        self.resolution = float(info.resolution)
        self.origin_x = float(info.origin.position.x)
        self.origin_y = float(info.origin.position.y)
        self.data = msg.data


def _apply_tf(tf, x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
    """TransformStamped を点 (x, y, z) に適用する（回転は四元数から）。"""
    t = tf.transform.translation
    q = tf.transform.rotation
    x2, y2, z2, w2 = float(q.x), float(q.y), float(q.z), float(q.w)
    # 回転行列（単位四元数）で回す。
    r00 = 1.0 - 2.0 * (y2 * y2 + z2 * z2)
    r01 = 2.0 * (x2 * y2 - z2 * w2)
    r02 = 2.0 * (x2 * z2 + y2 * w2)
    r10 = 2.0 * (x2 * y2 + z2 * w2)
    r11 = 1.0 - 2.0 * (x2 * x2 + z2 * z2)
    r12 = 2.0 * (y2 * z2 - x2 * w2)
    r20 = 2.0 * (x2 * z2 - y2 * w2)
    r21 = 2.0 * (y2 * z2 + x2 * w2)
    r22 = 1.0 - 2.0 * (x2 * x2 + y2 * y2)
    return (r00 * x + r01 * y + r02 * z + float(t.x),
            r10 * x + r11 * y + r12 * z + float(t.y),
            r20 * x + r21 * y + r22 * z + float(t.z))


class MeasureClearance(Node):

    def __init__(self, **kwargs):
        super().__init__('measure_clearance', **kwargs)

        # --- R2: 裸の数値を書かない。registry 由来は生成 yaml が渡す ---
        # arm_clearance_scan_max_age_ms は INTEGER（measure_link の
        # measure_link_timeout_ms と同じ理由）。
        self.declare_parameter(
            'arm_clearance_scan_max_age_ms', Parameter.Type.INTEGER)
        self._max_age_ms = int(
            self.get_parameter('arm_clearance_scan_max_age_ms').value)
        # 8-g P3: 並進の「進む先」の free_*_m の材料（floor_margin_m と死角）。
        # 生成 yaml が渡す。無い環境では既定（リミッタの既定余裕と空の死角）。
        self.declare_parameter('floor_margin_m', 0.30)
        self._floor_margin_m = float(self.get_parameter('floor_margin_m').value)
        # 8-g P4: 腕の範囲の確認で死角の判定から除く近い部分
        # （arm_clearance_blind_near_m）。生成 yaml が渡す。
        self.declare_parameter('corner_clear_radius_m', 0.689)
        self._corner_radius_m = float(
            self.get_parameter('corner_clear_radius_m').value)
        self.declare_parameter('arm_clearance_blind_near_m', 0.65)
        self._blind_near_m = float(
            self.get_parameter('arm_clearance_blind_near_m').value)
        # lidar_filter と同じ理由（DOUBLE_ARRAY の既定 [] 推論を避ける）。
        self.declare_parameter(
            'blind_angle_ranges', [],
            ParameterDescriptor(dynamic_typing=True))
        _blind_deg = list(self.get_parameter('blind_angle_ranges').value or [])
        self._blind_rad = tuple(
            (math.radians(float(_blind_deg[i])), math.radians(float(_blind_deg[i + 1])))
            for i in range(0, len(_blind_deg) - 1, 2))
        # 実行時に blind_angle_ranges が変わったら読み直す（試験 d は
        # 生成 yaml を渡した同じノードの値を書き換える）。
        self.add_on_set_parameters_callback(self._on_params_set)

        self._scan = None          # 最新の LaserScan
        self._scan_rx_ms = None    # 受信時刻（ms。自分の時計）
        self._costmap = None       # 最新の OccupancyGrid（届いていなければ None）
        # P4: `/system/state` の最新値（未受信は「受けない状態」）。
        self._sys_mode = None
        self._sys_state = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._setup_io()
        self.create_timer(_STATUS_PERIOD_S, self._on_timer)

    # ------------------------------------------------------------
    def _setup_io(self):
        self.create_subscription(
            LaserScan, '/scan_filtered', self._on_scan,
            qos_profile_sensor_data)
        costmap_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._on_costmap,
            costmap_qos)
        pub_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_status = self.create_publisher(
            String, '/measure/corner_status', pub_qos)
        # P4: 状態を覚える（状態は変えないので state_manager を通さない）。
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(
            SystemState, '/system/state', self._on_state, state_qos)
        self._srv_arm = self.create_service(
            CheckArmClearance, '/measure/check_arm_clearance',
            self._on_check_arm_clearance)

    # ------------------------------------------------------------
    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def _on_scan(self, msg):
        self._scan = msg
        self._scan_rx_ms = self._now_ms()

    def _on_state(self, msg):
        self._sys_mode = str(msg.mode or '')
        self._sys_state = str(msg.state or '')

    def _on_params_set(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'blind_angle_ranges':
                deg = list(p.value or [])
                self._blind_rad = tuple(
                    (math.radians(float(deg[i])), math.radians(float(deg[i + 1])))
                    for i in range(0, len(deg) - 1, 2))
            elif p.name == 'corner_clear_radius_m':
                self._corner_radius_m = float(p.value)
            elif p.name == 'arm_clearance_blind_near_m':
                self._blind_near_m = float(p.value)
        return SetParametersResult(successful=True)

    def _on_costmap(self, msg):
        self._costmap = msg

    # ------------------------------------------------------------
    def _scan_points_base(self):
        """`/scan_filtered` の有効点を `base_link` へ変換した (x, y) の列。

        TF が取れなければ None（確認できない）。"""
        scan = self._scan
        if scan is None:
            return None
        frame = str(scan.header.frame_id or 'laser_link')
        try:
            tf = self._tf_buffer.lookup_transform(
                'base_link', frame, rclpy.time.Time())
        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException,
                tf2_ros.ConnectivityException):
            return None
        angle_min = float(scan.angle_min)
        angle_inc = float(scan.angle_increment)
        rmin = float(scan.range_min)
        rmax = float(scan.range_max)
        points = []
        for i, r in enumerate(scan.ranges):
            r = float(r)
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            a = angle_min + i * angle_inc
            bx, by, _ = _apply_tf(tf, r * math.cos(a), r * math.sin(a))
            points.append((bx, by))
        return points

    def _costmap_blocked(self):
        """局所コストマップの占有セルが円の中にあるか。
        届いていなければ False（スキャンだけ。§6.2 P5b）。
        TF が取れなければ None（確認できない。呼び側で clear=false に倒す）。"""
        cm = self._costmap
        if cm is None:
            return False
        frame = str(cm.header.frame_id or '')
        try:
            # base_link 中心をコストマップの座標系へ変換する。
            tf = self._tf_buffer.lookup_transform(
                frame, 'base_link', rclpy.time.Time())
        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException,
                tf2_ros.ConnectivityException):
            return None
        cx, cy, _ = _apply_tf(tf, 0.0, 0.0)
        return circle_blocked_by_costmap(
            _GridView(cm), cx, cy, self._corner_radius_m,
            blocked_min=CORNER_LETHAL_MIN)

    # ------------------------------------------------------------
    # P4: 腕が振れる範囲の確認（/measure/check_arm_clearance）
    # ------------------------------------------------------------
    def _costmap_basis(self):
        """base_link 原点と +x・+y をコストマップの座標系へ変換したもの。

        届いていなければ None、TF が取れなければ 'tf_fail' を返す
        （呼び側で「確認できない」に倒す。角の円の確認と同じ）。
        """
        cm = self._costmap
        if cm is None:
            return None
        frame = str(cm.header.frame_id or '')
        try:
            tf = self._tf_buffer.lookup_transform(
                frame, 'base_link', rclpy.time.Time())
        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException,
                tf2_ros.ConnectivityException):
            return 'tf_fail'
        ox, oy, _ = _apply_tf(tf, 0.0, 0.0)
        ex, ey, _ = _apply_tf(tf, 1.0, 0.0)
        fx, fy, _ = _apply_tf(tf, 0.0, 1.0)
        return (ox, oy, (ex - ox, ey - oy), (fx - ox, fy - oy))

    def _on_check_arm_clearance(self, request, response):
        points = [(float(p.x), float(p.y)) for p in request.region.points]
        sides = [str(s) for s in request.sides]
        # 拒否の順: bad_argument → state_not_accepting（§6.5）。拒否のとき
        # 配列は空。状態は変えない。
        if validate_arm_request(points, sides):
            response.accepted = False
            response.reject_reason = 'bad_argument'
            response.sides = []
            response.results = []
            response.unknown_reasons = []
            response.checked_at = Time()
            return response
        if (self._sys_mode, self._sys_state) not in _ACCEPT_STATES:
            response.accepted = False
            response.reject_reason = 'state_not_accepting'
            response.sides = []
            response.results = []
            response.unknown_reasons = []
            response.checked_at = Time()
            return response
        now_ms = self._now_ms()
        scan_points = self._scan_points_base()
        tf_ok = scan_points is not None
        scan_age_ms = (None if self._scan_rx_ms is None
                       else now_ms - self._scan_rx_ms)
        basis = self._costmap_basis()
        if basis == 'tf_fail':
            tf_ok = False
            basis = None
        results = []
        reasons = []
        for side in sides:
            polygon = rotate_polygon(points, SIDE_ROTATION_RAD[side])
            blocked = any_point_in_region(scan_points, polygon) \
                if scan_points is not None else False
            if not blocked and basis is not None:
                ox, oy, bx, by = basis
                blocked = polygon_blocked_by_costmap(
                    _GridView(self._costmap), ox, oy, bx, by, polygon)
            blind_hit = region_overlaps_blind(
                polygon, self._blind_rad, self._blind_near_m)
            result, reason = decide_side(
                tf_ok, scan_age_ms, float(self._max_age_ms),
                blocked, blind_hit)
            results.append(result)
            reasons.append(reason)
        response.accepted = True
        response.reject_reason = ''
        response.sides = list(sides)
        response.results = results
        response.unknown_reasons = reasons
        # checked_at は判定に使ったスキャンの時刻（無ければ 0）。
        response.checked_at = (self._scan.header.stamp
                               if self._scan is not None else Time())
        return response

    def _on_timer(self):
        now_ms = self._now_ms()
        points = self._scan_points_base()
        tf_ok = points is not None
        scan_age_ms = None if self._scan_rx_ms is None else now_ms - self._scan_rx_ms
        scan_blocked = points_in_circle(points, self._corner_radius_m) if points is not None else False
        costmap_blocked = self._costmap_blocked() if tf_ok else False
        if costmap_blocked is None:
            tf_ok = False
            costmap_blocked = False
        clear, reason = decide_clear(
            tf_ok, scan_age_ms, float(self._max_age_ms),
            scan_blocked, costmap_blocked)
        # 8-g P3: 並進の「進む先」の空き距離（TF 無し・古いスキャン・死角は 0）。
        scan_ok = bool(tf_ok and scan_age_ms is not None
                       and scan_age_ms <= float(self._max_age_ms))
        scan_free_f = scan_free_b = 0.0
        if scan_ok and points is not None:
            scan_free_f = free_distance(
                points, True, self._blind_rad, self._floor_margin_m)
            scan_free_b = free_distance(
                points, False, self._blind_rad, self._floor_margin_m)
        map_ok = bool(tf_ok and self._costmap is not None
                      and costmap_blocked is not None)
        map_free_f = map_free_b = 0.0
        if map_ok:
            map_free_f = self._costmap_free(True)
            map_free_b = self._costmap_free(False)
        msg = String()
        msg.data = json.dumps({
            "clear": bool(clear), "reason": reason,
            "free_forward_m": float(combine_free(scan_free_f, map_free_f,
                                                 scan_ok, map_ok)),
            "free_backward_m": float(combine_free(scan_free_b, map_free_b,
                                                  scan_ok, map_ok)),
        })
        self._pub_status.publish(msg)

    def _costmap_free(self, forward: bool) -> float:
        """局所コストマップの占有セルからの free 距離（届いていなければ None 換算の 0）。"""
        cm = self._costmap
        if cm is None:
            return 0.0
        frame = str(cm.header.frame_id or '')
        try:
            tf = self._tf_buffer.lookup_transform(
                frame, 'base_link', rclpy.time.Time())
        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException,
                tf2_ros.ConnectivityException):
            return 0.0
        cx, cy, _ = _apply_tf(tf, 0.0, 0.0)
        # base_link の +x を grid 座標系へ（回転だけ）。
        q = tf.transform.rotation
        x2, y2, z2, w2 = float(q.x), float(q.y), float(q.z), float(q.w)
        dir_x = 1.0 - 2.0 * (y2 * y2 + z2 * z2)
        dir_y = 2.0 * (x2 * y2 + z2 * w2)
        return free_distance_from_grid(
            _GridView(cm), cx, cy, forward, dir_x, dir_y, self._blind_rad,
            self._floor_margin_m)


def main(args=None):
    rclpy.init(args=args)
    node = MeasureClearance()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
