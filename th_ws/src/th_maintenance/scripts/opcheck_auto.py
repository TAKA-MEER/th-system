#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""起動時の自動点検 `opcheck_auto`（Spec-ops.md §2.6）。

INIT 中から常時起動し、自動で判定できる項目（ESTOP／IMU／LIDAR）だけを
判定して総合ステータスを `/opcheck/auto_status`（std_msgs/String に JSON、
transient_local）に 1 Hz で出す。データが揃うまでは `overall: CHECKING`
（判定中）。

**機体は絶対に動かさない**: `/cmd_vel_behavior` には publish しない
（publisher を持たない）。モーター確認は自動では行わない（人の点検）。

開発モード（`/system/dev_mode` の `effective` に `opcheck`）では、判定は
止めずに配信し続け、`suppressed: true` にする（画面が「消している」表示に
するための合図）。未受信・3 秒以上更新なし・読めない JSON では消さない
（auto_check_core.is_suppressed。dev_mode_core.hpp と同じ考え方）。

パラメータは `opcheck_runner` と同じ値を使う（launch が生成済み
`opcheck_runner.yaml` をそのまま渡す）。宣言は PARS（opcheck_runner.py と
同値）に `blind_angle_ranges` を加えた形にし、生成 yaml の余分なキーで
起動失敗しないようにする。
"""

import json
import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                          QoSReliabilityPolicy, qos_profile_sensor_data)
    from rcl_interfaces.msg import ParameterDescriptor
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, String, UInt8
    RS_OK = True
except ImportError:
    rclpy = None
    Node = object
    RS_OK = False

from th_maintenance.auto_check_core import (
    combine_overall,
    is_suppressed,
    judge_estop_auto,
    judge_imu_auto,
    judge_lidar_auto,
)
from th_maintenance.check_core import CheckParams

# opcheck_runner.py の PARS と同値（test_maintenance_registry_driven.py が
# registry と照合する正本は向こう。こちらは同じ生成 yaml を読むための写し）。
PARS: dict = {
    "motor_deadband_mps": 0.03,
    "motor_follow_min_ratio": 0.5,
    "imu_bias_max_rad_s": 0.2,
    "imu_wz_implausible_rad_s": 10.0,
    "opcheck_imu_window_s": 2.0,
    "opcheck_deadman_timeout_s": 0.5,
    "opcheck_spin_w_rad_s": 0.3,
    "opcheck_blind_tolerance_deg": 5.0,
    "opcheck_scan_coverage_gap_deg": 2.0,
    "v_check": 0.05,
    # 整数で宣言する（registry の value: 300 は生成 yaml に整数で載る。rclpy は
    # INTEGER↔DOUBLE を変換せず、300.0 で宣言すると起動時に
    # InvalidParameterTypeException で落ちる。2026-10-02 実機で opcheck_runner が
    # これで起動せず、始業点検の MOTOR が指令を一切出さなかった）。
    "scan_stale_ms": 300,
    # SG-B21: _ESTOP_STALE_MS を registry 行にした（opcheck_runner と同名・同値）。
    "opcheck_estop_stale_ms": 3000,
}

_PUBLISH_PERIOD_S = 1.0


class OpcheckAuto(Node):
    """起動時の自動点検。購読と判定と配信だけ（駆動への出力は持たない）。"""

    def __init__(self):
        if not RS_OK:
            raise ImportError("rclpy が無い環境で OpcheckAuto を実体化した")
        super().__init__("opcheck_auto")
        for name, value in PARS.items():
            self.declare_parameter(name, value)
        self.declare_parameter("blind_angle_ranges", [],
                               ParameterDescriptor(dynamic_typing=True))

        self._p = CheckParams(
            motor_deadband_mps=float(self.get_parameter("motor_deadband_mps").value),
            motor_follow_min_ratio=float(self.get_parameter("motor_follow_min_ratio").value),
            imu_bias_max_rad_s=float(self.get_parameter("imu_bias_max_rad_s").value),
            imu_wz_implausible_rad_s=float(
                self.get_parameter("imu_wz_implausible_rad_s").value),
            opcheck_imu_window_s=float(self.get_parameter("opcheck_imu_window_s").value),
            opcheck_deadman_timeout_s=float(
                self.get_parameter("opcheck_deadman_timeout_s").value),
            opcheck_spin_w_rad_s=float(self.get_parameter("opcheck_spin_w_rad_s").value),
            opcheck_blind_tolerance_deg=float(
                self.get_parameter("opcheck_blind_tolerance_deg").value),
            opcheck_scan_coverage_gap_deg=float(
                self.get_parameter("opcheck_scan_coverage_gap_deg").value),
            v_check=float(self.get_parameter("v_check").value),
            scan_stale_ms=float(self.get_parameter("scan_stale_ms").value),
        )

        # ── 入力の最新値 ──
        self._estop_pressed = False
        self._estop_last_ms = None
        self._imu_wz = []
        self._imu_window_start_ms = self._now_ms()
        self._imu_bias = 0.0
        self._imu_max_wz = None
        self._imu_last_ms = None
        self._imu_calib = 0
        self._scan_last_ms = None
        self._scan_prev_stamp = None
        self._scan_period = None
        self._scan_ranges: list = []
        self._scan_angle_inc = math.radians(1.0)
        self._scan_angle_min = 0.0  # rad。初回受信まで scan_alive が偽なので値は使われない
        # /system/dev_mode（JSON 文字列）と受信時刻
        self._dev_raw = None
        self._dev_recv_ms = None

        # ── I/O ──
        self.create_subscription(Bool, "/safety/estop_hw", self._on_estop_hw, 10)
        self.create_subscription(Imu, "/esp32/imu_data", self._on_imu, 10)
        self.create_subscription(
            UInt8, "/esp32/imu_calib_status", self._on_imu_calib, 10)
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        dev_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(String, "/system/dev_mode", self._on_dev_mode, dev_qos)

        status_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(String, "/opcheck/auto_status", status_qos)

        self.create_timer(_PUBLISH_PERIOD_S, self._on_timer)
        self.get_logger().info("opcheck_auto 起動（自動点検のみ。駆動への出力なし）")

    @staticmethod
    def _now_ms() -> float:
        return time.monotonic() * 1000.0

    # ── 購読 ──
    def _on_estop_hw(self, msg: Bool):
        self._estop_pressed = bool(msg.data)
        self._estop_last_ms = self._now_ms()

    def _on_imu(self, msg: Imu):
        now = self._now_ms()
        self._imu_last_ms = now
        window_ms = self._p.opcheck_imu_window_s * 1000.0
        if now - self._imu_window_start_ms >= window_ms and self._imu_wz:
            # 直近窓の統計を確定し、新しい窓を始める
            self._imu_bias = sum(self._imu_wz) / len(self._imu_wz)
            self._imu_max_wz = max(abs(w) for w in self._imu_wz)
            self._imu_wz = []
            self._imu_window_start_ms = now
        self._imu_wz.append(msg.angular_velocity.z)

    def _on_imu_calib(self, msg: UInt8):
        self._imu_calib = int(msg.data)

    def _on_scan(self, msg: LaserScan):
        self._scan_last_ms = self._now_ms()
        stamp = msg.header.stamp
        if self._scan_prev_stamp is not None:
            dt = (stamp.sec - self._scan_prev_stamp.sec) + \
                 (stamp.nanosec - self._scan_prev_stamp.nanosec) / 1.0e9
            if 0.0 < dt < 5.0:
                self._scan_period = dt
        self._scan_prev_stamp = stamp
        self._scan_ranges = list(msg.ranges)
        self._scan_angle_min = msg.angle_min
        if msg.angle_increment > 0.0:
            self._scan_angle_inc = msg.angle_increment

    def _on_dev_mode(self, msg: String):
        self._dev_raw = msg.data
        self._dev_recv_ms = self._now_ms()

    def _configured_blind(self):
        value = self.get_parameter("blind_angle_ranges").value
        return [float(v) for v in (value or [])]

    # ── 判定と配信（1 Hz） ──
    def _on_timer(self):
        now = self._now_ms()

        estop_alive = (self._estop_last_ms is not None
                       and now - self._estop_last_ms
                       <= int(self.get_parameter("opcheck_estop_stale_ms").value))
        imu_alive = (self._imu_last_ms is not None
                     and now - self._imu_last_ms <= self._p.scan_stale_ms)
        scan_alive = (self._scan_last_ms is not None
                      and now - self._scan_last_ms <= self._p.scan_stale_ms)

        if self._estop_last_ms is None and self._imu_last_ms is None \
                and self._scan_last_ms is None:
            from th_maintenance.auto_check_core import AutoItemVerdict
            checking = AutoItemVerdict("CHECKING")
            overall = combine_overall(checking, checking, checking)
        else:
            estop = judge_estop_auto(estop_alive, self._estop_pressed)
            bias = self._imu_bias
            if self._imu_wz:
                bias = sum(self._imu_wz) / len(self._imu_wz)
            imu = judge_imu_auto(self._imu_calib, bias, self._imu_max_wz,
                                 imu_alive, self._p)
            lidar = judge_lidar_auto(
                alive=scan_alive,
                period_s=self._scan_period,
                ranges=self._scan_ranges,
                angle_increment_deg=math.degrees(self._scan_angle_inc),
                configured_ranges=self._configured_blind(),
                p=self._p,
                angle_min_deg=math.degrees(self._scan_angle_min))
            overall = combine_overall(estop, imu, lidar)

        suppressed = is_suppressed(self._dev_raw, now, self._dev_recv_ms)

        payload = {
            "overall": overall.overall,
            "suppressed": suppressed,
            "items": {
                name: {"result": v.result, "reason": v.reason}
                for name, v in overall.items.items()
            },
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpcheckAuto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
