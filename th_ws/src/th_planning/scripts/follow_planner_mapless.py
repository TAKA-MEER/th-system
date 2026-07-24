#!/usr/bin/env python3
"""
follow_planner_mapless.py — ROS2 ノード本体
=====================================
mapless_follow_core.py のロジックを ROS2 に接続する。
SLAM 占有格子・Nav2 を一切使わない、MAP不要の純粋軌跡追従モード。

担当:
  - /person/status  購読 → core への位置入力
  - /scan_filtered  購読 → 進路上障害物チェック（生 LiDAR スキャン）
  - /robot/mode     購読 → FOLLOWING_MAPLESS 中のみ動作
  - TF (odom→base_link) → 軌跡・退避ゴールの絶対座標系
  - /cmd_vel_retreat パブリッシュ（Nav2 は使わず常に直接速度指令）
"""

import math
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import SetParametersResult
from th_system_msgs.msg import RobotMode, PersonStatus

import tf2_ros

from th_planning.mapless_follow_core import (
    MaplessFollowParams, MaplessFollowCore, should_stop_for_lost,
)
from th_planning.follow_planner_core import to_absolute, to_relative


class FollowPlannerMapless(Node):
    def __init__(self):
        super().__init__("follow_planner_mapless")
        self._declare_all_params()
        p = self._load_params()
        self._core = MaplessFollowCore(p)
        self._lost_debounce_sec = self.get_parameter("lost_debounce_sec").value
        self.add_on_set_parameters_callback(self._on_set_params)
        self._current_mode = RobotMode.IDLE
        self._person_pos  = None
        self._person_abs  = None   # odom系での試験員位置 (受信時に固定、自己移動補償用)
        self._person_lost = True
        self._lost_since: float | None = None   # is_lost が True になった時刻
        self._stopped_during_loss = False       # 今回のロスト中に実際に停止したか
        self._scan: LaserScan | None = None

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub_retreat = self.create_publisher(Twist, "/cmd_vel_retreat", 10)
        self.create_subscription(RobotMode,    "/robot/mode",      self._cb_mode,   10)
        self.create_subscription(PersonStatus, "/person/status",   self._cb_person, 10)
        self.create_subscription(
            LaserScan, "/scan_filtered", self._cb_scan,
            rclpy.qos.QoSProfile(depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))

        self._last_reason = None
        hz = self.get_parameter("update_rate_hz").value
        self._timer = self.create_timer(1.0 / hz, self._loop)
        self.get_logger().info("follow_planner_mapless 起動（MAP不要軌跡追従モード）")

    def _declare_all_params(self):
        D = self.declare_parameter
        D("lookback_distance",             1.0)
        D("trail_sample_interval_m",      0.05)
        D("trail_max_points",           2000)
        D("stop_distance",                 1.0)
        D("resume_distance",               1.3)
        D("obstacle_check_distance_m",     1.0)
        D("obstacle_check_half_width_deg", 20.0)
        D("v_max",                        0.3)
        D("k_ang",                         2.0)
        D("stop_radius_m",                0.2)
        D("w_max_rad_s",                   1.0)
        D("max_linear_accel_mps2",        1.0)
        D("max_linear_decel_mps2",        2.0)
        D("max_angular_accel_rad_s2",     3.0)
        D("lost_debounce_sec",            0.3)
        D("update_rate_hz",              10.0)
        D("odom_frame",           "odom")
        D("base_frame",           "base_link")

    def _load_params(self) -> MaplessFollowParams:
        g = self.get_parameter
        return MaplessFollowParams(
            lookback_distance             = g("lookback_distance").value,
            trail_sample_interval_m       = g("trail_sample_interval_m").value,
            trail_max_points              = int(g("trail_max_points").value),
            stop_distance                 = g("stop_distance").value,
            resume_distance               = g("resume_distance").value,
            obstacle_check_distance_m     = g("obstacle_check_distance_m").value,
            obstacle_check_half_width_deg = g("obstacle_check_half_width_deg").value,
            v_max                         = g("v_max").value,
            k_ang                         = g("k_ang").value,
            stop_radius_m                 = g("stop_radius_m").value,
            w_max_rad_s                   = g("w_max_rad_s").value,
            max_linear_accel_mps2         = g("max_linear_accel_mps2").value,
            max_linear_decel_mps2         = g("max_linear_decel_mps2").value,
            max_angular_accel_rad_s2      = g("max_angular_accel_rad_s2").value,
            update_rate_hz                = g("update_rate_hz").value,
        )

    def _on_set_params(self, params):
        # _load_params() は起動時に一度だけ MaplessFollowParams へ値をコピーする
        # ため、標準の SetParameters を呼ぶだけでは動作に反映されない。
        # WebUI 設定パネルからのライブ反映を機能させるため、ここで core.params
        # にも反映する（MaplessFollowParams は非 frozen dataclass のため可能）。
        #
        # mapless_follow_core.py の計算式で除数・sqrt の被開数として使われる値
        # (v_max, k_ang, stop_radius_m 等) に 0 以下を許すとゼロ除算/定義域
        # エラーで FOLLOWING_MAPLESS 中にノードがクラッシュするため拒否する。
        for p in params:
            if p.name in self._POSITIVE_PARAMS and p.value <= 0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{p.name} は正の値である必要があります")
        for p in params:
            if hasattr(self._core.params, p.name):
                setattr(self._core.params, p.name, p.value)
        return SetParametersResult(successful=True)

    _POSITIVE_PARAMS = frozenset({
        "lookback_distance", "trail_sample_interval_m", "trail_max_points",
        "stop_distance", "resume_distance", "obstacle_check_distance_m",
        "obstacle_check_half_width_deg", "v_max", "k_ang", "stop_radius_m",
        "w_max_rad_s", "max_linear_accel_mps2", "max_linear_decel_mps2",
        "max_angular_accel_rad_s2", "update_rate_hz",
    })

    def _cb_mode(self, msg: RobotMode):
        prev = self._current_mode
        self._current_mode = msg.mode
        if prev == RobotMode.FOLLOWING_MAPLESS and msg.mode != RobotMode.FOLLOWING_MAPLESS:
            self._core.reset()
            self._stop()

    def _cb_person(self, msg: PersonStatus):
        was_lost = self._person_lost
        self._person_lost = msg.is_lost

        if msg.is_lost:
            if not was_lost:
                self._lost_since = self.get_clock().now().nanoseconds * 1e-9
                self._stopped_during_loss = False
            return

        self._person_pos = (msg.position.x, msg.position.y)
        # 受信時点の自己位置で odom 絶対座標に固定する。
        # DR-SPAAM は CPU 推論で約 2Hz しか更新されないため、base_link 相対の
        # まま保持すると次の検知までロボットの自己移動分だけ距離が実際より
        # 遠く見え続け、停止判定が遅れて追従対象に衝突する (実機で確認)。
        pose = self._robot_pose_odom()
        self._person_abs = to_absolute(pose, self._person_pos) if pose is not None else None

        # 軌跡クリアは実際に停止に至った(=lost_debounce_sec 以上ロストした)
        # 場合のみ行う。DR-SPAAM の単発検知ミス程度の瞬断(数百ms未満)では
        # 軌跡はまだ有効なので消さない。消すと再開直後にlookback目標が
        # 現在位置へ戻り旋回が跳ねる(症状: フラフラ揺れる一因)。
        if was_lost and self._stopped_during_loss:
            self._core.clear_trail()
        self._lost_since = None

    def _cb_scan(self, msg: LaserScan):
        self._scan = msg

    def _robot_pose_odom(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self.get_parameter("odom_frame").value,
                self.get_parameter("base_frame").value,
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().debug(
                f"odom TF 取得失敗: {e}", throttle_duration_sec=2.0)
            return None
        trans = tf.transform.translation
        rot   = tf.transform.rotation
        yaw   = 2.0 * math.atan2(rot.z, rot.w)
        return (trans.x, trans.y, yaw)

    def _stop(self):
        self._pub_retreat.publish(Twist())

    def _loop(self):
        if self._current_mode != RobotMode.FOLLOWING_MAPLESS:
            return
        if self._person_pos is None:
            # まだ一度も検知していない (デバウンス対象外、確定的に停止)
            self._core.notify_lost()
            self._stop()
            if self._last_reason != "person_lost":
                self.get_logger().warn("停止中（理由: person_lost）", throttle_duration_sec=2.0)
            self._last_reason = "person_lost"
            return

        # is_lost によるロストはデバウンスする: DR-SPAAM は約2Hzでしか推論
        # されないため、1フレームの検知ミスだけで is_lost が一瞬 true になり
        # 得る。lost_debounce_sec 未満なら「まだ追従対象を見失っていない」
        # とみなし、自己移動補償した最後の既知位置で追従を続ける
        # (障害物・接近側の即時停止仕様は変更しない。is_lost のみ対象)。
        now = self.get_clock().now().nanoseconds * 1e-9
        lost_elapsed = (now - self._lost_since) if self._lost_since is not None else 0.0
        if should_stop_for_lost(self._person_lost, lost_elapsed, self._lost_debounce_sec):
            # ロスト中に何も publish しないと、直前の速度指令(前進中ならその速度)が
            # /cmd_vel_retreat に残ったままになり、ロボットが停止せず進み続けてしまう。
            # 追従対象に接近しすぎて LiDAR の死角/最小測距限界でロストした瞬間に
            # 起きうるため、毎周期明示的に停止指令を送り続ける。
            self._stopped_during_loss = True
            self._core.notify_lost()
            self._stop()
            if self._last_reason != "person_lost":
                self.get_logger().warn("停止中（理由: person_lost）", throttle_duration_sec=2.0)
            self._last_reason = "person_lost"
            return

        robot_pose_odom = self._robot_pose_odom()

        # 自己移動補償: 検知メッセージ間 (約0.5秒以上) もロボットは動き続けるため、
        # 受信時に固定した odom 絶対位置を現在の自己位置で相対座標に引き直す。
        # これで接近中の距離が受信間隔中も単調に減り、停止判定の遅れを防ぐ。
        px, py = self._person_pos
        if self._person_abs is not None and robot_pose_odom is not None:
            px, py = to_relative(robot_pose_odom, self._person_abs)

        scan_ranges = self._scan.ranges if self._scan is not None else None
        scan_angle_min = self._scan.angle_min if self._scan is not None else 0.0
        scan_angle_increment = self._scan.angle_increment if self._scan is not None else 0.0

        out = self._core.update(
            person_x=px, person_y=py,
            robot_pose_odom=robot_pose_odom,
            scan_ranges=scan_ranges,
            scan_angle_min=scan_angle_min,
            scan_angle_increment=scan_angle_increment)

        cmd = Twist()
        cmd.linear.x  = out.linear_x
        cmd.angular.z = out.angular_z
        self._pub_retreat.publish(cmd)

        if out.reason != self._last_reason:
            if out.reason != "tracking":
                self.get_logger().warn(
                    f"停止中（理由: {out.reason}）", throttle_duration_sec=2.0)
            else:
                self.get_logger().info("追従再開")
        self._last_reason = out.reason


def main(args=None):
    rclpy.init(args=args)
    node = FollowPlannerMapless()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
