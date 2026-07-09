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
from th_system_msgs.msg import RobotMode, PersonStatus

import tf2_ros

from th_planning.mapless_follow_core import (
    MaplessFollowParams, MaplessFollowCore,
)


class FollowPlannerMapless(Node):
    def __init__(self):
        super().__init__("follow_planner_mapless")
        self._declare_all_params()
        p = self._load_params()
        self._core = MaplessFollowCore(p)
        self._current_mode = RobotMode.IDLE
        self._person_pos  = None
        self._person_lost = True
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
            update_rate_hz                = g("update_rate_hz").value,
        )

    def _cb_mode(self, msg: RobotMode):
        prev = self._current_mode
        self._current_mode = msg.mode
        if prev == RobotMode.FOLLOWING_MAPLESS and msg.mode != RobotMode.FOLLOWING_MAPLESS:
            self._core.reset()
            self._stop()

    def _cb_person(self, msg: PersonStatus):
        was_lost = self._person_lost
        self._person_lost = msg.is_lost
        if not msg.is_lost:
            self._person_pos = (msg.position.x, msg.position.y)
            if was_lost:
                self._core.clear_trail()

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
        if self._person_lost or self._person_pos is None:
            return

        px, py = self._person_pos
        robot_pose_odom = self._robot_pose_odom()

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
