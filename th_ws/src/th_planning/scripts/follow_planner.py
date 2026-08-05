#!/usr/bin/env python3
"""
follow_planner.py — ROS2 ノード本体
=====================================
follow_planner_core.py のロジックを ROS2 に接続する。

担当:
  - /person/status      購読 → core への位置入力
  - /local_costmap/costmap 購読 → 退避方向探索・軌跡蓄積用の絶対姿勢(TF)取得、退避方向自由空間チェック
  - /robot/mode         購読 → FOLLOWING/MOVING_TO_PANEL 中のみ動作
  - NavigateToPose      アクションクライアント (base_link → map 変換後に送出)
  - /cmd_vel_retreat    パブリッシュ (PREPARE 旋回・EVADING 走行・停止)
  - /follow/status      パブリッシュ (内部状態の可視化。WebUI の音声通知が購読する)
"""

import math
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from th_system_msgs.msg import RobotMode, PersonStatus, FollowStatus

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from th_planning.follow_planner_core import (
    FollowParams, FollowPlannerCore,
)

OBSTACLE_COST_THRESHOLD = 90


class FollowPlanner(Node):
    def __init__(self):
        super().__init__("follow_planner")
        self._declare_all_params()
        p = self._load_params()
        self._core = FollowPlannerCore(p)
        self._current_mode = RobotMode.IDLE
        self._person_pos   = None
        self._person_lost  = True
        self._costmap      = None
        self._nav_goal_handle = None
        # base_link → costmap フレーム の変換を 1 制御周期に 1 回だけ取得してキャッシュする。
        # (costmap_clear_fn は 1 ループで数百回呼ばれるため、毎回 TF ルックアップすると過負荷になる)
        self._cached_tf: tuple | None = None

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub_retreat = self.create_publisher(Twist, "/cmd_vel_retreat", 10)
        self._pub_status  = self.create_publisher(FollowStatus, "/follow/status", 10)
        self.create_subscription(RobotMode,     "/robot/mode",          self._cb_mode,     10)
        self.create_subscription(PersonStatus,  "/person/status",       self._cb_person,   10)
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._cb_costmap,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 状態 publish の間引き用 (follow_planner_mapless.py と同じ方針)
        self._last_status_key  = None
        self._last_status_time = 0.0
        self._status_active    = False   # 直前周期で自モードだったか
        hz = self.get_parameter("update_rate_hz").value
        self._timer = self.create_timer(1.0 / hz, self._loop)
        self.get_logger().info("follow_planner 起動（高度追従ロジック）")

    STATUS_HEARTBEAT_SEC = 1.0

    def _publish_status(self, state: str, reason: str,
                        person_distance: float = -1.0, linear_x: float = 0.0,
                        escape_angle=None):
        """内部状態を /follow/status に流す。変化時 + 1Hz のハートビート。"""
        key = (state, reason, escape_angle is not None)
        now = self.get_clock().now().nanoseconds * 1e-9
        if key == self._last_status_key and (now - self._last_status_time) < self.STATUS_HEARTBEAT_SEC:
            return
        self._last_status_key  = key
        self._last_status_time = now

        msg = FollowStatus()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = "base_link"
        msg.planner          = "map"
        msg.state            = state
        msg.reason           = reason
        msg.person_distance  = float(person_distance)
        msg.linear_x         = float(linear_x)
        msg.has_escape_angle = escape_angle is not None
        msg.escape_angle     = float(escape_angle) if escape_angle is not None else 0.0
        self._pub_status.publish(msg)

    def _declare_all_params(self):
        D = self.declare_parameter
        D("lookback_distance",          1.0)
        D("trail_sample_interval_m",   0.05)
        D("trail_max_points",         2000)
        D("d_prepare",                  3.0)
        D("d_evade",                    2.0)
        D("distance_hysteresis_m",      0.2)
        D("evade_scan_directions",     16)
        D("evade_scan_max_dist",        3.0)
        D("evade_route_length_m",       2.0)
        D("retreat_check_clearance",    0.5)
        D("retreat_speed",             0.15)
        D("evade_k_ang",                2.0)
        D("evade_stop_radius_m",        0.2)
        D("evade_orient_tolerance_rad", 0.05)
        D("goal_deadzone_m",           0.30)
        D("update_rate_hz",          10.0)
        D("base_frame",          "base_link")
        D("map_frame",           "map")

    def _load_params(self) -> FollowParams:
        g = self.get_parameter
        return FollowParams(
            lookback_distance          = g("lookback_distance").value,
            trail_sample_interval_m    = g("trail_sample_interval_m").value,
            trail_max_points           = int(g("trail_max_points").value),
            d_prepare                  = g("d_prepare").value,
            d_evade                    = g("d_evade").value,
            distance_hysteresis_m      = g("distance_hysteresis_m").value,
            evade_scan_directions      = int(g("evade_scan_directions").value),
            evade_scan_max_dist        = g("evade_scan_max_dist").value,
            evade_route_length_m       = g("evade_route_length_m").value,
            retreat_check_clearance    = g("retreat_check_clearance").value,
            retreat_speed              = g("retreat_speed").value,
            evade_k_ang                = g("evade_k_ang").value,
            evade_stop_radius_m        = g("evade_stop_radius_m").value,
            evade_orient_tolerance_rad = g("evade_orient_tolerance_rad").value,
            goal_deadzone_m            = g("goal_deadzone_m").value,
            update_rate_hz             = g("update_rate_hz").value,
        )

    def _cb_mode(self, msg: RobotMode):
        active = (RobotMode.FOLLOWING, RobotMode.MOVING_TO_PANEL)
        prev   = self._current_mode
        self._current_mode = msg.mode
        if prev in active and msg.mode not in active:
            self._core.reset()
            self._stop_retreat()

    def _cb_person(self, msg: PersonStatus):
        was_lost = self._person_lost
        self._person_lost = msg.is_lost
        if not msg.is_lost:
            self._person_pos = (msg.position.x, msg.position.y)
            if was_lost:
                # ロスト→再検出の遷移: デッドゾーンと軌跡をクリアして直ちに新ゴールを送出
                self._core.clear_prev_goal()
                self._core.clear_trail()

    def _cb_costmap(self, msg: OccupancyGrid):
        self._costmap = msg

    def _update_costmap_tf(self):
        """base_link → costmap フレーム の変換を 1 制御周期に 1 回だけ取得する。

        重要: costmap の origin はそのメッセージの header.frame_id（local_costmap は
        通常 'odom'）で表現されている。以前は固定で 'map' に変換していたため、
        SLAM/AMCL で map→odom にズレが生じると誤ったセルを参照していた。
        cm.header.frame_id を変換先にすることでフレーム不一致を解消する。
        """
        cm = self._costmap
        if cm is None:
            self._cached_tf = None
            return
        target = cm.header.frame_id or self.get_parameter("map_frame").value
        try:
            tf = self._tf_buffer.lookup_transform(
                target, self.get_parameter("base_frame").value,
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().debug(
                f"costmap TF 取得失敗 ({target}←base_link): {e}",
                throttle_duration_sec=2.0)
            self._cached_tf = None
            return
        trans = tf.transform.translation
        rot   = tf.transform.rotation
        yaw   = 2.0 * math.atan2(rot.z, rot.w)
        self._cached_tf = (trans.x, trans.y, math.cos(yaw), math.sin(yaw))

    def _costmap_is_free(self, x: float, y: float) -> bool:
        cm  = self._costmap
        tfc = self._cached_tf
        # costmap 未受信 / TF 未確立時は「空き」とみなす（保守的: 退避や追従を妨げない）
        if cm is None or tfc is None:
            return True
        tx, ty, cos_yaw, sin_yaw = tfc
        # base_link 相対座標 (x, y) を costmap フレームへ変換
        mx = tx + x * cos_yaw - y * sin_yaw
        my = ty + x * sin_yaw + y * cos_yaw
        ox  = cm.info.origin.position.x
        oy  = cm.info.origin.position.y
        res = cm.info.resolution
        W   = cm.info.width
        H   = cm.info.height
        cx = int((mx - ox) / res)
        cy = int((my - oy) / res)
        if cx < 0 or cx >= W or cy < 0 or cy >= H:
            return True
        return cm.data[cy * W + cx] < OBSTACLE_COST_THRESHOLD

    def _retreat_clearance(self, dx: float, dy: float, limit: float = 2.0) -> float:
        step = 0.1
        for i in range(1, int(limit / step) + 1):
            if not self._costmap_is_free(dx * step * i, dy * step * i):
                return step * (i - 1)
        return limit

    def _to_map_pose(self, x: float, y: float, yaw: float):
        ps = PoseStamped()
        ps.header.stamp    = rclpy.time.Time().to_msg()  # 最新の利用可能なTFを使用（未来外挿を回避）
        ps.header.frame_id = self.get_parameter("base_frame").value
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        try:
            return self._tf_buffer.transform(
                ps, self.get_parameter("map_frame").value,
                timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f"TF 変換失敗: {e}", throttle_duration_sec=2.0)
            return None

    def _send_nav_goal(self, pose: PoseStamped):
        if not self._nav_client.wait_for_server(timeout_sec=0.05):
            return
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        f = self._nav_client.send_goal_async(goal)
        def _on_goal_response(fut):
            result = fut.result()
            self._nav_goal_handle = result if result.accepted else None
        f.add_done_callback(_on_goal_response)

    def _stop_retreat(self):
        self._pub_retreat.publish(Twist())

    def _loop(self):
        if self._current_mode not in (RobotMode.FOLLOWING, RobotMode.MOVING_TO_PANEL):
            # 自モードでない間は黙る (follow_planner_mapless.py の同じ箇所を参照)。
            # モードを抜けた直後の 1 回だけ INACTIVE を出して残留状態を解除する。
            if self._status_active:
                self._status_active = False
                self._publish_status("INACTIVE", "inactive")
            return
        self._status_active = True
        if self._person_lost or self._person_pos is None:
            # ロスト中に何も publish しないと、EVADING/PREPARE で直前に出していた
            # 速度指令が /cmd_vel_retreat に残ったままになり停止しない
            # (mapless 版と同じ理由で follow_planner_mapless.py 参照)。
            self._stop_retreat()
            self._publish_status("TRACKING", "person_lost")
            return

        px, py = self._person_pos
        distance = math.hypot(px, py)
        t  = self.get_clock().now().nanoseconds * 1e-9
        # costmap フレームへの変換をこの周期で一度だけ取得（以降の costmap 参照・絶対姿勢で再利用）
        self._update_costmap_tf()

        robot_pose_odom = None
        if self._cached_tf is not None:
            tx, ty, cos_yaw, sin_yaw = self._cached_tf
            robot_pose_odom = (tx, ty, math.atan2(sin_yaw, cos_yaw))

        evade_scan_max_dist = self._core.params.evade_scan_max_dist
        out = self._core.update(
            person_x=px, person_y=py, t=t,
            clearance_fn=lambda dx, dy: self._retreat_clearance(dx, dy, limit=evade_scan_max_dist),
            robot_pose_odom=robot_pose_odom)

        # 退避方向は odom 絶対角で保持されているので機体相対に直してから流す
        escape_rel = (self._core.escape_angle_rel(robot_pose_odom[2])
                      if robot_pose_odom is not None else None)
        self._publish_status(
            state=self._core.state.name,   # "TRACKING" / "PREPARE" / "EVADING"
            reason=out.reason,
            person_distance=distance,
            linear_x=out.linear_x,
            escape_angle=escape_rel)

        if out.kind == "retreat":
            cmd = Twist()
            cmd.linear.x  = out.linear_x
            cmd.angular.z = out.angular_z
            self._pub_retreat.publish(cmd)

        elif out.kind == "stop":
            self._stop_retreat()
            # 「本当に逃げ場がない」場合と「自己位置が取れていない」場合を
            # 区別してログに出す (音声通知 C4 は前者にのみ反応する)
            if out.reason == "retreat_blocked":
                self.get_logger().warn(
                    "近接退避不可（壁際）— その場停止",
                    throttle_duration_sec=2.0)
            else:
                self.get_logger().warn(
                    "自己位置(TF/costmap)未確立 — その場停止",
                    throttle_duration_sec=2.0)

        elif out.kind == "nav_goal":
            map_pose = self._to_map_pose(out.goal_x, out.goal_y, out.goal_yaw)
            if map_pose:
                self._send_nav_goal(map_pose)
            else:
                # TF失敗時はデッドゾーンをクリアして次ループで再試行
                self._core.clear_prev_goal()


def main(args=None):
    rclpy.init(args=args)
    node = FollowPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
