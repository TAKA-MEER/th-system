#!/usr/bin/env python3
"""
person_predictor.py — 試験員ロスト時の位置予測ノード (完全版)
================================================================
4.2 節の実装:
  - 直近の速度ベクトルで等速直線運動外挿
  - 予測上限超過後: 捜索行動（その場旋回）
  - /person/search_mode (Bool) で捜索中フラグを発行
"""

import math
import collections

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from th_system_msgs.msg import PersonStatus, RobotMode


class PersonPredictor(Node):
    def __init__(self):
        super().__init__("person_predictor")

        # ── パラメータ ──────────────────────────────────────
        self.declare_parameter("history_sec",      2.0)
        self.declare_parameter("max_predict_sec",  5.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("search_angular_vel", 0.3)  # rad/s 捜索旋回速度
        self.declare_parameter("search_timeout_sec", 10.0) # 捜索打ち切り時間

        self._history_sec      = self.get_parameter("history_sec").value
        self._max_predict_sec  = self.get_parameter("max_predict_sec").value
        self._search_ang_vel   = self.get_parameter("search_angular_vel").value
        self._search_timeout   = self.get_parameter("search_timeout_sec").value
        rate_hz                = self.get_parameter("publish_rate_hz").value

        # ── 状態 ────────────────────────────────────────────
        self._history: collections.deque = collections.deque()
        self._last_known_x   = 0.0
        self._last_known_y   = 0.0
        self._vel_x = 0.0
        self._vel_y = 0.0
        self._lost_since: float | None = None
        self._current_mode = RobotMode.IDLE
        self._search_active = False   # 直前周期で捜索旋回コマンドを出していたか

        # ── Publishers ─────────────────────────────────────
        self._pub_pos    = self.create_publisher(
            PointStamped, "/person/predicted_position", 10)
        self._pub_search = self.create_publisher(
            Bool, "/person/search_mode", 10)
        self._pub_search_cmd = self.create_publisher(
            Twist, "/cmd_vel_retreat", 10)  # 捜索旋回: twist_mux 優先度20(Nav2より高)で出力

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(
            PersonStatus, "/person/status", self._cb_status, 10)
        self.create_subscription(
            RobotMode, "/robot/mode", self._cb_mode, 10)

        # ── 発行タイマー ────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info("person_predictor 起動")

    def _cb_mode(self, msg: RobotMode):
        self._current_mode = msg.mode

    def _cb_status(self, msg: PersonStatus):
        t = self.get_clock().now().nanoseconds * 1e-9

        if not msg.is_lost:
            x, y = msg.position.x, msg.position.y
            self._last_known_x = x
            self._last_known_y = y
            self._lost_since   = None

            self._history.append((t, x, y))
            cutoff = t - self._history_sec
            while self._history and self._history[0][0] < cutoff:
                self._history.popleft()

            self._estimate_velocity()
        else:
            if self._lost_since is None:
                self._lost_since = t
                self.get_logger().warn(
                    f"試験員ロスト: {msg.lost_reason} — 予測開始")

    def _estimate_velocity(self):
        if len(self._history) < 2:
            self._vel_x = 0.0
            self._vel_y = 0.0
            return
        oldest = self._history[0]
        newest = self._history[-1]
        dt = newest[0] - oldest[0]
        if dt < 0.1:
            return
        self._vel_x = (newest[1] - oldest[1]) / dt
        self._vel_y = (newest[2] - oldest[2]) / dt

    def _publish(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        is_searching = False
        rotating     = False   # この周期で捜索旋回コマンドを実際に出したか

        if self._lost_since is None:
            pred_x = self._last_known_x
            pred_y = self._last_known_y
        else:
            elapsed = t - self._lost_since

            if elapsed < self._max_predict_sec:
                # 等速直線外挿
                pred_x = self._last_known_x + self._vel_x * elapsed
                pred_y = self._last_known_y + self._vel_y * elapsed
            elif elapsed < self._max_predict_sec + self._search_timeout:
                # 捜索旋回フェーズ
                pred_x = self._last_known_x
                pred_y = self._last_known_y
                is_searching = True

                # FOLLOWING / FOLLOWING_MAPLESS モード中のみ旋回速度を発行
                if self._current_mode in (RobotMode.FOLLOWING, RobotMode.FOLLOWING_MAPLESS):
                    cmd = Twist()
                    cmd.angular.z = self._search_ang_vel
                    self._pub_search_cmd.publish(cmd)
                    rotating = True
            else:
                # 捜索打ち切り — IDLE 遷移は mode_manager に委ねる
                pred_x = self._last_known_x
                pred_y = self._last_known_y
                is_searching = True
                self.get_logger().warn(
                    "捜索タイムアウト — 再検出できなかった",
                    throttle_duration_sec=5.0)

        # 捜索旋回をやめた瞬間（再検出・捜索打ち切り・モード離脱）に
        # /cmd_vel_retreat へゼロを 1 回送り、停止後に旋回指令が
        # twist_mux のタイムアウト(0.5s)まで残って誤旋回するのを防ぐ。
        if self._search_active and not rotating:
            self._pub_search_cmd.publish(Twist())
        self._search_active = rotating

        # 予測位置発行
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x = pred_x
        msg.point.y = pred_y
        self._pub_pos.publish(msg)

        # 捜索フラグ発行
        self._pub_search.publish(Bool(data=is_searching))


def main(args=None):
    rclpy.init(args=args)
    node = PersonPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
