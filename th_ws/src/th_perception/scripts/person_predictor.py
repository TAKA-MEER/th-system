#!/usr/bin/env python3
"""
person_predictor.py — 試験員ロスト時の位置予測ノード (完全版)
================================================================
4.2 節の実装:
  - 直近の速度ベクトルで等速直線運動外挿
  - 予測上限超過後: 捜索行動（その場旋回）
  - /person/search_mode (Bool) で捜索中フラグを発行
  - /person/search_status (SearchStatus) で段階を発行
    (Bool では「捜索中」と「打ち切り後」を区別できないため。Bool 側は
     既存の購読者・テストのためそのまま残す)
"""

import math
import collections

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from th_system_msgs.msg import PersonStatus, RobotMode, SearchStatus


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
        # 段階 publish の間引き用。変化時 + 1Hz のハートビートに絞る
        self._last_phase      = None
        self._last_phase_time = 0.0

        # ── Publishers ─────────────────────────────────────
        self._pub_pos    = self.create_publisher(
            PointStamped, "/person/predicted_position", 10)
        self._pub_search = self.create_publisher(
            Bool, "/person/search_mode", 10)
        self._pub_search_status = self.create_publisher(
            SearchStatus, "/person/search_status", 10)
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

    STATUS_HEARTBEAT_SEC = 1.0

    def _publish_search_status(self, phase: str, lost_elapsed: float):
        """捜索段階を /person/search_status に流す。変化時 + 1Hz のハートビート。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        if phase == self._last_phase and (now - self._last_phase_time) < self.STATUS_HEARTBEAT_SEC:
            return
        self._last_phase      = phase
        self._last_phase_time = now

        msg = SearchStatus()
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.header.frame_id   = "base_link"
        msg.phase             = phase
        msg.lost_elapsed_sec  = float(lost_elapsed)
        self._pub_search_status.publish(msg)

    def _publish(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        is_searching = False
        rotating     = False   # この周期で捜索旋回コマンドを実際に出したか
        phase        = "NONE"
        lost_elapsed = 0.0

        if self._lost_since is None:
            pred_x = self._last_known_x
            pred_y = self._last_known_y
        else:
            elapsed = t - self._lost_since
            lost_elapsed = elapsed

            if elapsed < self._max_predict_sec:
                # 等速直線外挿
                pred_x = self._last_known_x + self._vel_x * elapsed
                pred_y = self._last_known_y + self._vel_y * elapsed
                phase  = "PREDICTING"
            elif elapsed < self._max_predict_sec + self._search_timeout:
                # 捜索旋回フェーズ
                pred_x = self._last_known_x
                pred_y = self._last_known_y
                is_searching = True
                phase = "SEARCHING"

                # FOLLOWING / FOLLOWING_MAPLESS モード中のみ旋回速度を発行
                #
                # FACING は意図的に含めない: face_planner はロスト中も毎周期ゼロ Twist を
                # /cmd_vel_retreat へ明示発行し続ける(ESP32 キープアライブが古い非ゼロ指令を
                # 再送し続ける事象を防ぐため。VISION.md §5 参照)。同じトピックへこの捜索旋回が
                # 同程度の周期で割り込むと、両者が交互に勝つ形でジャダー(旋回とゼロの点滅)が
                # 起きる。展示ブースでは近接遮蔽によるロストが起きやすくこの経路を頻繁に踏むため、
                # FACING は捜索旋回をせず停止して待つ(2026-08-07 決定)。
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
                phase = "GAVE_UP"
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

        # 捜索フラグ発行 (従来互換。SEARCHING / GAVE_UP のとき true)
        self._pub_search.publish(Bool(data=is_searching))

        # 段階発行 (捜索中と打ち切り後を区別できる形)
        self._publish_search_status(phase, lost_elapsed)


def main(args=None):
    rclpy.init(args=args)
    node = PersonPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
