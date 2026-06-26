#!/usr/bin/env python3
# ============================================================
# manual_command_handler — タブレット手動操作ハンドラー
# ============================================================
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose
from th_system_msgs.msg import RobotMode
from th_system_msgs.srv import SetMode

import math
import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class ManualCommandHandler(Node):
    def __init__(self):
        super().__init__('manual_command_handler')

        # ── パラメータ ──────────────────────────────────────
        self.declare_parameter('heartbeat_timeout_sec', 1.0)
        self.declare_parameter('check_period_ms',       200)
        self.declare_parameter('base_frame',            'base_link')
        self.declare_parameter('map_frame',             'map')

        self._hb_timeout = self.get_parameter('heartbeat_timeout_sec').value
        check_ms         = self.get_parameter('check_period_ms').value
        self._base_frame = self.get_parameter('base_frame').value

        # ── 状態 ────────────────────────────────────────────
        self._current_mode    = RobotMode.IDLE
        self._last_hb_time    = None   # heartbeat 受信時刻
        self._goal_handle     = None
        self._in_manual       = False

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(RobotMode, '/robot/mode', self._cb_mode, 10)
        self.create_subscription(PoseStamped, '/manual/target_pose',
                                 self._cb_target_pose, 10)
        self.create_subscription(Empty, '/manual/heartbeat',
                                 self._cb_heartbeat, 10)

        # ── TF ───────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Nav2 アクション & mode_manager サービス ──────────
        self._nav_client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._set_mode_cli = self.create_client(SetMode, '/mode_manager/set_mode')

        # ── heartbeat 監視タイマー ────────────────────────────
        self._hb_timer = self.create_timer(
            check_ms / 1000.0, self._check_heartbeat)

        self.get_logger().info('manual_command_handler 起動')

    def _cb_mode(self, msg: RobotMode):
        self._current_mode = msg.mode
        self._in_manual = (msg.mode == RobotMode.MANUAL)
        if not self._in_manual:
            self._last_hb_time = None   # MANUAL 離脱時はリセット

    def _cb_heartbeat(self, _msg: Empty):
        self._last_hb_time = self.get_clock().now().nanoseconds * 1e-9

    def _cb_target_pose(self, msg: PoseStamped):
        if self._current_mode != RobotMode.MANUAL:
            return

        if not self._nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 未起動')
            return

        # base_link 座標を map フレームへ変換してから Nav2 へ送る
        # (タイムスタンプを now() に更新し、TF の最新変換を使用する)
        msg.header.stamp = self.get_clock().now().to_msg()
        map_frame = self.get_parameter('map_frame').value
        try:
            map_pose = self._tf_buffer.transform(
                msg, map_frame, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f'TF 変換失敗 (base_link→map): {e}')
            return

        # 既存ゴールをキャンセル
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = map_pose

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self._goal_handle = goal_handle
        else:
            self.get_logger().warn('Nav2 ゴール拒否')

    def _check_heartbeat(self):
        if not self._in_manual:
            return
        if self._last_hb_time is None:
            return  # MANUAL 移行直後はまだ受信前

        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._last_hb_time

        if elapsed > self._hb_timeout:
            self.get_logger().warn(
                f'heartbeat タイムアウト ({elapsed:.1f}s) → IDLE へ')
            self._request_mode(RobotMode.IDLE, 'manual_command_handler')
            # ゴールキャンセル
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
            self._last_hb_time = None

    def _request_mode(self, mode: int, requester: str):
        if not self._set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('mode_manager 未起動')
            return
        req = SetMode.Request()
        req.requested_mode = mode
        req.requester      = requester
        self._set_mode_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = ManualCommandHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
