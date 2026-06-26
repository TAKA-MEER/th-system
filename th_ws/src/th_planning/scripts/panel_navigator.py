#!/usr/bin/env python3
# ============================================================
# panel_navigator — 配電盤ナビゲーションノード
# ============================================================
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from th_system_msgs.msg import RobotMode, PanelArrival
from th_system_msgs.srv import GoToPanel, CompleteInspection, SetMode

from action_msgs.msg import GoalStatus

import yaml
import os
import math


class PanelNavigator(Node):
    def __init__(self):
        super().__init__('panel_navigator')

        # ── パラメータ ──────────────────────────────────────
        self.declare_parameter('panels_yaml',
            os.path.join(
                os.path.dirname(__file__), '..', 'config', 'panels.yaml'))
        self.declare_parameter('arrival_threshold_m', 0.3)
        self.declare_parameter('map_frame', 'map')

        panels_yaml = self.get_parameter('panels_yaml').value
        self._arrival_threshold = self.get_parameter('arrival_threshold_m').value
        self._map_frame         = self.get_parameter('map_frame').value

        # ── 配電盤座標の読み込み ─────────────────────────────
        self._panels = self._load_panels(panels_yaml)
        self.get_logger().info(f'配電盤 {len(self._panels)} 件登録')

        # ── 状態 ────────────────────────────────────────────
        self._current_mode    = RobotMode.IDLE
        self._current_panel   = None
        self._goal_handle     = None

        # ── Publishers ─────────────────────────────────────
        self._pub_arrived = self.create_publisher(
            PanelArrival, '/panel_navigator/arrived', 10)
        self._pub_interrupted = self.create_publisher(
            PanelArrival, '/panel_navigator/at_panel_interrupted', 10)

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(RobotMode, '/robot/mode', self._cb_mode, 10)

        # ── サービス ─────────────────────────────────────────
        self.create_service(GoToPanel, '/panel_navigator/go_to_panel',
                            self._cb_go_to_panel)
        self.create_service(CompleteInspection,
                            '/panel_navigator/complete_inspection',
                            self._cb_complete)

        # ── Nav2 アクション & mode_manager サービス ──────────
        self._nav_client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._set_mode_cli = self.create_client(SetMode, '/mode_manager/set_mode')

        self.get_logger().info('panel_navigator 起動')

    # ── パネルYAML読み込み ─────────────────────────────────
    def _load_panels(self, path: str) -> dict:
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
            return {p['id']: p for p in data.get('panels', [])}
        except Exception as e:
            self.get_logger().warn(f'panels.yaml 読み込みエラー: {e}')
            return {}

    # ── モード監視 ─────────────────────────────────────────
    def _cb_mode(self, msg: RobotMode):
        prev = self._current_mode
        self._current_mode = msg.mode

        # AT_PANEL → MANUAL: 中断通知
        if (prev == RobotMode.AT_PANEL and
                msg.mode == RobotMode.MANUAL and
                self._current_panel):
            interrupted_msg = PanelArrival()
            interrupted_msg.header.stamp = self.get_clock().now().to_msg()
            interrupted_msg.panel_id     = self._current_panel
            self._pub_interrupted.publish(interrupted_msg)
            self.get_logger().warn(
                f'AT_PANEL 中断 (MANUAL 切替): panel={self._current_panel}')

    # ── /panel_navigator/go_to_panel ──────────────────────
    def _cb_go_to_panel(self, req: GoToPanel.Request,
                         res: GoToPanel.Response) -> GoToPanel.Response:
        panel_id = req.panel_id
        if panel_id not in self._panels:
            res.success = False
            res.message = f'未登録パネル: {panel_id}'
            return res

        panel = self._panels[panel_id]
        self._current_panel = panel_id

        # mode_manager に MOVING_TO_PANEL 遷移を要求
        self._request_mode(RobotMode.MOVING_TO_PANEL, 'panel_navigator')

        # Nav2 ゴール送出
        goal_pose = PoseStamped()
        goal_pose.header.stamp    = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self._map_frame
        goal_pose.pose.position.x = panel['x']
        goal_pose.pose.position.y = panel['y']
        yaw = panel.get('yaw', 0.0)
        goal_pose.pose.orientation.z = math.sin(yaw / 2)
        goal_pose.pose.orientation.w = math.cos(yaw / 2)

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            res.success = False
            res.message = 'Nav2 が応答しない'
            return res

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback)
        send_future.add_done_callback(
            lambda f: self._nav_goal_response(f, panel_id))

        res.success = True
        res.message = f'パネル {panel_id} へ移動開始'
        return res

    def _nav_goal_response(self, future, panel_id: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 ゴール拒否')
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._nav_result(f, panel_id))

    def _nav_feedback(self, feedback_msg):
        pass  # 必要に応じて進捗表示

    def _nav_result(self, future, panel_id: str):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'配電盤 {panel_id} 到着')
            # 到着通知
            arrived = PanelArrival()
            arrived.header.stamp = self.get_clock().now().to_msg()
            arrived.panel_id     = panel_id
            self._pub_arrived.publish(arrived)
            # AT_PANEL へ遷移
            self._request_mode(RobotMode.AT_PANEL, 'panel_navigator')
        else:
            self.get_logger().warn(f'Nav2 結果: status={result.status}')
            self._request_mode(RobotMode.IDLE, 'panel_navigator')

    # ── /panel_navigator/complete_inspection ──────────────
    def _cb_complete(self, req: CompleteInspection.Request,
                      res: CompleteInspection.Response) -> CompleteInspection.Response:
        self.get_logger().info(f'作業完了: panel={req.panel_id}')
        self._request_mode(RobotMode.FOLLOWING, 'panel_navigator')
        self._current_panel = None
        res.success = True
        return res

    # ── mode_manager サービス呼び出し ──────────────────────
    def _request_mode(self, mode: int, requester: str):
        if not self._set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('mode_manager サービス応答なし')
            return
        req = SetMode.Request()
        req.requested_mode = mode
        req.requester      = requester
        self._set_mode_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = PanelNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
