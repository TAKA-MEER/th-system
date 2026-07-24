#!/usr/bin/env python3
# ============================================================
# summon_navigator — 呼び寄せノード
#
# /person/status (base_link 相対) を要求時点の base_link→map TF で
# 変換し、Nav2 に一度だけゴールを送る単発方式。IDLE からのみ起動でき、
# 到着/失敗のいずれでも IDLE に戻る（AT_PANEL のような保持状態は持たない）。
# ============================================================
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from th_system_msgs.msg import RobotMode, PersonStatus
from th_system_msgs.srv import SetMode

from action_msgs.msg import GoalStatus

import math
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from th_planning.summon_navigator_core import (
    SummonParams, is_target_valid, compute_stop_goal,
)


class SummonNavigator(Node):
    def __init__(self):
        super().__init__('summon_navigator')

        # ── パラメータ ──────────────────────────────────────
        self.declare_parameter('stop_distance_m', 1.0)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('nav_wait_timeout_sec', 2.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self._params = SummonParams(
            stop_distance_m=self.get_parameter('stop_distance_m').value,
            min_confidence=self.get_parameter('min_confidence').value,
        )
        self._nav_wait_timeout = self.get_parameter('nav_wait_timeout_sec').value
        self._map_frame        = self.get_parameter('map_frame').value
        self._base_frame       = self.get_parameter('base_frame').value

        # ── 状態 ────────────────────────────────────────────
        self._current_mode = RobotMode.IDLE
        self._person       = None   # 最新の PersonStatus
        self._goal_handle   = None

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(RobotMode, '/robot/mode', self._cb_mode, 10)
        self.create_subscription(PersonStatus, '/person/status', self._cb_person, 10)

        # ── サービス ─────────────────────────────────────────
        self.create_service(Trigger, '/summon_navigator/call', self._cb_call)

        # ── TF ───────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Nav2 アクション & mode_manager サービス ──────────
        self._nav_client   = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._set_mode_cli = self.create_client(SetMode, '/mode_manager/set_mode')

        self.get_logger().info('summon_navigator 起動')

    # ── モード監視 ─────────────────────────────────────────
    def _cb_mode(self, msg: RobotMode):
        prev = self._current_mode
        self._current_mode = msg.mode

        # SUMMONING から他モードへ抜けたら (IDLE ボタンでのキャンセル・
        # フォルト強制・ESTOP 等) 残っている Nav2 ゴールを止める。
        # これがないとモード表示だけ変わりロボットは走り続けてしまう。
        if (prev == RobotMode.SUMMONING and
                msg.mode != RobotMode.SUMMONING and
                self._goal_handle is not None):
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            self.get_logger().info('SUMMONING 離脱によりゴールをキャンセル')

    def _cb_person(self, msg: PersonStatus):
        self._person = msg

    # ── /summon_navigator/call ────────────────────────────
    def _cb_call(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self._current_mode != RobotMode.IDLE:
            res.success = False
            res.message = 'IDLE 中のみ呼び寄せできます'
            return res

        if self._person is None:
            res.success = False
            res.message = '試験員の位置情報がありません'
            return res

        ok, reason = is_target_valid(
            self._person.is_lost, self._person.confidence, self._params)
        if not ok:
            res.success = False
            res.message = reason
            return res

        goal = compute_stop_goal(
            self._person.position.x, self._person.position.y, self._params)
        if goal is None:
            res.success = True
            res.message = '既に十分近い'
            return res

        # mode_manager に SUMMONING 遷移を要求 (fire-and-forget。
        # panel_navigator と同様、サービスコールバック内から別サービスの
        # 応答を待つと単一スレッド Executor でハングしうるため)
        self._request_mode(RobotMode.SUMMONING, 'summon_navigator')

        goal_x, goal_y, yaw = goal
        goal_pose = PoseStamped()
        goal_pose.header.stamp    = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self._base_frame
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation.z = math.sin(yaw / 2)
        goal_pose.pose.orientation.w = math.cos(yaw / 2)

        try:
            map_pose = self._tf_buffer.transform(
                goal_pose, self._map_frame, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f'TF 変換失敗 (base_link→map): {e}')
            self._request_mode(RobotMode.IDLE, 'summon_navigator')
            res.success = False
            res.message = 'TF 変換に失敗しました'
            return res

        if not self._nav_client.wait_for_server(timeout_sec=self._nav_wait_timeout):
            self.get_logger().error('Nav2 が応答しない')
            self._request_mode(RobotMode.IDLE, 'summon_navigator')
            res.success = False
            res.message = 'Nav2 が応答しない'
            return res

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = map_pose
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._nav_goal_response)

        res.success = True
        res.message = '呼び寄せ開始'
        return res

    def _nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 ゴール拒否')
            self._request_mode(RobotMode.IDLE, 'summon_navigator')
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        result = future.result()
        self._goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('呼び寄せ: 試験員に到着')
        else:
            self.get_logger().warn(f'呼び寄せ: Nav2 結果 status={result.status}')
        self._request_mode(RobotMode.IDLE, 'summon_navigator')

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
    node = SummonNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
