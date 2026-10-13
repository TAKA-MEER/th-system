"""
test_nav2_progress_checker_node.py — Nav2 の進捗判定は旋回も進捗に数える
launch_testing 試験（2026-10-13 実機 R-41「走り出して少しして一時停止」）。

実機で、盤前移動の `Received a goal` のちょうど 10.0 秒後に controller_server が
`Failed to make progress` で follow_path を打ち切った。旧設定の
`SimpleProgressChecker` は位置しか見ないので、走り出す前の「その場で経路の向きへ
旋回」(0.4 rad/s) が進捗に数えられなかった。実機の `PoseProgressChecker` は位置か
向きのどちらかが動けば進捗とみなす。

本番ノード（nav2_controller の controller_server）を、実機が読むのと同じ
`config/nav2_params.yaml`（launch が複写して `generated/nav2/nav2_params.yaml` に
する元）で単体起動し、偽の TF（odom→base_link）で次を縛る。
  - configure が通り、`progress_checker.*` が yaml のとおりに効く
    （プラグイン名・required_movement_radius・required_movement_angle・
    movement_time_allowance）
  - 位置は動かず向きだけ 0.4 rad/s で回っていれば、10 秒を超えても打ち切られない
  - 位置も向きも動かなければ、10 秒前後で打ち切られる
    （固着の検知が壊れていないこと）
  - launch が作る生成 yaml でも progress_checker が静的な設定のまま残る
ROS（rclpy・nav2）が要るのでホストでは走らない。
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
_NAV2_STATIC = os.path.join(_WS_ROOT, 'src', 'th_bringup', 'config', 'nav2_params.yaml')

with open(_NAV2_STATIC, encoding='utf-8') as _f:
    _CFG = yaml.safe_load(_f)
_PC = _CFG['controller_server']['ros__parameters']['progress_checker']
_TIME_ALLOWANCE = float(_PC['movement_time_allowance'])
_ROT_VEL = float(_CFG['controller_server']['ros__parameters']['FollowPath']
                 ['rotate_to_heading_angular_vel'])


@pytest.mark.launch_test
def generate_test_description():
    ctrl = launch_ros.actions.Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[_NAV2_STATIC],
        remappings=[('cmd_vel', '/cmd_vel_nav')])
    return launch.LaunchDescription([ctrl, launch_testing.actions.ReadyToTest()]), {}


class TestProgressChecker(unittest.TestCase):
    _activated = False

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('progress_checker_test')
        cls.exec = MultiThreadedExecutor(num_threads=4)
        cls.exec.add_node(cls.node)
        cls.thr = threading.Thread(target=cls.exec.spin, daemon=True)
        cls.thr.start()
        cls.tf_pub = cls.node.create_publisher(TFMessage, '/tf', 10)
        cls.odom_pub = cls.node.create_publisher(Odometry, '/odom', 10)
        # 偽ロボット: x, y, yaw。mode は 'rotate'（cmd_vel の角速度だけ積分）か 'frozen'
        cls.pose = [0.0, 0.0, 0.0]
        cls.mode = 'frozen'
        cls.node.create_subscription(Twist, '/cmd_vel_nav', cls._on_cmd, 10)
        cls.last_t = time.monotonic()
        cls.cmd_w = 0.0
        cls.node.create_timer(0.05, cls._tick)
        cls.get_param = cls.node.create_client(
            GetParameters, '/controller_server/get_parameters')
        cls.change = cls.node.create_client(
            ChangeState, '/controller_server/change_state')
        cls.get_state = cls.node.create_client(
            GetState, '/controller_server/get_state')

    @classmethod
    def tearDownClass(cls):
        cls.exec.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_cmd(cls, msg):
        cls.cmd_w = msg.angular.z

    @classmethod
    def _tick(cls):
        now = time.monotonic()
        dt = now - cls.last_t
        cls.last_t = now
        if cls.mode == 'rotate':
            cls.pose[2] += cls.cmd_w * dt   # 位置は動かさない（向きだけ）
        stamp = cls.node.get_clock().now().to_msg()
        for parent, child, x, y, yaw in (('map', 'odom', 0.0, 0.0, 0.0),
                                         ('odom', 'base_link', *cls.pose)):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
            cls.tf_pub.publish(TFMessage(transforms=[t]))
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = cls.pose[0]
        o.pose.pose.orientation.z = math.sin(cls.pose[2] / 2.0)
        o.pose.pose.orientation.w = math.cos(cls.pose[2] / 2.0)
        o.twist.twist.angular.z = cls.cmd_w if cls.mode == 'rotate' else 0.0
        cls.odom_pub.publish(o)

    def _transition(self, tid):
        self.assertTrue(self.change.wait_for_service(timeout_sec=30.0))
        req = ChangeState.Request()
        req.transition.id = tid
        fut = self.change.call_async(req)
        end = time.monotonic() + 60.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        self.assertTrue(fut.done(), 'change_state が返らない')
        return fut.result().success

    def _param(self, name):
        self.assertTrue(self.get_param.wait_for_service(timeout_sec=30.0))
        fut = self.get_param.call_async(GetParameters.Request(names=[name]))
        end = time.monotonic() + 10.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        v = fut.result().values[0]
        if v.type == ParameterType.PARAMETER_STRING:
            return v.string_value
        if v.type == ParameterType.PARAMETER_DOUBLE:
            return v.double_value
        if v.type == ParameterType.PARAMETER_INTEGER:
            return v.integer_value
        return None

    def _ensure_active(self):
        cls = type(self)
        if cls._activated:
            return
        self.assertTrue(self._transition(Transition.TRANSITION_CONFIGURE),
                        'controller_server の configure が失敗（プラグイン読み込み不可の疑い）')
        self.assertTrue(self._transition(Transition.TRANSITION_ACTIVATE),
                        'controller_server の activate が失敗')
        cls._activated = True

    def _send_path(self, yaw_goal):
        """原点から x 方向（yaw_goal）へ 3 m の経路を FollowPath で送る。"""
        client = ActionClient(self.node, FollowPath, '/follow_path')
        self.assertTrue(client.wait_for_server(timeout_sec=30.0))
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(0, 31):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = 0.1 * i * math.cos(yaw_goal)
            ps.pose.position.y = 0.1 * i * math.sin(yaw_goal)
            ps.pose.orientation.z = math.sin(yaw_goal / 2.0)
            ps.pose.orientation.w = math.cos(yaw_goal / 2.0)
            path.poses.append(ps)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        gf = client.send_goal_async(goal)
        end = time.monotonic() + 10.0
        while not gf.done() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(gf.done() and gf.result().accepted, 'FollowPath が受理されない')
        return gf.result(), time.monotonic()

    def _run_goal(self, mode, yaw_goal, watch_s):
        """watch_s 秒のあいだ結果（打ち切り）を見る。打ち切られた経過秒 or None。"""
        self._ensure_active()
        type(self).pose[:] = [0.0, 0.0, 0.0]
        type(self).mode = mode
        handle, t0 = self._send_path(yaw_goal)
        rf = handle.get_result_async()
        end = t0 + watch_s
        while not rf.done() and time.monotonic() < end:
            time.sleep(0.05)
        out = None
        if rf.done():
            out = (rf.result().status, time.monotonic() - t0)
        else:
            handle.cancel_goal_async()
            time.sleep(0.5)
        type(self).mode = 'frozen'
        time.sleep(1.0)
        return out

    # ---- 設定の到達（configure とパラメータ）------------------------------
    def test_01_configure_and_params_reach_plugin(self):
        self.assertEqual(_PC['plugin'], 'nav2_controller::PoseProgressChecker')
        self.assertTrue(self._transition(Transition.TRANSITION_CONFIGURE),
                        'configure 失敗（PoseProgressChecker を読み込めない）')
        self.assertEqual(self._param('progress_checker.plugin'),
                         'nav2_controller::PoseProgressChecker')
        self.assertAlmostEqual(self._param('progress_checker.required_movement_radius'),
                               float(_PC['required_movement_radius']))
        self.assertAlmostEqual(self._param('progress_checker.required_movement_angle'),
                               float(_PC['required_movement_angle']))
        self.assertAlmostEqual(self._param('progress_checker.movement_time_allowance'),
                               _TIME_ALLOWANCE)
        # 旋回 0.4 rad/s で時間枠内に確実に届き、揺れでは届かない値
        ang = float(_PC['required_movement_angle'])
        self.assertLess(ang, _ROT_VEL * _TIME_ALLOWANCE / 2.0)
        self.assertGreaterEqual(ang, 0.3)

    def test_02_activate(self):
        # test_01 で configure 済みなので activate だけ
        self.assertTrue(self._transition(Transition.TRANSITION_ACTIVATE))
        type(self)._activated = True

    # ---- 挙動 ---------------------------------------------------------------
    def test_10_rotation_only_counts_as_progress(self):
        # RPP が回る向き（経路が真後ろでなく横を向く）。0.4 rad/s で位置は動かさない
        res = self._run_goal('rotate', math.radians(120.0), _TIME_ALLOWANCE + 2.5)
        self.assertIsNone(
            res, f'旋回中なのに進捗なしで打ち切られた: {res}'
                 '（位置が動かず向きだけ回る状態を進捗に数えていない）')

    def test_11_no_motion_aborts_around_allowance(self):
        res = self._run_goal('frozen', math.radians(120.0), _TIME_ALLOWANCE + 6.0)
        self.assertIsNotNone(res, '位置も向きも動かないのに打ち切られない（固着を検知できない）')
        status, elapsed = res
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)
        self.assertGreater(elapsed, _TIME_ALLOWANCE - 1.5)
        self.assertLess(elapsed, _TIME_ALLOWANCE + 4.0)


class TestGeneratedKeepsProgressChecker(unittest.TestCase):
    """launch が複写する生成 yaml（patch_nav2_goal_tolerance）でも設定が残る。"""

    def test_patch_keeps_progress_checker(self):
        sys.path.insert(0, os.path.join(_WS_ROOT, 'src', 'th_bringup', 'launch'))
        import params_generation as pg
        patched = pg.patch_nav2_goal_tolerance(_CFG, 0.2)
        got = patched['controller_server']['ros__parameters']['progress_checker']
        self.assertEqual(got, _PC)
