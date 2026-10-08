"""
test_teach_start_yaw_node.py
==============================
1b-7 S-13（SG-C5）の launch_testing（route_recorder 単体の本番の経路）。

記録開始の姿勢（yaw）が /route/status の最上位 start_yaw に載ること。
S-13「開始時の向き」表示の源。effect/state/odom のトピックが本番の
受け渡し口なので、FSM が無くても発行側の配線を縛れる。
_build_status_msg の publish の 1 行を消す変異で赤くなる。

rclpy 要＝ホスト除外（CLAUDE.md の除外一覧に載せない。報告の
「ROS 要の新しい試験ファイル」に列挙する）。TIMEOUT 120。
"""
import json
import math
import os
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from nav_msgs.msg import Odometry
from th_system_msgs.msg import (RouteStatus, StateEffect, SystemState)


_WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def _generated_yaml(node: str) -> str:
    """本番の bringup と同じ生成 yaml を先に読ませる（test_opcheck_runner_node.py
    と同じ作法。rclpy は INTEGER↔DOUBLE を変換しないので、dict だけ渡すと
    本番の型ずれがすり抜ける）。"""
    for d in (os.environ.get('TH_GENERATED_DIR', ''), '/root/th_data/generated',
              os.path.join(_WS_ROOT, 'data', 'generated')):
        path = os.path.join(d, f'{node}.yaml') if d else ''
        if path and os.path.exists(path):
            return path
    raise FileNotFoundError(f'生成 yaml {node}.yaml が見つからない')


@pytest.mark.launch_test
def generate_test_description():
    route_recorder = launch_ros.actions.Node(
        package='th_planning',
        executable='route_recorder.py',
        name='route_recorder',
        parameters=[_generated_yaml('route_recorder'), {
            'autosave_period_ms': 0,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        route_recorder,
        launch_testing.actions.ReadyToTest(),
    ]), {}


_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


def _odom_msg(x: float, yaw: float) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class TestTeachStartYawNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_start_yaw_client')
        self._statuses = []
        self.node.create_subscription(
            RouteStatus, '/route/status', self._statuses.append, 10)
        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', _STATE_QOS)
        self.pub_effect = self.node.create_publisher(
            StateEffect, '/system/effect', 10)
        self.pub_odom = self.node.create_publisher(Odometry, '/odom', 10)
        self._spin(1.0)  # discovery 待ち

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────
    def _spin(self, duration: float = 0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_status_with_points(self, timeout: float = 8.0):
        """points > 0 の RouteStatus を待つ（記録が開いて点が載ったもの）。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            for st in self._statuses:
                if st.points > 0:
                    return st
        return None

    # ── 本体 ──────────────────────────────────────────────
    def test_start_yaw_published_on_record_start(self):
        """記録開始の姿勢の yaw が /route/status.start_yaw に載る。

        publish の 1 行（_build_status_msg の start_yaw 代入）を消すと
        既定 0.0 のままになり、この試験が赤くなる。
        """
        yaw = 1.0
        # 開始姿勢を先に流す（recorder は start_record 時の最新 odom を使う）。
        for _ in range(5):
            self.pub_odom.publish(_odom_msg(0.0, yaw))
            self._spin(0.1)
        state = SystemState()
        state.mode = 'TEACH_MANUAL'
        state.state = 'REC'
        for _ in range(3):
            self.pub_state.publish(state)
            self._spin(0.2)
        eff = StateEffect()
        eff.name = 'start_record'
        eff.dest = 'route_recorder'
        eff.args_json = json.dumps({'route_id': 'test_start_yaw'})
        self.pub_effect.publish(eff)
        # 記録中は odom を流し続ける（途絶えると連続性切れで別経路に入る）。
        deadline = time.time() + 8.0
        st = None
        x = 0.0
        while time.time() < deadline and st is None:
            x += 0.05
            self.pub_odom.publish(_odom_msg(x, yaw))
            self._spin(0.1)
            for s in self._statuses:
                if s.points > 0:
                    st = s
                    break
        assert st is not None, '記録開始後に points > 0 の /route/status が来ない'
        assert abs(float(st.start_yaw) - yaw) < 1e-3, \
            f'start_yaw が記録開始の姿勢と違う: {st.start_yaw} != {yaw}'
        # 後始末：記録を閉じる（次の試験・再送に残さない）。
        eff = StateEffect()
        eff.name = 'discard_route'
        eff.dest = 'route_recorder'
        eff.args_json = '{}'
        self.pub_effect.publish(eff)
        self._spin(0.5)
