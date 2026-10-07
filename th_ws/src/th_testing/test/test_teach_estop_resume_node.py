"""
test_teach_estop_resume_node.py
================================
1b-7 SG-B1 の launch_testing（state_manager＋route_recorder の本番の経路）。

教示中に非常停止 → 解除 → 戻る、の流れで「記録中」表示と実際の記録が一致すること。
旧挙動（モードを出た瞬間に自動保存して閉じる）では、戻って PAUSE →「はい」→ REC
へ進んでも recorder が閉じたままなので点が増えず、「記録中」なのに記録していない
状態になった。route_recorder は明示の finalize/discard でのみ閉じるため、復帰後は
続きから点が増えることを、FSM の状態（REC）と /route/status の点数で縛る。

rclpy 要＝ホスト除外（CLAUDE.md の除外一覧に載せる）。TIMEOUT 120。
"""
import json
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
from std_msgs.msg import Bool
from th_system_msgs.msg import StateEffect, StateEvent, SystemState, RouteStatus
from th_system_msgs.srv import UiTrigger


JOG_LEASE_MS = 300
LINK_WAIT_TIMEOUT_MS = 60_000
UI_ACTIVE_WINDOW_S = 5
SCREEN_STALE_MS = 60_000

ROUTE_ID = 'test_teach_estop_resume'


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[{
            'jog_lease_ms': JOG_LEASE_MS,
            'link_wait_timeout_ms': LINK_WAIT_TIMEOUT_MS,
            'ui_active_window_s': UI_ACTIVE_WINDOW_S,
            'screen_stale_ms': SCREEN_STALE_MS,
        }],
        output='screen',
    )
    route_recorder = launch_ros.actions.Node(
        package='th_planning',
        executable='route_recorder.py',
        name='route_recorder',
        parameters=[{
            # 途中経過 .wip を書かせない（この試験ではファイルを作らない）。
            'autosave_period_ms': 0,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        state_manager,
        route_recorder,
        launch_testing.actions.ReadyToTest(),
    ]), {}


_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


def _odom_msg(x: float) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.orientation.w = 1.0
    return msg


class TestTeachEstopResumeNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_teach_estop_resume_client')
        self._state_history = []
        self.node.create_subscription(
            SystemState, '/system/state', self._state_history.append, _STATE_QOS)
        self._status_history = []
        self.node.create_subscription(
            RouteStatus, '/route/status', self._status_history.append, 10)
        self._effect_history = []
        self.node.create_subscription(
            StateEffect, '/system/effect', self._effect_history.append, 10)
        self.pub_event = self.node.create_publisher(StateEvent, '/system/event', 10)
        self.pub_odom = self.node.create_publisher(Odometry, '/odom', 10)
        self.pub_ui_estop = self.node.create_publisher(Bool, '/safety/estop_ui', 10)
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0), \
            'state_manager が起動していない'
        self._x = 0.0

    def tearDown(self):
        # 試験で作った経路ファイルが残らないように掃除する（.json は作らない
        # はずだが、変異時はできるので全部消す）。
        routes_dir = '/root/th_data/routes'
        for ext in ('.json', '.wip', '.prev'):
            path = os.path.join(routes_dir, ROUTE_ID + ext)
            try:
                if os.path.exists(path):
                    os.remove(path)
            except OSError:
                pass
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────
    def _spin(self, duration: float = 0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _trigger(self, trigger: str, arg: dict = None) -> object:
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = 'test'
        future = self.cli_trigger.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        return future.result()

    def _publish_event(self, event: str):
        msg = StateEvent()
        msg.event = event
        msg.source_node = 'test'
        msg.arg_json = ''
        self.pub_event.publish(msg)
        self._spin(0.3)

    def _wait_state(self, mode: str, state: str, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._state_history and \
                    (self._state_history[-1].mode, self._state_history[-1].state) == (mode, state):
                return True
        return False

    def _feed_odom(self, steps: int, dx: float = 0.15, rate_hz: float = 20.0):
        """x を dx ずつ進めながら /odom を流す（間引き 0.10 m を超える刻み）。"""
        period = 1.0 / rate_hz
        for _ in range(steps):
            self._x += dx
            self.pub_odom.publish(_odom_msg(self._x))
            time.sleep(period)
        self._spin(0.8)  # sample(10Hz)＋status(2Hz) の反映待ち

    def _latest_points(self) -> int:
        self._spin(0.6)
        assert self._status_history, '/route/status を1件も受信していない'
        return int(self._status_history[-1].points)

    # ── 本体 ──────────────────────────────────────────────
    def test_estop_return_continues_recording(self):
        # 起動（INIT/CHECK）→ 疎通 OK → IDLE。
        assert self._wait_state('INIT', 'CHECK', timeout=10.0), \
            'state_manager が INIT/CHECK で起動していない'
        self._publish_event('evt.link_ok')
        assert self._wait_state('IDLE', 'NONE'), 'IDLE に進まない'

        # 教示（手動）に入り、記録開始 → REC。
        res = self._trigger('ui.enter_mode', {'mode': 'TEACH_MANUAL'})
        assert res.accepted, f'TEACH_MANUAL に入れない: {res.reject_reason_key}'
        assert self._wait_state('TEACH_MANUAL', 'ROUTE_SEL')
        res = self._trigger('ui.route_select', {'new': True, 'id': ROUTE_ID})
        assert res.accepted, f'記録開始できない: {res.reject_reason_key}'
        assert self._wait_state('TEACH_MANUAL', 'REC')

        # 走らせて点を積む（記録中の証拠）。
        self._feed_odom(20)
        points_before = self._latest_points()
        assert points_before >= 5, f'記録中に点が増えない: {points_before}'

        # UI 非常停止 → ESTOP。非常停止中も走らせてみる（記録に積まれないこと）。
        self.pub_ui_estop.publish(Bool(data=True))
        assert self._wait_state('ESTOP', 'NONE'), 'ESTOP に入らない'
        self._feed_odom(10)
        points_estop = self._latest_points()
        assert points_estop == points_before, \
            f'ESTOP 中に点が増えた（{points_before} → {points_estop}）。' \
            '非常停止中に記録してはいけない'

        # 解除 → ESTOP のまま再開確認 →「戻る」→ TEACH/PAUSE（勝手に走り出さない）。
        self.pub_ui_estop.publish(Bool(data=False))
        self._spin(0.5)
        res = self._trigger('ui.resume_yes')
        assert res.accepted, f'「戻る」が拒否された: {res.reject_reason_key}'
        assert self._wait_state('TEACH_MANUAL', 'PAUSE'), \
            'TEACH/PAUSE に戻らない（戻り先が PAUSE でない）'

        # 「はい」→ REC。旧挙動では recorder が閉じているので点が増えない。
        res = self._trigger('ui.resume_yes')
        assert res.accepted, f'再開が拒否された: {res.reject_reason_key}'
        assert self._wait_state('TEACH_MANUAL', 'REC'), 'REC に戻らない'
        self._feed_odom(20)
        points_after = self._latest_points()
        assert points_after > points_before, \
            f'戻っても記録が再開しない（{points_before} → {points_after}）。' \
            '「記録中」なのに記録していない SG-B1 の再発'

        # 自動保存されていないこと（明示の保存をしていないので .json は無い）。
        assert not os.path.exists(f'/root/th_data/routes/{ROUTE_ID}.json'), \
            '保存していないのに .json ができている（自動保存が残っている）'
