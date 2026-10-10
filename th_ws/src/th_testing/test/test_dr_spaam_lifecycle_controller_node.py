"""test_dr_spaam_lifecycle_controller_node.py — DR-SPAAM 起動/停止アクチュエータの結合試験

brief-tracker-default-off §3.2: dr_spaam_lifecycle_controller を実際に起動し、
`/system/state` の `tracker_enabled` を見て DR-SPAAM の lifecycle サービス
（GetState / ChangeState）へ activate / deactivate を飛ばす"本番の経路"を縛る。
DR-SPAAM 本体の代わりに、テストノードが fake の lifecycle サービスを立てる。

変異チェック: 「tracker_enabled を見ずに常に activate する」等の配線バグを、
本ノードのテストは変化なし状態（OFF・INACTIVE）で ChangeState が来ないことで弾く。
core の transition ID が lifecycle_msgs の実値からずれると、activate のつもりの
要求が CLEANUP 等に化けて fake の状態が期待と違う方向へ動くため、やはり赤になる
（2026-10-10 の実機事故の再発防止。fake は core の定数と比べない）。
"""
import time
import unittest

import pytest
import rclpy
from rclpy.node import Node

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.msg import Transition as LifecycleTransition
from lifecycle_msgs.srv import ChangeState, GetState
from th_system_msgs.msg import SystemState

# dr_spaam_lifecycle_controller.py の _state_qos() と同じ TRANSIENT_LOCAL。
# plain int（既定 VOLATILE）で publisher を作ると durability 不一致で
# コントローラの購読に一切届かない（QoS incompatible。DDS がサイレントに
# マッチさせないだけで例外は出ないため気づきにくい）。
_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

class _FakeLifecycle(Node):
    """DR-SPAAM になりきる 2 サービス。本物の lifecycle 遷移表で応答する。

    受け付けるのは lifecycle_msgs の実値（CONFIGURE=1 / CLEANUP=2 /
    ACTIVATE=3 / DEACTIVATE=4）のみで、core の定数とは比べない。
    不正な遷移には success=false を返す（本物と同様）。
    """

    def __init__(self):
        super().__init__('fake_dr_spaam_lifecycle')
        self.state_id = LifecycleState.PRIMARY_STATE_INACTIVE
        self.change_requests = []

        self.create_service(
            GetState, '/dr_spaam/dr_spaam_ros/get_state', self._on_get_state)
        self.create_service(
            ChangeState, '/dr_spaam/dr_spaam_ros/change_state', self._on_change_state)

    def _on_get_state(self, request, response):
        response.current_state.id = self.state_id
        response.current_state.label = {
            LifecycleState.PRIMARY_STATE_UNCONFIGURED: 'unconfigured',
            LifecycleState.PRIMARY_STATE_INACTIVE: 'inactive',
            LifecycleState.PRIMARY_STATE_ACTIVE: 'active'}.get(
            self.state_id, 'unknown')
        return response

    def _on_change_state(self, request, response):
        tid = request.transition.id
        self.change_requests.append(tid)
        ok = (
            (tid == LifecycleTransition.TRANSITION_ACTIVATE
             and self.state_id == LifecycleState.PRIMARY_STATE_INACTIVE,
             LifecycleState.PRIMARY_STATE_ACTIVE),
            (tid == LifecycleTransition.TRANSITION_DEACTIVATE
             and self.state_id == LifecycleState.PRIMARY_STATE_ACTIVE,
             LifecycleState.PRIMARY_STATE_INACTIVE),
            (tid == LifecycleTransition.TRANSITION_CONFIGURE
             and self.state_id == LifecycleState.PRIMARY_STATE_UNCONFIGURED,
             LifecycleState.PRIMARY_STATE_INACTIVE),
            (tid == LifecycleTransition.TRANSITION_CLEANUP
             and self.state_id == LifecycleState.PRIMARY_STATE_INACTIVE,
             LifecycleState.PRIMARY_STATE_UNCONFIGURED),
        )
        for valid, next_state in ok:
            if valid:
                self.state_id = next_state
                response.success = True
                return response
        response.success = False
        return response


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package='th_perception',
        executable='dr_spaam_lifecycle_controller.py',
        name='dr_spaam_lifecycle_controller',
        parameters=[{'reconcile_period_s': 0.1}],
        output='screen',
    )
    return launch.LaunchDescription([
        controller,
        launch_testing.actions.ReadyToTest(),
    ])


class TestDrSpaamLifecycleControllerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_dr_spaam_lifecycle_controller_client')
        self.fake = _FakeLifecycle()

        # /system/state は TRANSIENT_LOCAL（state_manager の既定）で購読される。
        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', _STATE_QOS)
        self._tracker = False
        self._pb_timer = self.node.create_timer(
            0.1, lambda: self.pub_state.publish(
                SystemState(tracker_enabled=self._tracker)))
        self._spin(0.5)

    def tearDown(self):
        if getattr(self, '_pb_timer', None) is not None:
            self._pb_timer.cancel()
            self.node.destroy_timer(self._pb_timer)
            self._pb_timer = None
        self.fake.destroy_node()
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            rclpy.spin_once(self.fake, timeout_sec=0.05)

    def _publish_tracker(self, enabled: bool):
        self._tracker = enabled
        self.pub_state.publish(SystemState(tracker_enabled=enabled))

    def _wait_transition(self, transition_id: int, timeout: float = 5.0) -> bool:
        """指定 transition_id が DR-SPAAM へ要求されるまで待つ。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if transition_id in self.fake.change_requests:
                return True
            self._spin(0.1)
        return False

    # ════════════════════════════════════════════════════════
    def test_no_transition_while_disabled_and_inactive(self):
        """既定状態（OFF・INACTIVE）では DR-SPAAM に何も要求しない。"""
        assert self.fake.change_requests == []

    def test_activate_when_enabled_then_deactivate_when_disabled(self):
        """ON（S-20/S-21 の人検出開始）→ activate、OFF → deactivate。"""
        self._publish_tracker(True)
        assert self._wait_transition(LifecycleTransition.TRANSITION_ACTIVATE), \
            'tracker_enabled=true なのに activate が DR-SPAAM へ届かない'

        self._publish_tracker(False)
        assert self._wait_transition(LifecycleTransition.TRANSITION_DEACTIVATE), \
            'tracker_enabled=false なのに deactivate が DR-SPAAM へ届かない'

    def test_reconcile_reactivates_after_external_deactivate(self):
        """突き合わせ: ON のまま外部から deactivate されても次周期で activate し直す。"""
        self._publish_tracker(True)
        assert self._wait_transition(LifecycleTransition.TRANSITION_ACTIVATE)
        self.fake.change_requests.clear()

        # 外部（別の何か）が DR-SPAAM を deactivate した状態を取り戻す
        self.fake.state_id = LifecycleState.PRIMARY_STATE_INACTIVE
        assert self._wait_transition(LifecycleTransition.TRANSITION_ACTIVATE), \
            'ON のまま DR-SPAAM が INACTIVE に戻されたのに再 activate されない'