"""test_person_tracker_bridge_node.py — 人物検出の OFF ゲート（本番の経路）の結合試験

brief-tracker-default-off §3.3: person_tracker_bridge を実際に起動し、
`/system/state.tracker_enabled` を見て、OFF 中は
  - /person/targets ・ /person/status が is_lost=True / lost_reason="disabled" /
    confidence=0.0 を出し、候補は空・selected_index=-1 になる
  - evt.target_lost を出さない
  - PersonTracker（SelectTarget）へ select サービスを呼ばない
ことを"本番の経路"で確かめる（純関数の apply_disabled だけでは呼び出し側の
数行の配線が縛れないため）。

変異チェック: 「bridge が /system/state を購読せず常に実データを出す」変異（②）
を、OFF 中に target_lost が出ないこと・select が呼ばれないことで弾く。
"""
import json
import time
import unittest

import pytest
import rclpy
from rclpy.node import Node

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Point
from multiple_sensor_person_tracking.msg import FollowingPosition, PersonCandidates
from multiple_sensor_person_tracking.srv import SelectTarget
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from th_system_msgs.msg import (PersonStatus, PersonTargets, StateEffect,
                                StateEvent, SystemState)


@pytest.mark.launch_test
def generate_test_description():
    bridge = launch_ros.actions.Node(
        package='th_perception',
        executable='person_tracker_bridge.py',
        name='person_tracker_bridge',
        parameters=[{'auto_select_hold_s': 0.2, 'tracker_lost_grace_ms': 0}],
        output='screen',
    )
    return launch.LaunchDescription([
        bridge,
        launch_testing.actions.ReadyToTest(),
    ])


class _FakeTrackerServices(Node):
    """PersonTracker になりきるサービス。呼ばれた回数を記録する。"""

    def __init__(self):
        super().__init__('fake_person_tracker')
        self.select_calls = []
        self.reset_calls = 0
        self.create_service(SelectTarget, '/person_tracker/select_target', self._on_select)
        self.create_service(Trigger, '/person_tracker/reset_tracking', self._on_reset)

    def _on_select(self, request, response):
        self.select_calls.append(request.candidate_index)
        response.success = True
        response.message = ''
        return response

    def _on_reset(self, request, response):
        self.reset_calls += 1
        response.success = True
        response.message = ''
        return response


class TestPersonTrackerBridgeNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_person_tracker_bridge_client')
        self.fake = _FakeTrackerServices()

        self._targets = []
        self._status = []
        self._events = []
        self.node.create_subscription(PersonTargets, '/person/targets', self._targets.append, 10)
        self.node.create_subscription(PersonStatus, '/person/status', self._status.append, 10)
        self.node.create_subscription(StateEvent, '/system/event', self._events.append, 10)

        self.pub_follow = self.node.create_publisher(FollowingPosition, _FOLLOW_TOPIC, 10)
        self.pub_stop = self.node.create_publisher(Bool, _STOP_TOPIC, 10)
        self.pub_cands = self.node.create_publisher(PersonCandidates, _CANDIDATES_TOPIC, 10)
        self.pub_effect = self.node.create_publisher(StateEffect, '/system/effect', 10)
        self.pub_state = self.node.create_publisher(SystemState, '/system/state', 1)

        self._spin(1.0)  # ブリッジの購読マッチングを待つ

    def tearDown(self):
        self.fake.destroy_node()
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            rclpy.spin_once(self.fake, timeout_sec=0.05)

    def _last_targets(self) -> PersonTargets:
        return self._targets[-1]

    def _wait_targets(self, timeout: float = 3.0):
        base = len(self._targets)
        deadline = time.time() + timeout
        while time.time() < deadline and len(self._targets) == base:
            self._spin(0.05)
        return len(self._targets) > base

    def _publish_tracker_state(self, enabled: bool):
        self.pub_state.publish(SystemState(tracker_enabled=enabled))
        self._spin(0.3)

    def _publish_tracking(self, status: int = 1):
        msg = FollowingPosition()
        msg.status = status
        msg.pose.position.x = 1.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub_follow.publish(msg)
        cands = PersonCandidates()
        if status == 1:
            p = Point()
            p.x = 1.0
            p.y = 0.0
            p.z = 0.0
            cands.positions.append(p)
        self.pub_cands.publish(cands)
        self.pub_stop.publish(Bool(data=False))
        self._wait_targets()

    def _publish_effect(self, name: str):
        ev = StateEffect()
        ev.dest = 'person_tracker'
        ev.name = name
        ev.args_json = json.dumps({'index': 0}) if name == 'set_target' else '{}'
        self.pub_effect.publish(ev)
        self._spin(0.5)

    # ════════════════════════════════════════════════════════
    def test_disabled_forces_lost_and_empty_candidates(self):
        """OFF 中：実データが流れてきても is_lost=True / disabled / 候補なし。"""
        self._publish_tracker_state(False)
        self._publish_tracking(status=1)   # 真には「検出あり」が来ている

        t = self._last_targets()
        assert t.is_lost is True
        assert t.lost_reason == 'disabled'
        assert t.confidence == 0.0
        assert len(t.candidates) == 0
        assert t.selected_index == -1

        s = self._status[-1]
        assert s.is_lost is True
        assert s.lost_reason == 'disabled'
        assert s.confidence == 0.0

    def test_disabled_suppresses_select_service_and_effects(self):
        """OFF 中：set_target / clear_selection の effect が来ても PersonTracker へ呼ばない。"""
        self._publish_tracker_state(False)
        self._publish_tracking(status=1)

        self._publish_effect('set_target')
        self._publish_effect('clear_selection')
        assert self.fake.select_calls == [], f'OFF 中に select が呼ばれた: {self.fake.select_calls}'
        assert self.fake.reset_calls == 0, 'OFF 中に reset が呼ばれた'

    def test_disabled_does_not_emit_target_lost(self):
        """OFF 中：真の検出が lost に変わり続けても evt.target_lost を出さない（変異②）。"""
        self._publish_tracker_state(False)
        self._publish_tracking(status=1)
        self._events.clear()
        self._publish_tracking(status=0)   # OFF のまま実データが lost 化
        self._spin(0.5)
        lost_events = [e for e in self._events if e.event == 'evt.target_lost']
        assert lost_events == [], 'OFF 中に evt.target_lost が出た（変異②）'

    def test_enable_unmasks_real_data_then_lost_edge(self):
        """ON 復帰：実データがそのまま出る。ON→OFF のち実際に lost になるなら 1 回だけ。"""
        self._publish_tracker_state(True)
        self._publish_tracking(status=1)
        t = self._last_targets()
        assert t.is_lost is False
        assert t.confidence > 0.0

        self._events.clear()
        self._publish_tracking(status=0)   # 真に lost
        lost_events = [e for e in self._events if e.event == 'evt.target_lost']
        assert len(lost_events) == 1, f'evt.target_lost が 1 回にならない: {lost_events}'


_FOLLOW_TOPIC = 'sobits_follower/multiple_sensor_person_tracking/following_position'
_STOP_TOPIC = 'sobits_follower/multiple_sensor_person_tracking/stop_following'
_CANDIDATES_TOPIC = 'sobits_follower/multiple_sensor_person_tracking/person_candidates'