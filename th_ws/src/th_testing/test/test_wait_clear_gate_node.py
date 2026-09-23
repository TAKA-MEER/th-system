"""test_wait_clear_gate_node.py — 退避待ちゲートの「disabled（人物検出 OFF）」結合試験

brief-tracker-default-off §3.3: wait_clear_gate を実際に起動し、
`/person/status` が is_lost=True / lost_reason="disabled"（人物検出 OFF 中に
bridge が出す強制値）の間は evt.clear_ok を出さない"本番の経路"を縛る。

見失い（disabled を含むどの lost_reason でも）は wait_clear_core の
target_visible=False に落ち、hold が 0 に戻る（test_wait_clear_core.py の
test_hold_resets_on_target_lost が純ロジック側）。ここではノード配線
（/person/status.is_lost → _person_map_xy()=None → 安全側）まで通して確認する。

制御対照: disabled を抜けて「見えている人」が十分離れている状態にすると
clear_ok が出る（ゲートが壊れているのではなく disabled だけが塞いでいる、
ということを同じセッションで示す）。
"""
import math
import time
import unittest

import pytest
import rclpy
from rclpy.node import Node

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import PersonStatus, StateEvent, SystemState


@pytest.mark.launch_test
def generate_test_description():
    gate = launch_ros.actions.Node(
        package='th_onsite',
        executable='wait_clear_gate.py',
        name='wait_clear_gate',
        parameters=[{
            'clear_distance_m': 0.5,
            'clear_hold_ms': 100,
            'clear_timeout_ms': 5000,
            'tick_hz': 20,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        gate,
        launch_testing.actions.ReadyToTest(),
    ])


class TestWaitClearGateNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_wait_clear_gate_client')

        self._events = []
        self.node.create_subscription(StateEvent, '/system/event', self._events.append, 10)

        # /system/state ・ /onsite/summon_goal は TRANSIENT_LOCAL で購読される
        self.pub_state = self.node.create_publisher(SystemState, '/system/state', 1)
        self.pub_goal = self.node.create_publisher(PoseStamped, '/onsite/summon_goal', 1)
        self.pub_person = self.node.create_publisher(PersonStatus, '/person/status', 10)
        self.pub_tf = self.node.create_publisher(TFMessage, '/tf_static', 1)

        # map ← base_link の恒等 TF（人座標の変換に使う）
        tf = TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.rotation.w = 1.0
        self.pub_tf.publish(TFMessage(transforms=[tf]))

        # SUMMON/WAIT_CLEAR に入れておく（transient_local でラッチ）
        self.pub_state.publish(SystemState(mode='SUMMON', state='WAIT_CLEAR'))
        self.pub_goal.publish(PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(position=Point(x=0.0, y=0.0, z=0.0))))
        self._spin(1.0)

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_person(self, is_lost: bool, lost_reason: str):
        person = PersonStatus()
        person.header.stamp = self.node.get_clock().now().to_msg()
        person.header.frame_id = 'base_link'
        person.position.x = 1.0
        person.position.y = 0.0
        person.position.z = 0.0
        person.confidence = 0.0 if is_lost else 0.9
        person.is_lost = is_lost
        person.lost_reason = lost_reason
        # 10Hz で流し続ける（gate は保持するだけだが実機と同じ状況にする）
        self.pub_person.publish(person)
        self._spin(0.1)

    def _clear_ok_events(self):
        return [e for e in self._events if e.event == 'evt.clear_ok']

    def _wait_event(self, event: str, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if any(e.event == event for e in self._events):
                return True
            self._spin(0.1)
        return False

    # ════════════════════════════════════════════════════════
    def test_disabled_person_never_fires_clear_ok(self):
        """disabled（人物検出 OFF 中）→ evt.clear_ok を一切出さない。"""
        for _ in range(15):            # 1.5 秒ぶん流し続ける（hold 100ms を大きく超える）
            self._publish_person(is_lost=True, lost_reason='disabled')
        assert self._clear_ok_events() == [], \
            'disabled 中に evt.clear_ok が出た'

    def test_disabled_blocks_then_far_person_fires(self):
        """対照: disabled 中は塞ぎ、抜けて「見えている人が十分離れる」と発火する。"""
        for _ in range(10):            # disabled で塞がれている状態
            self._publish_person(is_lost=True, lost_reason='disabled')
        assert self._clear_ok_events() == []

        # 見えている人（基地から1m。距離 1.0 ≥ clear_distance 0.5）
        self._events.clear()
        for _ in range(30):
            self._publish_person(is_lost=False, lost_reason='')
            if self._clear_ok_events():
                break
        assert self._clear_ok_events(), \
            '見えている人が十分離れているのに disabled のせいで発火しない（ゲートがへんな状態）'