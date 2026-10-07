"""
test_teach_record_broken_node.py
==================================
1b-7 SG-B18（SM-3.1.2-019）の launch_testing（route_recorder 単体の本番の経路）。

記録の連続性が切れたら（自己位置の飛び・オドメトリ途絶）、route_recorder が
/system/event に evt.record_broken を出すこと。state_manager は起動しない
（effect/state/event のトピックが本番の受け渡し口なので、FSM が無くても
発行側の配線を縛れる）。しきい値自体は純粋試験と W-03 の registry 駆動で縛る。

rclpy 要＝ホスト除外（CLAUDE.md の除外一覧に載せる）。TIMEOUT 120。
"""
import json
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
from th_system_msgs.msg import StateEffect, StateEvent, SystemState


ROUTE_JUMP = 'test_record_broken_jump'
ROUTE_GAP = 'test_record_broken_gap'


@pytest.mark.launch_test
def generate_test_description():
    route_recorder = launch_ros.actions.Node(
        package='th_planning',
        executable='route_recorder.py',
        name='route_recorder',
        parameters=[{
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


def _odom_msg(x: float) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.orientation.w = 1.0
    return msg


class TestTeachRecordBrokenNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_record_broken_client')
        self._events = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._events.append, 10)
        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', _STATE_QOS)
        self.pub_effect = self.node.create_publisher(
            StateEffect, '/system/effect', 10)
        self.pub_odom = self.node.create_publisher(Odometry, '/odom', 10)
        self._spin(1.0)  # discovery 待ち
        self._x = 0.0

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────
    def _spin(self, duration: float = 0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _begin_recording(self, route_id: str):
        """教示系・REC にして記録を始める（FSM の代わり）。"""
        state = SystemState()
        state.mode = 'TEACH_MANUAL'
        state.state = 'REC'
        for _ in range(3):
            self.pub_state.publish(state)
            self._spin(0.2)
        eff = StateEffect()
        eff.name = 'start_record'
        eff.dest = 'route_recorder'
        eff.args_json = json.dumps({'route_id': route_id})
        self.pub_effect.publish(eff)
        self._spin(0.5)

    def _isolate_recorder(self):
        """前の試験の記録・ラッチ・再送を落として分離する。

        同じ route_recorder プロセスを使い回すため、前の試験の記録が開いたまま
        （ラッチ済みなら _status_timer が evt.record_broken を出し続ける）だと、
        新しい試験の購読に混ざる。discard_route で前の記録を閉じる
        （閉じれば _sample_timer は何も見ない・再送も止まる）と、観測済みを捨てる。
        記録が開いていなければ discard は無視されるだけ。各試験の最初に呼ぶこと。
        前提 assert（正常走行で出ない）は残す。
        """
        eff = StateEffect()
        eff.name = 'discard_route'
        eff.dest = 'route_recorder'
        eff.args_json = '{}'
        self.pub_effect.publish(eff)
        self._spin(0.5)
        self._events.clear()

    def _feed_odom(self, steps: int, dx: float = 0.15, rate_hz: float = 20.0):
        period = 1.0 / rate_hz
        for _ in range(steps):
            self._x += dx
            self.pub_odom.publish(_odom_msg(self._x))
            time.sleep(period)
        self._spin(0.5)

    def _wait_broken_event(self, timeout: float = 8.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if any(e.event == 'evt.record_broken' for e in self._events):
                return True
        return False

    def _broken_from_recorder(self) -> bool:
        return any(e.event == 'evt.record_broken'
                   and getattr(e, 'source_node', '') == 'route_recorder'
                   for e in self._events)

    # ── 本体 ──────────────────────────────────────────────
    def test_jump_emits_record_broken(self):
        """5 m の飛び（手押し相当）で evt.record_broken が出る。"""
        self._isolate_recorder()
        self._begin_recording(ROUTE_JUMP)
        self._feed_odom(10)
        assert not any(e.event == 'evt.record_broken' for e in self._events), \
            '正常な走行で record_broken が出た'
        # 一気に 5 m 飛ばす（route_jump_m=0.5 を十分超える）。
        self._x += 5.0
        # 飛んだ後も odom を送り続ける（跳んだ位置から少しずつ進む）。
        # 送るのを止めて長く待つと途切れ判定（3 秒）が満たされ、jump 検出を
        # 殺す変異がすり抜ける。途切れの半分の 1.5 秒以内に出ることを縛る。
        deadline = time.time() + 1.5
        found = False
        while time.time() < deadline:
            self._x += 0.05
            self.pub_odom.publish(_odom_msg(self._x))
            self._spin(0.05)
            if any(e.event == 'evt.record_broken' for e in self._events):
                found = True
                break
        assert found, \
            '5 m 飛んで 1.5 秒以内に evt.record_broken が出ない'
        assert self._broken_from_recorder(), \
            'evt.record_broken の source_node が route_recorder でない'

    def test_odom_gap_emits_record_broken(self):
        """オドメトリ途絶（3 s 超）で evt.record_broken が出る。
        registry の route_gap_timeout_ms=3000 の本番値で見る。"""
        self._isolate_recorder()
        self._begin_recording(ROUTE_GAP)
        self._feed_odom(10)
        assert not any(e.event == 'evt.record_broken' for e in self._events), \
            '正常な走行で record_broken が出た'
        # /odom を止める（3.5 s。タイムアウト 3.0 s を超える）。
        self._spin(3.5)
        assert self._wait_broken_event(), \
            'オドメトリが 3.5 s 途絶えても evt.record_broken が出ない'
        assert self._broken_from_recorder()
