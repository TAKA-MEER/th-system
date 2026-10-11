"""
test_init_estop_held_guide_node.py
===================================
T-6（2026-10-10）— 物理非常停止が押されたまま起動したときの「解除してください」案内。

spec: Spec-modes.md §4.1 例外 2・SM-3.1.2-003 ／ Spec-ops.md（CL-B-6）。
state_manager・connectivity_checker（sim）・safety_monitor を本番どおりに起動し、起動直後から
`/safety/estop_hw=true` を流し続けた状態で次を縛る。

 (a) 押されている間は INIT/CHECK のまま IDLE へ進まない（疎通が揃っていても）
 (b) 案内を出すための状態が /system/state に載り続ける
     （mode=INIT かつ estop_hw=true。**画面を後から開いた購読者**でも読める
     transient_local の状態。一回きりの /system/effect は取りこぼされる）
 (c) false にしたら IDLE へ進む

注意（launch_testing の挙動）: モジュール直下の素の pytest 関数は収集されない
ので、全部メソッドにする。
"""
import os
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from th_system_msgs.msg import StateEffect, SystemState
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

SCAN_EXPECTED_POINTS = 8

_WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def _generated_yaml(node: str) -> str:
    # 本番の bringup と同じ生成 yaml を先に読ませる（test_fault_aggregate_node.py と同じ解決）。
    for d in (os.environ.get('TH_GENERATED_DIR', ''), '/root/th_data/generated',
              os.path.join(_WS_ROOT, 'data', 'generated')):
        path = os.path.join(d, f'{node}.yaml') if d else ''
        if path and os.path.exists(path):
            return path
    raise FileNotFoundError(f'生成 yaml {node}.yaml が見つからない')


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state', executable='state_manager.py', name='state_manager',
        parameters=[{'jog_lease_ms': 300, 'link_wait_timeout_ms': 60_000,
                     'ui_active_window_s': 5, 'screen_stale_ms': 60_000}],
        output='screen')
    connectivity_checker = launch_ros.actions.Node(
        package='th_state', executable='connectivity_checker.py',
        name='connectivity_checker',
        parameters=[{
            'esp32_alive_timeout_ms': 3000,
            'scan_expected_points': SCAN_EXPECTED_POINTS,
            'required_nodes': ['unused_in_sim'],
            'restart_max_count': 3, 'restart_wait_ms': 5000, 'sim': True,
        }],
        output='screen')
    # 本番の安全監視も同時に起動する（押下で ESTOP 等へ落とされないことを含めて縛る）。
    # 対象は /scan と /system/state だけで判定できる 2 つに絞る。
    safety_monitor = launch_ros.actions.Node(
        package='th_safety', executable='safety_monitor', name='safety_monitor',
        parameters=[_generated_yaml('safety_monitor'), {
            'startup_grace_sec': 1, 'enabled_targets': ['lidar', 'state']}],
        output='screen')
    return launch.LaunchDescription([
        safety_monitor, state_manager, connectivity_checker,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_manager,
         'connectivity_checker': connectivity_checker}


_STATE_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST)
_EFFECT_QOS = QoSProfile(
    depth=10, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)


class TestInitEstopHeldGuideNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_init_estop_held_client')
        self.states = []
        self.effects = []
        self.node.create_subscription(
            SystemState, '/system/state', self.states.append, _STATE_QOS)
        self.node.create_subscription(
            StateEffect, '/system/effect', self.effects.append, _EFFECT_QOS)
        self.pub_scan = self.node.create_publisher(LaserScan, '/scan', 5)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_ui = self.node.create_publisher(Bool, '/safety/estop_ui', 10)
        self._timers = []

    def tearDown(self):
        for t in self._timers:
            t.cancel()
            self.node.destroy_timer(t)
        self.node.destroy_node()

    def _spin(self, duration=0.2):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_state(self, mode, state, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self.states and (self.states[-1].mode, self.states[-1].state) == (mode, state):
                return True
        return False

    def _hw_value(self):
        return self._hw

    def _start_hw(self, value):
        self._hw = value
        self.pub_hw.publish(Bool(data=value))
        self._timers.append(self.node.create_timer(
            0.1, lambda: self.pub_hw.publish(Bool(data=self._hw))))

    def _start_scan(self):
        def _pub():
            m = LaserScan()
            m.ranges = [1.0] * SCAN_EXPECTED_POINTS
            self.pub_scan.publish(m)
        _pub()
        self._timers.append(self.node.create_timer(0.2, _pub))

    def test_held_at_boot_stays_init_with_guide_then_releases_to_idle(self):
        # 起動直後から押されたまま流し続ける。疎通（/scan）は揃っているので、
        # IDLE へ進めない原因は物理ボタンだけ。
        self._start_hw(True)
        self._start_scan()
        end = time.time() + 15.0
        while not self.states and time.time() < end:
            self._spin(0.1)

        # (a)(b) 5 秒たっても INIT/CHECK のまま、状態は「押されている」を載せている。
        self._spin(5.0)
        assert self.states, '/system/state を受信していない'
        cur = self.states[-1]
        assert (cur.mode, cur.state) == ('INIT', 'CHECK'), \
            f'押されたまま起動で INIT/CHECK から出た: {cur.mode}/{cur.state}'
        assert cur.estop_hw is True, 'state.estop_hw が真になっていない（案内の根拠が無い）'
        assert not any(s.mode in ('IDLE', 'ESTOP', 'CARRY') for s in self.states), \
            f'途中で IDLE/ESTOP/CARRY に入った: {[(s.mode, s.state) for s in self.states]}'

        # (b') 後から画面を開いた購読者でも読める: 新しい購読者に最新の状態が届く。
        late = []
        sub = self.node.create_subscription(
            SystemState, '/system/state', late.append, _STATE_QOS)
        self._spin(1.0)
        self.node.destroy_subscription(sub)
        assert late and (late[-1].mode, late[-1].estop_hw) == ('INIT', True), \
            '後から購読した画面に「INIT かつ押下」が届かない'

        # 起動時案内の effect（state_manager が本番どおり publish する）。
        guides = [e for e in self.effects if e.name == 'guide']
        assert guides, 'T-INIT-03 の guide effect が一度も出ていない'
        assert '"estop_held_at_boot"' in guides[0].args_json, guides[0].args_json

        # (c) 戻したら IDLE へ進む。
        self._hw = False
        assert self._wait_state('IDLE', 'NONE', timeout=8.0), \
            '物理ボタンを戻しても IDLE へ進まない'
