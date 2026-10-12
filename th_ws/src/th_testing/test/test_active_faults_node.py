"""
test_active_faults_node.py — safety_monitor の /safety/active_faults と、
それを入力にする fault_recorder の本番経路（Spec-safety §3.5.5）。

/safety/fault は「代表 1 件」しか運ばないので、複数同時や重大フォルト継続中の
回復フォルトが見えない。履歴の記録はそのため /safety/active_faults（立っている
種別の集合）を使う。ここでは本物の safety_monitor に入力を止めたり再開したりして、

  a. 単独の発生・解消が集合に出る
  b. 2 種別が同時に出る（辞書順）。片方だけ解消すると残りだけになる
  c. 重大(LIMITER_DEAD)の継続中に立った回復(LIDAR_LOST)が集合に出る
     （/safety/fault には出ない）。severities が types と同じ順
  d. 後から購読した側に最新の集合が届く（transient_local）
  e. 本物の fault_recorder がそれを履歴にする（種別ごとに別の 1 件・ファイルへ追記）

を縛る。fault_recorder は safety_monitor が出す QoS・同じ順序でそのまま受ける。
"""

import json
import os
import tempfile
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
from sensor_msgs.msg import LaserScan
from th_system_msgs.msg import (ActiveFaults, FaultHistory, FaultStatus,
                                LimiterStatus, WheelFeedback)

_WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
_LOG_DIR = tempfile.mkdtemp(prefix='fault_log_test_')
WINDOW_MS = 400


def _generated_yaml(node: str) -> str:
    for d in (os.environ.get('TH_GENERATED_DIR', ''), '/root/th_data/generated',
              os.path.join(_WS_ROOT, 'data', 'generated')):
        path = os.path.join(d, f'{node}.yaml') if d else ''
        if path and os.path.exists(path):
            return path
    raise FileNotFoundError(f'生成 yaml {node}.yaml が見つからない')


@pytest.mark.launch_test
def generate_test_description():
    safety = launch_ros.actions.Node(
        package='th_safety', executable='safety_monitor', name='safety_monitor',
        parameters=[_generated_yaml('safety_monitor'), {
            'lidar_timeout_ms': 400, 'esp32_timeout_ms': 400,
            'limiter_dead_ms': 1500, 'critical_fault_hold_ms': 100,
            'check_period_ms': 50, 'startup_grace_sec': 1,
            'enabled_targets': ['lidar', 'esp32', 'limiter'],
        }], output='screen')
    recorder = launch_ros.actions.Node(
        package='th_state', executable='fault_recorder.py', name='fault_recorder',
        parameters=[{'fault_outcome_window_ms': WINDOW_MS,
                     'fault_log_max_bytes': 1000000,
                     'fault_log_dir': _LOG_DIR}], output='screen')
    return launch.LaunchDescription([
        safety, recorder, launch_testing.actions.ReadyToTest(),
    ]), {'safety_monitor': safety, 'fault_recorder': recorder}


_LATCH = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    history=QoSHistoryPolicy.KEEP_LAST)


def _scan(stamp):
    m = LaserScan()
    m.header.stamp = stamp
    m.angle_min, m.angle_max, m.angle_increment = -3.14, 3.14, 0.01
    m.range_min, m.range_max = 0.1, 12.0
    m.ranges = [1.0] * 628
    return m


class TestActiveFaults(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_active_faults')
        self.sets = []        # (types, severities)
        self.fault = []
        self.hist = []
        self.node.create_subscription(
            ActiveFaults, '/safety/active_faults',
            lambda m: self.sets.append((list(m.types), list(m.severities))), _LATCH)
        self.node.create_subscription(FaultStatus, '/safety/fault', self.fault.append, 10)
        self.node.create_subscription(
            FaultHistory, '/system/fault_history', self.hist.append, _LATCH)
        self.p_scan = self.node.create_publisher(LaserScan, '/scan', 10)
        self.p_wf = self.node.create_publisher(WheelFeedback, '/esp32/wheel_feedback', 10)
        self.p_lim = self.node.create_publisher(LimiterStatus, '/safety/limiter_status', 10)
        time.sleep(3.5)
        self.settle()

    def tearDown(self):
        self.node.destroy_node()

    def scan(self):
        self.p_scan.publish(_scan(self.node.get_clock().now().to_msg()))

    def wf(self):
        self.p_wf.publish(WheelFeedback())

    def lim(self):
        self.p_lim.publish(LimiterStatus(alive=True, action='PASS'))

    def pump(self, sec, alive=()):
        end = time.time() + sec
        while time.time() < end:
            for f in alive:
                f()
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def until(self, cond, alive=(), timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.1, alive)
            if cond():
                return True
        return False

    def settle(self):
        """全入力を送って、集合が空で安定するまで。"""
        all_alive = (self.scan, self.wf, self.lim)
        # 前の試験で立っていた可能性があるので、いったん何か出るまで待ってから戻す
        self.until(lambda: bool(self.sets) and self.sets[-1][0] == [], all_alive, 60.0)
        self.pump(2.0, all_alive)
        assert self.sets and self.sets[-1][0] == [], f'空にならない: {self.sets[-3:]}'

    def test_a_single_fault_appears_and_clears(self):
        alive = (self.wf, self.lim)
        assert self.until(lambda: self.sets[-1] == (['LIDAR_LOST'], ['RECOVERABLE']), alive), \
            f'LIDAR_LOST の集合が出ない: {self.sets[-3:]}'
        assert self.until(lambda: self.sets[-1][0] == [], (self.scan, self.wf, self.lim)), \
            '解消で空の集合が出ない'

    def test_b_two_types_then_one_clears(self):
        assert self.until(
            lambda: self.sets[-1][0] == ['ESP32_DISCONNECTED', 'LIDAR_LOST'], (self.lim,)), \
            f'2 種別が集合に出ない: {self.sets[-3:]}'
        assert self.until(
            lambda: self.sets[-1][0] == ['ESP32_DISCONNECTED'], (self.scan, self.lim)), \
            f'片方の解消で残りだけにならない: {self.sets[-3:]}'
        assert self.until(lambda: self.sets[-1][0] == [], (self.scan, self.wf, self.lim))

    def test_c_recoverable_during_critical_is_visible(self):
        assert self.until(lambda: self.sets[-1][0] == ['LIMITER_DEAD'],
                          (self.scan, self.wf)), f'{self.sets[-3:]}'
        mark = len(self.fault)
        assert self.until(
            lambda: self.sets[-1] == (['LIDAR_LOST', 'LIMITER_DEAD'],
                                      ['RECOVERABLE', 'CRITICAL']), (self.wf,)), \
            f'重大継続中の回復フォルトが集合に出ない: {self.sets[-3:]}'
        # /safety/fault 側には LIDAR_LOST が出ない（代表が重大のまま）。だから別に出す
        assert not any(f.active and f.fault_type == 'LIDAR_LOST' for f in self.fault[mark:])
        assert self.until(lambda: self.sets[-1][0] == [], (self.scan, self.wf, self.lim))

    def test_d_late_subscriber_gets_latest(self):
        got = []
        n = rclpy.create_node('test_active_faults_late')
        n.create_subscription(ActiveFaults, '/safety/active_faults',
                              lambda m: got.append(list(m.types)), _LATCH)
        end = time.time() + 5
        while time.time() < end and not got:
            rclpy.spin_once(n, timeout_sec=0.1)
            self.scan(), self.wf(), self.lim()
        n.destroy_node()
        assert got and got[-1] == [], f'後から購読して最新が届かない: {got}'

    def test_e_recorder_records_each_type_and_writes_file(self):
        assert self.until(
            lambda: self.sets[-1][0] == ['ESP32_DISCONNECTED', 'LIDAR_LOST'], (self.lim,))
        assert self.until(lambda: self.sets[-1][0] == [], (self.scan, self.wf, self.lim))
        self.pump(WINDOW_MS / 1000.0 + 0.8, (self.scan, self.wf, self.lim))
        assert self.hist, '履歴が publish されない'
        latest = {r.fault_type: r for r in self.hist[-1].records}
        assert 'LIDAR_LOST' in latest and 'ESP32_DISCONNECTED' in latest, \
            [r.fault_type for r in self.hist[-1].records]
        for t in ('LIDAR_LOST', 'ESP32_DISCONNECTED'):
            r = latest[t]
            assert not r.ongoing and r.duration_s >= 0.0 and r.severity == 'RECOVERABLE'
        path = os.path.join(_LOG_DIR, 'fault_log.jsonl')
        assert os.path.exists(path), 'ファイルが作られない'
        lines = [json.loads(x) for x in open(path, encoding='utf-8') if x.strip()]
        finals = {d['fault_type'] for d in lines if d['final']}
        assert {'LIDAR_LOST', 'ESP32_DISCONNECTED'} <= finals, lines
