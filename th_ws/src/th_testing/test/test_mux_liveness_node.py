"""
test_mux_liveness_node.py
=========================
SG-A11（2026-10-08 ユーザー決定）: `MUX_DEAD` の生存確認を、safety_monitor ノード本体と
**本物の twist_mux** を実際に起動して縛る Docker launch テスト。

現行の流れ判定（detect_mux_dead）は、入力が無くて出力が黙っている停止中に twist_mux が
死んでも見えない。生存確認は「/cmd_vel_muxed の publisher に twist_mux ノードが居るか」を
見る。最初は記録だけ（mux_report_only。人物追跡の person_report_only と同じ形）。

2 台の safety_monitor を立てる（mux_report_only は起動時パラメータで、1 台では
両方を縛れない。test_person_tracker_lost_gate_node.py と同じ理由）:
  A: mux_report_only=false（止める側）
  B: パラメータを渡さない（既定＝記録だけ）
入力（/cmd_vel_behavior 等）は一切流さない＝ずっと停止中。

  a. twist_mux が生きている間は、猶予・保持を十分に越えても A も B も MUX_DEAD を出さない
     （起動直後・停止中の誤検知がない）
  b. twist_mux を `kill -TERM` で落とす → A は MUX_DEAD（CRITICAL）を出し fault_lock が立つ
  c. 同じとき B は FaultStatus も fault_lock も出さず、ログに「記録だけ」が残る
     （ESTOP にならない）

`enabled_targets` は launch ファイルの値を使う: bringup.launch.py の SAFETY_ENABLED_TARGETS と
gazebo.launch.py の SAFETY_ENABLED_TARGETS_SIM／_REAL のすべてに `mux` が入っているときだけ
`mux` を有効にする。1 つでも抜けると監視が立たず（b）が赤になる。

注意（CLAUDE.md の既知の癖）: 試験は TestCase のメソッドにする。メソッドは名前順に走り、
前の状態を引き継ぐ（a → b → c の順。c は b の後でも成り立つ観測）。
twist_mux は `kill -TERM` で落とす（`-9` は DDS を壊す）。
"""
import ast
import os
import signal
import time
import unittest

import pytest
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import FaultStatus


CHECK_PERIOD_MS = 50
STARTUP_GRACE_SEC = 1
STARTUP_DEADLINE_SEC = 3
HOLD_MS = 200
LIVENESS_GRACE_MS = 500

_LAUNCH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', '..', 'th_bringup', 'launch')


def _targets_in(path: str, var: str):
    """launch ファイル中の `var = [...]` の文字列要素を AST で静的に読む。"""
    with open(os.path.join(_LAUNCH_DIR, path), encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=path)
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == var for t in node.targets)
                and isinstance(node.value, (ast.List, ast.Tuple))):
            return [el.value for el in node.value.elts if isinstance(el, ast.Constant)]
    raise AssertionError(f'{path}: {var} の代入が見つからない')


def _launch_enables_mux() -> bool:
    lists = [
        _targets_in('bringup.launch.py', 'SAFETY_ENABLED_TARGETS'),
        _targets_in('gazebo.launch.py', 'SAFETY_ENABLED_TARGETS_SIM'),
        _targets_in('gazebo.launch.py', 'SAFETY_ENABLED_TARGETS_REAL'),
    ]
    return all('mux' in t for t in lists)


def _safety_node(name: str, ns: str, report_only):
    params = {
        'check_period_ms': CHECK_PERIOD_MS,
        'startup_grace_sec': STARTUP_GRACE_SEC,
        'startup_deadline_sec': STARTUP_DEADLINE_SEC,
        'critical_fault_hold_ms': HOLD_MS,
        'mux_liveness_grace_ms': LIVENESS_GRACE_MS,
        # launch の値で mux が有効になっているときだけ有効化する（F-5・O-7）。
        'enabled_targets': ['mux'] if _launch_enables_mux() else ['person'],
    }
    if report_only is not None:
        params['mux_report_only'] = report_only
    return launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name=name,
        parameters=[params],
        remappings=[('/safety/fault', ns + '/fault'),
                    ('/safety/fault_lock', ns + '/fault_lock'),
                    ('/safety/estop', ns + '/estop')],
        output='screen',
    )


@pytest.mark.launch_test
def generate_test_description():
    twist_mux = launch_ros.actions.Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[{
            'topics': {
                'behavior': {'topic': '/cmd_vel_behavior', 'timeout': 0.5, 'priority': 20},
                'nav': {'topic': '/cmd_vel_nav', 'timeout': 0.5, 'priority': 10},
            },
            'locks': {
                'estop': {'topic': '/safety/estop', 'timeout': 0.5, 'priority': 255},
            },
        }],
        remappings=[('cmd_vel_out', '/cmd_vel_muxed')],
        output='screen',
    )
    node_a = _safety_node('safety_mux_a', '/test/mux_a', False)
    node_b = _safety_node('safety_mux_b', '/test/mux_b', None)
    return launch.LaunchDescription([
        twist_mux, node_a, node_b,
        launch_testing.actions.ReadyToTest(),
    ]), {'twist_mux': twist_mux, 'safety_mux_a': node_a, 'safety_mux_b': node_b}


def _twist_mux_pids():
    """この launch が起動した twist_mux のプロセス ID（/proc を走査する）。"""
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                argv = f.read().split(b'\0')
        except OSError:
            continue
        if argv and argv[0].endswith(b'/twist_mux') and b'__node:=twist_mux' in argv:
            pids.append(int(entry))
    return pids


class TestMuxLiveness(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_mux_liveness')
        n = cls.node
        cls.faults_a, cls.faults_b = [], []
        cls.lock_a, cls.lock_b = [], []
        n.create_subscription(FaultStatus, '/test/mux_a/fault', cls.faults_a.append, 10)
        n.create_subscription(FaultStatus, '/test/mux_b/fault', cls.faults_b.append, 10)
        n.create_subscription(Bool, '/test/mux_a/fault_lock',
                              lambda m: cls.lock_a.append(m.data), 10)
        n.create_subscription(Bool, '/test/mux_b/fault_lock',
                              lambda m: cls.lock_b.append(m.data), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _spin(cls, duration):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def _mux_faults(cls, faults):
        return [f for f in faults if f.fault_type == 'MUX_DEAD' and f.active]

    def test_a_alive_idle_twist_mux_is_not_dead(self):
        self.assertTrue(_launch_enables_mux(),
                        'bringup／gazebo の SAFETY_ENABLED_TARGETS のどれかに mux が無い')
        # 起動猶予 + 未発見の上限 + 保持 + 余裕を十分に越えて待つ。入力は流さない（停止中）。
        self._spin(STARTUP_GRACE_SEC + STARTUP_DEADLINE_SEC + 3.0)
        self.assertEqual(self._mux_faults(self.faults_a), [], '生きているのに A が MUX_DEAD')
        self.assertEqual(self._mux_faults(self.faults_b), [], '生きているのに B が MUX_DEAD')
        self.assertNotIn(True, self.lock_a, '生きているのに A の fault_lock が立った')
        self.assertNotIn(True, self.lock_b)

    def test_b_killed_twist_mux_is_detected_while_idle(self):
        pids = _twist_mux_pids()
        self.assertEqual(len(pids), 1, f'twist_mux のプロセスが 1 つでない: {pids}')
        os.kill(pids[0], signal.SIGTERM)   # -9 は DDS を壊すので使わない
        deadline = time.time() + 8.0
        while time.time() < deadline and not self._mux_faults(self.faults_a):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        found = self._mux_faults(self.faults_a)
        self.assertTrue(found, '停止中に twist_mux を落としても MUX_DEAD が出ない')
        self.assertEqual(found[0].severity, 'CRITICAL')
        self._spin(0.5)
        self.assertIn(True, self.lock_a, '重大フォルトなのに fault_lock が立たない')

    def test_c_default_is_record_only(self, proc_output):
        # b で落とした後。B（既定）は記録だけ: ログに残り、FaultStatus も fault_lock も無い。
        proc_output.assertWaitFor('[MUX_DEAD 記録だけ 1 件目]', timeout=10)
        self._spin(1.0)
        self.assertEqual(self._mux_faults(self.faults_b), [],
                         '既定（記録だけ）なのに MUX_DEAD の FaultStatus が出た')
        self.assertEqual(
            [f for f in self.faults_b if f.active], [],
            '既定（記録だけ）なのに FaultStatus が出た（ESTOP になる）')
        self.assertNotIn(True, self.lock_b, '既定（記録だけ）なのに fault_lock が立った')
