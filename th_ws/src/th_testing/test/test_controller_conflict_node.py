"""
test_controller_conflict_node.py
================================
Spec-safety §6.4: 動作中、別の PC の制御系が現れたら重大フォルトとして止める。
ここでは `safety_monitor` 本体を実際に起動し、`/cmd_vel` に 2 つ目の publisher
を出したときの振る舞いを縛る（Docker launch テスト）。

2 台の safety_monitor を立てる（report_only は起動時パラメータで、1 台では
両方を縛れない。test_mux_liveness_node.py と同じ理由）:
  A: controller_conflict_report_only=false（止める側）
  B: パラメータを渡さない（既定＝記録だけ）

  a. /cmd_vel の publisher が 1 つだけの間は、猶予・保持を十分に越えても
     A も B も CONTROLLER_CONFLICT を出さない（起動直後・停止中の誤検知がない）
  b. 2 つ目の /cmd_vel publisher を出す → A は CONTROLLER_CONFLICT（CRITICAL）を出し
     fault_lock が立つ
  c. 同じとき B は FaultStatus も fault_lock も出さず、ログに「記録だけ」が残る

`enabled_targets` は launch ファイルの値を使う: bringup.launch.py の
SAFETY_ENABLED_TARGETS と gazebo.launch.py の SAFETY_ENABLED_TARGETS_SIM／_REAL の
すべてに `controller` が入っているときだけ `controller` を有効にする。
1 つでも抜けると監視が立たず (b) が赤になる。

注意（CLAUDE.md の既知の癖）: 試験は TestCase のメソッドにする。メソッドは名前順に走り、
前の状態を引き継ぐ（a → b → c の順。c は b の後でも成り立つ観測）。
2 つ目の publisher は試験内から subprocess で立てる（launch の宣言ではない）。
"""
import ast
import os
import signal
import subprocess
import sys
import time
import unittest

import pytest
import rclpy
from std_msgs.msg import Bool

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import FaultStatus


CHECK_PERIOD_MS = 50
STARTUP_GRACE_SEC = 1
STARTUP_DEADLINE_SEC = 3
HOLD_MS = 500

_LAUNCH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', '..', 'th_bringup', 'launch')

_FAKE_CMD_VEL_PUB = """
import signal, sys
def _h(signum, frame):
    sys.exit(0)
signal.signal(signal.SIGTERM, _h)
import rclpy
from geometry_msgs.msg import Twist
rclpy.init()
_node = rclpy.create_node('fake_cmd_vel_pub')
_pub = _node.create_publisher(Twist, '/cmd_vel', 10)
while rclpy.ok():
    rclpy.spin_once(_node, timeout_sec=0.1)
"""


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


def _launch_enables_controller() -> bool:
    lists = [
        _targets_in('bringup.launch.py', 'SAFETY_ENABLED_TARGETS'),
        _targets_in('gazebo.launch.py', 'SAFETY_ENABLED_TARGETS_SIM'),
        _targets_in('gazebo.launch.py', 'SAFETY_ENABLED_TARGETS_REAL'),
    ]
    return all('controller' in t for t in lists)


def _safety_node(name: str, ns: str, report_only):
    params = {
        'check_period_ms': CHECK_PERIOD_MS,
        'startup_grace_sec': STARTUP_GRACE_SEC,
        'startup_deadline_sec': STARTUP_DEADLINE_SEC,
        'controller_conflict_hold_ms': HOLD_MS,
        # launch の値で controller が有効になっているときだけ有効化する（F-5・O-7）。
        'enabled_targets': ['controller'] if _launch_enables_controller() else ['person'],
    }
    if report_only is not None:
        params['controller_conflict_report_only'] = report_only
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
    node_a = _safety_node('safety_controller_a', '/test/controller_a', False)
    node_b = _safety_node('safety_controller_b', '/test/controller_b', None)
    return launch.LaunchDescription([
        node_a, node_b,
        launch_testing.actions.ReadyToTest(),
    ]), {'safety_controller_a': node_a, 'safety_controller_b': node_b}


class TestControllerConflict(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_controller_conflict')
        n = cls.node
        cls.faults_a, cls.faults_b = [], []
        cls.lock_a, cls.lock_b = [], []
        n.create_subscription(FaultStatus, '/test/controller_a/fault', cls.faults_a.append, 10)
        n.create_subscription(FaultStatus, '/test/controller_b/fault', cls.faults_b.append, 10)
        n.create_subscription(Bool, '/test/controller_a/fault_lock',
                              lambda m: cls.lock_a.append(m.data), 10)
        n.create_subscription(Bool, '/test/controller_b/fault_lock',
                              lambda m: cls.lock_b.append(m.data), 10)
        cls.extra_pub = None

    @classmethod
    def tearDownClass(cls):
        if cls.extra_pub is not None:
            cls.extra_pub.terminate()
            cls.extra_pub.wait(timeout=10)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _spin(cls, duration):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def _conflict_faults(cls, faults):
        return [f for f in faults if f.fault_type == 'CONTROLLER_CONFLICT' and f.active]

    def test_a_single_cmd_vel_publisher_is_not_a_conflict(self):
        self.assertTrue(_launch_enables_controller(),
                        'bringup／gazebo の SAFETY_ENABLED_TARGETS のどれかに controller が無い')
        # /cmd_vel の publisher は 0（試験は obstacle_limiter を起こさない）:
        # 猶予・保持を十分に越えても出ないことを確かめる。
        self._spin(STARTUP_GRACE_SEC + STARTUP_DEADLINE_SEC + 3.0)
        self.assertEqual(self._conflict_faults(self.faults_a), [],
                         'publisher が居ないのに A が CONTROLLER_CONFLICT')
        self.assertEqual(self._conflict_faults(self.faults_b), [],
                         'publisher が居ないのに B が CONTROLLER_CONFLICT')
        self.assertNotIn(True, self.lock_a)
        self.assertNotIn(True, self.lock_b)

    def test_b_second_cmd_vel_publisher_is_detected(self):
        # 2 つ目の /cmd_vel publisher を出す（1 つ目は... 本試験は obstacle_limiter を
        # 起こさないので、この 1 つで publisher 数は 1 になる。では足りないので、
        # /cmd_vel に publisher を 2 つ出す）。
        procs = []
        for _ in range(2):
            procs.append(subprocess.Popen(
                [sys.executable, '-c', _FAKE_CMD_VEL_PUB],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        type(self).extra_pub = procs[0]
        try:
            deadline = time.time() + 15.0
            while time.time() < deadline and not self._conflict_faults(self.faults_a):
                rclpy.spin_once(self.node, timeout_sec=0.05)
            found = self._conflict_faults(self.faults_a)
            self.assertTrue(found, '/cmd_vel に publisher を 2 つ出しても CONTROLLER_CONFLICT が出ない')
            self.assertEqual(found[0].severity, 'CRITICAL')
            self._spin(0.5)
            self.assertIn(True, self.lock_a, '重大フォルトなのに fault_lock が立たない')
        finally:
            for p in procs:
                p.terminate()
            for p in procs:
                p.wait(timeout=10)
            type(self).extra_pub = None

    def test_c_default_is_record_only(self, proc_output):
        # b の後。B（既定）は記録だけ: ログに残り、FaultStatus も fault_lock も無い。
        # b で publisher を落としたあとでも記録だけのログは残っている。
        proc_output.assertWaitFor('[CONTROLLER_CONFLICT 記録だけ 1 件目]', timeout=10)
        self._spin(1.0)
        self.assertEqual(self._conflict_faults(self.faults_b), [],
                         '既定（記録だけ）なのに CONTROLLER_CONFLICT の FaultStatus が出た')
        self.assertEqual(
            [f for f in self.faults_b if f.active], [],
            '既定（記録だけ）なのに FaultStatus が出た（ESTOP になる）')
        self.assertNotIn(True, self.lock_b, '既定（記録だけ）なのに fault_lock が立った')
