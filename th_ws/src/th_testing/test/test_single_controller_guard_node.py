"""
test_single_controller_guard_node.py
====================================
Spec-safety §6.4「制御系は同時に 1 つ」: 起動時の拒否を、実 rclpy のグラフで縛る。

  a. 別の制御系がいないときは拒否しない（通る）
  b. 別プロセスで偽の `obstacle_limiter` 名のノードを立てた状態で拒否の関数を
     本当に呼び、中止（RuntimeError）されることを確かめる

rclpy が要るので Docker で回す（ホストの pytest では除外。CLAUDE.md の除外一覧）。
メディアスの時刻表ではなく、グラフの実状態で判定していることを縛るのが要点。
"""
import os
import subprocess
import sys
import time
import unittest

import rclpy

_REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_LAUNCH_DIR = os.path.join(_REPO_SRC, "th_bringup", "launch")
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

import single_controller_guard as scg  # noqa: E402

_FAKE_LIMITER = """
import signal, sys
def _h(signum, frame):
    sys.exit(0)
signal.signal(signal.SIGTERM, _h)
import rclpy
rclpy.init()
_node = rclpy.create_node('obstacle_limiter')
while rclpy.ok():
    rclpy.spin_once(_node, timeout_sec=0.1)
"""


class TestSingleControllerGuard(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_single_controller_guard')
        cls.fake = None

    @classmethod
    def tearDownClass(cls):
        if cls.fake is not None:
            cls.fake.terminate()
            cls.fake.wait(timeout=10)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_node_visible(self, name, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if name in self.node.get_node_names():
                return True
        return False

    def test_a_no_other_controller_passes(self):
        # 偽の obstacle_limiter はまだ立てていない（test_b が立てる）。
        lines = scg.run_startup_check(watch_sec=0.5)
        self.assertEqual(lines, [], '別の制御系が居ないのに起動を拒否した')

    def test_b_fake_obstacle_limiter_aborts_startup(self):
        fake = subprocess.Popen(
            [sys.executable, '-c', _FAKE_LIMITER],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        type(self).fake = fake
        try:
            self.assertTrue(
                self._wait_node_visible('obstacle_limiter'),
                '偽の obstacle_limiter がグラフに現れない')
            with self.assertRaises(RuntimeError) as ctx:
                scg.run_startup_check(watch_sec=0.5)
            msg = str(ctx.exception)
            self.assertIn('別の PC の制御系が動いています', msg)
            self.assertIn('obstacle_limiter', msg)
        finally:
            fake.terminate()
            fake.wait(timeout=10)
            type(self).fake = None
