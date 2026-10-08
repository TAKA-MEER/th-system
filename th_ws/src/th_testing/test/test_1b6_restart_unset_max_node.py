"""
test_1b6_restart_unset_max_node.py
==================================
1b-6 SG-B7 — 本番の設定で疎通確認の時間切れが制御系の再起動に届く。

`restart_max_count` は registry で placeholder（O-d4）のため生成 yaml に載らず、
本番では None になる。前任の実装ではこのとき「上限に達した」とみなして一度も
再起動しなかった（試験が dict で 2 に上書きしていたため緑だった）。

ここでは生成 yaml だけを渡し（restart_max_count を上書きしない）、start.sh の下
（control_attempt:=1）で動いている場合に限り再起動が撃たれることを見る。
kill は restart_kill_log の偽物に差し替える（本物を撃つと launch_testing ごと死ぬ）。
start.sh の下でない場合（control_attempt:=0）に撃たない側は connectivity_core の
restart_allowed の単体試験で縛る。

変異: restart_allowed の None の扱いを「中止」に戻す → 赤。
"""
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

GENERATED_DIR = '/root/th_data/generated'
_TMP_DIR = tempfile.mkdtemp(prefix='th_test_1b6u_')
_KILL_LOG = os.path.join(_TMP_DIR, 'restart_kill.log')


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state', executable='state_manager.py', name='state_manager',
        parameters=[os.path.join(GENERATED_DIR, 'state_manager.yaml'),
                    {'link_wait_timeout_ms': 1500}],
        output='screen')
    connectivity_checker = launch_ros.actions.Node(
        package='th_state', executable='connectivity_checker.py',
        name='connectivity_checker',
        parameters=[os.path.join(GENERATED_DIR, 'connectivity_checker.yaml'),
                    {'sim': True, 'restart_kill_log': _KILL_LOG,
                     'control_attempt': '1'}],
        output='screen')
    return launch.LaunchDescription([
        state_manager, connectivity_checker, launch_testing.actions.ReadyToTest(),
    ]), {}


class TestRestartUnsetMax(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_timeout_restarts_under_start_sh_with_unset_max(self):
        deadline = time.time() + 70.0
        lines = []
        while time.time() < deadline:
            try:
                with open(_KILL_LOG, encoding='utf-8') as f:
                    lines = [ln for ln in f.read().splitlines() if ln.strip()]
            except OSError:
                lines = []
            if lines:
                break
            time.sleep(0.5)
        self.assertEqual(len(lines), 1, lines)
