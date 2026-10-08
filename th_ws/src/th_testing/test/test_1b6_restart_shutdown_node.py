"""
test_1b6_restart_shutdown_node.py
====================================
1b-6（SG-B6・SG-B7）— connectivity_checker と state_manager を同時起動する試験。

SG-B7: state_manager の sys.link_timeout → T-INIT-02 の restart_control_stack
effect → connectivity_checker が restart_control_stack() を呼ぶ（本番の経路）。
SIGTERM は restart_dryrun で撃たず、回数だけ数える（本物はプロセスグループへ
SIGTERM を送るため試験で撃つと launch_testing・colcon ごと死ぬ）。
観測点は /system/link_status の restart.calls / restart.attempt。

SG-B6: 未保存がある間は /shutdown/execute が拒否し、無くなれば印ファイルを
置いて成功する（shutdown_dryrun で SIGTERM は撃たない）。未保存の種は
/route/status（route）・/map_session/status（venue_map）。

生成 yaml（data/generated/）を先に渡してから dict で上書きする
（rclpy は INTEGER↔DOUBLE を変換しない。brief-common の決まり）。

注意（launch_testing の挙動）: このファイルは @pytest.mark.launch_test を持つ
ため、その中の unittest.TestCase だけを実行する。モジュール直下の素の pytest
関数は収集されないので、全部メソッドにする。メソッド名の abc_ 接頭辞は
実行順（ alphabetical ）の制御用：上限（max 2 回）の試験は呼び出し回数の
積み上げが前提なので順序を固定する。
"""
import json
import os
import tempfile
import time
import unittest

import pytest
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from std_msgs.msg import String
from std_srvs.srv import Trigger
from th_system_msgs.msg import MapSessionStatus, RouteStatus, StateEffect


GENERATED_DIR = '/root/th_data/generated'

# launch 時に決まる印ファイルの置き場（generate_test_description と TestCase で
# 共有。コンテナ内の使い捨てパス。試験後に消す）。
_TMP_DIR = tempfile.mkdtemp(prefix='th_test_1b6_')
_MARKER_PATH = os.path.join(_TMP_DIR, 'control_stop')

LINK_WAIT_TIMEOUT_MS = 1500  # T-INIT-02 を試験中に発火させる（本番は 10000）
RESTART_MAX_COUNT = 2        # 上限の試験用（timeout 由来＋直接 effect で使い切る）
CONTROL_ATTEMPT = '3'        # S-00 表示用（start.sh が付ける回数の代役）


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[
            os.path.join(GENERATED_DIR, 'state_manager.yaml'),
            {
                'link_wait_timeout_ms': LINK_WAIT_TIMEOUT_MS,
                'shutdown_marker_path': _MARKER_PATH,
                'shutdown_dryrun': True,
            },
        ],
        output='screen',
    )
    connectivity_checker = launch_ros.actions.Node(
        package='th_state',
        executable='connectivity_checker.py',
        name='connectivity_checker',
        parameters=[
            os.path.join(GENERATED_DIR, 'connectivity_checker.yaml'),
            {
                'esp32_alive_timeout_ms': 3000,
                'scan_expected_points': 8,
                'required_nodes': ['unused_in_sim'],  # sim=True のため判定に使わない
                'restart_max_count': RESTART_MAX_COUNT,
                'restart_wait_ms': 5000,
                'restart_dryrun': True,
                'control_attempt': CONTROL_ATTEMPT,
                'sim': True,
            },
        ],
        output='screen',
    )
    return launch.LaunchDescription([
        state_manager,
        connectivity_checker,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_manager,
         'connectivity_checker': connectivity_checker}


_LINK_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)

# /map_session/status の発行側（slam_control）と同じ QoS。VOLATILE で出すと
# TRANSIENT_LOCAL の購読と不整合で届かない。
_MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


class Test1b6RestartShutdownNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_1b6_restart_shutdown_client')

        self._link_payloads = []
        self.node.create_subscription(
            String, '/system/link_status', self._link_payloads.append, 10)
        self.pub_effect = self.node.create_publisher(StateEffect, '/system/effect', 10)
        # /route/status は VOLATILE（state_manager の購読と同じ）。/scan は
        # 出さない（gate を偽のままにして INIT に留め、T-INIT-02 を発火させる）。
        self.pub_route = self.node.create_publisher(RouteStatus, '/route/status', 10)
        self.pub_map = self.node.create_publisher(
            MapSessionStatus, '/map_session/status', _MAP_QOS)
        self.cli_prepare = self.node.create_client(Trigger, '/shutdown/prepare')
        self.cli_execute = self.node.create_client(Trigger, '/shutdown/execute')
        self._spin(1.0)

    def tearDown(self):
        self.node.destroy_node()
        if os.path.exists(_MARKER_PATH):
            os.remove(_MARKER_PATH)

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _restart_calls(self):
        if not self._link_payloads:
            return None
        try:
            payload = json.loads(self._link_payloads[-1].data)
        except (ValueError, TypeError):
            return None
        return (payload.get('restart') or {}).get('calls')

    def _restart_attempt(self):
        if not self._link_payloads:
            return None
        try:
            payload = json.loads(self._link_payloads[-1].data)
        except (ValueError, TypeError):
            return None
        return (payload.get('restart') or {}).get('attempt')

    def _wait_calls(self, want, timeout=25.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            calls = self._restart_calls()
            if calls is not None and calls >= want:
                return calls
            self._spin(0.5)
        return self._restart_calls()

    def _send_effect(self, name, dest):
        msg = StateEffect()
        msg.name = name
        msg.dest = dest
        msg.args_json = '{}'
        # 購読のマッチング前に消えることがあるため数回出す（本番は latched ではない）。
        for _ in range(3):
            self.pub_effect.publish(msg)
            self._spin(0.2)

    def _call(self, cli, timeout=10.0):
        fut = cli.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if fut.done():
                return fut.result()
        self.fail('サービス応答が来ない')

    def _publish_route(self, points, saved, route_id=''):
        msg = RouteStatus()
        msg.points = points
        msg.saved = saved
        msg.current.id = route_id
        for _ in range(3):
            self.pub_route.publish(msg)
            self._spin(0.2)

    def _publish_map(self, slot, dirty):
        msg = MapSessionStatus()
        msg.slot = slot
        msg.dirty = dirty
        for _ in range(3):
            self.pub_map.publish(msg)
            self._spin(0.2)

    # ── SG-B7 ─────────────────────────────────────────────────
    def test_a_link_timeout_restarts(self):
        # 本番の経路: sys.link_timeout → T-INIT-02 の effect → restart が 1 回呼ばれる。
        # /scan を出さないので INIT に留まり、T-INIT-02 が発火する。
        calls = self._wait_calls(1)
        self.assertIsNotNone(calls)
        self.assertGreaterEqual(calls, 1)
        # 購読 1 行（_on_effect）を消す変異では calls が進まずここで赤になる。
        self.assertEqual(self._restart_attempt(), int(CONTROL_ATTEMPT))

    def test_b_direct_effect_restarts(self):
        # 自分宛ての effect でもう 1 回（上限 2 回目）。dest の判定を外す変異では
        # test_d が赤になる（ここでは正しい dest で呼ぶ）。
        self._send_effect('restart_control_stack', 'connectivity_checker')
        calls = self._wait_calls(2)
        self.assertEqual(calls, 2)

    def test_c_over_limit_not_called(self):
        # 上限（restart_max_count=2）に達したら呼ばれない。上限の比較を外す変異では
        # calls が 3 になって赤くなる。
        self._send_effect('restart_control_stack', 'connectivity_checker')
        self._spin(3.0)
        self.assertEqual(self._restart_calls(), 2)

    def test_d_wrong_dest_ignored(self):
        # 別宛ての effect では呼ばれない（dest 判定）。上限いっぱいなので calls は
        # 動かないことが正しい。購読自体の有無は test_a が縛る。
        before = self._restart_calls()
        self._send_effect('restart_control_stack', 'WebUI')
        self._spin(3.0)
        self.assertEqual(self._restart_calls(), before)

    # ── SG-B6 ─────────────────────────────────────────────────
    def test_e_route_unsaved_blocks_execute(self):
        # 未保存の教示記録がある間は execute が拒否される。
        self._publish_route(points=5, saved=False)
        res = self._call(self.cli_prepare)
        self.assertTrue(res.success)
        self.assertIn('route', json.loads(res.message))
        res = self._call(self.cli_execute)
        self.assertFalse(res.success)
        self.assertEqual(res.message, 'unsaved_remains')
        self.assertFalse(os.path.exists(_MARKER_PATH))
        # 未保存の判定を常に空にする変異では prepare が [] になって赤になる。

    def test_f_saved_route_executes_with_marker(self):
        # 保存済みになれば execute が通り、印ファイルを置く（SIGTERM は dryrun）。
        self._publish_route(points=5, saved=True)
        res = self._call(self.cli_execute)
        self.assertTrue(res.success, res.message)
        self.assertTrue(os.path.exists(_MARKER_PATH))

    def test_g_venue_dirty_blocks_execute(self):
        # /map_session/status の VENUE dirty は venue_map として検出する。
        self._publish_map(slot='VENUE', dirty=True)
        res = self._call(self.cli_prepare)
        self.assertTrue(res.success)
        self.assertIn('venue_map', json.loads(res.message))
        res = self._call(self.cli_execute)
        self.assertFalse(res.success)
        # 後始末（次の試験に持ち越さない）。
        self._publish_map(slot='VENUE', dirty=False)
        res = self._call(self.cli_prepare)
        self.assertNotIn('venue_map', json.loads(res.message))
