"""
test_stop_only_freshness_node.py
================================
1b-5（SG-A10）— 停止中だけのガードのうち、state_manager では作れない条件を
試験用の /system/state 発行者で縛る launch 試験（`slam_control`・
`config_manager` は本物。state_manager は起動しない）。

ここで縛るもの:
  1. /system/state 未受信では 3 経路とも拒否される（起動直後の安全側）
  2. 走行系（REPLAY/RUN）・PREP/RETURN・PREP/PAUSE では拒否される
  3. IDLE・PREP/MAPPING でもジョグ中は拒否される
  4. 受信後に途絶すると拒否に戻る（鮮度。state_stale_ms=1500 の既定。
     新鮮なうちは通り、2 秒超の沈黙で拒否される）

試験名に番号を付けているのは、発行者の状態が launch 全体で共有されるため
実行順（unittest は名前順）を固定する必要があるため。番号は順序の意味だけ。

変異チェック:
  ① stop_only_allows() を常に True → 01〜04 が赤
  ② _cb_state で _state_at を更新しない → 04 の前半（新鮮で通る）が赤
"""
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

from std_srvs.srv import SetBool, Trigger
from th_system_msgs.msg import SystemState
from th_system_msgs.srv import OpenMapSession, SetTunableParams

_STATE_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)

GUARD_WORD = '停止中（IDLE・PREP の地図作業中）のみ'


@pytest.mark.launch_test
def generate_test_description():
    slam_control = launch_ros.actions.Node(
        package='th_config_manager', executable='slam_control.py', name='slam_control',
        output='screen')
    config_manager = launch_ros.actions.Node(
        package='th_config_manager', executable='config_manager.py', name='config_manager',
        output='screen')
    return launch.LaunchDescription([
        slam_control, config_manager,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestStopOnlyFreshness(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_stop_only_freshness_client')
        # slam_toolbox の代役（起動時の _startup を速やかに通す。toggle の
        # 切り替え先としても成功を返す）。
        self.node.create_service(
            SetBool, '/slam_toolbox/set_localization_mode', self._localization_ok)
        self.pub_state = self.node.create_publisher(SystemState, '/system/state', _STATE_QOS)
        self.cli_toggle = self.node.create_client(Trigger, '/slam_control/toggle_mapping')
        self.cli_discard = self.node.create_client(Trigger, '/slam_control/discard_map')
        self.cli_open = self.node.create_client(OpenMapSession, '/map_session/open')
        self.cli_cfg = self.node.create_client(
            SetTunableParams, '/config_manager/set_tunable_params')
        assert self.cli_toggle.wait_for_service(timeout_sec=15.0), 'slam_control 未起動'
        assert self.cli_cfg.wait_for_service(timeout_sec=15.0), 'config_manager 未起動'

    @staticmethod
    def _localization_ok(req, resp):
        resp.success = True
        resp.message = ''
        return resp

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────
    def _spin(self, duration=0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _call(self, client, req, timeout=6.0):
        future = client.call_async(req)
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            self._spin(0.02)
        assert future.done(), 'サービス応答が返らない'
        return future.result()

    def _publish(self, mode, state='NONE', jog=False):
        msg = SystemState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = mode
        msg.state = state
        msg.jog_active = jog
        self.pub_state.publish(msg)
        self._spin(0.5)

    def _toggle_msg(self):
        return self._call(self.cli_toggle, Trigger.Request()).message

    def _discard_msg(self):
        return self._call(self.cli_discard, Trigger.Request()).message

    def _config_msg(self):
        req = SetTunableParams.Request()
        req.node_name = 'no_such_node_xyz'
        req.parameters = []
        return self._call(self.cli_cfg, req).message

    def _assert_denies(self, where):
        assert GUARD_WORD in self._toggle_msg(), f'{where}: toggle が通った'
        assert GUARD_WORD in self._discard_msg(), f'{where}: discard が通った'
        assert GUARD_WORD in self._config_msg(), f'{where}: config が通った'

    # ══════════════════════════════════════════════════════
    def test_01_never_received_denies(self):
        """/system/state 未受信（起動直後）では拒否される。"""
        self._spin(0.5)
        toggle = self._toggle_msg()
        assert GUARD_WORD in toggle, f'未受信で toggle が通った: {toggle}'
        assert '受信していない' in toggle, f'理由が未受信でない: {toggle}'
        config = self._config_msg()
        assert GUARD_WORD in config, f'未受信で config が通った: {config}'

    def test_02_driving_states_deny(self):
        """走行系・PREP/RETURN・PREP/PAUSE では拒否される（SD-9）。"""
        for mode, state in [('REPLAY', 'RUN'), ('PANEL_NAV', 'NAV'),
                            ('PREP', 'RETURN'), ('PREP', 'PAUSE'),
                            ('SUMMON', 'WAIT_CLEAR'), ('ESTOP', 'NONE')]:
            self._publish(mode, state)
            self._assert_denies(f'{mode}/{state}')

    def test_03_jog_denies(self):
        """IDLE・PREP/MAPPING でもジョグ中は拒否される。"""
        for mode, state in [('IDLE', 'NONE'), ('PREP', 'MAPPING')]:
            self._publish(mode, state, jog=True)
            toggle = self._toggle_msg()
            assert GUARD_WORD in toggle, f'{mode}/{state}+jog で通った: {toggle}'
            assert 'ジョグ中' in toggle, f'理由がジョグ中でない: {toggle}'

    def test_04_stale_denies_after_fresh_allows(self):
        """新鮮なうちは通り、途絶（2 秒超の沈黙）で拒否に戻る。"""
        self._publish('IDLE', 'NONE')
        toggle = self._call(self.cli_toggle, Trigger.Request())
        assert toggle.success is True and GUARD_WORD not in toggle.message, \
            f'新鮮な IDLE で拒否された: {toggle.message}'
        time.sleep(2.2)
        toggle = self._toggle_msg()
        assert GUARD_WORD in toggle, f'途絶後に通った: {toggle}'
        assert '古い' in toggle, f'理由が鮮度切れでない: {toggle}'
        config = self._config_msg()
        assert GUARD_WORD in config, f'途絶後に config が通った: {config}'
