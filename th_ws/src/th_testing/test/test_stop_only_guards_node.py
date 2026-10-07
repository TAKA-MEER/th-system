"""
test_stop_only_guards_node.py
============================
1b-5（SG-A10・SG-A6）— 停止中だけのガードを **本番の経路で通す** launch 試験。

`state_manager`（本物）と `slam_control`・`config_manager`（本物）を同時に起動し、
画面の代わりにテストが `/system/trigger`（ui.*）を叩いてモードを動かす。

ここで縛るもの（ソースの文字列検査ではなく、実際に動かして見る）:
  1. IDLE では地図操作・設定値変更が通る（ガード通過の証拠は下流の応答。
     代役の set_localization が成功を返すため toggle は success=True、
     slam_toolbox プロセスが居ないため discard は「プロセスが見つかりません」、
     存在しない地図の VENUE reload は「地図ファイルが無い」、
     未知ノードへの set は「未知のノード」。いずれもガード文言ではない）
  2. PREP/MAPPING（PREP の止まっている状態）でも通る
     （PREP の discard_map が壊れたガードのおかげで動いていた件。付け替え後も動く）
  3. PREP/RETURN（自律走行）・MANUAL（手動走行）では 4 経路とも拒否される
     （応答に「停止中（IDLE・PREP の地図作業中）のみ」が入る）

走行系モード（REPLAY 等）・/system/state 途絶・ジョグ中の拒否は、試験用の
発行者で /system/state を出す test_stop_only_freshness_node.py で縛る
（state_manager が居ると途絶を作れない・走行系に入る前提が重いため）。

変異チェック（実装管理担当 or 実装者が一時的に壊して実行。一覧は報告に記載）:
  ① stop_only_allows() を常に True → 拒否系の 2 件が赤
  ② slam_control の /system/state 購読を /robot/mode に戻す → 全件が赤
     （/robot/mode を出す者が居ないため未受信で拒否される）
"""
import json
import os
import shutil
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

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from th_system_msgs.msg import FaultStatus, Pin, PinList, SystemState
from th_system_msgs.srv import OpenMapSession, SetTunableParams, UiTrigger

MAP_DIR = tempfile.mkdtemp(prefix='stop_only_maps_')
VENUE_DIR = tempfile.mkdtemp(prefix='stop_only_venue_')

_STATE_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)

GUARD_WORD = '停止中（IDLE・PREP の地図作業中）のみ'


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state', executable='state_manager.py', name='state_manager',
        parameters=[{'link_wait_timeout_ms': 600_000, 'screen_stale_ms': 600_000,
                     'ui_active_window_s': 5}],
        output='screen')
    slam_control = launch_ros.actions.Node(
        package='th_config_manager', executable='slam_control.py', name='slam_control',
        parameters=[{'map_dir': MAP_DIR, 'venue_map_dir': VENUE_DIR}],
        output='screen')
    config_manager = launch_ros.actions.Node(
        package='th_config_manager', executable='config_manager.py', name='config_manager',
        output='screen')
    return launch.LaunchDescription([
        state_manager, slam_control, config_manager,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestStopOnlyGuards(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        shutil.rmtree(MAP_DIR, ignore_errors=True)
        shutil.rmtree(VENUE_DIR, ignore_errors=True)

    def setUp(self):
        self.node = rclpy.create_node('test_stop_only_guards_client')
        # slam_toolbox の代役。無いと slam_control が起動時の _startup で
        # サービス出現を最大 60 秒待ってサービス応答が遅れる。toggle の
        # 切り替え先としても成功を返す（ガード通過の証拠が success=True になる）。
        self.srv_localization = self.node.create_service(
            SetBool, '/slam_toolbox/set_localization_mode',
            lambda req, resp: self._localization_ok(req, resp))
        self._state_history = []
        self.node.create_subscription(
            SystemState, '/system/state', self._state_history.append, _STATE_QOS)
        self.pub_fault = self.node.create_publisher(
            FaultStatus, '/safety/fault', 5)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_ui_estop = self.node.create_publisher(Bool, '/safety/estop_ui', 10)
        self.pub_pins = self.node.create_publisher(PinList, '/onsite/pins', _STATE_QOS)

        self._setup_services()
        self._publish_home_pin()
        self._reset_to_idle()
        self._state_history.clear()

    @staticmethod
    def _localization_ok(req, resp):
        resp.success = True
        resp.message = ''
        return resp

    def _setup_services(self):
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        self.cli_toggle = self.node.create_client(Trigger, '/slam_control/toggle_mapping')
        self.cli_discard = self.node.create_client(Trigger, '/slam_control/discard_map')
        self.cli_open = self.node.create_client(OpenMapSession, '/map_session/open')
        self.cli_cfg = self.node.create_client(
            SetTunableParams, '/config_manager/set_tunable_params')
        assert self.cli_trigger.wait_for_service(timeout_sec=15.0), 'state_manager 未起動'
        assert self.cli_toggle.wait_for_service(timeout_sec=15.0), 'slam_control 未起動'
        assert self.cli_cfg.wait_for_service(timeout_sec=15.0), 'config_manager 未起動'

    def tearDown(self):
        self._reset_to_idle()
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

    def _trigger(self, trigger, arg=None):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = 'test'
        return self._call(self.cli_trigger, req)

    def _mode_state(self):
        self._spin(0.2)
        if not self._state_history:
            return None
        last = self._state_history[-1]
        return (last.mode, last.state)

    def _wait_mode_state(self, mode, state, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._mode_state() == (mode, state):
                return True
            self._spin(0.1)
        return False

    def _publish_home_pin(self):
        """ui.return_home の前提（home_pin_exists）を満たす HOME ピン。"""
        from geometry_msgs.msg import Pose
        msg = PinList()
        pin = Pin()
        pin.id = 'home'
        pin.name = 'home'
        pin.kind = 'HOME'
        pin.pose = Pose()
        msg.pins = [pin]
        self.pub_pins.publish(msg)
        self._spin(0.3)

    def _reset_to_idle(self):
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.1)
        self.pub_hw.publish(Bool(data=True))
        self._spin(0.1)
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.1)
        self.pub_ui_estop.publish(Bool(data=False))
        self._spin(0.1)
        self._trigger('ui.finish')
        if not self._wait_mode_state('IDLE', 'NONE', timeout=5.0):
            self.fail(f'IDLE に戻らない ({self._mode_state()})')

    # ── 測定（ガード通過／拒否の証拠） ─────────────────────
    def _toggle(self):
        return self._call(self.cli_toggle, Trigger.Request())

    def _discard(self):
        return self._call(self.cli_discard, Trigger.Request())

    def _venue_reload_missing(self):
        req = OpenMapSession.Request()
        req.slot = 'VENUE'
        req.session_id = 'no_such_venue_map_xyz'
        req.mode = 'reload'
        req.has_initial_pose = False
        return self._call(self.cli_open, req)

    def _config_unknown_node(self):
        req = SetTunableParams.Request()
        req.node_name = 'no_such_node_xyz'
        req.parameters = []
        return self._call(self.cli_cfg, req)

    def _assert_allows(self, where):
        """4 経路ともガードを通過する（下流の応答が返る。ガード文言は無い）。"""
        r = self._toggle()
        assert r.success is True and GUARD_WORD not in r.message, \
            f'{where}: toggle がガードで拒否された: {r.message}'
        r = self._discard()
        assert r.success is False and GUARD_WORD not in r.message, \
            f'{where}: discard がガードで拒否された: {r.message}'
        assert 'プロセスが見つかりません' in r.message, \
            f'{where}: プロセス不在の応答でない: {r.message}'
        r = self._venue_reload_missing()
        assert r.success is False and GUARD_WORD not in r.message, \
            f'{where}: VENUE reload がガードで拒否された: {r.message}'
        assert '地図ファイルが無い' in r.message, \
            f'{where}: ファイル不在の応答でない: {r.message}'
        r = self._config_unknown_node()
        assert r.success is False and GUARD_WORD not in r.message, \
            f'{where}: config がガードで拒否された: {r.message}'
        assert '未知のノード' in r.message, \
            f'{where}: 未知ノードの応答でない: {r.message}'

    def _assert_denies(self, where):
        """4 経路ともガードで拒否される（停止中の文言）。"""
        r = self._toggle()
        assert r.success is False and GUARD_WORD in r.message, \
            f'{where}: toggle が通った: {r.message}'
        r = self._discard()
        assert r.success is False and GUARD_WORD in r.message, \
            f'{where}: discard が通った: {r.message}'
        r = self._venue_reload_missing()
        assert r.success is False and GUARD_WORD in r.message, \
            f'{where}: VENUE reload が通った: {r.message}'
        r = self._config_unknown_node()
        assert r.success is False and GUARD_WORD in r.message, \
            f'{where}: config が通った: {r.message}'

    # ══════════════════════════════════════════════════════
    def test_idle_allows(self):
        """IDLE では地図操作・設定値変更が通る（ガード通過）。"""
        assert self._mode_state() == ('IDLE', 'NONE')
        self._assert_allows('IDLE')

    def test_manual_denies(self):
        """MANUAL（手動走行）では 4 経路とも拒否される（SD-9）。"""
        res = self._trigger('ui.enter_mode', {'mode': 'MANUAL'})
        assert res.accepted, f'MANUAL に入らない: {res.reason}'
        # MANUAL の入口状態は RUN（state_core.ESTOP_RESUME_RUN の run_state）。
        assert self._wait_mode_state('MANUAL', 'RUN', timeout=5.0),             f'MANUAL/RUN に入らない ({self._mode_state()})'
        self._assert_denies('MANUAL')

    def test_prep_mapping_allows(self):
        """PREP/MAPPING（止まっている状態）では通る。PREP の地図破棄が
        付け替え後も動くことの担保（SG-A10 の注意書き）。"""
        res = self._trigger('ui.enter_mode', {'mode': 'PREP'})
        assert res.accepted, f'PREP に入らない: {res.reason}'
        assert self._wait_mode_state('PREP', 'MAPPING', timeout=10.0), \
            f'PREP/MAPPING に入らない ({self._mode_state()})'
        self._assert_allows('PREP/MAPPING')

    def test_prep_return_denies(self):
        """PREP/RETURN（自律走行）では 4 経路とも拒否される（SD-9）。"""
        res = self._trigger('ui.enter_mode', {'mode': 'PREP'})
        assert res.accepted, f'PREP に入らない: {res.reason}'
        assert self._wait_mode_state('PREP', 'MAPPING', timeout=10.0)
        for _ in range(5):
            self._trigger('ui.return_home')
            if self._wait_mode_state('PREP', 'RETURN', timeout=3.0):
                break
        assert self._mode_state() == ('PREP', 'RETURN'), \
            f'PREP/RETURN に入らない ({self._mode_state()})'
        self._assert_denies('PREP/RETURN')
