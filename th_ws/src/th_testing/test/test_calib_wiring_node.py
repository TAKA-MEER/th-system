"""
test_calib_wiring_node.py
=========================
WP-MAINT-02 — 校正（CALIB）を **本番の経路で通す** launch 試験。

`state_manager`（本物）と `calib_runner`（本物）を同時に起動し、画面の代わりにテストが
`/system/trigger`（ui.*）と `/calib/submit` を叩く。`esp32_bridge` はパラメータを受ける
代役ノード（`wheel_radius_scale` / `wheel_base`）、ロボット本体は `/cmd_vel_behavior` を積分して
`/odom` を返す物理モデル（真の補正値と現在の補正値の差で、止まる位置が実際にずれる）。

ここで縛るもの（ソースの文字列検査ではなく、実際に動かして見る）:
  1. 直進の通し: 走る → 実測を入れる → プレビュー → 適用 → 検証走行 → 確定。current.yaml が
     書かれ、esp32_bridge の wheel_radius_scale が真の値へ収束する
  2. 検証 NG で**適用前に戻る**（esp32_bridge の値が元に戻り、確定されない。T-CAL-06）
  3. **A10 を超える値は適用しない**（プレビューが sane にならず S4 へ進めない・bridge に届かない）
  4. 中断（abort）で走行が即座に止まり、適用済みなら戻り、確定されない
  5. 校正中のフォルト・非常停止で確定しない（§7 #3）
  6. CALIB 以外では動かない（effect・サービスを直接叩いても走らない）
  7. state_manager ↔ calib_runner の配線（effect 配送・evt 返送・preview_sane の受け渡し）

変異チェック（実装管理担当 or 実装者が一時的に壊して実行。一覧は報告に記載）:
  ① calib_runner._fail_verify の _revert_runtime() を消す → test_verify_ng_reverts_to_previous が赤
  ② calib_runner._in_calib() を True 固定 → test_effects_and_services_ignored_outside_calib が赤
  ③ calib_runner._discard() を何もしない → test_abort_stops_motion_and_reverts が赤
  ④ state_manager の _on_calib_status() を何もしない → test_linear_full_flow_commits が赤
  ⑤ calib_core.a10_ok() を常に True → test_a10_violation_is_never_applied が赤
"""
import json
import math
import os
import shutil
import tempfile
import time
import unittest

import pytest
import rclpy
import yaml
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from th_system_msgs.msg import (CalibStatus, FaultStatus, StateEffect, StateEvent,
                                SystemState)
from th_system_msgs.srv import RollbackCalib, StartCalib, SubmitCalib, UiTrigger

CALIB_DIR = tempfile.mkdtemp(prefix='calib_wiring_')

DIST_M = 0.30
ROT_DEG = 90.0
V_CALIB = 0.30
LIN_TOL = 0.05
ROT_TOL = 5.0

_STATE_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST)


@pytest.mark.launch_test
def generate_test_description():
    state_manager = launch_ros.actions.Node(
        package='th_state', executable='state_manager.py', name='state_manager',
        parameters=[{'link_wait_timeout_ms': 600_000, 'screen_stale_ms': 600_000,
                     'ui_active_window_s': 5}],
        output='screen')
    calib_runner = launch_ros.actions.Node(
        package='th_maintenance', executable='calib_runner.py', name='calib_runner',
        parameters=[{
            'calib_dir': CALIB_DIR,
            'params_digest_path': os.path.join(CALIB_DIR, 'no_such_digest.json'),
            'calib_linear_distance_m': DIST_M,
            'calib_rotation_deg': ROT_DEG,
            'v_calib': V_CALIB,
            'calib_linear_tolerance_ratio': LIN_TOL,
            'calib_rotation_tolerance_deg': ROT_TOL,
            'calib_run_timeout_s': 20.0,
        }],
        output='screen')
    return launch.LaunchDescription([
        state_manager, calib_runner, launch_testing.actions.ReadyToTest(),
    ]), {}


class _Sim:
    """ロボットの物理。補正値（bridge の現在値）と真の値の差で止まる位置がずれる。"""

    def __init__(self):
        self.k_true = 0.95        # 真の wheel_radius_scale
        self.base_true = 0.38     # 真の wheel_base
        self.reset()

    def reset(self):
        self.odom_x = 0.0
        self.odom_yaw = 0.0
        self.phys_dist = 0.0
        self.phys_yaw = 0.0


class TestCalibWiring(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        shutil.rmtree(CALIB_DIR, ignore_errors=True)

    def setUp(self):
        # 前のテストの確定値を消す（calib_runner は毎回ディスクから読む）。
        for name in os.listdir(CALIB_DIR):
            path = os.path.join(CALIB_DIR, name)
            shutil.rmtree(path) if os.path.isdir(path) else os.remove(path)

        self.node = rclpy.create_node('test_calib_wiring_client')
        # esp32_bridge の代役（calib_runner が /esp32_bridge/set_parameters を呼ぶ）
        self.bridge = rclpy.create_node('esp32_bridge')
        self.bridge.declare_parameter('wheel_radius_scale', 1.0)
        self.bridge.declare_parameter('wheel_base', 0.39)
        self.bridge_sets = []
        self.bridge.add_on_set_parameters_callback(self._on_bridge_set)

        self.sim = _Sim()
        self._last_cmd = Twist()
        self._last_cmd_t = 0.0
        self.cmds = []
        self.node.create_subscription(Twist, '/cmd_vel_behavior', self._on_cmd, 10)
        self.pub_odom = self.node.create_publisher(Odometry, '/odom', 10)
        self.status = None
        self.node.create_subscription(CalibStatus, '/calib/status', self._on_status, 10)
        self.state = None
        self.node.create_subscription(SystemState, '/system/state', self._on_state, _STATE_QOS)
        self.events = []
        self.node.create_subscription(StateEvent, '/system/event', self.events.append, 10)
        self.effects = []
        self.node.create_subscription(StateEffect, '/system/effect', self.effects.append, 10)
        self.pub_fault = self.node.create_publisher(FaultStatus, '/safety/fault', 5)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_effect = self.node.create_publisher(StateEffect, '/system/effect', 10)

        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        self.cli_submit = self.node.create_client(SubmitCalib, '/calib/submit')
        self.cli_start = self.node.create_client(StartCalib, '/calib/start')
        self.cli_rollback = self.node.create_client(RollbackCalib, '/calib/rollback')
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0), 'state_manager 未起動'
        assert self.cli_submit.wait_for_service(timeout_sec=10.0), 'calib_runner 未起動'

        self._reset_to_idle()
        self.events.clear()
        self.effects.clear()
        self.cmds.clear()
        self.bridge_sets.clear()

    def tearDown(self):
        self._reset_to_idle()
        self.node.destroy_node()
        self.bridge.destroy_node()

    # ── 代役 bridge・物理 ────────────────────────────────────
    def _on_bridge_set(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            self.bridge_sets.append((p.name, p.value))
        return SetParametersResult(successful=True)

    def _bridge_value(self, name):
        return self.bridge.get_parameter(name).value

    def _on_cmd(self, msg):
        self._last_cmd = msg
        self._last_cmd_t = time.time()
        self.cmds.append(msg)

    def _on_status(self, msg):
        self.status = msg

    def _on_state(self, msg):
        self.state = msg

    def _sim_step(self, dt):
        cmd = self._last_cmd if time.time() - self._last_cmd_t < 0.3 else Twist()
        v, w = cmd.linear.x, cmd.angular.z
        k_bridge = self._bridge_value('wheel_radius_scale')
        base_bridge = self._bridge_value('wheel_base')
        self.sim.odom_x += v * dt
        self.sim.phys_dist += abs(v) * dt * self.sim.k_true / k_bridge
        self.sim.odom_yaw += w * dt
        self.sim.phys_yaw += abs(w) * dt * base_bridge / self.sim.base_true
        odom = Odometry()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.pose.pose.position.x = self.sim.odom_x
        odom.pose.pose.orientation.z = math.sin(self.sim.odom_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.sim.odom_yaw / 2.0)
        self.pub_odom.publish(odom)

    def _spin(self, duration=0.2):
        deadline = time.time() + duration
        last = time.time()
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.bridge, timeout_sec=0.01)
            now = time.time()
            if now - last >= 0.04:
                self._sim_step(now - last)
                last = now

    def _wait(self, cond, timeout=8.0, what=''):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.05)
            if cond():
                return True
        self.fail(f'タイムアウト: {what}')

    # ── 操作 ────────────────────────────────────────────────
    def _call(self, client, req, timeout=4.0):
        future = client.call_async(req)
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            self._spin(0.02)
        return future.result()

    def _trigger(self, trigger, arg=None):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = 'test'
        return self._call(self.cli_trigger, req)

    def _submit(self, item, measured):
        req = SubmitCalib.Request()
        req.item = item
        req.measured = float(measured)
        req.arg_json = json.dumps({'operator': 'tester'})
        return self._call(self.cli_submit, req)

    def _reset_to_idle(self):
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self._spin(0.1)
        self.pub_hw.publish(Bool(data=True))
        self._spin(0.1)
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.2)
        self._trigger('ui.finish')
        self._spin(0.3)
        if not (self.state and self.state.mode == 'IDLE'):
            self._trigger('ui.finish')
            self._spin(0.3)

    def _state_is(self, mode, state):
        return self.state is not None and self.state.mode == mode and self.state.state == state

    def _result_is(self, result):
        return self.status is not None and self.status.result == result

    def _enter_calib(self, item):
        self.sim.reset()
        res = self._trigger('ui.enter_mode', {'mode': 'CALIB'})
        assert res.accepted, res.reject_reason_key
        self._wait(lambda: self._state_is('CALIB', 'LIST'), what='CALIB/LIST')
        assert self._trigger('ui.calib_item', {'item': item}).accepted
        self._wait(lambda: self._state_is('CALIB', 'S1'), what='S1')

    def _run_to_s3(self):
        assert self._trigger('ui.calib_next').accepted
        self._wait(lambda: self._state_is('CALIB', 'S3'), what='S3（測定走行の完了）')

    def _preview_and_apply(self, item, measured):
        res = self._submit(item, measured)
        assert res.success, f'プレビューが sane にならない: {res.preview_after}'
        self._wait(lambda: self._result_is('PREVIEW_OK'), what='PREVIEW_OK')
        # state_manager が /calib/status から sane をラッチするのを待つ
        deadline = time.time() + 5.0
        while time.time() < deadline:
            r = self._trigger('ui.calib_next')
            if r.accepted:
                break
            self._spin(0.2)
        assert r.accepted, f'S3→S4 に進めない（preview_sane の配線）: {r.reject_reason_key}'
        self._wait(lambda: self._state_is('CALIB', 'S4'), what='S4')

    def _cmd_is_zero_now(self):
        self._spin(0.4)
        recent = [c for c in self.cmds[-5:]]
        return all(abs(c.linear.x) < 1e-9 and abs(c.angular.z) < 1e-9 for c in recent)

    # ════════════════════════════════════════════════════════
    # 1. 直進の通し
    # ════════════════════════════════════════════════════════
    def test_linear_full_flow_commits(self):
        self._enter_calib('LINEAR')
        self._run_to_s3()
        # 真の補正は 0.95。止まった地点の実距離を「巻き尺で測った値」として入力する
        measured = self.sim.phys_dist
        assert abs(measured - DIST_M * 0.95) < 0.02, f'物理モデルが想定と違う: {measured}'
        self.sim.reset()
        self._preview_and_apply('LINEAR', measured)
        # 適用: bridge の wheel_radius_scale が真の値へ（検証走行の前に反映される）
        self._wait(lambda: abs(self._bridge_value('wheel_radius_scale') - 0.95) < 0.03,
                   what='bridge への適用')
        self._wait(lambda: self._result_is('WAIT_VERIFY'), what='検証走行の完了')
        verify_measured = self.sim.phys_dist
        assert abs(verify_measured - DIST_M) < 0.02, f'補正後の実距離が指定距離に合わない: {verify_measured}'
        res = self._submit('LINEAR', verify_measured)
        assert res.success, res.preview_after
        self._wait(lambda: self._state_is('CALIB', 'LIST'), what='LIST へ戻る（確定）')
        self._wait(lambda: os.path.exists(os.path.join(CALIB_DIR, 'current.yaml')),
                   what='current.yaml')
        with open(os.path.join(CALIB_DIR, 'current.yaml'), encoding='utf-8') as f:
            cur = yaml.safe_load(f)
        entry = cur['items']['LINEAR']
        assert abs(entry['values']['wheel_radius_scale'] - 0.95) < 0.03, entry
        assert entry['verification']['result'] == 'OK'
        assert entry['operator'] == 'tester'
        names = [e.name for e in self.effects if e.dest == 'calib_runner']
        for expected in ('begin_wizard', 'run_measurement', 'build_preview',
                         'apply_and_verify', 'commit_calib'):
            assert expected in names, f'{expected} が calib_runner へ配送されていない: {names}'
        # 走行は /cmd_vel_behavior だけ（このノードは /cmd_vel へ出さない）
        assert self.cmds, '/cmd_vel_behavior に何も出ていない'
        assert self._cmd_is_zero_now(), '確定後も /cmd_vel_behavior が 0 にならない'

    # ════════════════════════════════════════════════════════
    # 2. 検証 NG で適用前に戻る（T-CAL-06）
    # ════════════════════════════════════════════════════════
    def test_verify_ng_reverts_to_previous(self):
        self._enter_calib('LINEAR')
        self._run_to_s3()
        measured = self.sim.phys_dist
        self.sim.reset()
        self._preview_and_apply('LINEAR', measured)
        self._wait(lambda: abs(self._bridge_value('wheel_radius_scale') - 0.95) < 0.03,
                   what='bridge への適用')
        self._wait(lambda: self._result_is('WAIT_VERIFY'), what='検証走行の完了')
        # 実測を大きく外れた値（許容 5% に対し 20% 誤差）→ 検証 NG
        res = self._submit('LINEAR', DIST_M * 0.8)
        assert not res.success
        self._wait(lambda: self._state_is('CALIB', 'S2'), what='検証 NG で S2 へ（T-CAL-06）')
        self._wait(lambda: abs(self._bridge_value('wheel_radius_scale') - 1.0) < 1e-9,
                   what='適用前の値へ戻る')
        assert any(e.event == 'evt.calib_verify_ng' for e in self.events)
        assert any(e.name == 'revert_calib' for e in self.effects)
        assert not os.path.exists(os.path.join(CALIB_DIR, 'current.yaml')), \
            '検証 NG なのに補正値が確定された'
        # S2 での再走行（/calib/start）→ S3 へ
        self.sim.reset()
        req = StartCalib.Request()
        req.item = 'LINEAR'
        res = self._call(self.cli_start, req)
        assert res.started, res.message
        self._wait(lambda: self._state_is('CALIB', 'S3'), what='再走行後に S3')
        # 二重起動は拒否
        res = self._call(self.cli_start, req)
        assert not res.started

    # ════════════════════════════════════════════════════════
    # 3. A10 を超える値は適用しない
    # ════════════════════════════════════════════════════════
    def test_a10_violation_is_never_applied(self):
        self.sim.k_true = 0.85          # 真の補正が 15% ずれている（機械的異常）
        self._enter_calib('LINEAR')
        self._run_to_s3()
        measured = self.sim.phys_dist
        assert abs(measured - DIST_M * 0.85) < 0.02
        res = self._submit('LINEAR', measured)
        assert not res.success, 'A10 を超える補正のプレビューが sane になった'
        self._wait(lambda: self._result_is('PREVIEW_INSANE'), what='PREVIEW_INSANE')
        for _ in range(5):
            r = self._trigger('ui.calib_next')
            assert not r.accepted, 'A10 超過のまま S4 へ進めた'
            self._spin(0.1)
        assert self._state_is('CALIB', 'S3')
        assert not [s for s in self.bridge_sets if s[0] == 'wheel_radius_scale'], \
            f'A10 超過の値が esp32_bridge へ送られた: {self.bridge_sets}'
        assert abs(self._bridge_value('wheel_radius_scale') - 1.0) < 1e-9

    # ════════════════════════════════════════════════════════
    # 4. 中断（abort）
    # ════════════════════════════════════════════════════════
    def test_abort_stops_motion_and_reverts(self):
        self._enter_calib('LINEAR')
        assert self._trigger('ui.calib_next').accepted
        self._wait(lambda: any(abs(c.linear.x) > 0.01 for c in self.cmds), what='走り出す')
        assert self._trigger('ui.abort').accepted
        self._wait(lambda: self._state_is('CALIB', 'LIST'), what='LIST へ（T-CAL-07）')
        dist_at_abort = self.sim.phys_dist
        assert self._cmd_is_zero_now(), '中断したのに /cmd_vel_behavior が 0 にならない'
        self._spin(0.6)
        assert self.sim.phys_dist - dist_at_abort < 0.01, '中断後も走り続けている'
        assert dist_at_abort < DIST_M * 0.95 * 0.9, '中断の前に走り終えていた（試験の前提崩れ）'
        assert not os.path.exists(os.path.join(CALIB_DIR, 'current.yaml'))

    def test_abort_during_verify_reverts_applied_value(self):
        self._enter_calib('LINEAR')
        self._run_to_s3()
        measured = self.sim.phys_dist
        self.sim.reset()
        self._preview_and_apply('LINEAR', measured)
        self._wait(lambda: abs(self._bridge_value('wheel_radius_scale') - 0.95) < 0.03,
                   what='bridge への適用')
        assert self._trigger('ui.abort').accepted
        self._wait(lambda: abs(self._bridge_value('wheel_radius_scale') - 1.0) < 1e-9,
                   what='中断で適用前へ戻る')
        assert not os.path.exists(os.path.join(CALIB_DIR, 'current.yaml'))

    # ════════════════════════════════════════════════════════
    # 5. フォルト・非常停止で確定しない（§7 #3）
    # ════════════════════════════════════════════════════════
    def test_fault_during_verify_does_not_commit(self):
        self._enter_calib('LINEAR')
        self._run_to_s3()
        measured = self.sim.phys_dist
        self.sim.reset()
        self._preview_and_apply('LINEAR', measured)
        self._wait(lambda: abs(self._bridge_value('wheel_radius_scale') - 0.95) < 0.03,
                   what='bridge への適用')
        self.pub_fault.publish(FaultStatus(
            active=True, fault_type='LIDAR_LOST', severity='RECOVERABLE'))
        self._wait(lambda: abs(self._bridge_value('wheel_radius_scale') - 1.0) < 1e-9,
                   what='フォルトで適用前へ戻る')
        assert self._cmd_is_zero_now()
        assert not os.path.exists(os.path.join(CALIB_DIR, 'current.yaml'))
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))

    def test_estop_during_run_stops_and_does_not_commit(self):
        self._enter_calib('LINEAR')
        assert self._trigger('ui.calib_next').accepted
        self._wait(lambda: any(abs(c.linear.x) > 0.01 for c in self.cmds), what='走り出す')
        self.pub_hw.publish(Bool(data=True))
        self._wait(lambda: self._state_is('ESTOP', 'NONE') or
                   (self.state is not None and self.state.mode == 'ESTOP'), what='ESTOP')
        assert self._cmd_is_zero_now(), '非常停止で /cmd_vel_behavior が 0 にならない'
        dist = self.sim.phys_dist
        self._spin(0.6)
        assert self.sim.phys_dist - dist < 0.01
        assert not os.path.exists(os.path.join(CALIB_DIR, 'current.yaml'))
        self.pub_hw.publish(Bool(data=False))

    # ════════════════════════════════════════════════════════
    # 6. CALIB 以外では動かない
    # ════════════════════════════════════════════════════════
    def test_effects_and_services_ignored_outside_calib(self):
        assert self.state.mode == 'IDLE'
        for name in ('begin_wizard', 'run_measurement'):
            eff = StateEffect()
            eff.header.stamp = self.node.get_clock().now().to_msg()
            eff.name = name
            eff.dest = 'calib_runner'
            eff.args_json = json.dumps({'item': 'LINEAR'})
            self.pub_effect.publish(eff)
            self._spin(0.2)
        self.sim.reset()
        self._spin(1.0)
        assert not any(abs(c.linear.x) > 1e-9 or abs(c.angular.z) > 1e-9 for c in self.cmds), \
            'CALIB 以外で effect を受けて走った'
        req = StartCalib.Request()
        req.item = 'LINEAR'
        assert not self._call(self.cli_start, req).started
        assert not self._submit('LINEAR', 0.3).success
        rb = RollbackCalib.Request()
        rb.item = 'LINEAR'
        rb.generation = 1
        assert not self._call(self.cli_rollback, rb).success

    # ════════════════════════════════════════════════════════
    # ロールバック（§7 #6）
    # ════════════════════════════════════════════════════════
    def test_rollback_restores_previous_and_applies_to_bridge(self):
        from th_maintenance.calib_store import CalibStore
        store = CalibStore(CALIB_DIR)
        store.commit('LINEAR', {'wheel_radius_scale': 0.97}, '2026-10-01T00:00:00',
                     {'result': 'OK'})
        store.commit('LINEAR', {'wheel_radius_scale': 0.93}, '2026-10-02T00:00:00',
                     {'result': 'OK'})
        res = self._trigger('ui.enter_mode', {'mode': 'CALIB'})
        assert res.accepted
        self._wait(lambda: self._state_is('CALIB', 'LIST'), what='CALIB/LIST')
        rb = RollbackCalib.Request()
        rb.item = 'LINEAR'
        rb.generation = 1
        assert self._call(self.cli_rollback, rb).success
        self._wait(lambda: abs(self._bridge_value('wheel_radius_scale') - 0.97) < 1e-9,
                   what='ロールバック値が bridge へ')
        assert store.current_entry('LINEAR')['values']['wheel_radius_scale'] == 0.97
        # 存在しない世代・不明項目は拒否
        rb.generation = 3
        assert not self._call(self.cli_rollback, rb).success
        # S1 以降（校正の途中）では拒否
        assert self._trigger('ui.calib_item', {'item': 'LINEAR'}).accepted
        self._wait(lambda: self._state_is('CALIB', 'S1'), what='S1')
        rb.generation = 1
        assert not self._call(self.cli_rollback, rb).success


if __name__ == '__main__':
    unittest.main()
