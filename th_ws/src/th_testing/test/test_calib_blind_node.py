"""
test_calib_blind_node.py
========================
校正 BLIND（LiDAR 死角マスク）を **本物のノードで通す** launch 試験。

`state_manager`・`calib_runner`・`lidar_filter`・`obstacle_limiter`・`opcheck_runner` を実際に起動し、
画面の代わりにテストが `/system/trigger`（ui.*）と `/calib/submit` を叩く。`/scan` はテストが
出す（全周 1° 刻み。前方 ±10° に「支柱」＝近距離の恒常的な写り込みがある）。

死角マスクは「その角度の点を障害物として見ない」設定で、**3 つのノードが読む**:
  - `lidar_filter`   : `/scan_filtered` から選んだ角度の点を消す（地図用）
  - `obstacle_limiter`: その方向へ進むとき速度上限を v_reverse へ絞る（安全判定。C++。実行中更新あり）
  - `opcheck_runner` : 始業点検の死角ズレの比較先
校正で変えたマスクは**この 3 つに同時に効く**。地図用と安全判定で食い違う時間を作らない。

ここで縛るもの:
  1. 通し: 選択 → プレビュー → 適用 → 検証 → 確定。3 ノードすべてに同じ値が入り、`/scan_filtered` から
     選んだ角度の点が実際に消え、obstacle_limiter の前方の上限が v_reverse に絞られ、current.yaml が書かれる
  2. 幅の上限を超える選択（1 区間 31°・総幅 91°・9 区間）は適用に進めない。どのノードにも届かない
  3. 検証 NG（写り込みを覆っていない選択）は**3 ノードすべて適用前へ戻り**、確定されない
  4. 適用中の中断・フォルトで 3 ノードとも適用前へ戻り、確定されない
  5. obstacle_limiter が（自分の上限検査で）拒否したら、先に入った lidar_filter・opcheck_runner も戻る
  6. obstacle_limiter は実行中に届いた上限違反の値を自分で拒否し、古い値のまま動く
  7. ロールバック（履歴の世代）が 3 ノードに効く
  8. CALIB 以外では受け付けない

変異チェック（一覧は報告に記載）:
  ① blind_core.check_limits の上限判定を外す → test_over_limit_*  が赤
  ② calib_runner._discard / _revert_runtime の戻しを消す → test_abort_*  が赤
  ③ _blind_verify_tick が不合格でも VERIFIED にする → test_verify_ng_*  が赤
  ④ calib_runner の BLIND_TARGET_NODES から 1 つ外す → test_full_flow_*  が赤（3 ノード確認）
  ⑥ obstacle_limiter の上限検査（validate_blind_ranges）を外す → test_limiter_rejects_*  が赤
  ⑦ obstacle_limiter の更新コールバックが params_ に反映しない → test_full_flow_*  が赤（上限の絞り）
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
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from th_system_msgs.msg import (CalibStatus, FaultStatus, LimiterStatus, StateEffect,
                                StateEvent, SystemState)
from th_system_msgs.srv import RollbackCalib, SubmitCalib, UiTrigger

CALIB_DIR = tempfile.mkdtemp(prefix='calib_blind_')

V_CALIB = 0.30
V_REVERSE = 0.25
POLE = (-10.0, 10.0)      # 前方の「支柱」（近距離の恒常的な写り込み）
TARGETS = ('obstacle_limiter', 'lidar_filter', 'opcheck_runner')

_STATE_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST)


@pytest.mark.launch_test
def generate_test_description():
    tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher', name='laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'], output='screen')
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
            'calib_blind_tolerance_deg': 5.0,
            'calib_blind_verify_frames': 8.0,
            'calib_blind_verify_timeout_s': 20.0,
        }],
        output='screen')
    lidar_filter = launch_ros.actions.Node(
        package='th_perception', executable='lidar_filter.py', name='lidar_filter',
        output='screen')
    limiter = launch_ros.actions.Node(
        package='th_safety', executable='obstacle_limiter', name='obstacle_limiter',
        parameters=[{'v_slow': 0.30, 'v_reverse': V_REVERSE, 'v_calib': V_CALIB,
                     'obstacle_floor_distance_m': 0.05}],
        output='screen')
    opcheck = launch_ros.actions.Node(
        package='th_maintenance', executable='opcheck_runner.py', name='opcheck_runner',
        output='screen')
    return launch.LaunchDescription([
        tf, state_manager, calib_runner, lidar_filter, limiter, opcheck,
        launch_testing.actions.ReadyToTest(),
    ]), {}


def _in_range(angle_deg, lo, hi):
    off = (angle_deg - lo) % 360.0
    return off <= (hi - lo) % 360.0 + 1e-9


class TestCalibBlind(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        shutil.rmtree(CALIB_DIR, ignore_errors=True)

    def setUp(self):
        for name in os.listdir(CALIB_DIR):
            path = os.path.join(CALIB_DIR, name)
            shutil.rmtree(path) if os.path.isdir(path) else os.remove(path)
        self.node = rclpy.create_node('test_calib_blind_client')
        self.send_scan = True
        self.pole = POLE
        self.status = None
        self.state = None
        self.limiter = None
        self.filtered = None
        self.events = []
        self.node.create_subscription(CalibStatus, '/calib/status',
                                      lambda m: setattr(self, 'status', m), 10)
        self.node.create_subscription(SystemState, '/system/state',
                                      lambda m: setattr(self, 'state', m), _STATE_QOS)
        self.node.create_subscription(StateEvent, '/system/event', self.events.append, 10)
        self.node.create_subscription(
            LimiterStatus, '/safety/limiter_status', lambda m: setattr(self, 'limiter', m),
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(
            LaserScan, '/scan_filtered', lambda m: setattr(self, 'filtered', m),
            qos_profile_sensor_data)
        self.pub_scan = self.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_muxed = self.node.create_publisher(Twist, '/cmd_vel_muxed', 1)
        self.pub_estop = self.node.create_publisher(Bool, '/safety/estop', 1)
        self.pub_lock = self.node.create_publisher(Bool, '/safety/fault_lock', 1)
        self.pub_fault = self.node.create_publisher(FaultStatus, '/safety/fault', 5)
        self.pub_hw = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.cli_trigger = self.node.create_client(UiTrigger, '/system/trigger')
        self.cli_submit = self.node.create_client(SubmitCalib, '/calib/submit')
        self.cli_rollback = self.node.create_client(RollbackCalib, '/calib/rollback')
        self.cli_get = {n: self.node.create_client(GetParameters, f'/{n}/get_parameters')
                        for n in TARGETS}
        self.cli_set = {n: self.node.create_client(SetParameters, f'/{n}/set_parameters')
                        for n in TARGETS + ('calib_runner',)}
        assert self.cli_trigger.wait_for_service(timeout_sec=15.0), 'state_manager 未起動'
        assert self.cli_submit.wait_for_service(timeout_sec=15.0), 'calib_runner 未起動'
        for n, c in self.cli_get.items():
            assert c.wait_for_service(timeout_sec=30.0), f'{n} 未起動'
        for n, c in self.cli_set.items():
            assert c.wait_for_service(timeout_sec=30.0), f'{n} 未起動'
        self._last_pub = 0.0
        self._reset_to_idle()
        # 3 ノードを空（マスクなし）へ揃える（前の試験の影響を残さない）
        self._set_param_all([])
        self.events.clear()

    def tearDown(self):
        self.send_scan = True
        self._reset_to_idle()
        self.node.destroy_node()

    # ── 入力の配信（/scan・限界値用の周辺トピック）──────────────
    def _make_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi - math.radians(1.0)
        msg.angle_increment = math.radians(1.0)
        msg.range_min = 0.05
        msg.range_max = 12.0
        out = []
        for i in range(360):
            a = -180.0 + i
            near = self.pole is not None and _in_range(a, *self.pole)
            out.append(0.3 if near else 3.0)
        msg.ranges = out
        return msg

    def _publish_inputs(self):
        if self.send_scan:
            self.pub_scan.publish(self._make_scan())
        cmd = Twist()
        cmd.linear.x = 0.5
        self.pub_muxed.publish(cmd)
        self.pub_estop.publish(Bool(data=False))
        self.pub_lock.publish(Bool(data=False))
        self._last_pub = time.time()

    def _spin(self, duration=0.2):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if time.time() - self._last_pub >= 0.05:
                self._publish_inputs()

    def _wait(self, cond, timeout=10.0, what=''):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.05)
            if cond():
                return True
        self.fail(f'タイムアウト: {what}')

    # ── サービス ────────────────────────────────────────────
    def _call(self, client, req, timeout=5.0):
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

    def _submit_ranges(self, ranges):
        req = SubmitCalib.Request()
        req.item = 'BLIND'
        req.measured = 0.0
        req.arg_json = json.dumps({'operator': 'tester', 'ranges': ranges})
        return self._call(self.cli_submit, req)

    def _get(self, node):
        req = GetParameters.Request()
        req.names = ['blind_angle_ranges']
        res = self._call(self.cli_get[node], req)
        v = res.values[0]
        return list(v.double_array_value) if v.type == ParameterType.PARAMETER_DOUBLE_ARRAY else []

    def _set_param(self, node, name, value):
        req = SetParameters.Request()
        pm = ParameterMsg()
        pm.name = name
        if isinstance(value, list):
            pm.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                      double_array_value=[float(v) for v in value])
        else:
            pm.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                      double_value=float(value))
        req.parameters.append(pm)
        return self._call(self.cli_set[node], req).results[0]

    def _set_param_all(self, flat):
        for n in TARGETS:
            r = self._set_param(n, 'blind_angle_ranges', flat)
            assert r.successful, (n, r.reason)

    def _values(self):
        return {n: self._get(n) for n in TARGETS}

    def _all_equal(self, flat):
        v = self._values()
        return all(v[n] == [float(x) for x in flat] for n in TARGETS)

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

    def _enter_blind(self):
        assert self._trigger('ui.enter_mode', {'mode': 'CALIB'}).accepted
        self._wait(lambda: self._state_is('CALIB', 'LIST'), what='CALIB/LIST')
        assert self._trigger('ui.calib_item', {'item': 'BLIND'}).accepted
        self._wait(lambda: self._state_is('CALIB', 'S1'), what='S1')
        assert self._trigger('ui.calib_next').accepted
        self._wait(lambda: self._state_is('CALIB', 'S2'), what='S2（選択待ち）')
        self._wait(lambda: self.status is not None and self.status.item == 'BLIND'
                   and self.status.result == 'RUNNING', what='RUNNING（選択待ち）')

    def _select_to_s3(self, ranges):
        res = self._submit_ranges(ranges)
        assert res.success, f'プレビューが sane にならない: {res.preview_after}'
        self._wait(lambda: self._state_is('CALIB', 'S3'), what='S3')
        self._wait(lambda: self._result_is('PREVIEW_OK'), what='PREVIEW_OK')
        return res

    def _go_s4(self):
        deadline = time.time() + 6.0
        r = None
        while time.time() < deadline:
            r = self._trigger('ui.calib_next')
            if r.accepted:
                break
            self._spin(0.2)
        assert r.accepted, f'S3→S4 に進めない: {r.reject_reason_key}'
        self._wait(lambda: self._state_is('CALIB', 'S4'), what='S4')

    def _current_yaml(self):
        path = os.path.join(CALIB_DIR, 'current.yaml')
        if not os.path.exists(path):
            return None
        with open(path, encoding='utf-8') as f:
            return yaml.safe_load(f)

    def _finite_in(self, lo, hi):
        """最新の /scan_filtered で [lo,hi] 度の中にまだ有限な点の数。"""
        f = self.filtered
        n = 0
        for i, r in enumerate(f.ranges):
            a = math.degrees(f.angle_min + i * f.angle_increment)
            if _in_range(a, lo, hi) and math.isfinite(r):
                n += 1
        return n

    def _limiter_cap(self):
        return None if self.limiter is None else self.limiter.applied_limit_mps

    # ════════════════════════════════════════════════════════
    # 1. 通し
    # ════════════════════════════════════════════════════════
    def test_full_flow_commits_and_reaches_all_three_nodes(self):
        # 適用前: マスクなし。前方は支柱の写り込みが /scan_filtered にそのまま出る
        self._wait(lambda: self.filtered is not None and self._finite_in(*POLE) > 0,
                   what='/scan_filtered の支柱')
        self._wait(lambda: self._limiter_cap() is not None, what='limiter_status')
        before_cap = self._limiter_cap()
        self._enter_blind()
        res = self._select_to_s3([[-12.0, 12.0]])
        after = json.loads(res.preview_after)
        assert after['masked_points'] >= 20, after       # プレビュー: 消える点の数
        assert after['blind_angle_ranges'] == [-12.0, 12.0], after
        # プレビューの段階ではどのノードにも入っていない（適用は S4）
        assert self._all_equal([]), self._values()
        self._go_s4()
        self._wait(lambda: self._state_is('CALIB', 'LIST'), timeout=30.0, what='LIST（確定）')
        # 3 ノードすべてに同じ値が入っている
        assert self._all_equal([-12.0, 12.0]), self._values()
        # lidar_filter: 選んだ角度の点が /scan_filtered から消えている
        self._wait(lambda: self._finite_in(-12.0, 12.0) == 0, what='/scan_filtered から支柱が消える')
        assert self._finite_in(15.0, 170.0) > 100, '選んでいない角度まで消えている'
        # obstacle_limiter: 前方がマスクに入ったので上限が v_reverse に絞られる（安全判定にも効く）
        self._wait(lambda: abs(self._limiter_cap() - V_REVERSE) < 1e-3, timeout=5.0,
                   what=f'前方の上限が v_reverse へ（before={before_cap}）')
        # 確定: current.yaml
        cur = self._current_yaml()
        entry = cur['items']['BLIND']
        assert entry['values']['blind_angle_ranges'] == [-12.0, 12.0], entry
        assert entry['verification']['result'] == 'OK'
        assert entry['operator'] == 'tester'
        names = [e.event for e in self.events]
        assert 'evt.calib_step_done' in names

    # ════════════════════════════════════════════════════════
    # 2. 幅の上限
    # ════════════════════════════════════════════════════════
    def test_over_limit_selection_is_rejected_and_never_applied(self):
        self._enter_blind()
        four_wide = [[i * 80.0 - 170.0, i * 80.0 - 147.0] for i in range(4)]    # 23° × 4 = 92°
        nine = [[i * 40.0 - 170.0, i * 40.0 - 165.0] for i in range(9)]         # 9 区間
        for bad, why in (([[0.0, 31.0]], '1 区間 31°'), (four_wide, '総幅 92°'), (nine, '9 区間')):
            res = self._submit_ranges(bad)
            assert not res.success, f'上限超過（{why}）のプレビューが sane になった'
            self._wait(lambda: self._result_is('PREVIEW_INSANE'), what=f'PREVIEW_INSANE（{why}）')
            assert self._state_is('CALIB', 'S2'), f'上限超過（{why}）で S3 へ進んだ'
            assert not self._trigger('ui.calib_next').accepted, f'上限超過（{why}）で進めた'
        assert self._all_equal([]), f'上限超過の値がノードへ届いた: {self._values()}'
        assert self._current_yaml() is None
        # 幅ゼロも弾く
        assert not self._submit_ranges([[40.0, 40.0]]).success
        # 上限内の選択ならそのまま進める（PREVIEW_INSANE は終わる）
        assert self._submit_ranges([[-12.0, 12.0]]).success

    def test_over_limit_resubmit_in_s3_blocks_apply(self):
        self._enter_blind()
        self._select_to_s3([[-12.0, 12.0]])
        res = self._submit_ranges([[0.0, 45.0]])
        assert not res.success
        self._wait(lambda: self._result_is('PREVIEW_INSANE'), what='S3 での再選択が上限超過')
        assert self._state_is('CALIB', 'S3')
        for _ in range(5):
            assert not self._trigger('ui.calib_next').accepted, '上限超過のまま S4 へ進めた'
            self._spin(0.1)
        assert self._all_equal([])

    # ════════════════════════════════════════════════════════
    # 3. 検証 NG は適用前へ戻す
    # ════════════════════════════════════════════════════════
    def test_verify_ng_reverts_all_three_and_does_not_commit(self):
        self._enter_blind()
        # 支柱（前方 ±10°）を覆わない選択 → 写り込みが選択の外に 20° 残る → 許容 5° 超過
        self._select_to_s3([[100.0, 120.0]])
        self._go_s4()
        self._wait(lambda: self._state_is('CALIB', 'S2'), timeout=30.0, what='検証 NG で S2 へ（T-CAL-06）')
        assert any(e.event == 'evt.calib_verify_ng' for e in self.events)
        self._wait(lambda: self._all_equal([]), what='3 ノードとも適用前へ戻る')
        assert self._current_yaml() is None, '検証 NG なのに確定された'
        assert 'offset_exceeds_tolerance' in json.loads(self.status.detail)['reason']

    # ════════════════════════════════════════════════════════
    # 4. 中断・フォルト
    # ════════════════════════════════════════════════════════
    def test_abort_after_apply_reverts_all_three(self):
        self._enter_blind()
        self._select_to_s3([[-12.0, 12.0]])
        # 適用後に /scan を止めて検証を待たせる（その間に中断する）
        self.send_scan = False
        self._go_s4()
        self._wait(lambda: self._all_equal([-12.0, 12.0]), what='3 ノードへ適用')
        assert self._trigger('ui.abort').accepted
        self._wait(lambda: self._all_equal([]), what='中断で 3 ノードとも適用前へ戻る')
        assert self._current_yaml() is None

    def test_fault_after_apply_reverts_all_three(self):
        self._enter_blind()
        self._select_to_s3([[-12.0, 12.0]])
        self.send_scan = False
        self._go_s4()
        self._wait(lambda: self._all_equal([-12.0, 12.0]), what='3 ノードへ適用')
        self.pub_fault.publish(FaultStatus(
            active=True, fault_type='LIDAR_LOST', severity='RECOVERABLE'))
        self._wait(lambda: self._all_equal([]), what='フォルトで 3 ノードとも適用前へ戻る')
        assert self._current_yaml() is None
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))

    # ════════════════════════════════════════════════════════
    # 5. obstacle_limiter が拒否したら、先に入ったノードも戻る
    # ════════════════════════════════════════════════════════
    def test_limiter_rejection_reverts_the_others(self):
        # calib_runner の上限だけ緩める（obstacle_limiter は 30° のまま）。
        r = self._set_param('calib_runner', 'blind_max_sector_deg', 60.0)
        assert r.successful, r.reason
        try:
            self._enter_blind()
            self._select_to_s3([[-20.0, 20.0]])          # 40°: runner は通すが limiter は拒否する
            self._go_s4()
            self._wait(lambda: self._state_is('CALIB', 'S2'), timeout=30.0,
                       what='limiter の拒否で S2 へ')
            assert 'apply_failed' in json.loads(self.status.detail)['reason']
            self._wait(lambda: self._all_equal([]), what='lidar_filter・opcheck_runner も適用前へ戻る')
            assert self._current_yaml() is None
        finally:
            self._set_param('calib_runner', 'blind_max_sector_deg', 30.0)

    # ════════════════════════════════════════════════════════
    # 6. obstacle_limiter 自身の上限検査（実行中更新）
    # ════════════════════════════════════════════════════════
    def test_limiter_rejects_over_limit_runtime_update_and_keeps_old_value(self):
        ok = self._set_param('obstacle_limiter', 'blind_angle_ranges', [-12.0, 12.0])
        assert ok.successful, ok.reason
        assert self._get('obstacle_limiter') == [-12.0, 12.0]
        for bad in ([0.0, 40.0],                                    # 1 区間 40°
                    [0.0, 25.0, 60.0, 85.0, 120.0, 145.0, -60.0, -35.0],  # 総幅 100°
                    [5.0, 5.0],                                     # 幅ゼロ
                    [0.0, 10.0, 20.0]):                             # 奇数長
            res = self._set_param('obstacle_limiter', 'blind_angle_ranges', bad)
            assert not res.successful, f'上限違反の値を受理した: {bad}'
        assert self._get('obstacle_limiter') == [-12.0, 12.0], '拒否したのに値が変わった'
        # 内部状態（安全判定）も古い値のまま: 前方がマスク内なので上限は v_reverse のまま
        self._spin(0.5)
        self._wait(lambda: abs(self._limiter_cap() - V_REVERSE) < 1e-3, what='古い値のまま絞り続ける')
        # 空（マスクなし）は受理。上限は元（前方）へ戻る
        assert self._set_param('obstacle_limiter', 'blind_angle_ranges', []).successful
        self._wait(lambda: self._limiter_cap() is not None and self._limiter_cap() > V_REVERSE + 1e-3,
                   what='マスクを外すと前方の上限が戻る')

    # ════════════════════════════════════════════════════════
    # 7. ロールバック
    # ════════════════════════════════════════════════════════
    def test_rollback_applies_history_to_all_three(self):
        from th_maintenance.calib_store import CalibStore
        store = CalibStore(CALIB_DIR)
        store.commit('BLIND', {'blind_angle_ranges': [40.0, 60.0]}, '2026-10-01T00:00:00',
                     {'result': 'OK'})
        store.commit('BLIND', {'blind_angle_ranges': [-60.0, -40.0]}, '2026-10-02T00:00:00',
                     {'result': 'OK'})
        assert self._trigger('ui.enter_mode', {'mode': 'CALIB'}).accepted
        self._wait(lambda: self._state_is('CALIB', 'LIST'), what='CALIB/LIST')
        rb = RollbackCalib.Request()
        rb.item = 'BLIND'
        rb.generation = 1
        assert self._call(self.cli_rollback, rb).success
        self._wait(lambda: self._all_equal([40.0, 60.0]), what='ロールバック値が 3 ノードへ')
        assert store.current_entry('BLIND')['values']['blind_angle_ranges'] == [40.0, 60.0]
        # 上限違反の履歴値へは戻せない
        store.commit('BLIND', {'blind_angle_ranges': [0.0, 80.0]}, '2026-10-03T00:00:00',
                     {'result': 'OK'})
        store.commit('BLIND', {'blind_angle_ranges': [10.0, 20.0]}, '2026-10-04T00:00:00',
                     {'result': 'OK'})
        rb.generation = 1      # [0,80] は 80° 幅
        assert not self._call(self.cli_rollback, rb).success
        self._wait(lambda: self._all_equal([40.0, 60.0]), what='拒否したら値は変わらない')

    # ════════════════════════════════════════════════════════
    # 8. CALIB 以外では動かない
    # ════════════════════════════════════════════════════════
    def test_ignored_outside_calib(self):
        assert self.state.mode == 'IDLE'
        for name in ('begin_wizard', 'run_measurement', 'apply_and_verify', 'commit_calib'):
            pub = self.node.create_publisher(StateEffect, '/system/effect', 10)
            eff = StateEffect()
            eff.header.stamp = self.node.get_clock().now().to_msg()
            eff.name = name
            eff.dest = 'calib_runner'
            eff.args_json = json.dumps({'item': 'BLIND'})
            pub.publish(eff)
            self._spin(0.2)
        assert not self._submit_ranges([[-12.0, 12.0]]).success
        assert self._all_equal([])
        assert self._current_yaml() is None


if __name__ == '__main__':
    unittest.main()
