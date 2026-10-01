"""
test_home_declarer_node.py — 待機場所宣言の配線試験（onsite §9 #10
「待機場所の宣言でずれが検出される」）。

正本: `docs/plan/detailed/DetailedDesign-onsite.md` §9 受け入れ条件 #10。
純ロジックは `test_home_declare_core.py` で済んでいる。ここでは**ノードの配線**
（`/onsite/pins` → HOME ピン抽出 → TF `map → base_link` との照合 →
`/onsite/declare_home` の応答 → `/onsite/home_declared` の publish／解除）を縛る。

===============================================================================
【Gazebo を使わない理由（設計書の読み替え）】
===============================================================================
設計書の手段は「Gazebo（ピンから 1 m ずらして置く）」だが、ずれの検出は
`home_declarer` が見る `map → base_link` TF と HOME ピンの差で決まる。
Gazebo で機体をずらして置く代わりに**試験から `/tf_static` で TF を与えて
ずらす**ことで、同じ照合経路（TF バッファ → `pose_offset` → 許容判定 →
`home_declared`）をそのまま検証できる。Gazebo・SLAM・Nav2 は起動しない。

===============================================================================
【外から入れてよいもの（これ以外は入れない）】
===============================================================================
- `/tf_static` に `map → base_link` … 自己位置の代役。試験ごとに姿勢を変える。
- `pins.yaml`（`pin_registrar` の `venue_dir` に置く初期 HOME ピン）…
  起動前の配置。試験中のピン変更は `/onsite/register_pin`（MAP_TAP。
  本番の口）で行う。
- `/onsite/declare_home` の呼び出し … 画面が叩くのと同じ経路。

**入れないもの**: `/onsite/pins` の publish（`pin_registrar` が出すべきもの。
直接出すと検証にならない）、`/onsite/home_declared` の publish（観測のみ）。
"""

from __future__ import annotations

import math
import os
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from th_system_msgs.srv import DeclareHome, RegisterPin


# ===========================================================================
# T-1: 判定に使う許容値は生成 YAML（registry.yaml 由来）から読む。直書きしない。
# （`data/generated/home_declarer.yaml` があればそこ、無ければ
# `th_params/config/registry.yaml`。本番の bringup も同じ値で起動する。）
# ===========================================================================
_TH_TESTING_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
_WS_ROOT = os.path.abspath(os.path.join(_TH_TESTING_ROOT, '..', '..'))
_WORKTREE_ROOT = os.path.abspath(os.path.join(_WS_ROOT, '..'))


def _find_generated_dir() -> str:
    candidates = [
        os.environ.get('TH_GENERATED_DIR', ''),
        '/root/th_data/generated',
        os.path.join(_WS_ROOT, 'data', 'generated'),
    ]
    for path in candidates:
        if path and os.path.isdir(path):
            return path
    pytest.fail(
        '生成パラメータ置き場が見つからない（試した: '
        + ', '.join(c or '(env 未設定)' for c in candidates) + '）。')


def _resolve_tolerances() -> tuple[float, float]:
    """(tolerance_m, tolerance_deg)。生成 yaml 優先、無ければ registry.yaml。"""
    gen_path = os.path.join(_find_generated_dir(), 'home_declarer.yaml')
    if os.path.exists(gen_path):
        with open(gen_path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = (data.get('home_declarer') or {}).get('ros__parameters') or {}
        try:
            return float(params['home_declare_tolerance_m']), \
                float(params['home_declare_tolerance_deg'])
        except KeyError:
            pytest.fail(f'{gen_path} に home_declare_tolerance_* が無い。')
    reg_path = os.path.join(
        _WS_ROOT, 'src', 'th_params', 'config', 'registry.yaml')
    if not os.path.exists(reg_path):
        pytest.fail(f'生成 yaml も registry も無い ({gen_path}, {reg_path})。')
    with open(reg_path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    vals = {}
    for entry in data.get('parameters', []) or []:
        if entry.get('name') in ('home_declare_tolerance_m',
                                 'home_declare_tolerance_deg'):
            vals[entry['name']] = float(entry['value'])
    if len(vals) != 2:
        pytest.fail(f'{reg_path} に home_declare_tolerance_* が揃っていない。')
    return vals['home_declare_tolerance_m'], vals['home_declare_tolerance_deg']


_TOL_M, _TOL_DEG = _resolve_tolerances()

# ── HOME ピン（`pin_registrar` の `venue_dir` に置く `pins.yaml`） ──────────
# 一時ファイルは worktree 内の `.briefs/tmp/` に置く（`/tmp` は使わない）。
_VENUE_DIR = os.path.join(_WORKTREE_ROOT, '.briefs', 'tmp', 'hd_venue')
_HOME_ID = 'h1'
_HOME_XY = (0.0, 0.0)
_HOME_YAW = 0.0


def _prepare_venue_dir() -> None:
    os.makedirs(_VENUE_DIR, exist_ok=True)
    pins = {
        'pins': [{
            'id': _HOME_ID,
            'name': 'home',
            'kind': 'HOME',
            'pose': {'x': _HOME_XY[0], 'y': _HOME_XY[1], 'yaw': _HOME_YAW},
            'registered_at': 0,
        }],
        'map_instance_id': '',
    }
    with open(os.path.join(_VENUE_DIR, 'pins.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(pins, f, allow_unicode=True, default_flow_style=False)


_prepare_venue_dir()


def _tl_qos() -> QoSProfile:
    """`/onsite/pins`・`/onsite/home_declared` と同じ TRANSIENT_LOCAL。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)


# ===========================================================================
# launch: 本番ノードそのものを起動する
# ===========================================================================
@pytest.mark.launch_test
def generate_test_description():
    registrar = launch_ros.actions.Node(
        package='th_onsite',
        executable='pin_registrar.py',
        name='pin_registrar',
        parameters=[{'venue_dir': _VENUE_DIR}],
        output='screen',
    )
    declarer = launch_ros.actions.Node(
        package='th_onsite',
        executable='home_declarer.py',
        name='home_declarer',
        parameters=[{
            'home_declare_tolerance_m': _TOL_M,
            'home_declare_tolerance_deg': _TOL_DEG,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        registrar, declarer,
        launch_testing.actions.ReadyToTest(),
    ]), {'pin_registrar': registrar, 'home_declarer': declarer}


class TestHomeDeclarerNode(unittest.TestCase):
    """onsite §9 #10。メソッド名は ABC 順に走るが、各々が TF と宣言状態を
    自分で作り直すため順序に依存しない。"""

    # ── 起動／後始末 ──────────────────────────────────────────
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('hd_client')
        self._lock = threading.Lock()
        self._last_declared: bool | None = None
        self._robot_xy_yaw = (0.0, 0.0, 0.0)  # (x, y, yaw_deg)

        cbg = ReentrantCallbackGroup()
        self.node.create_subscription(
            Bool, '/onsite/home_declared', self._on_declared, _tl_qos(),
            callback_group=cbg)

        self.pub_tf = self.node.create_publisher(TFMessage, '/tf_static', _tl_qos())
        self._publish_tf()
        self._tf_timer = self.node.create_timer(1.0, self._publish_tf,
                                                callback_group=cbg)

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin,
                                             daemon=True)
        self._spin_thread.start()
        # 代役サーバを作り直す試験ごとに discovery が追いつくまで置く。
        time.sleep(2.0)

    def tearDown(self):
        try:
            self._executor.shutdown()
        except Exception:
            pass
        self._spin_thread.join(timeout=5.0)
        try:
            self.node.destroy_timer(self._tf_timer)
        except Exception:
            pass
        self.node.destroy_node()

    # ── 自己位置の代役（`/tf_static` の `map → base_link`） ──────────
    def _publish_tf(self):
        x, y, yaw_deg = self._robot_xy_yaw
        yaw = math.radians(yaw_deg)
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.pub_tf.publish(TFMessage(transforms=[t]))

    def _set_robot(self, x: float, y: float, yaw_deg: float):
        """自己位置を変える。tf2 バッファへの到達待ち（timer 再送ぶん）を含む。"""
        self._robot_xy_yaw = (x, y, yaw_deg)
        for _ in range(3):
            self._publish_tf()
            time.sleep(0.2)
        time.sleep(1.0)

    def _on_declared(self, msg: Bool):
        with self._lock:
            self._last_declared = bool(msg.data)

    # ── 補助 ──────────────────────────────────────────────────
    def _call_srv(self, srv_type, srv_name: str, req, timeout: float = 10.0):
        cli = self.node.create_client(srv_type, srv_name)
        try:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.fail(f'{srv_name} が 10s 待っても現れない')
            fut = cli.call_async(req)
            deadline = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not fut.done():
                self.fail(f'{srv_name} の応答が {timeout}s 来ない')
            return fut.result()
        finally:
            self.node.destroy_client(cli)

    def _declare(self, force: bool = False):
        req = DeclareHome.Request()
        req.force = force
        return self._call_srv(DeclareHome, '/onsite/declare_home', req)

    def _register_home_tap(self, tap1: tuple[float, float],
                           tap2: tuple[float, float]):
        """本番の口（`/onsite/register_pin` MAP_TAP）で HOME ピンを置き直す。

        `/onsite/edit_pin` は改名・削除しかできず座標を変えられないため、
        座標を変える正規の口は `register_pin`（MAP_TAP は TF 不要で決定的）。
        HOME は 1 個のみで置換される（`pin_registrar._place_pin_effect`）。
        """
        req = RegisterPin.Request()
        req.kind = 'HOME'
        req.name = 'home'
        req.method = 'MAP_TAP'
        req.tap1_x, req.tap1_y = float(tap1[0]), float(tap1[1])
        req.tap2_x, req.tap2_y = float(tap2[0]), float(tap2[1])
        res = self._call_srv(RegisterPin, '/onsite/register_pin', req)
        self.assertTrue(res.success, f'register_pin(MAP_TAP) が失敗: {res.message}')

    def _wait_declared(self, value: bool, timeout: float = 8.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._last_declared is value:
                    return True
            time.sleep(0.05)
        with self._lock:
            return self._last_declared is value

    def _ensure_undeclared(self):
        """宣言を `false`・HOME ピンを原点に戻す（各試験の入口の掃除）。

        ピンを遠方へ移して原点に戻す 2 段構え。姿勢が変わるたび
        `home_declarer` が宣言を `false` に戻す（同一姿勢の置き直しでは
        戻らないため、必ず異なる座標を経由する）。
        """
        self._register_home_tap((5.0, 0.0), (6.0, 0.0))
        self._register_home_tap((_HOME_XY[0], _HOME_XY[1]),
                                (_HOME_XY[0] + 1.0, _HOME_XY[1]))
        if not self._wait_declared(False, timeout=8.0):
            self.fail('HOME ピンを置き直しても home_declared が false に戻らない')

    def _mark_passed(self, note: str = ''):
        line = f'[hd] {self._testMethodName} PASSED {note}'.rstrip() + '\n'
        print(line, flush=True)

    # ═══════════════════════════════════════════════════════════════════
    # 1: 許容内（ピンから 0.1 m・5°）→ 成功・ずれ一致・declared=true
    # ═══════════════════════════════════════════════════════════════════
    def test_within_tolerance_declares(self):
        self._ensure_undeclared()
        self._set_robot(0.1, 0.0, 5.0)
        res = self._declare(force=False)
        self.assertTrue(res.success, f'許容内なのに失敗: {res.message}')
        self.assertAlmostEqual(res.offset_m, 0.1, delta=0.03,
                               msg=f'offset_m がずれている: {res.offset_m}')
        self.assertAlmostEqual(res.offset_deg, 5.0, delta=1.0,
                               msg=f'offset_deg がずれている: {res.offset_deg}')
        self.assertTrue(
            self._wait_declared(True),
            'declare 成功なのに /onsite/home_declared が true にならない')
        self._mark_passed(f'(offset={res.offset_m:.3f}m/{res.offset_deg:.2f}deg)')

    # ═══════════════════════════════════════════════════════════════════
    # 2: 位置のずれが大きい（ピンから 1.0 m）→ 失敗・declared=false のまま
    # ═══════════════════════════════════════════════════════════════════
    def test_position_offset_rejected(self):
        """§9 #10 の核心（設計書の「ピンから 1 m ずらして置く」に相当）。"""
        self._ensure_undeclared()
        self._set_robot(1.0, 0.0, 0.0)
        res = self._declare(force=False)
        self.assertFalse(res.success, '1.0 m ずれているのに成功した')
        self.assertAlmostEqual(res.offset_m, 1.0, delta=0.03,
                               msg=f'offset_m がずれている: {res.offset_m}')
        time.sleep(1.0)
        with self._lock:
            last = self._last_declared
        self.assertIs(last, False,
                      f'失敗したのに home_declared が false のままでない: {last}')
        self._mark_passed(f'(offset_m={res.offset_m:.3f})')

    # ═══════════════════════════════════════════════════════════════════
    # 3: 向きのずれが大きい（位置は同じ・30°）→ 失敗
    # ═══════════════════════════════════════════════════════════════════
    def test_yaw_offset_rejected(self):
        self._ensure_undeclared()
        self._set_robot(0.0, 0.0, 30.0)
        res = self._declare(force=False)
        self.assertFalse(res.success, '30° ずれているのに成功した')
        self.assertAlmostEqual(res.offset_deg, 30.0, delta=1.0,
                               msg=f'offset_deg がずれている: {res.offset_deg}')
        self._mark_passed(f'(offset_deg={res.offset_deg:.2f})')

    # ═══════════════════════════════════════════════════════════════════
    # 4: force=true なら 2 の状況でも成功・declared=true
    # ═══════════════════════════════════════════════════════════════════
    def test_force_declares_despite_offset(self):
        self._ensure_undeclared()
        self._set_robot(1.0, 0.0, 0.0)
        res = self._declare(force=True)
        self.assertTrue(res.success, f'force なのに失敗: {res.message}')
        self.assertTrue(
            self._wait_declared(True),
            'force 宣言が成功したのに /onsite/home_declared が true にならない')
        self._mark_passed(f'(offset_m={res.offset_m:.3f})')

    # ═══════════════════════════════════════════════════════════════════
    # 5: ピンが変わると宣言が解除される（本番の口 = register_pin MAP_TAP）
    # ═══════════════════════════════════════════════════════════════════
    def test_pin_change_clears_declaration(self):
        self._ensure_undeclared()
        self._set_robot(0.0, 0.0, 0.0)
        res = self._declare(force=False)
        self.assertTrue(res.success, f'宣言できない: {res.message}')
        self.assertTrue(self._wait_declared(True), '宣言後に true にならない')
        # HOME ピンを (2, 0) へ移す → 宣言が false に戻る。
        self._register_home_tap((2.0, 0.0), (3.0, 0.0))
        self.assertTrue(
            self._wait_declared(False),
            'HOME ピンを変えたのに home_declared が false に戻らない')
        # 移した先ではずれが検出される（成功せず offset ≈ 2.0 m）。
        res2 = self._declare(force=False)
        self.assertFalse(res2.success, 'ピンから 2 m 離れているのに成功した')
        self.assertAlmostEqual(res2.offset_m, 2.0, delta=0.05,
                               msg=f'offset_m がずれている: {res2.offset_m}')
        self._mark_passed('(register_pin MAP_TAP で HOME を (2,0) へ移動)')

    # ═══════════════════════════════════════════════════════════════════
    # 6: 許容値ちょうどの境界で成功／失敗が切り替わる
    # ═══════════════════════════════════════════════════════════════════
    def test_tolerance_boundary_switches(self):
        under = max(0.0, _TOL_M - 0.05)
        over = _TOL_M + 0.05
        self._ensure_undeclared()
        self._set_robot(under, 0.0, 0.0)
        res_ok = self._declare(force=False)
        self.assertTrue(
            res_ok.success,
            f'許容値の直前 ({under:.2f} m < {_TOL_M}) なのに失敗: {res_ok.message}')
        # いったん解除してから直後を試す（成功は解除しないため）。
        self._ensure_undeclared()
        self._set_robot(over, 0.0, 0.0)
        res_ng = self._declare(force=False)
        self.assertFalse(
            res_ng.success,
            f'許容値の直後 ({over:.2f} m > {_TOL_M}) なのに成功した')
        self._mark_passed(f'(tol={_TOL_M}m under={under:.2f} over={over:.2f})')


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
