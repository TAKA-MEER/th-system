"""
test_venue_nav_arrival_tol_node.py — 到着許容差の一本化（SG-B15・1b-11①）の
launch_testing 試験。

`venue_navigator` の到着判定（`venue_nav_core.arrived`）が、生成 yaml
（= registry の `arrival_xy_tol_m`）の値で動くこと。境界の内外・ちょうど境界
（`<` の厳密さ。`<=` に緩めると赤くなる）を見る。

正本: `docs/plan/spec/Spec-onsite.md` §6.2（0.12 m / 0.12 rad の 1 つの値）。

===============================================================================
【本番の経路】
===============================================================================
本番ノード（`state_manager` / `pin_registrar` / `venue_navigator`）を
launch_testing で直接起動する。`venue_navigator` には**生成 yaml を先に渡し、
`blocked_recheck_period_s` だけ dict で上書きする**（rclpy は上書き順に読む。
生成 yaml が正で dict は試験用の短縮）。
Nav2 の代役は立てない（compute 未 ready → `evt.blocked` の実経路を通り、
到着圏内なら recheck が `evt.unblocked` で NAV に戻して到着フォールバックが
`evt.arrived` を出す。`evt.blocked` / `evt.unblocked` / `evt.arrived` は
いずれも venue が出すべきものなので試験からは送らない）。

===============================================================================
【外から入れてよいもの（これ以外は入れない）】
===============================================================================
- `/system/event` に `evt.link_ok` … 起動の前提（`INIT/CHECK → IDLE`）。
- `/tf_static` に `map → base_link` … ロボット位置。到着判定の基準。
  距離 d は生成 yaml の tol から導く（tol±0.01・ちょうど tol）。直書きしない。
- `/system/trigger`（`UiTrigger`）と `/onsite/select_pin`（`GoToPanel`）…
  画面が叩くのと同じ経路。

**入れないもの**: `/system/state`（`state_manager` の出力）、`/system/effect` の
publish、`evt.blocked` / `evt.unblocked` / `evt.arrived` の直送。
"""

from __future__ import annotations

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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import StateEvent, SystemState
from th_system_msgs.srv import GoToPanel, UiTrigger


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


_GENERATED = _find_generated_dir()
_VENUE_YAML = os.path.join(_GENERATED, 'venue_navigator.yaml')


def _generated_tol() -> float:
    """判定の合否しきい値。生成 yaml（registry 由来）から読む。直書きしない。"""
    if not os.path.exists(_VENUE_YAML):
        pytest.fail(f'生成パラメータ {_VENUE_YAML} が無い。')
    with open(_VENUE_YAML, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    params = (data.get('venue_navigator') or {}).get('ros__parameters') or {}
    if 'arrival_xy_tol_m' not in params:
        pytest.fail(f'{_VENUE_YAML} に arrival_xy_tol_m が無い。')
    return float(params['arrival_xy_tol_m'])


_TOL = _generated_tol()
# 境界の内外。厳密な `<` なので、ちょうど tol では到着にならない。
_D_OUT = _TOL + 0.01
_D_IN = _TOL - 0.01

# ── 盤ピン（`pin_registrar` の `venue_dir` に置く `pins.yaml`） ────────────────
# 一時ファイルは worktree 内の `.briefs/tmp/` に置く（`/tmp` は使わない）。
_VENUE_DIR = os.path.join(_WORKTREE_ROOT, '.briefs', 'tmp', 'vnat_venue')
_PIN_ID = 'p1'
_GOAL_XY = (3.0, 0.0)


def _prepare_venue_dir() -> None:
    os.makedirs(_VENUE_DIR, exist_ok=True)
    pins = {
        'pins': [{
            'id': _PIN_ID,
            'name': 'panel1',
            'kind': 'PANEL',
            'pose': {'x': _GOAL_XY[0], 'y': _GOAL_XY[1], 'yaw': 0.0},
            'registered_at': 0,
        }],
        'map_instance_id': '',
    }
    with open(os.path.join(_VENUE_DIR, 'pins.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(pins, f, allow_unicode=True, default_flow_style=False)


_prepare_venue_dir()

# ── blocked 再探索周期の launch 上書き ─────────────────────────────────────
# registry の既定 2.0s では到着→unblocked の観測に時間がかかる。要件は到着判定
# の境界であって周期の長さではないため 0.3s に縮める。
_BLOCKED_PERIOD_S = 0.3

_SERVICE_RETRY_BUDGET_SEC = 30.0


def _tl_qos() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)


# ===========================================================================
# launch: 本番ノードそのものを起動する（生成 yaml を先に渡す）
# ===========================================================================
@pytest.mark.launch_test
def generate_test_description():
    state_mgr = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[os.path.join(_GENERATED, 'state_manager.yaml')],
        output='screen',
    )
    registrar = launch_ros.actions.Node(
        package='th_onsite',
        executable='pin_registrar.py',
        name='pin_registrar',
        parameters=[os.path.join(_GENERATED, 'pin_registrar.yaml'),
                    {'venue_dir': _VENUE_DIR}],
        output='screen',
    )
    navigator = launch_ros.actions.Node(
        package='th_onsite',
        executable='venue_navigator.py',
        name='venue_navigator',
        parameters=[_VENUE_YAML,
                    {'blocked_recheck_period_s': _BLOCKED_PERIOD_S}],
        output='screen',
    )
    return launch.LaunchDescription([
        state_mgr, registrar, navigator,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_mgr, 'pin_registrar': registrar,
          'venue_navigator': navigator}


class TestVenueNavArrivalTol(unittest.TestCase):
    """到着判定の境界。メソッドは ABC 順に走るが、各自入り直すため順序非依存。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('vnat_client')
        self._lock = threading.Lock()
        self.states: list[tuple[str, str]] = []
        self.event_names: list[str] = []
        self._robot_x = _GOAL_XY[0] - _D_OUT  # 既定は圏外

        cbg = ReentrantCallbackGroup()
        self.node.create_subscription(
            SystemState, '/system/state', self._on_state, _tl_qos(),
            callback_group=cbg)
        self.node.create_subscription(
            StateEvent, '/system/event', self._on_event, 10,
            callback_group=cbg)
        self.pub_event = self.node.create_publisher(StateEvent, '/system/event', 10)
        self.pub_tf = self.node.create_publisher(TFMessage, '/tf_static', _tl_qos())
        self._publish_tf()
        self._tf_timer = self.node.create_timer(1.0, self._publish_tf,
                                                callback_group=cbg)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin,
                                             daemon=True)
        self._spin_thread.start()
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

    # ── 購読・発行 ──────────────────────────────────────────
    def _on_state(self, msg: SystemState):
        with self._lock:
            self.states.append((msg.mode, msg.state))

    def _on_event(self, msg: StateEvent):
        with self._lock:
            self.event_names.append(msg.event)

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self._robot_x)
        t.transform.rotation.w = 1.0
        self.pub_tf.publish(TFMessage(transforms=[t]))

    def _set_distance(self, d: float):
        """ゴールの d [m] 手前にロボットを置く（即時再送して TF を更新）。"""
        with self._lock:
            self._robot_x = _GOAL_XY[0] - d
        self._publish_tf()

    # ── 待ち・呼び出し ──────────────────────────────────────
    def _mode_state(self) -> tuple[str, str] | None:
        with self._lock:
            return self.states[-1] if self.states else None

    def _wait_mode_state(self, mode: str, state: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._mode_state() == (mode, state):
                return True
            time.sleep(0.05)
        return self._mode_state() == (mode, state)

    def _snap_events(self) -> list[str]:
        with self._lock:
            return list(self.event_names)

    def _wait_event(self, name: str, since: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if name in self._snap_events()[since:]:
                return True
            time.sleep(0.05)
        return name in self._snap_events()[since:]

    def _call_srv(self, srv_type, srv_name: str, req, timeout: float = 8.0):
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

    def _call_trigger(self, trigger: str, arg_json: str = '{}'):
        deadline = time.monotonic() + _SERVICE_RETRY_BUDGET_SEC
        while time.monotonic() < deadline:
            req = UiTrigger.Request()
            req.trigger = trigger
            req.arg_json = arg_json
            req.requester = 'vnat_test'
            res = self._call_srv(UiTrigger, '/system/trigger', req)
            if res is not None and res.accepted:
                return res
            time.sleep(0.5)
        self.fail(f'/system/trigger({trigger}) が受理されない')

    def _publish_link_ok(self):
        ev = StateEvent()
        ev.header.stamp = self.node.get_clock().now().to_msg()
        ev.event = 'evt.link_ok'
        ev.source_node = 'vnat_test'
        ev.arg_json = '{}'
        self.pub_event.publish(ev)

    def _reset_to_home(self):
        ms = self._mode_state()
        if ms is None:
            time.sleep(0.5)
            ms = self._mode_state()
        if ms is None or ms[0] == 'INIT':
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                self._publish_link_ok()
                if self._wait_mode_state('IDLE', 'NONE', timeout=0.6):
                    return
            self.fail(f'evt.link_ok 後に IDLE にならない ({self._mode_state()})')
            return
        if ms[0] in ('PANEL_NAV', 'SUMMON', 'HOME_NAV'):
            self._call_trigger('ui.abort')
            if not self._wait_mode_state('AT_HOME', 'IDLE_H', timeout=10.0):
                self.fail(f'ui.abort 後に AT_HOME にならない ({self._mode_state()})')

    def _enter_nav(self):
        since0 = len(self._snap_events())
        ms = self._mode_state()
        if ms is None or ms == ('INIT', 'CHECK'):
            self._reset_to_home()
        deadline = time.monotonic() + _SERVICE_RETRY_BUDGET_SEC
        while time.monotonic() < deadline:
            req = GoToPanel.Request()
            req.panel_id = _PIN_ID
            res = self._call_srv(GoToPanel, '/onsite/select_pin', req)
            if res is not None and res.success:
                break
            time.sleep(0.5)
        else:
            self.fail('/onsite/select_pin が成功しない')
        self._call_trigger('ui.goto', '{"kind":"PANEL"}')
        # Nav2 の代役が居ないので NAV はすぐ BLOCKED に移る（高負荷だと NAV を
        # 観測できずに BLOCKED だけ見えることがある）。PANEL_NAV に入ったことは
        # 状態を問わず、到着が既に出ていればそれも「入った」とみなす。
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            ms = self._mode_state()
            if (ms is not None and ms[0] == 'PANEL_NAV') or \
                    'evt.arrived' in self._snap_events()[since0:]:
                return since0
            time.sleep(0.05)
        self.fail(f'PANEL_NAV に入らない ({self._mode_state()})')

    # ═══════════════════════════════════════════════════════════════════
    # 境界の外 → 到着にならない／境界の内 → なる／ちょうど境界 → ならない
    # ═══════════════════════════════════════════════════════════════════
    def test_arrival_inside_tol(self):
        """tol - 0.01 m で evt.arrived が出る（生成 yaml の値で判定している）。"""
        self._set_distance(_D_IN)
        since = self._enter_nav()
        if not self._wait_event('evt.arrived', since, timeout=15.0):
            self.fail(f'tol-{0.01} m で evt.arrived が出ない '
                      f'(tol={_TOL}, events={self._snap_events()[since:]})')
        self._reset_to_home()

    def test_arrival_on_boundary_is_not_arrived(self):
        """ちょうど tol では到着にならない（判定は厳密な `<`）。"""
        self._set_distance(_TOL)
        self._enter_nav()
        since = len(self._snap_events())
        time.sleep(5.0)
        news = self._snap_events()[since:]
        self.assertNotIn('evt.arrived', news,
                         f'ちょうど tol で到着になった (tol={_TOL})')
        self._reset_to_home()

    def test_arrival_outside_tol(self):
        """tol + 0.01 m で evt.arrived が出ない。"""
        self._set_distance(_D_OUT)
        self._enter_nav()
        since = len(self._snap_events())
        time.sleep(5.0)
        news = self._snap_events()[since:]
        self.assertNotIn('evt.arrived', news,
                         f'tol+{0.01} m で到着になった (tol={_TOL})')
        self._reset_to_home()
