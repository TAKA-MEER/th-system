"""
test_venue_nav_align_min_w_node.py — その場旋回の速さの下限（A-5。2026-11-10）を
`venue_navigator` の本番の経路で縛る launch_testing 試験。

Spec-onsite.md §6.2.1: 向き合わせは、回すときは常にモーターの不感帯より上の
速さ（`turn_in_place_w_min_rps` = 0.40 rad/s 以上）で回し、目標の向きの許容に
入ったら 0 にして止める。比例制御のまま最後に指令を細らせると不感帯に入って
回らない（2026-11-10 実機）。

正本: Spec-onsite.md §6.2.1 ／ Spec-params.md「その場旋回の速さの下限」

===============================================================================
【本番の経路】
===============================================================================
本番ノード（`state_manager` / `pin_registrar` / `venue_navigator`）を
launch_testing で直接起動する。`venue_navigator` には**生成 yaml を先に渡し、
`turn_in_place_w_min_rps` だけ dict で 0.42 に上書きする**（rclpy は上書き順に
読む。生成 yaml が正で dict は試験用）。出力が 0.40 ではなく 0.42 になること
で、ノードがパラメータを本当に読んでいること（既定値の直書きでないこと）を見る。

===============================================================================
【外から入れてよいもの（これ以外は入れない）】
===============================================================================
- `/system/event` に `evt.link_ok` … 起動の前提（`INIT/CHECK → IDLE`）。
- `/tf_static` に `map → base_link` … ロボット位置・向き。到着判定の基準。
- `/system/trigger`（`UiTrigger`）と `/onsite/select_pin`（`GoToPanel`）…
  画面が叩くのと同じ経路。

**入れないもの**: `/system/state`（`state_manager` の出力）、`/system/effect` の
publish、`evt.arrived` / `evt.align_done` の直送。
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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import TransformStamped, Twist
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


def _generated_param(key: str) -> float:
    if not os.path.exists(_VENUE_YAML):
        pytest.fail(f'生成パラメータ {_VENUE_YAML} が無い。')
    with open(_VENUE_YAML, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    params = (data.get('venue_navigator') or {}).get('ros__parameters') or {}
    if key not in params:
        pytest.fail(f'{_VENUE_YAML} に {key} が無い。')
    return float(params[key])


_TOL = _generated_param('align_tolerance_rad')
_ARRIVAL_TOL = _generated_param('arrival_xy_tol_m')
# ノードが生成 yaml（0.40）ではなく上書き値（0.42）を読んだことを見分ける。
_MIN_W_OVERRIDE = 0.42
# 許容のすぐ外（比例だけなら 1.2*0.10=0.12 で不感帯）。
_ERR_OUT = _TOL + 0.01
_ERR_IN = _TOL - 0.02
_D_NEAR = _ARRIVAL_TOL - 0.01

# ── 盤ピン（`pin_registrar` の `venue_dir` に置く `pins.yaml`） ────────────────
_VENUE_DIR = os.path.join(_WORKTREE_ROOT, '.briefs', 'tmp', 'vnam_venue')
_PIN_OUT = 'p_out'      # 向きのずれ 0.10 rad（許容の外）
_PIN_IN = 'p_in'        # 向きのずれ 0（許容の中）
_PIN_POSE = {
    _PIN_OUT: (3.0, 1.0, _ERR_OUT),
    _PIN_IN: (3.0, -1.0, 0.0),
}


def _prepare_venue_dir() -> None:
    os.makedirs(_VENUE_DIR, exist_ok=True)
    pins = {
        'pins': [{
            'id': pin_id,
            'name': pin_id,
            'kind': 'PANEL',
            'pose': {'x': x, 'y': y, 'yaw': yaw},
            'registered_at': 0,
        } for pin_id, (x, y, yaw) in _PIN_POSE.items()],
        'map_instance_id': '',
    }
    with open(os.path.join(_VENUE_DIR, 'pins.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(pins, f, allow_unicode=True, default_flow_style=False)


_prepare_venue_dir()

_SERVICE_RETRY_BUDGET_SEC = 30.0


def _tl_qos() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)


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
                    {'turn_in_place_w_min_rps': _MIN_W_OVERRIDE}],
        output='screen',
    )
    return launch.LaunchDescription([
        state_mgr, registrar, navigator,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_mgr, 'pin_registrar': registrar,
          'venue_navigator': navigator}


class TestVenueNavAlignMinW(unittest.TestCase):
    """回すときの速さの下限と、許容に入ったら 0 になること。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('vnam_client')
        self._lock = threading.Lock()
        self.states: list[tuple[str, str]] = []
        self.event_names: list[str] = []
        self.cmds: list[float] = []
        self._robot = (0.0, 0.0, 0.0)

        cbg = ReentrantCallbackGroup()
        self.node.create_subscription(
            SystemState, '/system/state', self._on_state, _tl_qos(),
            callback_group=cbg)
        self.node.create_subscription(
            StateEvent, '/system/event', self._on_event, 10,
            callback_group=cbg)
        self.node.create_subscription(
            Twist, '/cmd_vel_behavior', self._on_cmd, 10,
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

    def _on_cmd(self, msg: Twist):
        with self._lock:
            self.cmds.append(float(msg.angular.z))

    def _publish_tf(self):
        yaw = self._robot[2]
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self._robot[0])
        t.transform.translation.y = float(self._robot[1])
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.pub_tf.publish(TFMessage(transforms=[t]))

    def _set_robot_near(self, pin_id: str):
        x, y, _ = _PIN_POSE[pin_id]
        self._robot = (x, y - _D_NEAR, 0.0)
        self._publish_tf()

    # ── 待ち・呼び出し ──────────────────────────────────────
    def _mode_state(self):
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
            req.requester = 'vnam_test'
            res = self._call_srv(UiTrigger, '/system/trigger', req)
            if res is not None and res.accepted:
                return res
            time.sleep(0.5)
        self.fail(f'/system/trigger({trigger}) が受理されない')

    def _publish_link_ok(self):
        ev = StateEvent()
        ev.header.stamp = self.node.get_clock().now().to_msg()
        ev.event = 'evt.link_ok'
        ev.source_node = 'vnam_test'
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

    def _enter_align(self, pin_id: str):
        """選んだピンへ「走行」し、到着後の ALIGN に入る。

        venue_navigator が /cmd_vel_behavior に出すのは ALIGN 中だけなので、
        呼び出し時点で cmds を空にすれば、以降はこの向き合わせの指令だけになる。
        """
        ms = self._mode_state()
        if ms is None or ms == ('INIT', 'CHECK'):
            self._reset_to_home()
        with self._lock:
            self.cmds.clear()
        self._set_robot_near(pin_id)
        deadline = time.monotonic() + _SERVICE_RETRY_BUDGET_SEC
        while time.monotonic() < deadline:
            req = GoToPanel.Request()
            req.panel_id = pin_id
            res = self._call_srv(GoToPanel, '/onsite/select_pin', req)
            if res is not None and res.success:
                break
            time.sleep(0.5)
        else:
            self.fail(f'/onsite/select_pin({pin_id}) が成功しない')
        self._call_trigger('ui.goto', '{"kind":"PANEL"}')
        if not self._wait_mode_state('PANEL_NAV', 'ALIGN', timeout=20.0):
            self.fail(f'PANEL_NAV/ALIGN に入らない ({self._mode_state()})')

    # ═══════════════════════════════════════════════════════════════════
    # 許容の外 → 下限以上（上書きした 0.42）で回る／許容の中 → 0
    # ═══════════════════════════════════════════════════════════════════
    def test_a_outside_tol_keeps_min_speed(self):
        """許容のすぐ外（0.10 > 0.09）では 0.42 rad/s 以上・符号は err と同じ。"""
        self._enter_align(_PIN_OUT)
        time.sleep(1.5)
        with self._lock:
            cmds = list(self.cmds)
        assert cmds, '/cmd_vel_behavior に何も出ていない'
        nonzero = [w for w in cmds if abs(w) > 1e-9]
        assert nonzero, '回す指令が出ていない'
        assert all(w > 0.0 for w in nonzero), f'符号が err と違う: {nonzero[:5]}'
        assert all(abs(w) >= _MIN_W_OVERRIDE - 1e-6 for w in nonzero), (
            f'下限 {_MIN_W_OVERRIDE} 未満の指令がある（ノードがパラメータを'
            f'読まず既定値 0.40 を使っている疑い）: {sorted(set(nonzero))[:5]}')
        assert any(abs(w) >= _MIN_W_OVERRIDE - 1e-6 for w in nonzero)
        # 0.40 ではなく上書き値 0.42 に追従していること（生成 yaml を読んでいる）。
        assert all(abs(abs(w) - _MIN_W_OVERRIDE) < 1e-6 for w in nonzero), (
            f'指令が上書き値 {_MIN_W_OVERRIDE} と違う: {sorted(set(nonzero))[:5]}')
        self._reset_to_home()

    def test_b_inside_tol_stops(self):
        """許容の中（0 < 0.09）では 0 にして止まる。"""
        self._enter_align(_PIN_IN)
        since = len(self._snap_events())
        assert self._wait_event('evt.align_done', since, timeout=10.0), (
            f'許容内なのに evt.align_done が出ない (events={self._snap_events()[since:]})')
        time.sleep(0.5)
        with self._lock:
            cmds = list(self.cmds)
        assert cmds, '/cmd_vel_behavior に停止指令が出ていない'
        assert all(abs(w) < 1e-9 for w in cmds), f'許容内で回した: {cmds}'


if __name__ == '__main__':
    unittest.main()
