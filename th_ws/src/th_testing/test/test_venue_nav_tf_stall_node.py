"""
test_venue_nav_tf_stall_node.py — Nav2 controller の TF 停滞の検知（記録だけ）
launch_testing 試験（2026-10-10 実機の `Transform data too old`）。

実機で、Nav2 の `controller_server` の TF バッファの map→odom が約 2 時間
止まったまま（`/tf` 自体は新しい）、`Transform data too old ... Data time`
固定・`Transform time` 固定のまま `follow_path` が約 10 秒ごとに打ち切られ、
`venue_navigator` は 12 秒周期で再送を繰り返して回復しなかった。止める・
再起動する動作はまだ入れない（記録だけ）。本試験はその検知が出ることと、
単発の `too old`（一過性）では出ないことを確かめる。

本番ノード（state_manager / pin_registrar / venue_navigator）を起動し、Nav2
の代役（ComputePathToPose と、受け付けたまま終わらない FollowPath）だけ
試験が務める。controller の `too old` は `/rosout` の代役発行で再現する。
  - `/rosout` に controller の `too old` が短時間に集中 → `evt.nav_tf_stall`
  - 単発（しきい値未満）では出ない
"""
from __future__ import annotations

import json
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
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)

from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Log
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import StateEvent, SystemState
from th_system_msgs.srv import GoToPanel, UiTrigger
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


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

# ── 盤ピン（`pin_registrar` の `venue_dir` に置く `pins.yaml`） ────────────────
# 一時ファイルは worktree 内の `.briefs/tmp/` に置く（`/tmp` は使わない）。
_VENUE_DIR = os.path.join(_WORKTREE_ROOT, '.briefs', 'tmp', 'vnat_tfstall_venue')
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

# ── 検知しきい値の launch 上書き ─────────────────────────────────────
# 本番の既定（30 件/30 秒）では試験が長い。要件は「集中すれば出る・単発では
# 出ない」の境界なので、小さい値に縮める。
_STALL_COUNT = 5
_STALL_WINDOW_S = 5.0
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
                    {'blocked_recheck_period_s': _BLOCKED_PERIOD_S,
                     'nav_tf_stall_count': _STALL_COUNT,
                     'nav_tf_stall_window_s': _STALL_WINDOW_S}],
        output='screen',
    )
    return launch.LaunchDescription([
        state_mgr, registrar, navigator,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_mgr, 'pin_registrar': registrar,
          'venue_navigator': navigator}


class TestVenueNavTfStall(unittest.TestCase):
    """Nav2 TF 停滞の検知。メソッドは各自入り直すため順序非依存。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('vnat_tfstall_client')
        self._lock = threading.Lock()
        self.states: list[tuple[str, str]] = []
        self.event_names: list[str] = []
        self.event_args: list[str] = []
        self._robot_x = _GOAL_XY[0] - 3.0  # 既定は圏外（原点）

        cbg = ReentrantCallbackGroup()
        self.node.create_subscription(
            SystemState, '/system/state', self._on_state, _tl_qos(),
            callback_group=cbg)
        self.node.create_subscription(
            StateEvent, '/system/event', self._on_event, 10,
            callback_group=cbg)
        self.pub_event = self.node.create_publisher(StateEvent, '/system/event', 10)
        self.pub_tf = self.node.create_publisher(TFMessage, '/tf', 10)
        self.pub_tf_static = self.node.create_publisher(
            TFMessage, '/tf_static', _tl_qos())
        self.pub_rosout = self.node.create_publisher(Log, '/rosout', 10)
        self._publish_tf()
        self._tf_timer = self.node.create_timer(0.1, self._publish_tf,
                                                callback_group=cbg)
        self.follow_calls = 0
        self._follow_stop = threading.Event()
        self._compute_srv = ActionServer(
            self.node, ComputePathToPose, 'compute_path_to_pose',
            self._exec_compute, callback_group=cbg)
        self._follow_srv = ActionServer(
            self.node, FollowPath, 'follow_path', self._exec_follow,
            callback_group=cbg)
        self._executor = MultiThreadedExecutor(num_threads=6)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin,
                                             daemon=True)
        self._spin_thread.start()
        time.sleep(2.0)

    def tearDown(self):
        self._follow_stop.set()
        try:
            self._executor.shutdown()
        except Exception:
            pass
        self._spin_thread.join(timeout=10.0)
        try:
            self.node.destroy_timer(self._tf_timer)
        except Exception:
            pass
        self.node.destroy_node()

    # ── Nav2・TF・rosout の代役 ───────────────────────────────────
    def _exec_compute(self, goal_handle):
        res = ComputePathToPose.Result()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x in (0.0, _GOAL_XY[0]):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = path.header.stamp
            ps.pose.position.x = float(x)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        res.path = path
        goal_handle.succeed()
        return res

    def _exec_follow(self, goal_handle):
        # 実機では controller が 10 秒ごとに打ち切った。ここでは検知の窓の
        # あいだ goal を生かしたままにする（打ち切り後の再送でも同じ）。
        with self._lock:
            self.follow_calls += 1
        self._follow_stop.wait(timeout=30.0)
        goal_handle.succeed()
        return FollowPath.Result()

    def _on_state(self, msg: SystemState):
        with self._lock:
            self.states.append((msg.mode, msg.state))

    def _on_event(self, msg: StateEvent):
        with self._lock:
            self.event_names.append(msg.event)
            self.event_args.append(msg.arg_json)

    def _publish_tf(self):
        now = self.node.get_clock().now().to_msg()
        od = TransformStamped()
        od.header.stamp = now
        od.header.frame_id = 'map'
        od.child_frame_id = 'odom'
        od.transform.rotation.w = 1.0
        base = TransformStamped()
        base.header.stamp = now
        base.header.frame_id = 'odom'
        base.child_frame_id = 'base_link'
        base.transform.translation.x = float(self._robot_x)
        base.transform.rotation.w = 1.0
        self.pub_tf.publish(TFMessage(transforms=[od, base]))

    def _burst_too_old(self, n: int):
        """controller の `too old`（map→odom）の代役を n 件出す。

        実機のロガー名は `tf_help`（nav2 の nav_2d_utils）。検知は名前で
        絞らないため、代役も実機と同じ名前にして本番の経路を縛る。
        """
        for _ in range(n):
            msg = Log()
            msg.stamp = self.node.get_clock().now().to_msg()
            msg.level = 40
            msg.name = 'tf_help'
            msg.msg = ('Transform data too old when converting from map to odom '
                       '(Data time / Transform time は省略)')
            self.pub_rosout.publish(msg)
            time.sleep(0.1)

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
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            ms = self._mode_state()
            if (ms is not None and ms[0] == 'PANEL_NAV') or \
                    'evt.arrived' in self._snap_events()[since0:]:
                return since0
            time.sleep(0.05)
        self.fail(f'PANEL_NAV に入らない ({self._mode_state()})')

    def _follow_count(self) -> int:
        with self._lock:
            return self.follow_calls

    def _wait_follow_active(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._follow_count() >= 1:
                return True
            time.sleep(0.05)
        return self._follow_count() >= 1

    def _reset_quietly(self):
        try:
            self._reset_to_home()
        except Exception:
            pass

    def test_burst_emits_nav_tf_stall(self):
        """`too old` が集中すれば `evt.nav_tf_stall` が出る（記録だけ）。"""
        try:
            since = self._enter_nav()
            self.assertTrue(self._wait_follow_active(),
                            'FollowPath が呼ばれない')
            self._burst_too_old(_STALL_COUNT + 3)
            self.assertTrue(
                self._wait_event('evt.nav_tf_stall', since, timeout=15.0),
                'too old の集中なのに evt.nav_tf_stall が出ない')
            with self._lock:
                idx = self.event_names.index('evt.nav_tf_stall')
                arg = json.loads(self.event_args[idx] or '{}')
            self.assertTrue(arg.get('wire_fresh', False),
                            f'/tf が新しいのに wire_fresh でない ({arg})')
        finally:
            self._reset_quietly()

    def test_single_too_old_is_ignored(self):
        """単発（しきい値未満）では `evt.nav_tf_stall` は出ない。"""
        try:
            since = self._enter_nav()
            self.assertTrue(self._wait_follow_active(),
                            'FollowPath が呼ばれない')
            self._burst_too_old(2)
            time.sleep(8.0)
            self.assertNotIn('evt.nav_tf_stall', self._snap_events()[since:],
                             '単発の too old なのに evt.nav_tf_stall が出た')
        finally:
            self._reset_quietly()
