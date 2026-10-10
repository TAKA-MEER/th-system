"""
test_venue_nav_tf_stall_node.py — Nav2 TF 停滞の検知（記録だけ）
launch_testing 試験（2026-10-10 実機の follow_path 連続打ち切り）。

実機で、Nav2 の `controller_server` の TF バッファの map→odom が止まったまま
（`/tf` 自体は新しい）、約 10 秒ごとに `Failed to make progress` で follow_path
が打ち切られ（status=6）、`venue_navigator` は 12 秒周期で blocked→再送→
unblocked を繰り返して回復しなかった。止める・再起動する動作はまだ入れない
（記録だけ）。旧版は `/rosout` の `Transform data too old`（tf_help ロガー）
を見ていたが、ノードに紐づかないロガーは `/rosout` に流れないため実機で
発火しなかった。本試験は作り直した信号（A: 打ち切りの連続＋位置不動、
B: `/rosout` の controller_server の進捗失敗）を本番ノードで縛る。

本番ノード（state_manager / pin_registrar / venue_navigator）を起動し、Nav2
の代役（ComputePathToPose と、**中断だけを返し続ける FollowPath**）で実機と
同じ系列（follow_path の打ち切り、位置がほぼ不動、TF 新しい）を作る。
判定の閾値は本番の値（3 回連続・20 cm・progress 併用）を直接使い、周期だけ
縮める（blocked 再探索 0.3 秒・打ち切り 0.4 秒）。

  - 3 回連続の打ち切り（静止・TF 新・progress 有り）→ `evt.nav_tf_stall`
  - 打ち切りを繰り返しても機体が進んでいれば → 出ない（誤検知の防止）
  - 単発の打ち切り → 出ない
  - `/tf` ごと古ければ → 出ない（SLAM 側を見る）
  - progress の裏付けが無ければ → 出ない（B の併用を縛る）
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

# ── 周期だけ縮める。判定の閾値（3 回・20 cm・progress 併用）は本番の値 ──
_BLOCKED_PERIOD_S = 0.3
_ABORT_DELAY_S = 0.4

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
        self._robot_x = 0.0  # 既定は静止（原点）
        self._stale_tf = False  # True: /tf の map→odom を古い時刻で出す
        self._send_progress = True  # False: B の裏付け無しを再現
        self._abort_budget: int | None = None  # None: 常に打ち切り

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
        # 実機では controller が約 10 秒ごとに `Failed to make progress` で
        # 打ち切った。代役も打ち切りの直前に同じ文言をノード名
        # `controller_server` で /rosout へ流す（実機と同じ経路）。
        with self._lock:
            self.follow_calls += 1
            send_progress = self._send_progress
            if self._abort_budget is not None:
                do_abort = self._abort_budget > 0
                if do_abort:
                    self._abort_budget -= 1
            else:
                do_abort = True
        if send_progress:
            msg = Log()
            msg.stamp = self.node.get_clock().now().to_msg()
            msg.level = 40
            msg.name = 'controller_server'
            msg.msg = 'Failed to make progress (代役の進捗失敗)'
            self.pub_rosout.publish(msg)
        time.sleep(_ABORT_DELAY_S)
        if do_abort:
            goal_handle.abort()
        else:
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
        now_ns = self.node.get_clock().now().nanoseconds
        if self._stale_tf:
            now_ns -= 30_000_000_000
        stamp = rclpy.time.Time(nanoseconds=now_ns).to_msg()
        od = TransformStamped()
        od.header.stamp = stamp
        od.header.frame_id = 'map'
        od.child_frame_id = 'odom'
        od.transform.rotation.w = 1.0
        base = TransformStamped()
        base.header.stamp = stamp
        base.header.frame_id = 'odom'
        base.child_frame_id = 'base_link'
        base.transform.translation.x = float(self._robot_x)
        base.transform.rotation.w = 1.0
        self.pub_tf.publish(TFMessage(transforms=[od, base]))

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

    def _follow_count(self) -> int:
        with self._lock:
            return self.follow_calls

    def _wait_follow_calls(self, n: int, timeout: float = 20.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._follow_count() >= n:
                return True
            time.sleep(0.05)
        return self._follow_count() >= n

    def _stall_arg_since(self, since: int) -> dict | None:
        with self._lock:
            for name, arg in zip(self.event_names[since:],
                                 self.event_args[since:]):
                if name == 'evt.nav_tf_stall':
                    return json.loads(arg or '{}')
        return None

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

    def _reset_quietly(self):
        try:
            self._reset_to_home()
        except Exception:
            pass

    def test_repeated_aborts_emit_stall(self):
        """打ち切りの連続（静止・TF 新・progress 有り）で出る（記録だけ）。"""
        try:
            since = self._enter_nav()
            self.assertTrue(self._wait_follow_calls(1),
                            'FollowPath が呼ばれない')
            self.assertTrue(
                self._wait_event('evt.nav_tf_stall', since, timeout=30.0),
                '打ち切りの連続なのに evt.nav_tf_stall が出ない')
            arg = self._stall_arg_since(since)
            self.assertIsNotNone(arg, 'evt.nav_tf_stall の引数が無い')
            assert arg is not None
            self.assertTrue(arg.get('wire_fresh', False),
                            f'/tf が新しいのに wire_fresh でない ({arg})')
            self.assertGreaterEqual(arg.get('abort_count', 0), 3,
                                    f'打ち切り回数が足りない ({arg})')
            self.assertGreaterEqual(arg.get('progress_count', 0), 1,
                                    f'progress の裏付けが無い ({arg})')
            self.assertLess(arg.get('moved_cm', 9999.0), 20.0,
                            f'静止なのに moved_cm が大きい ({arg})')
        finally:
            self._reset_quietly()

    def test_moving_between_aborts_does_not_emit(self):
        """打ち切りを繰り返しても機体が進んでいれば出ない（誤検知の防止）。

        判定の直前だけ動かすのでは足りない（止まった後の打ち切りが 3 回
        積み上がると出るのが正しい挙動）。観測窓の全体で動かし続ける。
        0.4 秒ごとに 0.3 m ずつ進めば、打ち切り間隔（0.7 秒以上）より密に
        錨が動くため、3 回連続の静止はあり得ない。
        """
        try:
            with self._lock:
                self._robot_x = -4.0
            since = self._enter_nav()
            self.assertTrue(self._wait_follow_calls(1),
                            'FollowPath が呼ばれない')
            for _ in range(15):
                time.sleep(0.4)
                with self._lock:
                    self._robot_x += 0.3
            self.assertNotIn('evt.nav_tf_stall', self._snap_events()[since:],
                             '進んでいるのに evt.nav_tf_stall が出た')
        finally:
            self._reset_quietly()

    def test_single_abort_does_not_emit(self):
        """単発の打ち切りでは出ない（回数の閾値を縛る）。"""
        try:
            with self._lock:
                self._abort_budget = 1
            since = self._enter_nav()
            self.assertTrue(
                self._wait_event('evt.blocked', since, timeout=15.0),
                '打ち切り後の evt.blocked が出ない')
            self._reset_to_home()
            time.sleep(3.0)
            self.assertNotIn('evt.nav_tf_stall', self._snap_events()[since:],
                             '単発の打ち切りなのに evt.nav_tf_stall が出た')
        finally:
            self._reset_quietly()

    def test_stale_wire_does_not_emit(self):
        """`/tf` ごと古ければ出ない（SLAM 側を見る）。

        tf2 の「最新」は受信順ではなく時刻印順なので、古い時刻印を出し
        始めた直後は setUp 時の新しい見本が「最新」として残る。11 秒待って
        追い出して（tf2 の保持は 10 秒）から NAV に入る。
        """
        try:
            with self._lock:
                self._stale_tf = True
            time.sleep(11.0)
            since = self._enter_nav()
            self.assertTrue(self._wait_follow_calls(4),
                            'FollowPath の再送が続かない')
            time.sleep(3.0)
            self.assertNotIn('evt.nav_tf_stall', self._snap_events()[since:],
                             '/tf が古いのに evt.nav_tf_stall が出た')
        finally:
            self._reset_quietly()

    def test_aborts_without_progress_does_not_emit(self):
        """progress の裏付けが無ければ出ない（B の併用を縛る）。"""
        try:
            with self._lock:
                self._send_progress = False
            since = self._enter_nav()
            self.assertTrue(self._wait_follow_calls(4),
                            'FollowPath の再送が続かない')
            time.sleep(3.0)
            self.assertNotIn('evt.nav_tf_stall', self._snap_events()[since:],
                             'progress 無しなのに evt.nav_tf_stall が出た')
        finally:
            self._reset_quietly()
