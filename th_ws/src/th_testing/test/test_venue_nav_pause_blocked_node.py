"""
test_venue_nav_pause_blocked_node.py — 盤前移動の「一時停止→再開で経路を引き直さない」
（#3）・「BLOCKED がタイムアウトしない」（#4）の launch_testing 試験。

正本: `docs/plan/detailed/DetailedDesign-onsite.md` §9 受け入れ条件 #3・#4。

#3 の手段について設計書は「`FollowPath` の goal id が同一」と書いているが、
再開時の実装は `FollowPath` に**新しいゴールとして**送り直すため goal id は変わる。
本試験では「`ComputePathToPose` が呼ばれない」かつ「再送された経路が一時停止前と
同一」で判定する（この読み替え。設計書自体は直さない）。

===============================================================================
【Gazebo・Nav2 本体を使わない理由】
===============================================================================
`gazebo.launch.py` には `venue_navigator` も盤ピンも無く、実 Nav2 では
compute の成功／失敗と `FollowPath` の ABORT を試験から切り替えられない。
よって**本番ノード（`state_manager` / `pin_registrar` / `venue_navigator`）を
launch_testing で直接起動し、Nav2 の縁だけを代役で与える**。

===============================================================================
【外から入れてよいもの（これ以外は入れない）】
===============================================================================
- `/system/event` に `evt.link_ok` を 1 回 … 起動の前提（`INIT/CHECK → IDLE`）。
  **`evt.blocked` / `evt.unblocked` は試験から入れてはいけない**
  （`venue_navigator` が出すべきもの。直接出すと検証にならない）。
- `/tf_static` に `map → base_link` の恒等変換 … 到着判定の基準。
  ロボット (0,0)・ゴール (3,0) で距離 3.0m ≫ `arrival_xy_tol_m` (0.15) のため、
  到着フォールバックで抜けることはない。
- `compute_path_to_pose` の ActionServer（代役。成功／空経路の失敗を切替）。
- `follow_path` の ActionServer（代役。ゴールを記録し cancel を受け付ける。
  「走り続ける」／「ABORT を返す」を切替）。
- `ClearEntireCostmap` サービス 2 つ（即座に空応答）。
- `/system/trigger`（`UiTrigger`）と `/onsite/select_pin`（`GoToPanel`）…
  画面が叩くのと同じ経路。

**入れないもの**: `/system/state`（`state_manager` の出力。代役を立てると
FSM 検証が無意味になる）、`/system/effect` の publish（購読して観測はする）。
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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import StateEffect, StateEvent, SystemState
from th_system_msgs.srv import GoToPanel, UiTrigger


# ===========================================================================
# T-1: 判定に使う値は生成 YAML（registry.yaml 由来）から読む。直書きしない。
# （state_manager に渡す 5 値は case_07 と同じもの。試験の合否しきい値ではないが
# 本番と同じ起動条件にする。`blocked_recheck_period_s` だけはブリーフの指示で
# 短縮し、期待時間は必ずその上書き値から導く。）
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


_GENERATED = _find_generated_dir()


def _generated_param(yaml_name: str, node_name: str, key: str):
    path = os.path.join(_GENERATED, yaml_name)
    if not os.path.exists(path):
        pytest.fail(f'生成パラメータ {path} が無い。')
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    params = (data.get(node_name) or {}).get('ros__parameters') or {}
    if key not in params:
        pytest.fail(f'{path} に {node_name}.ros__parameters.{key} が無い。')
    return params[key]


_STATE_MGR_PARAMS = {
    k: _generated_param('state_manager.yaml', 'state_manager', k)
    for k in ('jog_lease_ms', 'link_wait_timeout_ms', 'ui_active_window_s',
              'screen_stale_ms', 'target_confidence_min')
}

# ── blocked 再探索周期の launch 上書き（ブリーフが明示的に許可） ─────────────
# registry の既定 2.0s では 20 周期の観測に 40 秒かかる。要件は「タイムアウト
# しないこと」であって 2.0 秒という長さではないため 0.5s に縮める。
# **期待値は必ずこの定数から導く**（`_OBSERVE_SEC` / `_MIN_RECHECKS`）。
_BLOCKED_PERIOD_S = 0.5
_OBSERVE_PERIODS = 20
_OBSERVE_SEC = _BLOCKED_PERIOD_S * _OBSERVE_PERIODS  # = 10.0s
# 20 周期ぶん回れば最大 20 回の再探索。初回クリア往復・DDS 収束・タイマジッタの
# 余裕として 7 割（14 回）を下限にする。これを下回れば再探索が止まっている。
_MIN_RECHECKS = 14

# ── 盤ピン（`pin_registrar` の `venue_dir` に置く `pins.yaml`） ────────────────
# 一時ファイルは worktree 内の `.briefs/tmp/` に置く（`/tmp` は使わない）。
_VENUE_DIR = os.path.join(_WORKTREE_ROOT, '.briefs', 'tmp', 'vnpb_venue')
_PIN_ID = 'p1'
_GOAL_XY = (3.0, 0.0)  # 原点のロボットから 3.0m（到着許容 0.15m より十分遠い）


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

# ── サービス呼び出しの retry 上限（case_07 と同じ考え方。DDS discovery の
# 後追着で応答が届かないことがあるため、「押せる状態になるまで押し直す」） ──
_SERVICE_RETRY_BUDGET_SEC = 30.0


def _tl_qos() -> QoSProfile:
    """`/system/state` と同じ TRANSIENT_LOCAL RELIABLE。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)


# ===========================================================================
# launch: 本番ノードそのものを起動する
# ===========================================================================
@pytest.mark.launch_test
def generate_test_description():
    state_mgr = launch_ros.actions.Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[_STATE_MGR_PARAMS],
        output='screen',
    )
    registrar = launch_ros.actions.Node(
        package='th_onsite',
        executable='pin_registrar.py',
        name='pin_registrar',
        parameters=[{'venue_dir': _VENUE_DIR}],
        output='screen',
    )
    navigator = launch_ros.actions.Node(
        package='th_onsite',
        executable='venue_navigator.py',
        name='venue_navigator',
        parameters=[{'blocked_recheck_period_s': _BLOCKED_PERIOD_S}],
        output='screen',
    )
    return launch.LaunchDescription([
        state_mgr, registrar, navigator,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_mgr, 'pin_registrar': registrar,
          'venue_navigator': navigator}


class TestVenueNavPauseBlocked(unittest.TestCase):
    """盤前移動 #3・#4。メソッドは ABC 順（blocked → pause → reroute）に走るが、
    各自が `ui.abort` 等で片付けてから入り直すため順序に依存しない。"""

    # ── 起動／後始末 ──────────────────────────────────────────
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('vnpb_client')
        self._lock = threading.Lock()
        self.states: list[tuple[str, str]] = []  # (mode, state) 受信履歴
        self.event_names: list[str] = []
        self.effect_names: list[str] = []
        # Nav2 代役の観測
        self.compute_count = 0
        self.follow_goals: list[list[tuple[float, float]]] = []  # 経路xy列
        self.follow_cancel_count = 0
        self.follow_open = 0  # 実行中のゴール数（accept 済・未 return）
        # 代役の振る舞い切替
        self.compute_mode = 'success'  # 'success' | 'fail'(空経路)
        self.follow_mode = 'run'       # 'run'(cancelまで保持) | 'abort'
        self.follow_abort_delay_s = 0.5
        # `FollowPath` の cancel 要求から結果（CANCELED）を返すまでの遅延。
        # 不具合 B の再現（取り消し結果の到着前にモード離脱）で 0.5s にする。
        # 既定 0.0（即時）。
        self.follow_cancel_result_delay_s = 0.0
        # `FollowPath` の accept 遅延（既定 0＝即時）。#3 の試験だけ 0.3s に
        # 上書きする。本物の Nav2 の accept は行動サーバの executor 経由で
        # 即時ではなく、負荷で数百 ms 遅れる。`resume_follow_path` effect は
        # `/system/state(NAV)` より先に届くため、accept が遅いと「再送したのに
        # handle がまだ無い」窓が確実に開き、`_on_state` の NAV 入口が
        # `_start_nav()` を二重に呼ぶ不具合（SM-3.1.2-056 違反）を確定的に踏む。
        # accept が速いと窓が閉じてしまい、不具合があっても緑になる弱い試験に
        # なる。常時遅延にしないのは、accept 待ちのゴールに cancel が届かない
        # （handle 未確定のため venue が cancel を送れない）まま片付けると
        # accept 完了時に stale な handle が残り、次の試験の NAV 入口を抑止
        # してしまうため。遅延は #3 の再開窓だけに限定する。
        self.follow_accept_delay_s = 0.0

        cbg = ReentrantCallbackGroup()
        self.node.create_subscription(
            SystemState, '/system/state', self._on_state, _tl_qos(),
            callback_group=cbg)
        self.node.create_subscription(
            StateEvent, '/system/event', self._on_event, 10,
            callback_group=cbg)
        self.node.create_subscription(
            StateEffect, '/system/effect', self._on_effect, 10,
            callback_group=cbg)

        self.pub_event = self.node.create_publisher(StateEvent, '/system/event', 10)
        self.pub_tf = self.node.create_publisher(TFMessage, '/tf_static', _tl_qos())
        self._publish_tf()
        self._tf_timer = self.node.create_timer(1.0, self._publish_tf,
                                                callback_group=cbg)

        self._compute_server = ActionServer(
            self.node, ComputePathToPose, 'compute_path_to_pose',
            execute_callback=self._exec_compute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _req: CancelResponse.ACCEPT,
            callback_group=cbg)
        self._follow_server = ActionServer(
            self.node, FollowPath, 'follow_path',
            execute_callback=self._exec_follow,
            goal_callback=self._on_follow_goal,
            cancel_callback=self._on_follow_cancel,
            callback_group=cbg)
        self.node.create_service(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
            lambda _req, res: res, callback_group=cbg)
        self.node.create_service(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
            lambda _req, res: res, callback_group=cbg)

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin,
                                            daemon=True)
        self._spin_thread.start()
        # 試験ごとに代役サーバを作り直すため、venue 側クライアントの discovery
        # が旧 endpoint を向いている隙に goal が吸われて消えることがある
        # （`server_is_ready` は真なのに届かない）。2s 置いてから手順に入る。
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

    # ── 代役（Nav2 の縁） ─────────────────────────────────────
    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0
        self.pub_tf.publish(TFMessage(transforms=[t]))

    def _on_state(self, msg: SystemState):
        with self._lock:
            self.states.append((msg.mode, msg.state))

    def _on_event(self, msg: StateEvent):
        with self._lock:
            self.event_names.append(msg.event)

    def _on_effect(self, msg: StateEffect):
        with self._lock:
            self.effect_names.append(msg.name)

    def _exec_compute(self, goal_handle):
        req = goal_handle.request
        with self._lock:
            self.compute_count += 1
            mode = self.compute_mode
        if mode == 'fail':
            # 空経路 → venue_navigator は compute_failed 扱いで evt.blocked。
            goal_handle.succeed()
            return ComputePathToPose.Result(path=Path())
        gx = float(req.goal.pose.position.x)
        gy = float(req.goal.pose.position.y)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(5):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = gx * i / 4.0
            p.pose.position.y = gy * i / 4.0
            path.poses.append(p)
        goal_handle.succeed()
        return ComputePathToPose.Result(path=path)

    def _on_follow_goal(self, _goal_req):
        time.sleep(self.follow_accept_delay_s)
        return GoalResponse.ACCEPT

    def _on_follow_cancel(self, _cancel_req):
        with self._lock:
            self.follow_cancel_count += 1
        return CancelResponse.ACCEPT

    def _exec_follow(self, goal_handle):
        with self._lock:
            self.follow_goals.append(
                [(float(p.pose.position.x), float(p.pose.position.y))
                 for p in goal_handle.request.path.poses])
            self.follow_open += 1
        try:
            return self._exec_follow_body(goal_handle)
        finally:
            with self._lock:
                self.follow_open -= 1

    def _exec_follow_body(self, goal_handle):
        def _wait_cancel(deadline: float) -> bool:
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    # B の再現用: 結果を遅らせて返す（要求自体は即時に受ける）。
                    time.sleep(self.follow_cancel_result_delay_s)
                    goal_handle.canceled()
                    return True
                time.sleep(0.05)
            return False

        if self.follow_mode == 'run':
            if _wait_cancel(time.monotonic() + 60.0):
                return FollowPath.Result()
            goal_handle.succeed()
            return FollowPath.Result()
        # 'abort': 一旦 NAV が観測できるよう少し保持してから ABORT。
        # venue_navigator は ABORTED → evt.blocked。
        if _wait_cancel(time.monotonic() + self.follow_abort_delay_s):
            return FollowPath.Result()
        goal_handle.abort()
        return FollowPath.Result()

    # ── 補助（試験スレッド側。executor が裏で回っているので sleep で待つ） ──
    def _sleep(self, sec: float):
        time.sleep(sec)

    def _mark_passed(self, note: str = ''):
        """メソッド終了の証跡。pytest が stdout を捕捉する（-s 無し）ため、
        コンソールだけでなくファイルにも残す。`VNPB_MARKER_FILE` 未設定なら
        何もしない（ホスト単体実行でも壊れない）。"""
        line = f'[vnpb] {self._testMethodName} PASSED {note}'.rstrip() + '\n'
        print(line, flush=True)
        path = os.environ.get('VNPB_MARKER_FILE', '')
        if not path:
            return
        try:
            with open(path, 'a', encoding='utf-8') as f:
                f.write(line)
        except OSError:
            pass

    def _snap_states(self) -> list[tuple[str, str]]:
        with self._lock:
            return list(self.states)

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

    def _n_compute(self) -> int:
        with self._lock:
            return self.compute_count

    def _n_follow(self) -> int:
        with self._lock:
            return len(self.follow_goals)

    def _n_cancel(self) -> int:
        with self._lock:
            return self.follow_cancel_count

    def _n_open(self) -> int:
        with self._lock:
            return self.follow_open

    def _snap_events(self) -> list[str]:
        with self._lock:
            return list(self.event_names)

    def _wait_count(self, get, target: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if get() >= target:
                return True
            time.sleep(0.05)
        return get() >= target

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
        """受理されるまで押し直す（人が画面を操作する時の挙動と同じ）。"""
        deadline = time.monotonic() + _SERVICE_RETRY_BUDGET_SEC
        last = '(応答なし)'
        while time.monotonic() < deadline:
            req = UiTrigger.Request()
            req.trigger = trigger
            req.arg_json = arg_json
            req.requester = 'vnpb_test'
            res = self._call_srv(UiTrigger, '/system/trigger', req)
            if res is not None and res.accepted:
                return res
            last = getattr(res, 'reject_reason_key', '(応答なし)') if res else last
            self._sleep(0.5)
        self.fail(f'/system/trigger({trigger}) が受理されない '
                  f'(最終 reject={last!r})')

    def _call_select_pin(self, panel_id: str = _PIN_ID):
        deadline = time.monotonic() + _SERVICE_RETRY_BUDGET_SEC
        while time.monotonic() < deadline:
            req = GoToPanel.Request()
            req.panel_id = panel_id
            res = self._call_srv(GoToPanel, '/onsite/select_pin', req)
            if res is not None and res.success:
                return res
            self._sleep(0.5)
        self.fail(f'/onsite/select_pin({panel_id}) が成功しない')

    def _publish_link_ok(self):
        ev = StateEvent()
        ev.header.stamp = self.node.get_clock().now().to_msg()
        ev.event = 'evt.link_ok'
        ev.source_node = 'vnpb_test'
        ev.arg_json = '{}'
        self.pub_event.publish(ev)

    def _reset_to_home(self):
        """PANEL_NAV 系に居れば `ui.abort` で AT_HOME に戻す。
        INIT/IDLE なら `evt.link_ok` で IDLE にする。既に AT_HOME/IDLE なら何もしない。
        戻り値: (mode, state)。

        `ui.abort` 後は cancel の到達も待つ。`/system/effect` と `/system/state`
        は別トピックで順序保証が無いため、AT_HOME の state だけ見て次に入ると
        venue 側の `cancel_follow_path` が未処理で古い `_follow_goal_handle` が
        残り、次の NAV 入口で `_start_nav()` が抑止されて compute が呼ばれない
        （何も起きないままになる）。代役が cancel を受けた＝effect が処理された
        証拠になる。送出済みゴールが無ければ待たない。
        """
        ms = self._mode_state()
        if ms is None:
            self._sleep(0.5)
            ms = self._mode_state()
        if ms is None or ms[0] == 'INIT':
            self._publish_link_ok()
            if not self._wait_mode_state('IDLE', 'NONE', timeout=5.0):
                self.fail(f'evt.link_ok 後に IDLE にならない ({self._mode_state()})')
            return ('IDLE', 'NONE')
        if ms[0] in ('PANEL_NAV', 'SUMMON', 'HOME_NAV'):
            with self._lock:
                c0 = self.follow_cancel_count
                live = self.follow_open > 0
            self._call_trigger('ui.abort')
            if not self._wait_mode_state('AT_HOME', 'IDLE_H', timeout=5.0):
                self.fail(f'ui.abort 後に AT_HOME にならない ({self._mode_state()})')
            if live and not self._wait_count(
                    self._n_cancel, c0 + 1, timeout=5.0):
                self.fail('ui.abort の cancel が代役に届かない。'
                          'venue が cancel_follow_path を処理していない。')
            # accept 応答の飛行中（最大 follow_accept_delay_s）＋ effect/state の
            # 到着順不定の猶予。ここを待たず次に入ると venue 側に handle だけが
            # 残り、次の NAV 入口の _start_nav が抑止される。
            self._sleep(self.follow_accept_delay_s + 0.3)
            return ('AT_HOME', 'IDLE_H')
        return ms

    def _enter_nav(self):
        """`select_pin` → `ui.goto{kind:PANEL}`。IDLE/AT_HOME のどちらからでも入る。"""
        ms = self._mode_state()
        if ms is None or ms == ('INIT', 'CHECK'):
            self._reset_to_home()
        self._call_select_pin()
        self._call_trigger('ui.goto', '{"kind":"PANEL"}')

    # ═══════════════════════════════════════════════════════════════════
    # #4: BLOCKED がタイムアウトしない
    # ═══════════════════════════════════════════════════════════════════
    def test_blocked_does_not_time_out(self):
        """`DetailedDesign-onsite.md` §9 #4。

        `FollowPath` ABORT（初回）→ 以降 compute 失敗のまま、`BLOCKED` が
        `blocked_recheck_period_s`（上書き 0.5s）の 20 周期ぶん落ちず、
        再探索（compute）が止まらない。陽性対照で compute 成功に戻すと
        `evt.unblocked` で NAV に復帰する。
        """
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'fail'
            self.follow_mode = 'abort'
            self.compute_count = 0
            self.follow_goals.clear()
            self.states.clear()
            self.event_names.clear()

        self._enter_nav()
        # 初回は compute 失敗で直接 BLOCKED（follow は呼ばれないこともある）。
        if not self._wait_mode_state('PANEL_NAV', 'BLOCKED', timeout=10.0):
            self.fail(f'BLOCKED に入らない ({self._mode_state()})。'
                      f'compute={self._n_compute()} follow={self._n_follow()}')

        # ここから 20 周期ぶん観測する。
        with self._lock:
            mark = len(self.states)
            c0 = self.compute_count
        self._sleep(_OBSERVE_SEC)
        window = self._snap_states()[mark:]
        self.assertTrue(
            window,
            f'観測窓 ({_OBSERVE_SEC}s) に /system/state を1通も受信していない。')
        bad = [s for s in window if s != ('PANEL_NAV', 'BLOCKED')]
        self.assertEqual(
            bad, [],
            f'観測窓に PANEL_NAV/BLOCKED 以外が {len(bad)} 通混ざった（先頭: {bad[:5]}）.'
            f'タイムアウト・誤遷移の疑い。')
        dc = self._n_compute() - c0
        self.assertGreaterEqual(
            dc, _MIN_RECHECKS,
            f'観測窓の compute 回数が {dc} 回（下限 {_MIN_RECHECKS} = '
            f'{_OBSERVE_PERIODS} 周期の 7 割）。再探索が止まっている。')

        # ── 陽性対照: compute 成功に戻すと unblocked で NAV に復帰 ──
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            n_ev = len(self.event_names)
        if not self._wait_mode_state('PANEL_NAV', 'NAV', timeout=10.0):
            self.fail(f'compute 成功に戻しても NAV に復帰しない ({self._mode_state()})')
        with self._lock:
            new_events = self.event_names[n_ev:]
        self.assertIn(
            'evt.unblocked', new_events,
            'NAV に復帰したのに evt.unblocked が出ていない（遷移の根拠が無い）。')

        # 後片付け（保持中の FollowPath ゴールを cancel して終わる。
        # 置いたまま tearDown すると代役 ActionServer の execute が
        # 終了時エラーを吐く）
        self._reset_to_home()
        self._mark_passed(f'(compute_total={self._n_compute()})')

    # ═══════════════════════════════════════════════════════════════════
    # #3: PAUSE → 再開で再プランが走らない（SM-3.1.2-056「同じ経路の続きから。
    # 再検索はしない」）
    # ═══════════════════════════════════════════════════════════════════
    def test_pause_resume_does_not_replan(self):
        """`DetailedDesign-onsite.md` §9 #3。

        `ui.stop`（PAUSE）→ `ui.run`（NAV）で `resume_follow_path` がキャッシュ
        経路を再送するため、`ComputePathToPose` は増えず、2 件目の `FollowPath`
        ゴールは 1 件目と同一経路。
        """
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            self.follow_accept_delay_s = 0.3
            self.compute_count = 0
            self.follow_goals.clear()
            self.follow_cancel_count = 0

        self._enter_nav()
        if not self._wait_mode_state('PANEL_NAV', 'NAV', timeout=10.0):
            self.fail(f'NAV に入らない ({self._mode_state()})')
        self.assertTrue(
            self._wait_count(self._n_compute, 1, timeout=10.0),
            'NAV に入ったのに compute が 1 回も呼ばれない。')
        self.assertTrue(
            self._wait_count(self._n_follow, 1, timeout=10.0),
            'compute したのに FollowPath にゴールが届かない。')
        with self._lock:
            path_a = list(self.follow_goals[0])
        self.assertTrue(path_a, '1 件目の経路が空。代役が壊れている。')
        self.assertEqual(
            self._n_compute(), 1,
            f'NAV 突入時点の compute が 1 回でない ({self._n_compute()})。')

        # ── 一時停止: FollowPath が cancel される ──
        self._call_trigger('ui.stop')
        if not self._wait_mode_state('PANEL_NAV', 'PAUSE', timeout=5.0):
            self.fail(f'PAUSE に入らない ({self._mode_state()})')
        self.assertTrue(
            self._wait_count(self._n_cancel, 1, timeout=5.0),
            'ui.stop したのに代役 FollowPath に cancel が届かない。'
            'cancel_follow_path effect が効いていない。')
        self.assertEqual(
            self._n_compute(), 1,
            f'PAUSE で compute が増えた ({self._n_compute()})。'
            'cancel 時に再計算が走っている。')

        # ── 再開: キャッシュ再送のみ。compute は増えない・経路は同一 ──
        self._call_trigger('ui.run')
        if not self._wait_mode_state('PANEL_NAV', 'NAV', timeout=5.0):
            self.fail(f'再開後に NAV に戻らない ({self._mode_state()})')
        self.assertTrue(
            self._wait_count(self._n_follow, 2, timeout=10.0),
            '再開後に FollowPath に 2 件目のゴールが届かない。'
            'resume_follow_path effect が効いていない。')
        with self._lock:
            path_b = list(self.follow_goals[1])
        self.assertEqual(
            len(path_b), len(path_a),
            f'再送経路の点数が変わった ({len(path_a)} → {len(path_b)})。')
        for i, (pa, pb) in enumerate(zip(path_a, path_b)):
            self.assertAlmostEqual(
                pa[0], pb[0], places=9,
                msg=f'再送経路の {i} 点目の x が変わった ({pa} → {pb})。')
            self.assertAlmostEqual(
                pa[1], pb[1], places=9,
                msg=f'再送経路の {i} 点目の y が変わった ({pa} → {pb})。')
        # 再送後にしばらく待っても compute が増えない（再プラン禁止の核心）。
        self._sleep(3.0)
        self.assertEqual(
            self._n_compute(), 1,
            f'再開後に compute が増えた (1 → {self._n_compute()})。'
            'SM-3.1.2-056「再検索はしない」に反する再プランが走っている。')

        # 後片付け
        self._reset_to_home()
        self._mark_passed(f'(compute_total={self._n_compute()})')

    # ═══════════════════════════════════════════════════════════════════
    # 不具合 A: 受け付け前の一時停止で、取り消しが空振りして走り出す
    # ═══════════════════════════════════════════════════════════════════
    def test_cancel_before_accept_cancels_late_accept(self):
        """不具合 A（安全側。`_send_follow_path` → accept の窓で `ui.stop`）。

        代役 `follow_path` の受け付けを 1.0s 遅らせ、`compute` が飛んだ直後
        （受け付け前）に `ui.stop` → `PAUSE` する。正しくは受け付けのあと
        代役のゴールが取り消される（`PAUSE` の間に実行中ゴールが残らない）。
        今のコードは handle が `None` のため何も取り消せず、受け付け後に
        走行が続く（本番の不具合で赤。次のコミットで直す）。
        """
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            self.follow_accept_delay_s = 1.0
            self.follow_cancel_result_delay_s = 0.0
            self.compute_count = 0
            self.follow_goals.clear()
            self.follow_cancel_count = 0
            self.event_names.clear()
        try:
            self._enter_nav()
            # compute が飛んだ＝送信が飛んだ直後。accept（1.0s 遅延）の前。
            self.assertTrue(
                self._wait_count(self._n_compute, 1, timeout=10.0),
                'NAV に入ったのに compute が 1 回も呼ばれない。')
            # 送信が代役に届く猶予。accept 遅延 1.0s の窓の中に収める。
            self._sleep(0.2)
            with self._lock:
                n_ev = len(self.event_names)
            self._call_trigger('ui.stop')
            if not self._wait_mode_state('PANEL_NAV', 'PAUSE', timeout=5.0):
                self.fail(f'PAUSE に入らない ({self._mode_state()})')

            # 受け付け自体は届く（窓の確認）。
            self.assertTrue(
                self._wait_count(self._n_follow, 1, timeout=5.0),
                '代役に FollowPath ゴールが届かない。accept 遅延の窓が開いていない。')
            # 受け付けのあと、代役のゴールが取り消される。
            self.assertTrue(
                self._wait_count(self._n_cancel, 1, timeout=5.0),
                '受け付け前の ui.stop が、受け付け後のゴールに届いていない。'
                'cancel が空振りして走行が続いている（不具合 A）。')
            # PAUSE の間に実行中ゴールが残らない。
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and self._n_open() > 0:
                time.sleep(0.05)
            self.assertEqual(
                self._n_open(), 0,
                'PAUSE なのに代役のゴールが実行中のまま残っている（不具合 A）。')
            # 遅れて取り消した結果で偽の evt.blocked を出さない（変異 3 の検出）。
            self._sleep(1.0)
            new_events = self._snap_events()[n_ev:]
            self.assertNotIn(
                'evt.blocked', new_events,
                '自分で取り消した結果で evt.blocked が出ている。')
            self.assertEqual(
                self._mode_state(), ('PANEL_NAV', 'PAUSE'),
                f'PAUSE のはずが {self._mode_state()} に動いた。')
            self._mark_passed(f'(cancel={self._n_cancel()})')
        finally:
            with self._lock:
                self.follow_accept_delay_s = 0.0
            self._reset_to_home()

    # ═══════════════════════════════════════════════════════════════════
    # 不具合 B: 取り消し中にモードを離れると、あとから偽の evt.blocked が出る
    # ═══════════════════════════════════════════════════════════════════
    def test_exit_during_cancel_no_spurious_blocked(self):
        """不具合 B（`_reset_for_exit` が `_cancel_requested` を無条件に下ろす）。

        走行中（受け付け済み）に `ui.abort` → `AT_HOME`。代役は取り消しの
        結果を 0.5s 遅らせて返す（effect の cancel と state の離脱の順序差で
        `_reset_for_exit` が結果到着前に走る窓を開ける）。正しくは離脱後に
        `evt.blocked` が出ない。今のコードは印を消すため CANCELED を他人分
        と誤判定して `evt.blocked` を出し、`_blocked` が残る（本番の不具合で
        赤。次のコミットで直す）。
        """
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            self.follow_accept_delay_s = 0.0
            self.follow_cancel_result_delay_s = 0.5
            self.compute_count = 0
            self.follow_goals.clear()
            self.follow_cancel_count = 0
            self.event_names.clear()
        try:
            self._enter_nav()
            if not self._wait_mode_state('PANEL_NAV', 'NAV', timeout=10.0):
                self.fail(f'NAV に入らない ({self._mode_state()})')
            self.assertTrue(
                self._wait_count(self._n_follow, 1, timeout=10.0),
                'FollowPath にゴールが届かない（受け付け済みにできない）。')
            with self._lock:
                n_ev = len(self.event_names)
                c0 = self.follow_cancel_count

            # 走行中に離脱。cancel が投げられ、結果は 0.5s 後に届く。
            self._call_trigger('ui.abort')
            if not self._wait_mode_state('AT_HOME', 'IDLE_H', timeout=5.0):
                self.fail(f'ui.abort 後に AT_HOME にならない ({self._mode_state()})')
            self.assertTrue(
                self._wait_count(self._n_cancel, c0 + 1, timeout=5.0),
                'ui.abort の cancel が代役に届かない。')
            # 遅れた取り消し結果が届くまで待ち、偽の blocked が無いこと。
            self._sleep(2.0)
            new_events = self._snap_events()[n_ev:]
            self.assertNotIn(
                'evt.blocked', new_events,
                '離脱後の取り消し結果で偽の evt.blocked が出ている（不具合 B）。')

            # もう一度 NAV に入る。再探索周期を待たずにすぐ compute が走る
            # （`_blocked` が残っていれば入口で抑止され、再探索経由の遠回り）。
            with self._lock:
                n_ev2 = len(self.event_names)
                c1 = self.compute_count
            self._call_select_pin()
            self._call_trigger('ui.goto', '{"kind":"PANEL"}')
            if not self._wait_mode_state('PANEL_NAV', 'NAV', timeout=5.0):
                self.fail(f'再突入で NAV に入らない ({self._mode_state()})')
            t_nav = time.monotonic()
            t_comp = None
            deadline = t_nav + 5.0
            while time.monotonic() < deadline:
                if self._n_compute() >= c1 + 1:
                    t_comp = time.monotonic()
                    break
                time.sleep(0.05)
            self.assertIsNotNone(
                t_comp,
                '再突入後に compute が走らない（_blocked 残留で入口抑止の疑い）。')
            elapsed = t_comp - t_nav
            self.assertLess(
                elapsed, _BLOCKED_PERIOD_S,
                f'再突入の compute が {elapsed:.2f}s 後（周期 {_BLOCKED_PERIOD_S}s '
                '以上）。再探索タイマ経由の遠回りになっている。')
            new_events2 = self._snap_events()[n_ev2:]
            self.assertNotIn(
                'evt.unblocked', new_events2,
                '再突入で evt.unblocked が出ている（再探索経由で復帰した証拠）。')
            self._mark_passed(f'(reentry_compute_lag={elapsed:.2f}s)')
        finally:
            with self._lock:
                self.follow_cancel_result_delay_s = 0.0
            self._reset_to_home()

    # ═══════════════════════════════════════════════════════════════════
    # 不具合 A の出口経路版: 送出中にモード離脱（ESTOP 相当）→ 受け付け後に
    # _reset_for_exit 経由で取り消される
    # ═══════════════════════════════════════════════════════════════════
    def test_exit_during_send_cancels_late_accept(self):
        """送出～accept の窓で `ui.estop.press` → `ESTOP`（`_reset_for_exit`）。

        ESTOP 突入は venue に effect を送らない（`latch_prev` のみ）ため、
        `_cancel_follow_path` は何もできず、`_reset_for_exit` だけが頼り。
        正しくは受け付けのあと代役のゴールが取り消される。印が無ければ
        受け付け後に走行が続く（本番の不具合で赤。修正と一緒に直す）。
        """
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'run'
            self.follow_accept_delay_s = 1.0
            self.follow_cancel_result_delay_s = 0.0
            self.compute_count = 0
            self.follow_goals.clear()
            self.follow_cancel_count = 0
            self.event_names.clear()
        try:
            self._enter_nav()
            # compute が飛んだ＝送信が飛んだ直後。accept（1.0s 遅延）の前。
            self.assertTrue(
                self._wait_count(self._n_compute, 1, timeout=10.0),
                'NAV に入ったのに compute が 1 回も呼ばれない。')
            # 送信が代役に届く猶予。accept 遅延 1.0s の窓の中に収める。
            self._sleep(0.2)
            with self._lock:
                n_ev = len(self.event_names)
                c0 = self.follow_cancel_count
            # モード離脱（ESTOP。venue への effect は無い）。
            self._call_trigger('ui.estop.press')
            if not self._wait_mode_state('ESTOP', 'NONE', timeout=5.0):
                self.fail(f'ESTOP に入らない ({self._mode_state()})')

            # 受け付け自体は届く（窓の確認）。
            self.assertTrue(
                self._wait_count(self._n_follow, 1, timeout=5.0),
                '代役に FollowPath ゴールが届かない。accept 遅延の窓が開いていない。')
            # 受け付けのあと、代役のゴールが取り消される。
            self.assertTrue(
                self._wait_count(self._n_cancel, c0 + 1, timeout=5.0),
                '送出中のモード離脱が、受け付け後のゴールに届いていない。'
                '離脱後に走行が続いている（_reset_for_exit の取りこぼし）。')
            # 離脱後に実行中ゴールが残らない。
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and self._n_open() > 0:
                time.sleep(0.05)
            self.assertEqual(
                self._n_open(), 0,
                'ESTOP なのに代役のゴールが実行中のまま残っている。')
            # 遅れて取り消した結果で偽の evt.blocked を出さない。
            self._sleep(1.0)
            new_events = self._snap_events()[n_ev:]
            self.assertNotIn(
                'evt.blocked', new_events,
                '自分で取り消した結果で evt.blocked が出ている。')
            self.assertEqual(
                self._mode_state(), ('ESTOP', 'NONE'),
                f'ESTOP のはずが {self._mode_state()} に動いた。')
            self._mark_passed(f'(cancel={self._n_cancel()})')
        finally:
            with self._lock:
                self.follow_accept_delay_s = 0.0
            # ESTOP 復帰: release → resume_yes で PANEL_NAV/PAUSE に戻し、
            # あとはいつもの abort で AT_HOME へ（release 直行 IDLE の場合も
            # _reset_to_home がそのまま受け止める）。
            try:
                self._call_trigger('ui.estop.release')
                self._sleep(0.5)
                if self._mode_state() is not None and self._mode_state()[0] == 'ESTOP':
                    self._call_trigger('ui.resume_yes')
                    self._sleep(0.5)
            except Exception:
                pass
            self._reset_to_home()

    # ═══════════════════════════════════════════════════════════════════
    # 陽性対照（#3 の裏）: ui.reroute → replan で compute が増える
    # ═══════════════════════════════════════════════════════════════════
    def test_reroute_replans(self):
        """`T-PNAV-07` の `replan` effect で compute が走る。

        これが付かないと、#3 の「compute が増えない」が「compute を数えられて
        いない」せいに見分けが付かない。compute が数えられていることの証拠。
        """
        self._reset_to_home()
        with self._lock:
            self.compute_mode = 'success'
            self.follow_mode = 'abort'
            self.follow_abort_delay_s = 0.5
            self.compute_count = 0
            self.follow_goals.clear()
            self.effect_names.clear()

        self._enter_nav()
        # follow が一旦 NAV で走り、ABORT で BLOCKED に落ちる。
        if not self._wait_mode_state('PANEL_NAV', 'NAV', timeout=10.0):
            self.fail(f'NAV に入らない ({self._mode_state()})')
        if not self._wait_mode_state('PANEL_NAV', 'BLOCKED', timeout=10.0):
            self.fail(f'BLOCKED に入らない ({self._mode_state()})')

        c0 = self._n_compute()
        with self._lock:
            n_fx = len(self.effect_names)
        self._call_trigger('ui.reroute')
        self.assertTrue(
            self._wait_count(self._n_compute, c0 + 1, timeout=5.0),
            f'ui.reroute 後に compute が増えない ({c0} → {self._n_compute()})。'
            f'replan effect が compute に繋がっていない。')
        with self._lock:
            new_effects = self.effect_names[n_fx:]
        self.assertIn(
            'replan', new_effects,
            'compute は増えたのに replan effect が観測できない（根拠が無い）。')

        # 後片付け（次の試験のため AT_HOME に戻す）
        self._reset_to_home()
        self._mark_passed(f'(compute_total={self._n_compute()})')


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
