"""
_blocked_hold_harness.py — 「塞がれた経路は、先が空くまで走り直さない」
（Spec-onsite §6.3）の launch_testing 試験の共通部品。

本番ノード（`state_manager` / `pin_registrar` / `venue_navigator`）を
`launch_testing` で起動し、Nav2 の縁だけを代役で与える。形は
`test_venue_nav_pause_blocked_node.py` と同じで、足したのは障害物マップ
（`/global_costmap/costmap`）の代役だけ。

障害物マップの代役は**実機と同じ形**:
- QoS は RELIABLE + TRANSIENT_LOCAL（Nav2 Humble の costmap 発行側。
  `ros2 topic info -v` で実測 2026-10-12）。
- 座標系 `map`・解像度 0.05・全体を 1 Hz で配信（`always_send_full_costmap`）。
- 経路上に致死(100)／内接(99)の障害物帯を置ける（`set_obstacle`）。

`ComputePathToPose` は経路を返す。`FollowPath` は受け付けて数秒後に中断
（ABORTED = status 6）を返す（実機で障害物に当たった形）。
外から入れてよいものは pause_blocked 試験と同じ（`evt.link_ok` ・`/tf_static`・
Nav2 の代役・UI のトリガ）。`evt.blocked`／`evt.unblocked` は入れない。
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
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import Log
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import FaultStatus, StateEffect, StateEvent, SystemState
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

# ── 盤ピン（`pin_registrar` の `venue_dir` に置く `pins.yaml`） ────────────────
# 一時ファイルは worktree 内の `.briefs/tmp/` に置く（`/tmp` は使わない）。
_VENUE_DIR = os.path.join(_WORKTREE_ROOT, '.briefs', 'tmp', 'vnbh_venue')
_PIN_ID = 'p1'
_GOAL_XY = (3.0, 0.0)  # 原点のロボットから 3.0m（到着許容 0.15m より十分遠い）
_HOME_ID = 'home1'
_HOME_XY = (-3.0, 0.0)  # HOME_NAV 用。PANEL 系の試験には触らない


def _prepare_venue_dir() -> None:
    os.makedirs(_VENUE_DIR, exist_ok=True)
    pins = {
        'pins': [{
            'id': _PIN_ID,
            'name': 'panel1',
            'kind': 'PANEL',
            'pose': {'x': _GOAL_XY[0], 'y': _GOAL_XY[1], 'yaw': 0.0},
            'registered_at': 0,
        }, {
            'id': _HOME_ID,
            'name': 'home',
            'kind': 'HOME',
            'pose': {'x': _HOME_XY[0], 'y': _HOME_XY[1], 'yaw': 0.0},
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



_BLOCKED_PERIOD_S_DEFAULT = 0.5


def make_description(period_s: float, extra_params: dict | None = None):
    """本番ノードを起動する launch。`blocked_recheck_period_s` だけ上書きする。"""
    state_mgr = launch_ros.actions.Node(
        package='th_state', executable='state_manager.py',
        name='state_manager', parameters=[_STATE_MGR_PARAMS], output='screen')
    registrar = launch_ros.actions.Node(
        package='th_onsite', executable='pin_registrar.py',
        name='pin_registrar', parameters=[{'venue_dir': _VENUE_DIR}],
        output='screen')
    navigator = launch_ros.actions.Node(
        package='th_onsite', executable='venue_navigator.py',
        name='venue_navigator',
        parameters=[{'blocked_recheck_period_s': period_s,
                     **(extra_params or {})}], output='screen')
    return launch.LaunchDescription([
        state_mgr, registrar, navigator,
        launch_testing.actions.ReadyToTest(),
    ]), {'state_manager': state_mgr, 'pin_registrar': registrar,
          'venue_navigator': navigator}


class BlockedHoldBase(unittest.TestCase):

    # ── 起動／後始末 ──────────────────────────────────────────
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('vnbh_client')
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
        self.follow_abort_delay_s = 3.0   # 受け付けて数秒後に中断（実機の形）
        # `FollowPath` の cancel 要求から結果（CANCELED）を返すまでの遅延。
        # 不具合 B の再現（取り消し結果の到着前にモード離脱）で 0.5s にする。
        # 既定 0.0（即時）。
        self.follow_cancel_result_delay_s = 0.0
        # `ComputePathToPose` の実行遅延（既定 0＝即時）。離脱時破棄の試験だけ
        # 2.0s にする。カウントを先に増やしてから眠るので、試験側は
        # compute==1 を見て「計算が飛行中」を確定的に掴める。
        self.compute_result_delay_s = 0.0
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
        # 衝突による打ち切りの再現（Spec-onsite §6.3）。立てると、ABORT の直前に
        # 実機と同じ形（ノード名 controller_server・ERROR）で RPP の衝突のログを
        # /rosout に流す。follow_times は FollowPath を受けた時刻（monotonic）。
        self.follow_collision = False
        self.follow_times: list[float] = []
        self.robot_x = 0.0

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
        self.pub_rosout = self.node.create_publisher(Log, '/rosout', 10)
        self.pub_fault = self.node.create_publisher(FaultStatus, '/safety/fault', 10)
        # `/onsite/summon_goal` は venue 側が TRANSIENT_LOCAL で購読している。
        # 既定 QoS（VOLATILE）では非互換で届かず、NAV 入口でゴール無し
        # → evt.blocked になる。
        self.pub_summon = self.node.create_publisher(
            PoseStamped, '/onsite/summon_goal', _tl_qos())
        self.pub_tf = self.node.create_publisher(TFMessage, '/tf_static', _tl_qos())
        self._publish_tf()
        self._tf_timer = self.node.create_timer(1.0, self._publish_tf,
                                                callback_group=cbg)

        # Spec-onsite §6.3: 障害物マップの代役。実機の発行側（Nav2 costmap）と同じ
        # RELIABLE + TRANSIENT_LOCAL で、経路上が空いた全体を 1 Hz で流す。
        # 「塞がれた経路は、先が空くまで再送しない」ので、再送を期待する試験には要る。
        # 既定は空き。`set_obstacle` で経路上に障害物帯を置く。
        self.costmap_obstacle = None   # (x0, x1, 値) 例 (1.0, 1.2, 100)
        self.costmap_auto = True       # False なら 1 Hz の自動配信を止める
        self.clear_call_times: list[float] = []  # コストマップ消去が呼ばれた時刻
        self._pub_costmap = self.node.create_publisher(
            OccupancyGrid, '/global_costmap/costmap', _tl_qos())
        self._costmap_timer = self.node.create_timer(
            1.0, self._costmap_tick, callback_group=cbg)
        self._publish_costmap()

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
            self._on_clear, callback_group=cbg)
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
        t.transform.translation.x = float(self.robot_x)
        t.transform.rotation.w = 1.0
        self.pub_tf.publish(TFMessage(transforms=[t]))

    def _on_clear(self, _req, res):
        with self._lock:
            self.clear_call_times.append(time.monotonic())
        return res

    def _costmap_tick(self):
        if self.costmap_auto:
            self._publish_costmap()

    def set_obstacle(self, obstacle):
        """障害物帯 (x0, x1, 値) か None（空き）に切り替え、すぐ配信する。"""
        self.costmap_obstacle = obstacle
        self._publish_costmap()

    def _publish_costmap(self):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.info.resolution = 0.05
        m.info.width = 200
        m.info.height = 200
        m.info.origin.position.x = -5.0
        m.info.origin.position.y = -5.0
        m.info.origin.orientation.w = 1.0
        data = [0] * (200 * 200)
        ob = self.costmap_obstacle
        if ob is not None:
            x0, x1, val = ob
            for iy in range(200):
                y = -5.0 + (iy + 0.5) * 0.05
                if not (-0.5 <= y <= 0.5):
                    continue
                for ix in range(200):
                    x = -5.0 + (ix + 0.5) * 0.05
                    if x0 <= x < x1:
                        data[iy * 200 + ix] = val
        m.data = data
        self._pub_costmap.publish(m)

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
            delay = self.compute_result_delay_s
        if delay > 0.0:
            time.sleep(delay)
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
            self.follow_times.append(time.monotonic())
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
        if self.follow_collision:
            msg = Log()
            msg.stamp = self.node.get_clock().now().to_msg()
            msg.level = 40
            msg.name = 'controller_server'
            msg.msg = 'RegulatedPurePursuitController detected collision ahead!'
            self.pub_rosout.publish(msg)
        goal_handle.abort()
        return FollowPath.Result()

    # ── 補助（試験スレッド側。executor が裏で回っているので sleep で待つ） ──
    def _sleep(self, sec: float):
        time.sleep(sec)

    def _mark_passed(self, note: str = ''):
        """メソッド終了の証跡。pytest が stdout を捕捉する（-s 無し）ため、
        コンソールだけでなくファイルにも残す。`VNPB_MARKER_FILE` 未設定なら
        何もしない（ホスト単体実行でも壊れない）。"""
        line = f'[vnbh] {self._testMethodName} PASSED {note}'.rstrip() + '\n'
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
            req.requester = 'vnbh_test'
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
        ev.source_node = 'vnbh_test'
        ev.arg_json = '{}'
        self.pub_event.publish(ev)

    def _publish_fault(self, active: bool,
                       fault_type: str = 'ESP32_DISCONNECTED',
                       severity: str = 'RECOVERABLE'):
        """回復フォルトの発生／解消（C-03 の経路。`PERSON_TRACKER_LOST` は
        PANEL_NAV を PAUSE に落とさないので使わない）。"""
        msg = FaultStatus()
        msg.active = active
        msg.fault_type = fault_type
        msg.severity = severity if active else 'RECOVERABLE'
        self.pub_fault.publish(msg)

    def _publish_setup_event(self, event: str):
        """起動の前提として試験から与えるイベント（`evt.link_ok` と同じ扱い）。
        `evt.blocked` / `evt.unblocked` は venue が出すべきものなので、
        ここでは送らない（SUMMON 到達に要る `two_point_done` / `clear_ok` 用）。"""
        ev = StateEvent()
        ev.header.stamp = self.node.get_clock().now().to_msg()
        ev.event = event
        ev.source_node = 'vnbh_test'
        ev.arg_json = '{}'
        self.pub_event.publish(ev)

    def _publish_summon_goal(self, x: float = 3.0, y: float = 0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.pub_summon.publish(pose)

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
            # 起動直後の初回試験では discovery が間に合わず link_ok の単発が
            # 落ちることがある（並行負荷で顕著）。IDLE になるまで送り直す。
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                self._publish_link_ok()
                if self._wait_mode_state('IDLE', 'NONE', timeout=0.6):
                    return ('IDLE', 'NONE')
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
            # 取り消し結果が届いて実行中ゴールが落ちるまで待つ。置いたまま
            # 次に入ると、古いゴールが次の試験の NAV 入口を抑止するか
            # （handle 残留で _start_nav が抑止）、古い経路を走らせてしまう。
            if live:
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline and self._n_open() > 0:
                    time.sleep(0.05)
                if self._n_open() > 0:
                    self.fail('ui.abort の cancel から 5s たっても代役の'
                              'ゴールが実行中のまま残っている。')
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

    def _enter_home_nav(self):
        """`ui.goto{kind:HOME}`。IDLE/AT_HOME のどちらからでも入る。
        select_pin は要らない（HOME ピンを venue が直接読む）。"""
        ms = self._mode_state()
        if ms is None or ms == ('INIT', 'CHECK'):
            self._reset_to_home()
        self._call_trigger('ui.goto', '{"kind":"HOME"}')

    def _enter_summon_nav(self):
        """`ui.goto{kind:SUMMON}` → POINT → `two_point_done` → WAIT_CLEAR →
        `clear_ok` → NAV。後ろ 2 つは本来 `pin_registrar` / `wait_clear_gate`
        が出すが、この試験は venue と FSM の配線が対象なので `link_ok` と同じ
        「起動の前提」として試験から与える（assert 対象の blocked/unblocked
        ではない）。SUMMON のゴールは `/onsite/summon_goal` で与える。"""
        ms = self._mode_state()
        if ms is None or ms == ('INIT', 'CHECK'):
            self._reset_to_home()
        self._publish_summon_goal()
        self._call_trigger('ui.goto', '{"kind":"SUMMON"}')
        if not self._wait_mode_state('SUMMON', 'POINT', timeout=10.0):
            self.fail(f'SUMMON/POINT に入らない ({self._mode_state()})')
        self._publish_setup_event('evt.two_point_done')
        if not self._wait_mode_state('SUMMON', 'WAIT_CLEAR', timeout=5.0):
            self.fail(f'SUMMON/WAIT_CLEAR に入らない ({self._mode_state()})')
        self._publish_setup_event('evt.clear_ok')
        if not self._wait_mode_state('SUMMON', 'NAV', timeout=10.0):
            self.fail(f'SUMMON/NAV に入らない ({self._mode_state()})')

