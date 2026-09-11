#!/usr/bin/env python3
# ============================================================
# venue_navigator — 試験場内 盤前移動・向き合わせ (WP-ONSITE-02)
# ============================================================
# PANEL_NAV / SUMMON の NAV → ALIGN → AT_PANEL を配線する。
# - NAV: Nav2 の ComputePathToPose → FollowPath（経路を1回計算してキャッシュ。
#   高水準アクションは使わない＝自動リプラン禁止・Spec-onsite §6）。
#   到着（result SUCCEEDED または xy が arrival_xy_tol_m 以内）で evt.arrived。
#   ABORTED/CANCELED → evt.blocked、同一ゴールで経路再探索を繰り返し成功で
#   evt.unblocked（タイムアウトしない・ゴールは変えない）。
# - cancel/resume_follow_path: 経路キャッシュを再送。replan だけ取り直し。
# - ALIGN: /cmd_vel_behavior で超信地旋回。収束で Twist() を出して evt.align_done。
#
# 純コア (venue_nav_core) に旋回誤差・旋回指令・到着判定を寄せる。
import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from th_system_msgs.msg import PinList, StateEffect, StateEvent, SystemState
from th_system_msgs.srv import GoToPanel

from th_onsite.venue_nav_core import (
    VenueNavParams, align_cmd_wz, arrived, find_home_goal,
    should_unblock_for_arrival,
)


def _yaw_from_quat(q) -> float:
    """クォータニオン (w,x,y,z フィールドを持つ) から yaw [rad] を返す。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VenueNavigator(Node):
    # このノードが駆動対象とするモード（NAV で走り、必要なら ALIGN）
    _NAV_MODES = ('PANEL_NAV', 'SUMMON', 'HOME_NAV')

    def __init__(self):
        super().__init__('venue_navigator')

        # ── パラメータ ──────────────────────────────────────
        # WAIVER(demo): W-13 — 数値は registry.yaml 経由でなくノード内リテラル既定値
        self.declare_parameter('align_tolerance_rad', 0.09)
        self.declare_parameter('align_kp', 1.2)
        self.declare_parameter('align_w_max_rps', 0.6)
        self.declare_parameter('blocked_recheck_period_s', 2.0)
        self.declare_parameter('arrival_xy_tol_m', 0.30)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self._params = VenueNavParams(
            align_tolerance_rad=float(self.get_parameter('align_tolerance_rad').value),
            align_kp=float(self.get_parameter('align_kp').value),
            align_w_max_rps=float(self.get_parameter('align_w_max_rps').value),
            blocked_recheck_period_s=float(
                self.get_parameter('blocked_recheck_period_s').value),
            arrival_xy_tol_m=float(self.get_parameter('arrival_xy_tol_m').value),
        )
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # ── 状態 ────────────────────────────────────────────
        self._mode = ""            # /system/state の mode（PANEL_NAV / SUMMON を対象）
        self._state = ""           # /system/state の state
        self._prev_key = None      # (mode, state) 遷移検出用
        self._pins = []            # /onsite/pins の保持
        self._pending_goal = None  # select_pin で保留した PIN goal (dict)
        self._summon_goal = None   # /onsite/summon_goal の最新 (PoseStamped)
        self._robot = None         # (x, y, yaw) 最新（TF map->base_link。到着判定/ALIGN は map 系で行う）
        self._path = None          # compute_path_to_pose のキャッシュ (nav_msgs/Path)
        self._arrived_latched = False
        self._align_latched = False
        self._follow_goal_handle = None
        self._cancel_requested = False   # 自分が cancel したかを区別する
        self._blocked = False            # BLOCKED 相当の内部状態
        self._unblocking = False         # 再探索成功で evt.unblocked を出す予約
        self._recheck_timer = None
        # 2026-09-10 実機バグ対策（venue-navigator-blocked-bugs）:
        # compute→follow の連鎖が in-flight か / blocked 再探索が in-flight か /
        # 到着処理を FSM に投げて NAV への復帰待ちか。取りこぼしに備え _started_at で
        # 10s の stale-escape を持つ。
        self._nav_chain_active = False
        self._nav_chain_started_at = 0.0
        self._recheck_in_flight = False
        self._recheck_started_at = 0.0
        self._arrival_pending = False
        # costmap クリア連鎖の締め切り（0.0 = in-flight でない）。サービスが
        # advertise されているのに応答しない（nav2 再起動中など）と done コール
        # バックが永久に来ず _recheck_in_flight が 10s 張り付くため、2s で
        # クリアを諦めて再計算へ進む。
        self._clear_deadline = 0.0
        # WS-9AC(2026-09-11): 幽霊マーク一掃は BLOCKED 突入ごとに 1 回だけ。
        # 毎周期クリアすると今まさに目の前にある本物の障害物マークまで消して
        # しまい、クリア直後の compute がすり抜けて FollowPath 側で ABORT する
        # 往復を招く（VISION.md WS-9AC）。
        self._cleared_this_episode = False

        # ── QoS ─────────────────────────────────────────────
        effect_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        event_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                               history=QoSHistoryPolicy.KEEP_LAST)
        state_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               history=QoSHistoryPolicy.KEEP_LAST)
        cmd_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)

        sub_cbg = ReentrantCallbackGroup()
        self._state_cbg = ReentrantCallbackGroup()
        # 2026-09-10: タイマがアクション結果コールバックに starve されないよう分離する
        # （旧版は全部ノード既定の MutuallyExclusive グループで、コールバック内の
        # 同期 wait_for_server が _blocked_recheck を止めていた）。
        self._align_cbg = MutuallyExclusiveCallbackGroup()
        self._recheck_cbg = MutuallyExclusiveCallbackGroup()
        self._action_cbg = ReentrantCallbackGroup()
        self._clear_cbg = MutuallyExclusiveCallbackGroup()

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(SystemState, '/system/state', self._on_state,
                                 state_qos, callback_group=self._state_cbg)
        self.create_subscription(StateEffect, '/system/effect', self._on_effect,
                                 effect_qos, callback_group=sub_cbg)
        self.create_subscription(PinList, '/onsite/pins', self._on_pins,
                                 state_qos, callback_group=sub_cbg)
        self.create_subscription(PoseStamped, '/onsite/summon_goal', self._on_summon,
                                 state_qos, callback_group=sub_cbg)

        # ロボット姿勢は TF map->base_link で取る（経路・ピンは map 系。odom 系だと
        # map->odom ぶんずれる。pin_registrar._map_pose と同じ非ブロッキング取得）。
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Publishers ──────────────────────────────────────
        self._pub_event = self.create_publisher(
            StateEvent, '/system/event', event_qos)
        self._pub_cmd_behavior = self.create_publisher(
            Twist, '/cmd_vel_behavior', cmd_qos)

        # ── Services ────────────────────────────────────────
        self.create_service(GoToPanel, '/onsite/select_pin', self._on_select_pin,
                            callback_group=sub_cbg)

        # ── Nav2 Action Clients ─────────────────────────────
        self._compute_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose',
            callback_group=self._action_cbg)
        self._follow_client = ActionClient(
            self, FollowPath, 'follow_path', callback_group=self._action_cbg)

        # ── costmap クリア（blocked 再探索の前に残留マークを消す）──
        self._clear_global_cli = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap',
            callback_group=self._clear_cbg)
        self._clear_local_cli = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap',
            callback_group=self._clear_cbg)

        # ── ALIGN 20Hz タイマ ───────────────────────────────
        self.create_timer(1.0 / 20.0, self._align_timer,
                          callback_group=self._align_cbg)

        # blocked 再探索タイマ（BLOCKED に入ったときに開始）
        self.create_timer(self._params.blocked_recheck_period_s,
                          self._blocked_recheck,
                          callback_group=self._recheck_cbg)

        self.get_logger().info(
            'venue_navigator 起動 (mode を監視: PANEL_NAV/SUMMON/HOME_NAV)')

    # ── 便利ヘルパー ──────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _emit_event(self, event: str, arg_json: str = "{}"):
        ev = StateEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.event = event
        ev.source_node = 'venue_navigator'
        ev.arg_json = arg_json
        self._pub_event.publish(ev)
        self.get_logger().info(f'/system/event 発行: {event}')

    def _go_blocked(self, arg_json: str = "{}"):
        """BLOCKED に落ちる全経路の唯一の窓口。内部フラグを必ず整合させてから
        evt.blocked を出す。旧版は _follow_result_done でしか _blocked=True に
        ならず、経路計算失敗（no_server / not_accepted / compute_failed / no_goal）
        では立たなかったため _blocked_recheck の `if not self._blocked` で
        再試行が武装解除されていた（2026-09-10 実機）。"""
        if not self._blocked:
            self._cleared_this_episode = False
        self._blocked = True
        self._nav_chain_active = False
        self._recheck_in_flight = False
        self._clear_deadline = 0.0
        self._emit_event('evt.blocked', arg_json)

    def _current_goal(self):
        """現モードのゴール dict（pose + yaw）。無ければ None。"""
        if self._mode == 'PANEL_NAV':
            if self._pending_goal is None:
                return None
            return self._pending_goal
        if self._mode == 'SUMMON':
            if self._summon_goal is None:
                return None
            sg = self._summon_goal
            return {'x': float(sg.pose.position.x),
                    'y': float(sg.pose.position.y),
                    'yaw': _yaw_from_quat(sg.pose.orientation)}
        if self._mode == 'HOME_NAV':
            # ゴールは待機場所ピン (kind == HOME)。無ければ None → _start_nav が
            # evt.blocked を出して待つ。
            return find_home_goal(self._pin_goals())
        return None

    def _pin_goals(self):
        """保持中の /onsite/pins を純関数 find_home_goal が使える dict 列に変換。"""
        out = []
        for pin in self._pins:
            if getattr(pin, 'kind', '') == 'HOME':
                p = pin.pose.position
                out.append({'kind': pin.kind,
                            'x': float(p.x), 'y': float(p.y),
                            'yaw': _yaw_from_quat(pin.pose.orientation)})
        return out

    # ── /system/state 受信 ────────────────────────────────
    def _on_state(self, msg: SystemState):
        self._mode = getattr(msg, 'mode', '')
        self._state = getattr(msg, 'state', '')
        key = (self._mode, self._state)
        if key == self._prev_key:
            return
        entered = key
        self._prev_key = key
        self.get_logger().info(f'/system/state: {entered}')

        # FSM が BLOCKED を publish している間は必ず内部でも blocked（一方向 re-arm。
        # 内部 _blocked が何かの取りこぼしで False に落ちても、次の recheck が
        # 再武装されるようにする。逆に内部 _blocked を勝手に False にはしない）。
        if self._state == 'BLOCKED':
            self._blocked = True

        # NAV に入った瞬間、まだ何も回っていなければ起動（二重チェーン防止）。
        if self._mode in self._NAV_MODES and self._state == 'NAV':
            if (self._follow_goal_handle is None
                    and not self._nav_chain_active
                    and not self._blocked
                    and not self._arrival_pending):
                self._start_nav()
            return

        # ALIGN に入ったら latched を外す（再入できるように）
        if self._state == 'ALIGN':
            self._arrived_latched = False
            self._align_latched = False
            self._arrival_pending = False
            return

        # モードから外れた（IDLE / ESTOP / PAUSE / AT_HOME を含む一部）
        if self._mode not in self._NAV_MODES:
            self._reset_for_exit()

    def _reset_for_exit(self):
        """モード離脱時に FollowPath を cancel し内部状態をリセット。"""
        if self._follow_goal_handle is not None:
            self._cancel_requested = True
            try:
                self._follow_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._follow_goal_handle = None
        self._path = None
        self._arrived_latched = False
        self._align_latched = False
        self._blocked = False
        self._cancel_requested = False
        self._nav_chain_active = False
        self._recheck_in_flight = False
        self._clear_deadline = 0.0
        self._cleared_this_episode = False
        self._arrival_pending = False

    # ── /system/effect 受信 ───────────────────────────────
    def _on_effect(self, msg: StateEffect):
        name = msg.name
        if name == 'cancel_follow_path':
            self._cancel_follow_path()
        elif name == 'resume_follow_path':
            self._resume_follow_path()
        elif name == 'replan':
            self._replan()

    def _cancel_follow_path(self):
        if self._follow_goal_handle is not None:
            self._cancel_requested = True
            try:
                self._follow_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._follow_goal_handle = None
        self._blocked = False
        self._nav_chain_active = False
        self._recheck_in_flight = False
        self._clear_deadline = 0.0
        self._cleared_this_episode = False
        # _path は保持（resume 用）

    def _resume_follow_path(self):
        self._blocked = False
        self._recheck_in_flight = False
        self._arrival_pending = False
        if self._path is None:
            self.get_logger().warn('resume: キャッシュ経路が無い。再探索します')
            self._start_nav()
            return
        self._send_follow_path(self._path)

    def _replan(self):
        self._blocked = False
        self._recheck_in_flight = False
        self._arrival_pending = False
        goal = self._current_goal()
        if goal is None:
            self.get_logger().warn('replan: ゴールが未設定')
            return
        self._compute_and_follow(goal)

    # ── /onsite/pins 受信 ─────────────────────────────────
    def _on_pins(self, msg: PinList):
        self._pins = list(msg.pins)

    # ── /onsite/summon_goal 受信 ──────────────────────────
    def _on_summon(self, msg: PoseStamped):
        self._summon_goal = msg

    # ── ロボット姿勢（TF map->base_link・非ブロッキング）──
    def _refresh_robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return
        t = tf.transform.translation
        self._robot = (t.x, t.y, _yaw_from_quat(tf.transform.rotation))

    # ── /onsite/select_pin ────────────────────────────────
    def _on_select_pin(self, request, response):
        for pin in self._pins:
            if getattr(pin, 'id', '') == request.panel_id:
                p = pin.pose.position
                self._pending_goal = {
                    'x': float(p.x),
                    'y': float(p.y),
                    'yaw': _yaw_from_quat(pin.pose.orientation),
                }
                response.success = True
                response.message = f'ゴール設定: {request.panel_id}'
                self.get_logger().info(
                    f'select_pin: {request.panel_id} -> 保留ゴール '
                    f'({self._pending_goal["x"]:.3f}, {self._pending_goal["y"]:.3f})')
                return response
        response.success = False
        response.message = f'ピンが見つかりません: {request.panel_id}'
        return response

    # ── NAV 起動 ──────────────────────────────────────────
    def _start_nav(self):
        goal = self._current_goal()
        if goal is None:
            self.get_logger().warn('NAV: 行き先が未設定。evt.blocked を出して待つ')
            self._go_blocked(json.dumps({'reason': 'no_goal'}))
            return
        # WS-9AC(2026-09-11): 地図作成中に付いた幽霊障害物マーク（global costmap は
        # rolling_window:false・raytrace_max_range:6.0 で視線外/6m 超が消えない。
        # VISION.md WS-9AC）を初回計算の前に一掃する。_nav_chain_active を先に
        # 立てて、クリアが in-flight の間に _on_state/_align_timer から
        # _start_nav が二重に呼ばれるのを防ぐ。
        self._nav_chain_active = True
        self._nav_chain_started_at = self._now()
        self._clear_costmaps_then(lambda: self._compute_and_follow(goal))

    def _compute_and_follow(self, goal):
        # 非ブロッキング判定。旧版は wait_for_server(timeout=5) をコールバック内で
        # 同期呼びして executor を詰まらせていた（2026-09-10 実機。CPU 張り付き）。
        if not self._compute_client.server_is_ready():
            self.get_logger().warn('compute_path_to_pose サーバ未 ready evt.blocked')
            self._go_blocked(json.dumps({'reason': 'no_server'}))
            return
        req = ComputePathToPose.Goal()
        req.goal = PoseStamped()
        req.goal.header.frame_id = self._map_frame
        req.goal.header.stamp = self.get_clock().now().to_msg()
        req.goal.pose.position.x = float(goal['x'])
        req.goal.pose.position.y = float(goal['y'])
        req.goal.pose.orientation.z = math.sin(float(goal['yaw']) / 2.0)
        req.goal.pose.orientation.w = math.cos(float(goal['yaw']) / 2.0)
        # 全コールバックは executor で走る（ブロッキング spin はしない）。
        self._compute_goal = goal
        self._nav_chain_active = True
        self._nav_chain_started_at = self._now()
        send_future = self._compute_client.send_goal_async(req)
        send_future.add_done_callback(self._compute_goal_done)

    def _compute_goal_done(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('compute_path 受理されず evt.blocked')
            self._go_blocked(json.dumps({'reason': 'not_accepted'}))
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._compute_result_done)

    def _compute_result_done(self, future):
        self._recheck_in_flight = False
        try:
            path = future.result().result.path
        except Exception:
            path = None
        if path is None or len(path.poses) == 0:
            self.get_logger().warn('経路計算失敗 evt.blocked')
            self._unblocking = False
            self._go_blocked(json.dumps({'reason': 'compute_failed'}))
            return
        self._path = path
        # blocked 中の再探索成功 → まず unblocked を出してから再開する
        if self._unblocking:
            self._unblocking = False
            self._blocked = False
            self.get_logger().info('blocked 解除: evt.unblocked + follow_path 再開')
            self._emit_event('evt.unblocked')
        self._send_follow_path(path)

    def _send_follow_path(self, path: Path):
        if not self._follow_client.server_is_ready():
            self.get_logger().warn('follow_path サーバ未 ready evt.blocked')
            self._go_blocked(json.dumps({'reason': 'no_follow_server'}))
            return
        req = FollowPath.Goal()
        req.path = path
        req.controller_id = 'FollowPath'
        self._cancel_requested = False
        send_future = self._follow_client.send_goal_async(
            req, feedback_callback=None)
        send_future.add_done_callback(self._follow_goal_done)

    def _follow_goal_done(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('follow_path 受理されず evt.blocked')
            self._go_blocked(json.dumps({'reason': 'follow_not_accepted'}))
            return
        self._follow_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._follow_result_done)

    def _follow_result_done(self, future):
        self._follow_goal_handle = None
        self._nav_chain_active = False
        # 到着判定は完了コールバックとロボット位置の両方で行う。
        status = None
        try:
            status = future.result().status
        except Exception:
            pass
        if self._cancel_requested:
            self._cancel_requested = False
            return  # 自分で cancel したので evt を出さない
        # result SUCCEEDED → arrived
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self._on_arrived()
            return
        # ABORTED / CANCELED（自分でない cancel）→ blocked
        self.get_logger().warn(f'follow_path 終了 status={status} → evt.blocked')
        self._go_blocked(json.dumps({'status': int(status)}))

    def _on_arrived(self):
        """到着処理。FSM は NAV でしか evt.arrived を受けない（T-PNAV-01 等）ので、
        NAV 以外（BLOCKED で到着圏内に居るなど）のときは evt.unblocked を出して
        FSM を NAV に戻し、次の _align_timer フォールバックに evt.arrived を任せる。
        ラッチは実際に evt.arrived を publish したときだけ立てる（旧版は先にラッチ
        して return し、_align_timer の到着フォールバックを恒久的に殺していた）。"""
        if self._arrived_latched:
            return
        if self._state != 'NAV':
            if not self._arrival_pending:
                self._arrival_pending = True
                self.get_logger().info(
                    f'arrived（state={self._state}）→ evt.unblocked で NAV に戻す')
                if self._blocked:
                    self._blocked = False
                    self._emit_event('evt.unblocked')
            return
        self._arrived_latched = True
        self._blocked = False
        self._arrival_pending = False
        self._nav_chain_active = False
        self._clear_deadline = 0.0
        if self._follow_goal_handle is not None:
            self._cancel_requested = True
            try:
                self._follow_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._follow_goal_handle = None
        self._emit_event('evt.arrived')

    # ── blocked 再探索タイマ ──────────────────────────────
    def _blocked_recheck(self):
        # 取りこぼしに備えた stale-escape（連鎖の done コールバックが 1 つ落ちても
        # 永久ロックしない）。
        now = self._now()
        if self._nav_chain_active and now - self._nav_chain_started_at > 10.0:
            self.get_logger().warn('nav_chain が 10s 応答なし → フラグ解放')
            self._nav_chain_active = False
        if self._recheck_in_flight and now - self._recheck_started_at > 10.0:
            self.get_logger().warn('recheck が 10s 応答なし → フラグ解放')
            self._recheck_in_flight = False
            self._clear_deadline = 0.0
        # costmap クリアが 2s 応答なし → クリアを諦めて再計算へ進む（サービスが
        # advertise 済みなのに無応答＝ nav2 再起動中などで done が来ないケース）。
        if (self._recheck_in_flight and not self._nav_chain_active
                and 0.0 < self._clear_deadline < now):
            self.get_logger().warn('costmap クリア無応答 → クリアを飛ばして再計算')
            self._clear_deadline = 0.0
            self._recheck_compute()
            return

        if not self._blocked:
            return
        if self._mode not in self._NAV_MODES:
            return
        goal = self._current_goal()
        if goal is None:
            return

        # 到着圏内で BLOCKED に落ちている（盤前で膠着）→ RPP を使わず、
        # evt.unblocked で NAV に戻して _align_timer の到着フォールバックに任せる。
        # ここに来る時点で self._blocked は True。pending が既に立っていても
        # 「BLOCKED かつ到着圏内」なら常に unblocked を出し直す（evt.unblocked を
        # BLOCKED から受けるのは T-PNAV-05 で許容。二重送信は無害）。
        self._refresh_robot_pose()
        robot_xy = self._robot[:2] if self._robot is not None else None
        if should_unblock_for_arrival(robot_xy, goal, self._params):
            self.get_logger().info('BLOCKED だが到着圏内 → evt.unblocked')
            self._arrival_pending = True
            self._blocked = False
            self._emit_event('evt.unblocked')
            return

        if self._nav_chain_active or self._recheck_in_flight:
            return
        self._recheck_in_flight = True
        self._recheck_started_at = now
        if self._cleared_this_episode:
            # WS-9AC(2026-09-11): 幽霊マーク一掃はこの BLOCKED episode で既に
            # 済んでいる。毎周期クリアすると今まさにある本物の障害物マークも
            # 消してしまい、クリア直後の compute がすり抜けて FollowPath 側で
            # ABORT する往復を招くため、以降は素の再計算だけ行う。
            self.get_logger().info('blocked 再探索: compute_path_to_pose（クリア済み）')
            self._recheck_compute()
        else:
            self._cleared_this_episode = True
            self.get_logger().info('blocked 再探索: costmap クリア → compute_path_to_pose')
            self._clear_costmaps_then(self._recheck_compute)

    def _clear_costmaps_then(self, done_cb):
        """global → local の順に ClearEntireCostmap を非同期で叩き、完了で done_cb。
        地図フレーム固定の global costmap に残る幽霊障害物マーク
        （rolling_window:false・raytrace_max_range:6.0 で 6m 超/視線外は消えない）を
        毎回の再探索前に一掃する。done が来ない場合は _blocked_recheck が
        _clear_deadline で 2s 後に打ち切る。"""
        self._clear_deadline = self._now() + 2.0

        def _after_local(_fut=None):
            if self._clear_deadline == 0.0:
                return  # 既に打ち切り済み（二重実行防止）
            self._clear_deadline = 0.0
            done_cb()

        def _after_global(_fut=None):
            if self._clear_local_cli.service_is_ready():
                f = self._clear_local_cli.call_async(ClearEntireCostmap.Request())
                f.add_done_callback(_after_local)
            else:
                _after_local()

        if self._clear_global_cli.service_is_ready():
            f = self._clear_global_cli.call_async(ClearEntireCostmap.Request())
            f.add_done_callback(_after_global)
        else:
            _after_global()

    def _recheck_compute(self):
        if self._nav_chain_active:
            return  # 打ち切り後に遅れて来た clear done コールバック等
        goal = self._current_goal()
        if goal is not None and self._blocked:
            self._unblocking = True
            self._compute_and_follow(goal)
        else:
            self._recheck_in_flight = False

    # ── 20Hz タイマ（NAV 到着フォールバック + ALIGN）────────
    def _align_timer(self):
        if self._mode not in self._NAV_MODES:
            return
        self._refresh_robot_pose()

        # NAV 中の到着フォールバック（follow_path の result が届かなかった場合の保険）
        if self._state == 'NAV' and not self._arrived_latched:
            goal = self._current_goal()
            if self._robot is not None and goal is not None:
                if arrived(self._robot[0], self._robot[1],
                           goal['x'], goal['y'], self._params):
                    self.get_logger().info('NAV 到着（xy tol 内）→ evt.arrived')
                    self._on_arrived()
                elif self._arrival_pending:
                    # BLOCKED から「到着圏内」で NAV に戻したのに実際は圏外だった
                    # （自己位置ジャンプ等）→ pending を解いて通常の NAV を起動する。
                    self.get_logger().info('NAV: 到着圏外 → arrival_pending 解除・NAV 再開')
                    self._arrival_pending = False
                    if (self._follow_goal_handle is None
                            and not self._nav_chain_active):
                        self._start_nav()
            return

        # HOME_NAV には ALIGN 状態が無い。向き合わせは仕様どおり行わない
        # （逃げの保険。上述の NAV 分岐で既に return しているため通常ここには来ない）。
        if self._mode == 'HOME_NAV':
            return

        if self._state != 'ALIGN':
            return
        if self._align_latched:
            return
        goal = self._current_goal()
        if goal is None or self._robot is None:
            return
        current_yaw = self._robot[2]
        target_yaw = float(goal['yaw'])
        wz, done = align_cmd_wz(current_yaw, target_yaw, self._params)
        if done:
            # 停止指令を1回出して latched
            stop = Twist()
            self._pub_cmd_behavior.publish(stop)
            self._align_latched = True
            self.get_logger().info('ALIGN 収束 → evt.align_done')
            self._emit_event('evt.align_done')
            return
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = float(wz)
        self._pub_cmd_behavior.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = VenueNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
