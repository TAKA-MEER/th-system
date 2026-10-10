#!/usr/bin/env python3
# ============================================================
# venue_navigator — 試験場内 盤前移動・向き合わせ (WP-ONSITE-02)
# ============================================================
# PANEL_NAV / SUMMON の NAV → ALIGN → AT_PANEL を配線する。
# - NAV: Nav2 の ComputePathToPose → FollowPath（経路を1回計算してキャッシュ。
#   高水準アクションは使わない＝自動リプラン禁止・Spec-onsite §6）。
#   到着（result SUCCEEDED または xy が arrival_xy_tol_m 以内）で evt.arrived。
#   ABORTED/CANCELED → evt.blocked。BLOCKED 中の自動再試行は保持した経路の
#   残りの再送だけ（再計算しない。SG-A15・Spec-onsite §2.1/§6）。
#   再送した FollowPath が受け付けられ走行中なのを次の周期で観測したら
#   evt.unblocked（＝決めた経路が通れた）。別ルートは ui.reroute → replan の
#   ときだけ計算する。最初の計画が失敗した（キャッシュ無し）ときの経路探しの
#   再試行は残す（WS-9AK の再発防止）。
# - cancel/resume_follow_path: 経路キャッシュを再送。replan だけ取り直し。
# - PAUSE に入ったら FollowPath を取り消して止まる（SG-A1。C-01/C-03 の共通行や
#   BLOCKED からの ui.stop には cancel_follow_path が無いが、共通行に足すと
#   PREP の地図作成中のジョグ（状態を保つだけ・復帰の遷移が無い）まで取り消して
#   戻る動作が詰むため、PAUSE 状態を見て取り消す。PREP/RETURN だけは PAUSE を
#   持つ（SG-A7）ので PREP/PAUSE でも取り消す）。再開までは動かない。
#   再開は残りの再送（再計算しない）。
# - ALIGN: /cmd_vel_behavior で超信地旋回。収束で Twist() を出して evt.align_done。
#
# 純コア (venue_nav_core) に旋回誤差・旋回指令・到着判定を寄せる。
import json
import math
from collections import deque

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
from rcl_interfaces.msg import Log
from th_system_msgs.msg import PinList, StateEffect, StateEvent, SystemState
from th_system_msgs.srv import GoToPanel

from th_onsite.venue_nav_core import (
    VenueNavParams, align_cmd_wz, arrived, find_home_goal, success_is_plausible,
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

    def _recovery_eligible(self):
        """WS-9AK(2026-09-11): _blocked_recheck（再探索ループ）の対象判定。
        _is_nav_state() とは意味が違う ── PANEL_NAV/SUMMON/HOME_NAV は
        state に関わらず対象（`state=='BLOCKED'` のときにこそ効く必要が
        あるのに、WS-9AH で誤って _is_nav_state()（state=='NAV' 限定）を
        流用してしまい、FSM が実際に BLOCKED を publish した瞬間に再探索が
        止まる回帰を起こした。実機で「途中で止まったまま。経路は物理的に
        空いている」で発覚）。PREP だけ state=='RETURN' に絞る（PREP には
        BLOCKED という FSM 状態が無いので、`_blocked` フラグの間ずっと
        state は RETURN のまま）。"""
        if self._mode in self._NAV_MODES:
            return True
        if self._mode == 'PREP':
            return self._state == 'RETURN'
        return False

    def _is_nav_state(self, mode=None, state=None):
        """WS-9AG(2026-09-11): 「今 compute/follow を回す対象か」を一箇所に
        集約する。PANEL_NAV/SUMMON/HOME_NAV は state=='NAV'、PREP は
        state=='RETURN'（1 ボタンで待機場所へ。Spec-onsite.md §2.1「自己位置
        推定して戻る」）。PREP の RETURN は HOME_NAV と同じく ALIGN を持たない
        一番単純な形（到着だけで完結）。PREP の他状態（MAPPING/REGISTER/
        EDIT/SAVED）や BLOCKED（PREP には無い）はここに含めない。"""
        mode = self._mode if mode is None else mode
        state = self._state if state is None else state
        if mode in self._NAV_MODES:
            return state == 'NAV'
        if mode == 'PREP':
            return state == 'RETURN'
        return False

    def __init__(self):
        super().__init__('venue_navigator')

        # ── パラメータ ──────────────────────────────────────
        # W-13 解除: 挙動値（向き合わせ・到着判定・再確認周期）は registry.yaml 駆動。
        # launch が渡す生成 yaml が上書きし、そちらが正になる。既定値は
        # 起動単体のフォールバックとして残す。
        # 移さないもの: map_frame/base_frame（フレーム名）。
        # 調整値ではなく配線設定のため registry の対象外
        # （th_config_manager/tunable_targets.py と同じ考え方）。
        self.declare_parameter('align_tolerance_rad', 0.09)
        self.declare_parameter('align_kp', 1.2)
        self.declare_parameter('align_w_max_rps', 0.6)
        self.declare_parameter('blocked_recheck_period_s', 2.0)
        # WS-9AJ(2026-09-11): nav2_params.yaml の xy_goal_tolerance（0.25→0.12）
        # と整合を取る。これは follow_path の result を取りこぼした場合の
        # 到着フォールバックなので、緩いままだと精度改善が骨抜きになる。
        # SG-B15(2026-10-08): registry の arrival_xy_tol_m（0.12）と同値。
        # 生成 yaml が正で、ここは起動単体のフォールバック。
        self.declare_parameter('arrival_xy_tol_m', 0.12)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # Nav2 TF 停滞の検知（2026-10-10 実機。記録だけ・動作は変えない）。
        # controller_server が map→odom の変換に繰り返し失敗すると
        # follow_path が約 10 秒ごとに打ち切られ（`Failed to make progress`）、
        # 再送を繰り返しても回復しない。主信号は自分で見ているもの：
        # follow_path の打ち切り（自分で cancel したものを除く）が N 回連続し、
        # そのあいだ機体の map 位置が M m 以上進んでいない（A）。
        # 補助に /rosout の controller_server の `Failed to make progress`
        # （B。ノード名付きのログなので /rosout に届く）を併用し、両方で発火。
        # 旧版は /rosout の `Transform data too old`（tf_help ロガー）を見て
        # いたが、ノードに紐づかないロガーは /rosout に流れないため実機で
        # 発火しなかった（2026-10-10 夕。検知の取りこぼし）。
        # 検知の調整値であり走行の挙動値ではないので registry の対象外。
        self.declare_parameter('nav_stall_abort_count', 3)
        self.declare_parameter('nav_stall_move_tol_m', 0.20)
        self.declare_parameter('nav_stall_progress_window_s', 120.0)

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
        self._tf_stall_aborts = int(
            self.get_parameter('nav_stall_abort_count').value)
        self._tf_stall_move_tol_m = float(
            self.get_parameter('nav_stall_move_tol_m').value)
        self._tf_stall_progress_window_s = float(
            self.get_parameter('nav_stall_progress_window_s').value)

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
        # WS-9AI(2026-09-11): HOME_NAV/PREP には ALIGN という FSM 状態を足さず、
        # venue_navigator 内部だけで到着後の向き合わせを挟む（evt.arrived を
        # 出す前に align_cmd_wz でその場旋回する）。PANEL_NAV/SUMMON は
        # 既存の ALIGN 状態を使うのでこのフラグの対象外。
        self._final_aligning = False
        self._spurious_timer = None
        self._follow_goal_handle = None
        # 自分が cancel した goal かを「通番の集合」で区別する。bool 1 個では
        # 取り消し結果の到着前に次の送信が走ると印を消してしまい、古い結果を
        # 他人分と誤判定する（再開直後の resume 送信が pause 取り消しの印を
        # 消して偽の evt.blocked → BLOCKED → 再探索の往復。2026-10-01 実測）。
        # 通番は単調増加で使い回さないため、古い印が未来の結果を誤って抑える
        # ことはない。印は対応する結果の到着で消費する。
        self._follow_seq = 0
        self._handle_seq = None
        self._cancel_seqs = set()
        # 不具合 A 対策 (2026-10-01): send_goal_async → accept 判明の窓では
        # handle が無いため cancel が空振りし、受け付け後に走行が続く。
        # 送出中かを _follow_send_in_flight で表し、窓の中で取り消しを求め
        # られたことを _cancel_before_accept に記録して、accept 時点で即座に
        # cancel する（その結果は _cancel_seqs に積んで evt.blocked を出さ
        # ない）。compute 中の取り消しは対象外（送出自体がまだ無い）。
        self._follow_send_in_flight = False
        self._cancel_before_accept = False
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
        # 往復を招く（Spec-onsite.md §6.1）。
        self._cleared_this_episode = False
        # Nav2 TF 停滞の検知用。打ち切り回数と最初の打ち切り時の機体位置
        # （錨。ここから動かなければ停滞）、controller の progress 失敗
        # （/rosout）の受信時刻の列と、episode ごとの発火ラッチ。
        self._stall_aborts = 0
        self._stall_anchor_xy = None
        self._progress_times = deque()
        self._nav_tf_stall_latched = False

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
        # Nav2 TF 停滞の検知（記録だけ）。/rosout は VOLATILE 購読にする
        # （rcl の発行側が TRANSIENT_LOCAL でも VOLATILE でも合うよう）。
        rosout_qos = QoSProfile(depth=100,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.VOLATILE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Log, '/rosout', self._on_rosout,
                                 rosout_qos, callback_group=sub_cbg)

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
        self._unblocking = False
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
        if self._mode in ('HOME_NAV', 'PREP'):
            # ゴールは待機場所ピン (kind == HOME)。無ければ None → _start_nav が
            # evt.blocked を出して待つ。WS-9AG: PREP/RETURN（1 ボタンで戻る）も
            # HOME_NAV と同じゴールを使う。
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

        # NAV 相当に入った瞬間、まだ何も回っていなければ起動（二重チェーン防止）。
        if self._is_nav_state():
            if (self._follow_goal_handle is None
                    and not self._nav_chain_active
                    and not self._blocked
                    and not self._arrival_pending):
                self._resume_or_start()
            return

        # SG-A1(2026-10-05): PAUSE に入ったら FollowPath を取り消して止まる。
        # 再開（ui.run / W-1 の「はい」等）まで動かない。_path は保持するので
        # 再開は残りの再送になる（SM-3.1.2-056）。
        # effect（transitions.yaml に cancel_follow_path を足す）方式は採らない。
        # C-01/C-03 は全モード共通の行で、足すと PREP の地図作成中のジョグ
        # （状態を保つだけ・復帰の遷移が無い）まで取り消して戻る動作が詰む。
        # PAUSE 状態を見て取り消す方が、C-01/C-03・T-PNAV-06/T-SUM-10/T-HNAV-06・
        # C-09c の全入口を一網打尽にでき、PAUSE を持たない PREP の状態
        # （MAPPING/REGISTER/EDIT/SAVED。そもそも PAUSE に入らない）にも影響しない。
        # SG-A7(2026-10-06): PREP/RETURN だけ PAUSE を持つので、PREP/PAUSE でも
        # 取り消す（「はい」で RETURN に戻ったら残りを再送。「いいえ」で MAPPING へ
        # 行ったら下の PREP 節で _reset_for_exit し経路を捨てる）。
        if self._state == 'PAUSE' and (self._mode in self._NAV_MODES
                                       or self._mode == 'PREP'):
            self._cancel_follow_path()
            return

        # ALIGN に入ったら latched を外す（再入できるように）。PREP には無い。
        if self._state == 'ALIGN':
            self._arrived_latched = False
            self._align_latched = False
            self._arrival_pending = False
            return

        # WS-9AG: PREP は RETURN / PAUSE 以外（MAPPING/REGISTER/EDIT/SAVED）に居る
        # ときは対象外。RETURN を抜けた直後（到着で EDIT／「いいえ」で MAPPING へ
        # 戻る等）の後始末も、他の 3 モードがモードごと外れたときと同じに行う。
        # PREP/PAUSE では経路を保持する（「はい」で RETURN に戻ったら残りを再送）。
        if self._mode == 'PREP' and self._state not in ('RETURN', 'PAUSE'):
            self._reset_for_exit()
            return

        # モードから外れた（IDLE / ESTOP / PAUSE / AT_HOME を含む一部）
        if self._mode not in self._NAV_MODES:
            self._reset_for_exit()

    def _reset_for_exit(self):
        """モード離脱時に FollowPath を cancel し内部状態をリセット。"""
        # 不具合 B 対策 (2026-10-01): 取り消しの結果 (CANCELED) が届く前に
        # 印を消すと「他人に取り消された」と誤判定し、偽の evt.blocked と
        # _blocked 残留を招く（次 NAV の即時起動が抑止され再探索経由の遠回り
        # になる）。結果待ちの印は残し、対応する結果の到着で消費する。
        # 送出中（handle 未確定）の離脱は _cancel_before_accept に記録し、
        # accept 時に late-cancel する（ESTOP 等は venue へ effect を送らない
        # ため、この経路だけが頼り。2026-10-01 指摘）。
        if self._follow_goal_handle is not None:
            if self._handle_seq is not None:
                self._cancel_seqs.add(self._handle_seq)
            try:
                self._follow_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._follow_goal_handle = None
            self._handle_seq = None
        elif self._follow_send_in_flight:
            self._cancel_before_accept = True
        self._path = None
        self._arrived_latched = False
        self._align_latched = False
        self._final_aligning = False
        self._blocked = False
        self._unblocking = False
        # _cancel_seqs は残す（結果待ちの取り消し。不具合 B 対策）。
        # _cancel_before_accept も残す（accept 時の late-cancel に使う）。
        # 下りる経路: _follow_result_done の消費／_follow_goal_done の不受理／
        # stale-escape。いずれも対応する結果か送信に紐づく。
        self._nav_chain_active = False
        self._recheck_in_flight = False
        self._clear_deadline = 0.0
        self._cleared_this_episode = False
        self._arrival_pending = False
        # 検知のラッチも episode ごとに下ろす（次の episode は改めて判定）。
        self._stall_reset()

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
            if self._handle_seq is not None:
                self._cancel_seqs.add(self._handle_seq)
            try:
                self._follow_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._follow_goal_handle = None
            self._handle_seq = None
        elif self._follow_send_in_flight:
            # 不具合 A (2026-10-01): 送信～accept の窓では handle が無く
            # cancel が空振りする。要求があったことだけ記録し、accept 時に
            # _follow_goal_done が即座に取り消す。
            self._cancel_before_accept = True
        self._blocked = False
        self._unblocking = False
        self._nav_chain_active = False
        self._recheck_in_flight = False
        self._clear_deadline = 0.0
        self._cleared_this_episode = False
        self._final_aligning = False
        # _path は保持（resume 用）

    def _resume_follow_path(self):
        self._blocked = False
        self._unblocking = False
        self._recheck_in_flight = False
        self._arrival_pending = False
        if self._path is None:
            self.get_logger().warn('resume: キャッシュ経路が無い。再探索します')
            self._start_nav()
            return
        if (self._follow_goal_handle is not None
                or self._follow_send_in_flight):
            # _resume_or_start と同じ理由（到着順不定の重複）。送出済みなら送らない。
            self.get_logger().info('resume: 送出済みのため再送しない')
            return
        self._send_follow_path(self._path)

    def _resume_or_start(self):
        """NAV 系への入口（effect を伴わない復帰＝C-04/C-05 の W-1「はい」等を含む）。
        キャッシュ経路があれば残りを送り直すだけで、計算し直さない
        （SM-3.1.2-056。f126534 で縛った挙動）。無ければ最初から計画する。"""
        if (self._follow_goal_handle is not None
                or self._follow_send_in_flight):
            # 再開要求の重複（resume effect と /system/state(NAV) は別トピックで
            # 到着順が不定。3ms 差で逆順に処理され二重送出になった実例あり）。
            # 送出済み・送出中なら何もしない。
            return
        if self._path is not None:
            self._send_follow_path(self._path)
        else:
            self._start_nav()

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

    # ── /rosout 受信（Nav2 TF 停滞の検知の補助 B・記録だけ）────
    # 2026-10-10 実機: controller_server の TF バッファの map→odom が約
    # 2 時間止まり、約 10 秒ごとに `Failed to make progress` で follow_path
    # が打ち切られ、再送では回復しなかった。止める・再起動する動作は入れ
    # ない（記録だけ）。FSM は evt.nav_tf_stall を知らないので無視する。
    # 注意: `Transform data too old` を出すのは `tf_help`（nav2 の
    # nav_2d_utils。ノードに紐づかないロガー）であり /rosout に流れない。
    # こちらで数えるのはノード名 `controller_server` の進捗失敗だけ。
    def _on_rosout(self, msg: Log):
        if msg.name != 'controller_server':
            return
        if 'failed to make progress' not in msg.msg.lower():
            return
        now = self._now()
        self._progress_times.append(now)
        while self._progress_times and \
                now - self._progress_times[0] > self._tf_stall_progress_window_s:
            self._progress_times.popleft()

    def _stall_reset(self):
        """停滞 episode の作り直し（到着・成功・離脱・新しい計画で呼ぶ）。"""
        self._stall_aborts = 0
        self._stall_anchor_xy = None
        self._progress_times.clear()
        self._nav_tf_stall_latched = False

    def _stall_note_abort(self):
        """follow_path の打ち切り（自分で cancel したものを除く）を数える。
        機体が動いていれば（再送で進めた正常系）錨を下ろし直して数え直す
        ので、本物の障害物で BLOCKED→再送→進めるを繰り返す系列では発火
        しない。動かずに打ち切りだけが積み上がるのが今回の故障の形。"""
        if not self._recovery_eligible():
            return
        self._refresh_robot_pose()
        if self._robot is None:
            return
        xy = (self._robot[0], self._robot[1])
        if self._stall_anchor_xy is None:
            self._stall_anchor_xy = xy
            self._stall_aborts = 1
        else:
            moved = math.hypot(xy[0] - self._stall_anchor_xy[0],
                               xy[1] - self._stall_anchor_xy[1])
            if moved >= self._tf_stall_move_tol_m:
                self._stall_anchor_xy = xy
                self._stall_aborts = 1
            else:
                self._stall_aborts += 1
        self._check_nav_tf_stall()

    def _check_nav_tf_stall(self):
        if self._nav_tf_stall_latched:
            return
        if self._stall_aborts < self._tf_stall_aborts:
            return
        # B（補助）: 直近に controller_server の進捗失敗があること。
        now = self._now()
        while self._progress_times and \
                now - self._progress_times[0] > self._tf_stall_progress_window_s:
            self._progress_times.popleft()
        if not self._progress_times:
            return
        # /tf 自体が新しいのに controller だけが古い＝今回の故障の形。
        # /tf ごと古い（wire_fresh=false）なら SLAM 側を見る。
        wire_fresh = False
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, 'odom', rclpy.time.Time())
            age = self._now() - (tf.header.stamp.sec +
                                 tf.header.stamp.nanosec / 1e9)
            wire_fresh = 0.0 <= age < 5.0
        except Exception:
            wire_fresh = False
        path_age_s = -1.0
        try:
            if self._path is not None:
                stamp = self._path.header.stamp
                path_age_s = self._now() - (stamp.sec + stamp.nanosec / 1e9)
        except Exception:
            path_age_s = -1.0
        self._nav_tf_stall_latched = True
        moved_cm = 0.0
        try:
            if self._stall_anchor_xy is not None and self._robot is not None:
                moved_cm = math.hypot(
                    self._robot[0] - self._stall_anchor_xy[0],
                    self._robot[1] - self._stall_anchor_xy[1]) * 100.0
        except Exception:
            moved_cm = 0.0
        arg = {
            'reason': 'controller_tf_stall',
            'abort_count': self._stall_aborts,
            'progress_count': len(self._progress_times),
            'moved_cm': round(moved_cm, 1),
            'wire_fresh': wire_fresh,
            'path_age_s': round(path_age_s, 1),
        }
        self.get_logger().warn(
            'Nav2 TF stall を検知（記録だけ）: follow_path が '
            f'{self._stall_aborts} 回連続で打ち切られ、そのあいだ機体は '
            f'{moved_cm:.0f} cm しか進んでいない。/tf は'
            f'{"新しい" if wire_fresh else "古い"}。再送では直らないので'
            '制御系を起動し直す（使い方 §7）。')
        self._emit_event('evt.nav_tf_stall', json.dumps(arg))

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
        # Spec-onsite.md §6.1）を初回計算の前に一掃する。_nav_chain_active を先に
        # 立てて、クリアが in-flight の間に _on_state/_align_timer から
        # _start_nav が二重に呼ばれるのを防ぐ。
        self._nav_chain_active = True
        self._nav_chain_started_at = self._now()
        # 新しい episode は検知も改めて判定する。
        self._stall_reset()
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
        # SG-A1(2026-10-05): 計算中に PAUSE に入っていたら送らず、キャッシュだけ
        # 残す。走り出すのは再開時の残り再送（_resume_or_start / resume effect）。
        if self._state == 'PAUSE':
            self._unblocking = False
            return
        # モード離脱後に届いた計算結果は捨てる（送らない・キャッシュも残さない）。
        # _reset_for_exit が _path=None にした後なので、残すと離脱先のモードで
        # 走り出すうえ、次の episode の NAV 入口が古い経路を送ってしまう。
        # ui.abort → 離脱 → 結果到着の順で実際に走り出した（09e8346 の競合と
        # 同じ型。IDLE_H で FollowPath が送られ次の試験を汚染した）。
        if not self._recovery_eligible():
            self._path = None
            self._unblocking = False
            self._recheck_in_flight = False
            return
        # blocked 中の再探索成功 → まず unblocked を出してから再開する
        # （経路未確定＝初回計画失敗時の再試行の経路。SG-A15 の残り再送は
        # _resend_cached 側で扱い、ここには来ない）。
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
        # _cancel_seqs はここで消さない。古い取り消しの結果は自分の通番で
        # 消費する（bool 時代はここで消して偽 blocked を出していた）。
        # SM-3.1.2-056「同じ経路の続きから。再検索はしない」:
        # resume_follow_path の effect は /system/state(NAV) より先に届く
        # （state_manager が effect→state の順に publish する）ため、follow の
        # accept 往復が終わる前に _on_state(NAV) が走る。その時点で handle は
        # まだ None なので、印が無いと _start_nav() が二重に走って compute が
        # 2 回目になる（再開なのに再検索）。送出時点で _nav_chain_active を
        # 立てて「送出中」を表し、二重起動を抑える。accept 完了後は handle が
        # 抑えるので印は下ろす（_follow_goal_done）。
        self._nav_chain_active = True
        self._nav_chain_started_at = self._now()
        # 送出中の印は send_goal_async の『前』に立てる。後に立てると、
        # MultiThreadedExecutor で done コールバックが先に走ったときに
        # 印だけ True のまま残り、次の正常な送信を late-cancel で殺す
        # （2026-10-01 指摘）。
        self._follow_send_in_flight = True
        self._follow_seq += 1
        seq = self._follow_seq
        send_future = self._follow_client.send_goal_async(
            req, feedback_callback=None)
        send_future.add_done_callback(
            lambda fut: self._follow_goal_done(fut, seq))

    def _follow_goal_done(self, future, seq):
        self._follow_send_in_flight = False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._cancel_before_accept = False
            self.get_logger().warn('follow_path 受理されず evt.blocked')
            self._go_blocked(json.dumps({'reason': 'follow_not_accepted'}))
            return
        self._follow_goal_handle = goal_handle
        self._handle_seq = seq
        # _send_follow_path で立てた「送出中」の印を下ろす。以降の NAV 入口は
        # handle が抑える。立てたままにすると長時間の follow 中に 10 秒の
        # stale-escape（_blocked_recheck 冒頭）が誤って発火する。
        self._nav_chain_active = False
        # SG-A15(2026-10-05): BLOCKED 中の残り再送（プローブ）の受け付け。
        # ここでは evt.unblocked を出さない ── 次の再探索周期で「まだ走行中」を
        # 観測できたときだけ通れたとみなす（受け付け直後の ABORT で
        # BLOCKED↔NAV が往復するのを防ぐ）。_recheck_in_flight は結果か
        # 次周期の観測まで立てたままにし、二重送信を抑える。
        if self._cancel_before_accept:
            # 不具合 A (2026-10-01): 窓の中で取り消しを求められていた。
            # 受け付けた端から取り消す。結果 (CANCELED) は自分由来なので
            # 通番を積んで evt.blocked を出さない。
            self._cancel_before_accept = False
            self._unblocking = False
            self._cancel_seqs.add(seq)
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self._follow_result_done(fut, seq))

    def _follow_result_done(self, future, seq):
        # 古い結果で生きている handle を捨てない。seq1 の取り消し → 再開で
        # seq2 が受け付け済み → seq1 の CANCELED が遅れて届く順だと、無条件に
        # 下ろすと走行中の seq2 の handle が消え、次の取り消しが空振りして
        # PAUSE 中に走り続ける（不具合 A と同じ型。2026-10-01 指摘）。
        # 自分の結果のときだけ下ろす。
        if seq == self._handle_seq:
            self._follow_goal_handle = None
            self._handle_seq = None
            self._nav_chain_active = False
        # 到着判定は完了コールバックとロボット位置の両方で行う。
        status = None
        try:
            status = future.result().status
        except Exception:
            pass
        if seq in self._cancel_seqs:
            self._cancel_seqs.discard(seq)
            self._unblocking = False
            return  # 自分で cancel したので evt を出さない
        # result SUCCEEDED → arrived
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self._unblocking = False
            self._recheck_in_flight = False
            self._stall_reset()
            self._refresh_robot_pose()
            goal = self._current_goal()
            robot_xy = self._robot[:2] if self._robot is not None else None
            if not success_is_plausible(robot_xy, goal):
                # Nav2 が自己位置の変換に失敗して即「成功」と返したとき。到着扱いに
                # せず、少し待って計画からやり直す(Nav2 の tf が戻れば進む)。
                self.get_logger().warn(
                    f'follow_path が成功を返したがゴールまで '
                    f'{math.hypot(robot_xy[0] - goal["x"], robot_xy[1] - goal["y"]):.2f} m '
                    f'離れている → 到着扱いにせず再計画する')
                self._schedule_spurious_retry()
                return
            self._arrive_or_align()
            return
        # ABORTED / CANCELED（自分でない cancel）→ blocked
        self.get_logger().warn(f'follow_path 終了 status={status} → evt.blocked')
        self._stall_note_abort()
        self._go_blocked(json.dumps({'status': int(status)}))

    def _schedule_spurious_retry(self, delay_s: float = 2.0):
        """成功の誤報のあと、delay_s 後に NAV を最初から計画し直す(1 回きりのタイマ)。"""
        if getattr(self, '_spurious_timer', None) is not None:
            return
        self._spurious_timer = self.create_timer(delay_s, self._spurious_retry)

    def _spurious_retry(self):
        timer = self._spurious_timer
        self._spurious_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        if (self._is_nav_state() and not self._arrived_latched
                and self._follow_goal_handle is None
                and not self._nav_chain_active):
            self.get_logger().info('成功の誤報のあと: NAV を計画からやり直す')
            self._start_nav()

    def _on_arrived(self):
        """到着処理。FSM は NAV でしか evt.arrived を受けない（T-PNAV-01 等）ので、
        NAV 以外（BLOCKED で到着圏内に居るなど）のときは evt.unblocked を出して
        FSM を NAV に戻し、次の _align_timer フォールバックに evt.arrived を任せる。
        ラッチは実際に evt.arrived を publish したときだけ立てる（旧版は先にラッチ
        して return し、_align_timer の到着フォールバックを恒久的に殺していた）。"""
        if self._arrived_latched:
            return
        if not self._is_nav_state():
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
            if self._handle_seq is not None:
                self._cancel_seqs.add(self._handle_seq)
            try:
                self._follow_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._follow_goal_handle = None
            self._handle_seq = None
        self._emit_event('evt.arrived')

    def _needs_final_align(self, mode=None):
        """WS-9AI(2026-09-11): PANEL_NAV/SUMMON は ALIGN という FSM 状態で
        向き合わせ済みなので対象外。HOME_NAV/PREP は無いので、ここで扱う。"""
        mode = self._mode if mode is None else mode
        return mode in ('HOME_NAV', 'PREP')

    def _arrive_or_align(self):
        """xy 到着（FollowPath 結果 or 到着フォールバックの両方から呼ぶ）を
        受けて、ALIGN を持たないモードなら evt.arrived の前にその場旋回を
        挟む（実機で「待機場所到着時に向きを直さない。当日の地図再読み込みが
        ピンの登録姿勢＝向きも前提にしているのでズレる」と判明。Spec-onsite.md §2.1.1
        WS-9AI）。ALIGN を持つモードは今までどおり素通しする。"""
        if not self._needs_final_align():
            self._on_arrived()
            return
        if self._final_aligning or self._arrived_latched:
            return
        self._final_aligning = True
        self._align_latched = False
        if self._follow_goal_handle is not None:
            if self._handle_seq is not None:
                self._cancel_seqs.add(self._handle_seq)
            try:
                self._follow_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._follow_goal_handle = None
            self._handle_seq = None
        self.get_logger().info('到着（xy tol 内）→ 向きを合わせてから evt.arrived')

    def _run_final_align(self):
        """_arrive_or_align() が立てた _final_aligning を実際に収束させる
        （ALIGN 状態の align_cmd_wz ループと同じ純関数を使い回す）。20Hz
        タイマから呼ぶ想定（self._robot は呼び出し側で更新済みのこと）。"""
        goal = self._current_goal()
        if goal is None or self._robot is None:
            # ゴールが消えた等の異常系。向き合わせは諦めてそのまま到着扱い。
            self._final_aligning = False
            self._on_arrived()
            return
        current_yaw = self._robot[2]
        target_yaw = float(goal['yaw'])
        wz, done = align_cmd_wz(current_yaw, target_yaw, self._params)
        if done:
            stop = Twist()
            self._pub_cmd_behavior.publish(stop)
            self._final_aligning = False
            self.get_logger().info('向き合わせ完了 → evt.arrived')
            self._on_arrived()
            return
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = float(wz)
        self._pub_cmd_behavior.publish(twist)

    # ── blocked 再探索タイマ ──────────────────────────────
    def _blocked_recheck(self):
        # 取りこぼしに備えた stale-escape（連鎖の done コールバックが 1 つ落ちても
        # 永久ロックしない）。
        now = self._now()
        if self._nav_chain_active and now - self._nav_chain_started_at > 10.0:
            self.get_logger().warn('nav_chain が 10s 応答なし → フラグ解放')
            self._nav_chain_active = False
            # accept が永久に来ない（Nav2 ごと死亡）とき、送出中・取り消し
            # 予約の印が残ると次の送信の受け付けを誤って殺すので一緒に下ろす。
            self._follow_send_in_flight = False
            self._cancel_before_accept = False
        if self._recheck_in_flight and now - self._recheck_started_at > 10.0:
            self.get_logger().warn('recheck が 10s 応答なし → フラグ解放')
            self._recheck_in_flight = False
            self._clear_deadline = 0.0
        # costmap クリアが 2s 応答なし → クリアを諦めて再計算へ進む（サービスが
        # advertise 済みなのに無応答＝ nav2 再起動中などで done が来ないケース）。
        if (self._recheck_in_flight and not self._nav_chain_active
                and 0.0 < self._clear_deadline < now):
            self.get_logger().warn('costmap クリア無応答 → クリアを飛ばして再試行')
            self._clear_deadline = 0.0
            self._recheck_send()
            return

        if not self._blocked:
            return
        if not self._recovery_eligible():
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
            self._unblocking = False
            self._emit_event('evt.unblocked')
            return

        # SG-A15(2026-10-05): 前周期に送った残り再送のプローブが受け付けられ、
        # まだ走行中＝決めた経路が通れた。evt.unblocked で NAV に戻す
        # （別ルートは探していない。受け付け直後の ABORT では往復しないよう、
        # 受け付け時点ではなく次の周期の観測で判定する）。
        # 再計算の経路（_path 無し→_compute_and_follow）の成功は
        # _compute_result_done 側で unblocked を出すので、ここには来ない
        # （_unblocking はその場で下ろす）。
        if self._unblocking and self._follow_goal_handle is not None:
            self.get_logger().info('blocked 解除: 残りの走り直しが通れた → evt.unblocked')
            self._unblocking = False
            self._recheck_in_flight = False
            if self._blocked:
                self._blocked = False
                self._emit_event('evt.unblocked')
            return

        if self._nav_chain_active or self._recheck_in_flight:
            return
        self._recheck_in_flight = True
        self._recheck_started_at = now
        self._recheck_send()

    def _recheck_send(self):
        """再探索タイマの送出部。経路未確定なら計算、確定済みなら残りの再送。"""
        if self._path is None:
            # まだ経路が決まっていない（最初の計画の失敗）。経路を探す再試行は
            # 残す（WS-9AK の再発防止。止まり続けたら実機で詰む）。
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
            return
        # SG-A15(2026-10-05): 経路決定後の自動再試行は、保持した経路の残りの
        # 再送だけ。別ルートは探さない（再検索 ui.reroute → _replan のときだけ
        # _compute_and_follow が走る）。コストマップの一掃は初回だけ
        # （Spec-onsite §6.1。毎周期消すと本物の障害物マークまで消す）。
        if self._cleared_this_episode:
            self.get_logger().info('blocked 再試行: 保持した経路の残りを再送（再計算しない）')
            self._resend_cached()
        else:
            self._cleared_this_episode = True
            self.get_logger().info('blocked 再試行: costmap クリア → 残りを再送')
            self._clear_costmaps_then(self._resend_cached)

    def _resend_cached(self):
        """保持した経路の残りを FollowPath に送り直す（_recheck_compute の
        残り再送版。送っただけでは unblocked を出さず、次の周期に「まだ
        走行中」を観測できたら通れたとみなす）。"""
        if self._nav_chain_active:
            return  # 打ち切り後に遅れて来た clear done コールバック等
        if self._path is None or not self._blocked:
            self._recheck_in_flight = False
            return
        self._unblocking = True
        self._send_follow_path(self._path)

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
        # WS-9AG: PREP/RETURN も到着フォールバックの対象（HOME_NAV と同じく
        # ALIGN を持たないので、下の ALIGN 分岐には届かない）。
        if self._mode not in self._NAV_MODES and self._mode != 'PREP':
            return
        self._refresh_robot_pose()

        # NAV 相当中の到着フォールバック（follow_path の result が届かなかった場合の保険）
        if self._is_nav_state() and not self._arrived_latched:
            # WS-9AI: ALIGN を持たないモード（HOME_NAV/PREP）の向き合わせは
            # ここで完結させる（_arrive_or_align() が立てたフラグを収束させる）。
            if self._final_aligning:
                self._run_final_align()
                return
            goal = self._current_goal()
            if self._robot is not None and goal is not None:
                if arrived(self._robot[0], self._robot[1],
                           goal['x'], goal['y'], self._params):
                    self.get_logger().info('NAV 到着（xy tol 内）')
                    self._arrive_or_align()
                elif self._arrival_pending:
                    # BLOCKED から「到着圏内」で NAV に戻したのに実際は圏外だった
                    # （自己位置ジャンプ等）→ pending を解いて通常の NAV を起動する。
                    self.get_logger().info('NAV: 到着圏外 → arrival_pending 解除・NAV 再開')
                    self._arrival_pending = False
                    if (self._follow_goal_handle is None
                            and not self._nav_chain_active):
                        self._start_nav()
            return

        # HOME_NAV / PREP には ALIGN という FSM 状態が無い（向き合わせ自体は
        # 上の _final_aligning 経路で行う。WS-9AI）。
        # （逃げの保険。上述の NAV 分岐で既に return しているため通常ここには来ない）。
        if self._mode in ('HOME_NAV', 'PREP'):
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
