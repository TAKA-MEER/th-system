#!/usr/bin/env python3
# ============================================================
# person_tracker_bridge — human_kenchi (LiDAR脚検出) ブリッジ
#
# human_kenchi (multiple_sensor_person_tracking::PersonTracker, leg モード) が
# 発行する following_position + stop_following を th_system_msgs/PersonStatus に
# 変換して /person/status に発行する。
#
# following_position.status:
#   0 = NO_EXISTS  (脚未検出)   → is_lost=True,  lost_reason="DETECTION_LOST"
#   1 = EXISTS_LEG (脚追跡中)   → is_lost=False, lost_reason=""
# stop_following (Bool): detection_lost or target_changed_latched_ (対象切替の疑い)。
#   status=EXISTS_LEG のまま stop_following=True の場合は対象切替とみなし
#   is_lost=True, lost_reason="TARGET_SWITCHED" にして追従側を停止させる
#   (切替そのものは PersonTracker が既に内部検知しているが、以前はこの信号が
#   どこにも購読されておらず追従ロジックに一切伝わっていなかった)。
#
# following_position と stop_following はそれぞれ別トピックで publish されるため、
# 2トピック間の到着順序に依存しないよう、どちらのコールバックが来ても直近の
# 両方の値から再計算・再 publish する(古くとも最大1トラッカー周期分のずれ)。
#
# target_frame launch 引数を "base_link" にして起動する前提のため、
# following_position.pose.position はそのまま base_link 基準相対座標として使える。
#
# WP-ONSITE-F1 追加:
#   - /person/targets (PersonTargets) を publish。候補一覧＋選択中を 1 本に。
#   - /system/effect の set_target / clear_selection を tracker のサービス呼び出しに変換。
#   - evt.target_lost (edge) / evt.auto_selected を /system/event に出す。
# ============================================================
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))  # person_tracker_bridge_core.py と同じディレクトリ

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from th_system_msgs.msg import PersonStatus, PersonTargets, StateEffect, StateEvent
from multiple_sensor_person_tracking.msg import FollowingPosition, PersonCandidates
from multiple_sensor_person_tracking.srv import SelectTarget
from std_srvs.srv import Trigger

from person_tracker_bridge_core import (
    classify,
    match_selected_index,
    auto_select_step,
    apply_lost_grace,
    AutoSelectState,
    LostGraceState,
    STATUS_EXISTS_LEG,
)

# WAIVER(demo): W-13 — ノード内リテラル既定値
DEFAULT_AUTO_SELECT_HOLD_S = 2.0
DEFAULT_MATCH_TOL_M = 0.35
DEFAULT_TARGET_CONFIDENCE_MIN = 0.5
# brief-onsite-ux2 F-4: 脚検出は歩行中に一瞬抜けるため、is_lost=True の
# フレームが1枚来ただけで即 evt.target_lost を出すと点滅して見える
# （実機ログで確定）。1500ms は仮の既定値。
DEFAULT_LOST_GRACE_MS = 1500


class PersonTrackerBridge(Node):
    def __init__(self):
        super().__init__('person_tracker_bridge')

        self._pub_status = self.create_publisher(PersonStatus, '/person/status', 10)
        self._pub_targets = self.create_publisher(
            PersonTargets, '/person/targets',
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST))

        event_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                               history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_event = self.create_publisher(StateEvent, '/system/event', event_qos)

        # ── パラメータ（# WAIVER(demo): W-13）────────────────
        self.declare_parameter('auto_select_hold_s', DEFAULT_AUTO_SELECT_HOLD_S)
        self.declare_parameter('match_tol_m', DEFAULT_MATCH_TOL_M)
        self.declare_parameter('target_confidence_min', DEFAULT_TARGET_CONFIDENCE_MIN)
        self.declare_parameter('candidates_topic',
                               'sobits_follower/multiple_sensor_person_tracking/person_candidates')
        self.declare_parameter('select_service', '/person_tracker/select_target')
        self.declare_parameter('reset_service', '/person_tracker/reset_tracking')
        # brief-onsite-ux2 F-4: is_lost のデバウンス（猶予）。
        self.declare_parameter('tracker_lost_grace_ms', DEFAULT_LOST_GRACE_MS)

        self._auto_select_hold_s = float(self.get_parameter('auto_select_hold_s').value)
        self._match_tol_m = float(self.get_parameter('match_tol_m').value)
        self._target_conf_min = float(self.get_parameter('target_confidence_min').value)
        candidates_topic = self.get_parameter('candidates_topic').value
        select_service = self.get_parameter('select_service').value
        reset_service = self.get_parameter('reset_service').value
        self._lost_grace_ms = int(self.get_parameter('tracker_lost_grace_ms').value)

        # ── 全購読を 1 つの MutuallyExclusive グループに入れる。
        # _recompute_and_publish は 4 コールバック（following / stop / candidates /
        # effect）から呼ばれ共有状態（_candidates / _selected_index / _auto_select /
        # _prev_lost）を触るので、MultiThreadedExecutor 下で同時実行させない。
        # サービスクライアントの done コールバックだけログ用に Reentrant で分離。
        sub_cbg = MutuallyExclusiveCallbackGroup()
        self._svc_cbg = ReentrantCallbackGroup()

        # ── 状態 ────────────────────────────────────────────
        self._last_status = STATUS_EXISTS_LEG  # 最初のメッセージが来るまでの楽観的初期値
        self._last_position = (0.0, 0.0)
        self._last_stamp = None
        self._stop_following = False
        self._candidates = []            # list[tuple[float,float]] base_link 系
        self._prev_lost = None           # evt.target_lost の edge 検出用
        self._auto_select = AutoSelectState()
        self._selected_index = -1        # マッチングによる選択中 index
        self._auto_selected = False      # 自動選択発火済みフラグ（1人継続で再発火しない）
        self._lost_grace = LostGraceState()  # brief-onsite-ux2 F-4: is_lost のデバウンス

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(
            FollowingPosition,
            'sobits_follower/multiple_sensor_person_tracking/following_position',
            self._following_cb,
            10,
            callback_group=sub_cbg,
        )
        self.create_subscription(
            Bool,
            'sobits_follower/multiple_sensor_person_tracking/stop_following',
            self._stop_cb,
            10,
            callback_group=sub_cbg,
        )
        self.create_subscription(
            PersonCandidates,
            candidates_topic,
            self._candidates_cb,
            10,
            callback_group=sub_cbg,
        )
        self.create_subscription(
            StateEffect,
            '/system/effect',
            self._on_effect,
            event_qos,
            callback_group=sub_cbg,
        )

        # ── Service clients (stub 運用では未提供 → wait_for_service で見極める) ──
        self._select_client = self.create_client(
            SelectTarget, select_service, callback_group=self._svc_cbg)
        self._reset_client = self.create_client(
            Trigger, reset_service, callback_group=self._svc_cbg)

        self.get_logger().info(
            'person_tracker_bridge 起動 '
            '(human_kenchi → /person/status + /person/targets, '
            f'auto_select_hold={self._auto_select_hold_s}s, match_tol={self._match_tol_m}m)')

    def _following_cb(self, msg: FollowingPosition):
        self._last_status = msg.status
        self._last_position = (msg.pose.position.x, msg.pose.position.y)
        self._last_stamp = msg.header.stamp
        self._recompute_and_publish()

    def _stop_cb(self, msg: Bool):
        self._stop_following = msg.data
        self._recompute_and_publish()

    def _candidates_cb(self, msg: PersonCandidates):
        self._candidates = [(p.x, p.y) for p in msg.positions]
        self._recompute_and_publish()

    # ── 再計算・再 publish ───────────────────────────────
    def _recompute_and_publish(self):
        now_ms = int(self.get_clock().now().nanoseconds / 1e6)

        decision = classify(self._last_status, self._stop_following)
        # brief-onsite-ux2 F-4: 猶予（デバウンス）。脚検出が歩行中に一瞬抜けた
        # だけの is_lost=True を tracker_lost_grace_ms の間は隠す（点滅対策）。
        # evt.target_lost は下の edge 検出がこの masked な is_lost しか見ないため、
        # 猶予中に一度も発行されない（復帰時にイベントを出さない、を自動的に満たす）。
        self._lost_grace, decision = apply_lost_grace(
            self._lost_grace, decision, now_ms, self._lost_grace_ms)
        is_lost = decision.is_lost
        lost_reason = decision.lost_reason

        # 選択中 index（追跡座標に最も近い候補）
        followed_xy = None if is_lost else self._last_position
        self._selected_index = match_selected_index(
            self._candidates, followed_xy, is_lost, self._match_tol_m)

        # 自動選択: 候補がちょうど1つ・まだ選ばれていない・追跡継続中のみ
        already_selected = self._selected_index >= 0 or self._auto_selected
        state, idx = auto_select_step(
            self._auto_select, len(self._candidates), now_ms,
            int(self._auto_select_hold_s * 1000), already_selected)
        self._auto_select = state
        if idx >= 0:
            self._auto_selected = True
            self._selected_index = idx
            self.get_logger().info(
                f'自動選択: 候補1つが {self._auto_select_hold_s}s 継続 → select_target(0)')
            self._call_select(idx)
            self._emit_event('evt.auto_selected', json.dumps({'index': idx}))

        # evt.target_lost の edge 検出（連続 lost では出し続けない）
        if self._prev_lost is not None and is_lost and not self._prev_lost:
            self._emit_event('evt.target_lost')
        self._prev_lost = is_lost

        # /person/targets を publish
        targets = PersonTargets()
        targets.header.stamp = self._last_stamp if self._last_stamp is not None else self.get_clock().now().to_msg()
        targets.header.frame_id = 'base_link'
        for (cx, cy) in self._candidates:
            p = Point()
            p.x = cx
            p.y = cy
            targets.candidates.append(p)
        targets.selected_index = self._selected_index
        targets.confidence = decision.confidence
        targets.is_lost = is_lost
        targets.lost_reason = lost_reason
        self._pub_targets.publish(targets)

        # /person/status は後方互換のため従来どおり
        out = PersonStatus()
        out.header.stamp = targets.header.stamp
        out.header.frame_id = 'base_link'
        out.position.x = self._last_position[0]
        out.position.y = self._last_position[1]
        out.position.z = 0.0
        out.confidence = decision.confidence
        out.is_lost = decision.is_lost
        out.lost_reason = decision.lost_reason
        self._pub_status.publish(out)

    # ── /system/effect ───────────────────────────────────
    def _on_effect(self, msg: StateEffect):
        if msg.dest != 'person_tracker':
            return
        if msg.name == 'set_target':
            try:
                args = json.loads(msg.args_json or '{}')
                index = int(args['index'])
            except Exception:
                self.get_logger().warn(
                    f'set_target: args_json 解釈不能 ({msg.args_json!r}) スキップ')
                return
            self.get_logger().info(f'set_target: select_target(candidate_index={index})')
            self._auto_selected = False
            self._call_select(index)
        elif msg.name == 'clear_selection':
            self.get_logger().info('clear_selection: reset_tracking()')
            self._auto_selected = False
            self._call_reset()
        else:
            self.get_logger().debug(f'effect {msg.name} は未処理 (dest={msg.dest})')

    # ── tracker サービス呼び出し（非同期・失敗はログのみ）──
    # WS-9AE(2026-09-11): wait_for_service(timeout_sec=0.5) は _on_effect（sub_cbg,
    # MutuallyExclusive）の中で同期ブロッキングしていた。この間 _following_cb/
    # _candidates_cb（同じグループ）が止まり、タップのたびに追跡パイプライン全体が
    # 最大 0.5s 固まる（venue_navigator で 2026-09-10 に直したのと同じアンチ
    # パターン）。サービスは起動直後を過ぎればほぼ常に ready なので、非ブロッキング
    # 判定に変える（一回のタップに対応する再試行ループが無いので、ready でない
    # ときは素直に諦める。旧版の「0.5s 待って探す」猶予は失うが、実運用ではその
    # 猶予より「毎タップ最大0.5s固まる」実害の方が大きい）。
    def _call_select(self, index: int):
        if not self._select_client.service_is_ready():
            self.get_logger().warn(
                f'select_target サービス未 ready スキップ index={index}')
            return
        req = SelectTarget.Request()
        req.candidate_index = index
        req.x = 0.0
        req.y = 0.0
        future = self._select_client.call_async(req)
        future.add_done_callback(self._select_done)

    def _select_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f'select_target 呼び出し失敗: {e}')
            return
        if resp is not None and not resp.success:
            self.get_logger().warn(f'select_target 失敗: {resp.message}')

    def _call_reset(self):
        if not self._reset_client.service_is_ready():
            self.get_logger().warn('reset_tracking サービス未 ready スキップ')
            return
        req = Trigger.Request()
        future = self._reset_client.call_async(req)
        future.add_done_callback(self._reset_done)

    def _reset_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f'reset_tracking 呼び出し失敗: {e}')
            return
        if resp is not None and not resp.success:
            self.get_logger().warn(f'reset_tracking 失敗: {resp.message}')

    # ── /system/event ────────────────────────────────────
    def _emit_event(self, event: str, arg_json: str = "{}"):
        ev = StateEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.event = event
        ev.source_node = 'person_tracker_bridge'
        ev.arg_json = arg_json
        self._pub_event.publish(ev)
        self.get_logger().info(f'/system/event 発行: {event} {arg_json}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():  # SIGTERM 時は既定シグナルハンドラが context を落としている
            rclpy.shutdown()


if __name__ == '__main__':
    main()