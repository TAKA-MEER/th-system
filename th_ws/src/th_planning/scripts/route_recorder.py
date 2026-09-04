#!/usr/bin/env python3
# ============================================================
# route_recorder — 教示経路の記録（demo-teach-replay / P3）
# ============================================================
# /system/effect の start_record / resume_record / finalize_route を受け、
# /odom のロボット姿勢を route_record_core で間引いて th_data/routes/<id>.json に
# 保存する。/route/catalog を latched で、/route/status を定期発行する。
# 姿勢サンプルは /system/state.state=="REC" の間だけ行い、PAUSE 中は積まない。
#
# 純コア (route_record_core) は P1 で完成済み。ここでは中身をコピペせず
# import して使う（追従ロジック follow_planner_core と同じ二層構造）。
import json
import math
import os
import time

import rclpy
import rclpy.time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import tf2_ros

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from th_system_msgs.msg import (RouteInfo, RouteList, RouteStatus, StateEffect,
                                SystemState)
from th_system_msgs.srv import OpenMapSession

# WS-9L: 教示の「保存」で地図を同期保存するための共通ヘルパー。
# 購読コールバック内からサービスを同期的に呼ぶには、main() の
# MultiThreadedExecutor + ReentrantCallbackGroup（/system/effect 購読）が必要。
from th_config_manager.service_call import call_and_wait


def _points_to_path(points, frame_id='odom', stamp=None):
    """[(x,y,yaw),...] を nav_msgs/Path（frame_id）にする。"""
    path = Path()
    path.header.frame_id = frame_id
    if stamp is not None:
        path.header.stamp = stamp
    for (x, y, _yaw) in points:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path

from th_planning.odom_source import pick_odom_source
from th_planning.route_record_core import (
    RouteRecorderCore, RouteRecordParams, autosave_path,
    _safe_id, decimate_polyline, finalize_route_file,
    list_finalized_route_files, owns_route_status, polyline_length, previous_path,
    route_from_dict,
    route_to_dict, save_route_atomic, should_autofinalize,
)


def _yaw_from_quat(q) -> float:
    """クォータニオン (w,x,y,z フィールドを持つ) から yaw [rad] を返す。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RouteRecorder(Node):
    def __init__(self):
        super().__init__('route_recorder')

        # ── パラメータ ──────────────────────────────────────
        # WAIVER(demo): W-03 — 数値は registry.yaml 経由でなくノード内リテラル既定値
        self.declare_parameter('routes_dir', '/root/th_data/routes')
        self.declare_parameter('sample_period_ms', 100)
        self.declare_parameter('sample_min_dist_m', 0.10)
        self.declare_parameter('sample_min_yaw_rad', 0.20)
        self.declare_parameter('status_period_ms', 500)
        # WS-9H: 記録中に途中経過を <id>.wip へ自動保存する間隔（ms）。0 以下なら
        # 自動保存しない。10 分級の教示中にルートが落ちても成果を失わないための保険。
        self.declare_parameter('autosave_period_ms', 10000)
        # WS-9F: /route/preview は表示専用なので、長距離で配信量が線形に増えて
        # 無線を食い潰すのを防ぐため間引いて publish する（既定 400 点）。
        self.declare_parameter('preview_max_points', 400)
        # WebUI の経路プレビューはロボット中心・固定倍率で描くので、この pose が
        # 画面全体の位置を決める。2Hz だと 1 更新あたり旋回 14°（w_max=0.5rad/s）＝
        # 半径 10m の点群外周で 47px の一発ジャンプになり「ガクガク」に見える
        # （2026-09-02 実機報告）。status(2Hz) から分離して 10Hz で出す。
        # 2026-09-01 に 10Hz→2Hz へ下げた理由はブロッキング lookup_transform で
        # executor が半分止まることだったが、その timeout は同じコミット(b7fa24a)で
        # 除去済み（_map_pose の docstring 参照）。レートを戻しても再現しない。
        self.declare_parameter('pose_period_ms', 100)
        # WS-8B: use_map_frame（launch が enable_route_slam のとき true）でだけ
        # slam_toolbox の map→base_link を使って map フレームで記録する。
        # 既定 false ＝ 従来どおり /odom のみ・TF リスナも作らない（通常起動で
        # 挙動を一切変えない）。
        self.declare_parameter('use_map_frame', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # WS-9K-B: 地図セッション ID。bringup.launch.py が 1 回だけ生成し、
        # route_recorder と replay_runner の両方へ同じ値を渡す（各ノードが自分で
        # 作ると一致しない）。map フレームで記録した経路はこの ID を持ち、
        # 再生側が同じセッションのときだけ追わせる。
        self.declare_parameter('map_session_id', '')
        # WS-9D: 自己位置源の EKF 出力 (/odometry/filtered) を優先し、古ければ
        # 生 /odom にフォールバックする。両ノードで同名・同既定にすること。
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_filtered_topic', '/odometry/filtered')
        self.declare_parameter('odom_stale_ms', 500)
        self._routes_dir = self.get_parameter('routes_dir').value
        sample_ms = self.get_parameter('sample_period_ms').value
        status_ms = self.get_parameter('status_period_ms').value
        self._use_map_frame = bool(self.get_parameter('use_map_frame').value)
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._map_session_id = self.get_parameter('map_session_id').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._odom_filtered_topic = self.get_parameter('odom_filtered_topic').value
        self._odom_stale_ms = int(self.get_parameter('odom_stale_ms').value)
        self._preview_max_points = int(self.get_parameter('preview_max_points').value)
        # WS-9H: 途中経過の自動保存。0 以下なら無効。_status_timer(2Hz) の中から
        # 前回時刻と比べて間隔が来たときだけ書く（専用タイマは増やさない）。
        self._autosave_period_s = (int(self.get_parameter('autosave_period_ms').value)
                                   / 1000.0)

        # ── 状態 ────────────────────────────────────────────
        self._recorder: RouteRecorderCore | None = None
        self._route_id: str | None = None
        self._name: str = ""
        self._rec_started_ms: int = 0
        # WS-9K-E2: 記録側の「ファイルに書けたか」フラグ。finalize_route_file に
        # 成功したときだけ true。「保存しました」は FSM の SAVED でなくこれを見る
        # （S13TeachManual.jsx）。新しい記録が始まったら false に戻す。
        self._saved = False
        self._pose: tuple[float, float, float] | None = None
        self._frame_id: str = 'odom'   # 記録中の経路フレーム（start_record で確定）
        self._mode: str = ""
        self._state: str = ""
        # WS-9D: 自己位置源の選択（生 odom / EKF 出力）。(pose, stamp_ms) の組。
        # stamp は受信時刻（_now_ms）を使い、ノード時計と同一基準にする。
        self._raw_sample: tuple[tuple[float, float, float], int] | None = None
        self._filtered_sample: tuple[tuple[float, float, float], int] | None = None
        self._current_source: str | None = None   # 直近の選択出所（切替ログ用）
        # WS-9H: 自動保存の状態（間引き用の記録）。start_record ごとに
        # _reset_autosave_state でまっさらに戻す（2 本目の教示で前の点数列が
        # 残って抑止されないため）。
        self._reset_autosave_state()

        self._tf_buffer = None
        self._tf_listener = None
        if self._use_map_frame:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Subscribers ────────────────────────────────────
        # WS-9L: /system/effect と /system/state は callback 内から別サービス
        # (/map_session/open save) を同期的に呼ぶ。単一スレッド executor だと応答
        # コールバックが動けずデッドロックするので、ReentrantCallbackGroup に乗せ、
        # main() 側で MultiThreadedExecutor を使う（slam_control.py と同じ構成）。
        sub_cbg = ReentrantCallbackGroup()
        effect_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(StateEffect, '/system/effect', self._on_effect,
                                 effect_qos, callback_group=sub_cbg)

        state_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(SystemState, '/system/state', self._on_state,
                                 state_qos, callback_group=sub_cbg)

        # WS-9L: 教示の「保存」で地図を凍結保存する。client は既定グループに置き、
        # effect コールバック（Reentrant）と並行実行できるようにする。
        self._map_session_client = self.create_client(
            OpenMapSession, '/map_session/open')

        # WS-9D: 同じ QoS で /odom（フォールバック源）と /odometry/filtered（EKF 出力、
        # 優先源）の両方を購読する。 /odom 購読はフォールバックに要るので消さない。
        odom_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                              history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, odom_qos)
        self.create_subscription(
            Odometry, self._odom_filtered_topic, self._on_odom_filtered, odom_qos)

        # ── Publishers ─────────────────────────────────────
        routes_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_routes = self.create_publisher(RouteList, '/route/catalog', routes_qos)

        status_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_status = self.create_publisher(RouteStatus, '/route/status', status_qos)
        self._pub_preview = self.create_publisher(Path, '/route/preview', status_qos)
        # WS-8B: WebUI が地図上にロボットを描くための pose（map or odom フレーム）。
        # rosbridge から TF ルックアップは扱いにくいのでノード側で解決して publish する。
        self._pub_robot_pose = self.create_publisher(
            PoseStamped, '/route/robot_pose', status_qos)

        # ── Timers ─────────────────────────────────────────
        self.create_timer(sample_ms / 1000.0, self._sample_timer)
        self.create_timer(status_ms / 1000.0, self._status_timer)
        # pose は表示の滑らかさに直結するので status とは別周期で回す。
        # use_map_frame が false のときは _publish_robot_pose が即 return するので
        # 通常起動での負荷増は無い。
        self.create_timer(
            self.get_parameter('pose_period_ms').value / 1000.0,
            self._publish_robot_pose)

        # 起動時に既存経路を latched publish する
        self._publish_routes_list()
        # WS-9H: 前回の教示が保存されずに終わった証拠（消し損ねた途中経過）を報告する。
        self._warn_leftover_autosaves()

        self.get_logger().info(
            f'route_recorder 起動 (routes_dir={self._routes_dir})')

    # ── フレーム解決 ──────────────────────────────────────
    def _map_pose(self):
        """map→base_link TF から (x, y, yaw) を返す。取れなければ None。

        **非ブロッキング**。timeout を渡すと単一スレッド executor 上で TF 到着
        コールバックが動けず必ずタイムアウトまで固まる（10Hz で呼ぶとノードが
        半分止まる）。バッファに既にあるものだけ読む。
        """
        if self._tf_buffer is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y, _yaw_from_quat(tf.transform.rotation))

    def _current_session_for_route(self) -> str:
        """記録中の経路に付けるセッション ID。map フレームのときだけ持つ。

        odom フレーム経路は地図に依存しないので ""（制限しない）。map フレームで
        記録するときは bringup が渡した共通セッション ID を付ける（WS-9K-B）。
        """
        if self._frame_id == self._map_frame:
            return self._map_session_id
        return ""

    def _publish_robot_pose(self):
        """WebUI 用に現在ロボット pose を map（あれば）or odom フレームで publish。"""
        if not self._use_map_frame:
            return
        mp = self._map_pose()
        if mp is not None:
            (x, y, yaw), frame = mp, self._map_frame
        else:
            op = self._odom_pose()
            if op is None:
                return
            (x, y, yaw), frame = op, 'odom'
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub_robot_pose.publish(ps)

    # ── 購読コールバック ──────────────────────────────────
    def _on_effect(self, msg: StateEffect):
        name = msg.name
        if name == 'start_record':
            args = json.loads(msg.args_json or '{}')
            route_id = args.get('route_id') or ''
            if not route_id:
                route_id = time.strftime('route_%Y%m%d_%H%M%S')
            self._route_id = route_id
            self._name = args.get('name') or route_id
            params = RouteRecordParams(
                sample_min_dist_m=self.get_parameter('sample_min_dist_m').value,
                sample_min_yaw_rad=self.get_parameter('sample_min_yaw_rad').value)
            self._recorder = RouteRecorderCore(params)
            # WS-9H-2: 新しい recorder は点数が 0 から数え直しになるので、前の教示の
            # 点数列が間引き比較に残らないよう自動保存の状態をまっさらに戻す。
            self._reset_autosave_state()
            # WS-9K-E2: 新しい記録はまだ保存していないので saved を false に戻す。
            self._saved = False
            # WS-8B: 記録開始時に map TF があれば経路全体を map フレームで記録する。
            # 無ければ従来どおり /odom。フレームは記録中ずっと固定（混ぜない）。
            mp = self._map_pose()
            if mp is not None:
                self._frame_id = self._map_frame
                self._recorder.start(*mp)
            else:
                # WS-9D: odom 開始姿勢も自己位置源の選択結果を使う。
                op = self._odom_pose()
                if op is not None:
                    self._frame_id = 'odom'
                    self._recorder.start(*op)
                else:
                    self._frame_id = 'odom'
                    self.get_logger().warn('odom 未受信のため (0,0,0) で記録開始')
                    self._recorder.start(0.0, 0.0, 0.0)
            self._rec_started_ms = self._now_ms()
            self.get_logger().info(
                f'記録開始: route_id={self._route_id} frame={self._frame_id}')
        elif name == 'resume_record':
            if self._recorder is None:
                self.get_logger().warn('resume_record を受けたが記録中でない（無視）')
                return
            self._recorder.resume()
        elif name == 'finalize_route':
            if self._recorder is None:
                self.get_logger().warn('finalize_route を受けたが記録中でない（無視）')
                return
            # WS-9H: 旧版退避(.prev)・原子的書き込み・途中経過(.wip)の後始末を
            # finalize_route_file が一括で担う（EXCEPTION-LEDGER W-04 を CLOSED に）。
            # WS-9K-D: 通常の finalize でも保存後に必ず記録を閉じる（_finalize_and_close
            # が self._recorder を None にする）。閉じないと SAVED 後にスティックへ
            # 触れた resume_record で保存済みの記録が再開してしまう。
            dest, route = self._finalize_and_close()
            if os.path.exists(previous_path(self._routes_dir, self._route_id)):
                prev_name = os.path.basename(
                    previous_path(self._routes_dir, self._route_id))
                self.get_logger().warn(
                    f'同名の経路があったため {prev_name} として退避しました')
            # RouteData は point_count / length_m を持たない（route_to_dict の出力だけが持つ）
            self.get_logger().info(
                f'保存: {dest} ({len(route.points)} 点, '
                f'{polyline_length(route.points):.2f} m)')
            self._publish_routes_list()
        else:
            self.get_logger().debug(f"無視する effect: {name}")

    def _on_state(self, msg: SystemState):
        new_mode = msg.mode
        # WS-9K-D: 記録中に教示系モードから出た（重大フォルト C-06a 等で TEACH_*
        # → ESTOP/IDLE）ら、FSM はもう finalize_route を発行できないため、ここで
        # 記録を保存して閉じる（無駄に捨てない）。通常の finalize は分岐側。
        if new_mode != self._mode and should_autofinalize(
                self._mode, new_mode, self._recorder is not None):
            self._autofinalize_mode_left(new_mode)
        self._mode = new_mode
        self._state = msg.state

    def _autofinalize_mode_left(self, new_mode: str):
        """記録中に教示系モードを出たので保存して閉じる（WS-9K-D）。

        finalize_route_file が成功したら warn で理由と保存先を残す。失敗時も
        記録は閉じる（次の教示で古い点数を引きずらない・進めない）。
        """
        try:
            dest, _route = self._finalize_and_close()
        except Exception as e:
            self.get_logger().error(f'教示中モード逸脱の自動保存に失敗: {e}')
            self._close_recording()
            return
        self.get_logger().warn(
            f'教示中にモードが {new_mode} へ移ったため、記録を自動保存した: {dest}')

    def _finalize_and_close(self):
        """現在の記録を RouteData に落とし finalize_route_file で保存して閉じる。

        map フレーム記録時だけ地図セッション ID を刻む（odom は ""＝制限しない）。
        WS-9K-D: 呼び出し元（通常の finalize_route / モード逸脱の自動保存）のどちら
        でも、保存後は必ず記録を閉じる（self._recorder = None）。いま閉じないと、
        SAVED 後にスティックへ触れた resume_record で保存済みの記録が再開してしまう。
        """
        route = self._recorder.finalize(
            self._route_id, self._name, self._now_ms(),
            frame_id=self._frame_id, map_session_id=self._current_session_for_route())
        dest = finalize_route_file(self._routes_dir, self._route_id,
                                   route_to_dict(route))
        # WS-9L: 経路が map フレームで記録されたときだけ、地図を凍結保存する
        # （route_id をセッション ID にして /map_session/open save）。経路の保存
        # が成功した直後に呼ぶ。失敗しても経路の保存は成功扱いのまま（_saved は
        # 真）で、warn にだけ残す。
        if self._frame_id == self._map_frame:
            self._save_map_for_route(self._route_id)
        # WS-9K-E2: ファイルに書けた（finalize_route_file が例外なしで返った）
        # ときだけ true。「保存しました」はこれを見る。FSM の SAVED は見ない。
        self._saved = True
        self._close_recording()
        return dest, route

    def _save_map_for_route(self, route_id: str):
        """教示の地図を /map_session/open で凍結保存する（失敗は warn のみ）。"""
        if not self._map_session_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'地図を保存できなかったので再生できない可能性がある: '
                f'/map_session/open に接続できません (id={route_id})')
            return
        # 送る側で正規化する（route_record_core._safe_id と同じ規則）。経路 JSON の
        # 保存名（finalized_path が内部で _safe_id する）と地図ファイル名を一致させる
        # ため。受ける側（slam_control_logic）は未正規化 id を拒否するだけ。
        session_id = _safe_id(route_id)
        req = OpenMapSession.Request(
            slot='ROUTE', session_id=session_id, mode='save')
        try:
            _resp, err = call_and_wait(
                self, self._map_session_client, req, 30.0)
        except Exception as e:   # noqa: BLE001
            self.get_logger().warn(
                f'地図を保存できなかったので再生できない可能性がある: {e} '
                f'(id={route_id})')
            return
        if err:
            self.get_logger().warn(
                f'地図を保存できなかったので再生できない可能性がある: {err} '
                f'(id={route_id})')
            return

    def _close_recording(self):
        """記録を閉じて自動保存の状態もまっさらに戻す。

        WS-9K-D: ソース上で self._recorder を None にする唯一の箇所。
        """
        self._recorder = None
        self._reset_autosave_state()

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, _yaw_from_quat(msg.pose.pose.orientation))
        self._raw_sample = (self._pose, self._now_ms())

    def _on_odom_filtered(self, msg: Odometry):
        p = msg.pose.pose.position
        self._filtered_sample = (
            (p.x, p.y, _yaw_from_quat(msg.pose.pose.orientation)), self._now_ms())

    # ── タイマ ────────────────────────────────────────────
    def _odom_pose(self):
        """自己位置源の選択を通した odom フレームの現在 pose（無ければ None）。"""
        pose, source = pick_odom_source(
            self._filtered_sample, self._raw_sample,
            self._now_ms(), self._odom_stale_ms)
        self._note_odom_source(source)
        return pose

    def _note_odom_source(self, source):
        """自己位置源が切り替わったときだけ 1 回ログする（毎ティック出さない）。"""
        if source == self._current_source:
            return
        prev = self._current_source
        self._current_source = source
        if prev is None:
            return  # 初回は切替ではない（移り変わりの起点）ので出さない
        if source == 'raw' and prev == 'filtered':
            self.get_logger().info(
                f'自己位置源を filtered → raw に切替'
                f'（EKF 出力が {self._odom_stale_ms}ms 途絶）')
        elif source is None:
            self.get_logger().info('自己位置源が利用不可（filtered/raw 両方途絶）')
        else:
            self.get_logger().info(f'自己位置源を {prev} → {source} に切替')

    def _sample_timer(self):
        # PAUSE 中は積まない。REC の間だけ姿勢を間引いて記録する。
        if self._recorder is None or self._state != 'REC':
            return
        # 記録フレームに合わせて pose を取る。map フレーム記録中に TF が
        # 一時的に切れたらそのサンプルは飛ばす（フレームを混ぜない）。
        if self._frame_id == self._map_frame:
            pose = self._map_pose()
        else:
            # WS-9D: EKF 出力が新鮮ならそれ、古ければ生 odom にフォールバック。
            pose = self._odom_pose()
        if pose is None:
            return
        self._recorder.add_pose(*pose)

    def _status_timer(self):
        # pose は専用の 10Hz タイマ（pose_period_ms）で出す。ここでは出さない。
        msg = RouteStatus()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.target_index = -1   # 記録側は目標点を持たない
        # WS-9K-E2: 「ファイルに書けたか」を載せる（FSM の SAVED ではない）。
        msg.saved = self._saved
        if self._recorder is None:
            msg.state = self._state or 'NONE'
            msg.recorded_m = 0.0
            msg.points = 0
            msg.elapsed_sec = 0.0
            preview_points = []
        else:
            msg.state = self._state or 'NONE'
            msg.recorded_m = float(self._recorder.recorded_m)
            msg.points = self._recorder.point_count
            msg.elapsed_sec = (self._now_ms() - self._rec_started_ms) / 1000.0
            preview_points = decimate_polyline(self._recorder.points,
                                               self._preview_max_points)
        # WS-9Q: 教示系モードのときだけ出す。再生中も出していたため points=0 が
        # replay_runner の実データと交互配信になり、state_manager の route_loaded が
        # 往復して「再生」が拒否されていた（実機 2026-09-04）。
        # /route/preview は WS-6.4 で同じ理由から既にゲート済みだった。
        if owns_route_status(self._mode, recorder=True):
            self._pub_status.publish(msg)
        # #4: 記録中だけ /route/preview を出す（空 Path を出すと再生側と交互配信になり
        # WebUI のプレビューが点滅する）。フレームは記録中の経路フレーム。
        if self._recorder is not None and preview_points:
            self._pub_preview.publish(_points_to_path(
                preview_points, frame_id=self._frame_id, stamp=stamp))
        # WS-9H: 記録中だけ、間隔が来て点が増えていたら途中経過を .wip へ自動保存する。
        self._maybe_autosave()

    def _reset_autosave_state(self):
        """自動保存の間引き用の記録をまっさらに戻す。

        start_record は毎回新しい RouteRecorderCore を作り点数が 0 から数え直しに
        なるため、前の教示の点数が残っていると 2 本目の自動保存が丸ごと抑止される。
        resume_record は同じ recorder を使い続けるので呼ばない。
        """
        self._last_autosave_s = 0.0
        self._last_autosaved_points = 0
        self._autosave_logged = False

    def _maybe_autosave(self):
        """記録中に途中経過を 1 世代前と混ざらない .wip へ定期的に保存する。

        専用タイマは持たない（executor 負荷を読みにくくするため）。_status_timer の
        中から前回時刻と比べて間隔が来たときだけ書く。点が増えていない間は同じ内容を
        書き直さない。ログは最初の 1 回だけ出す（毎回出さない。ログで CPU を食う前科）。
        """
        if self._recorder is None or self._autosave_period_s <= 0:
            return
        if self._recorder.point_count <= self._last_autosaved_points:
            return   # 点が増えていないときは書かない
        now_s = time.monotonic()
        if now_s - self._last_autosave_s < self._autosave_period_s:
            return
        route = self._recorder.finalize(
            self._route_id, self._name, self._now_ms(), frame_id=self._frame_id,
            map_session_id=self._current_session_for_route())
        path = autosave_path(self._routes_dir, self._route_id)
        save_route_atomic(path, route_to_dict(route))
        if not self._autosave_logged:
            self.get_logger().info(f'教示の途中経過を自動保存しています: {path}')
            self._autosave_logged = True
        self._last_autosave_s = now_s
        self._last_autosaved_points = self._recorder.point_count

    # ── 起動時チェック ─────────────────────────────────────
    def _warn_leftover_autosaves(self):
        """前回の教示が保存されずに終わった証拠（消し損ねた .wip）を warn で列挙する。

        自動で復元まではしない（FSM の状態を作り直す必要があり範囲が大きい）。
        """
        if not os.path.isdir(self._routes_dir):
            return
        for fname in sorted(os.listdir(self._routes_dir)):
            if fname.endswith('.wip'):
                path = os.path.join(self._routes_dir, fname)
                self.get_logger().warn(
                    f'前回の教示の途中経過が残っています: {path}'
                    f'（保存されずに終了した可能性）。使うときは .json にリネームしてください')

    # ── 経路一覧 publish ──────────────────────────────────
    def _publish_routes_list(self):
        if not os.path.isdir(self._routes_dir):
            self.get_logger().debug(
                f'routes_dir が存在しないため /route/catalog は空: {self._routes_dir}')
            self._pub_routes.publish(RouteList())
            return
        infos = []
        # WS-9H: .wip/.prev は一覧に出さない（除外判定は list_finalized_route_files）。
        for fname in list_finalized_route_files(self._routes_dir):
            path = os.path.join(self._routes_dir, fname)
            try:
                with open(path, encoding='utf-8') as f:
                    route = route_from_dict(json.load(f))
            except Exception as e:
                self.get_logger().warn(f'経路 JSON を読み込めないため skip: {path}: {e}')
                continue
            info = RouteInfo()
            info.id = route.id
            info.name = route.name
            info.generation = route.generation
            info.length_m = float(polyline_length(route.points))
            info.point_count = len(route.points)
            info.start_yaw = float(route.start_yaw)
            info.recorded_at = _ms_to_time(route.recorded_at_ms)
            infos.append(info)
        self._pub_routes.publish(RouteList(routes=infos))
        self.get_logger().debug(f'/route/catalog を publish: {len(infos)} 件')

    # ── ユーティリティ ────────────────────────────────────
    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000


def _ms_to_time(ms: int) -> TimeMsg:
    """エポックからのミリ秒を builtin_interfaces/Time に変換する。"""
    t = TimeMsg()
    t.sec = int(ms // 1000)
    t.nanosec = int((ms % 1000) * 1_000_000)
    return t


def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()
    # WS-9L: /system/effect コールバック内から /map_session/open を同期的に呼ぶため
    # MultiThreadedExecutor + ReentrantCallbackGroup を使う（slam_control.py と同じ
    # 構成。単一スレッド executor だと応答コールバックが動けず保存で固まる）。
    executor = MultiThreadedExecutor()
    node._executor = executor
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
