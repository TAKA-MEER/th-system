#!/usr/bin/env python3
# ============================================================
# replay_runner — 教示再生の走行（demo-teach-replay / P4）
# ============================================================
# /system/effect の load_route / rotate_to_start_yaw / resume_path を受け、
# route_replay_core の pure-pursuit で /cmd_vel_behavior (priority 20) に (v, w) を出す。
# 逆再生は点列反転。到着で evt.arrived を /system/event に返す。
#
# 特例（EXCEPTION-LEDGER W-01 / W-02。WS-8B で縮小）:
#   - map フレーム経路: slam_toolbox の map→base_link TF で追従。走行中も
#     map→odom 連続補正が効く（W-02 縮小）。load_route で保存 pgm 上の粗探索
#     （W-01 P2。localize_core の純関数）→ 最良候補を初期姿勢に reload →
#     TF 到着で evt.localize_done（arg_json{score, margin} 付き）。pgm 無しの
#     旧経路だけ始点決め打ち（localize_quality='unknown'）。
#   - odom フレーム経路（slam 未起動）: 従来どおり align_path_to_current で
#     現在地を始点とみなす・走行中補正なし。
# 速度指令の宛先は必ず /cmd_vel_behavior。/cmd_vel には直接 publish しない
# （obstacle_limiter だけが /cmd_vel の publisher。不変ルール WP-SAFE-03）。
# 純コア (route_replay_core / route_record_core) は import して使う（コピペしない）。
import json
import math
import os
import threading
import time

import rclpy
import rclpy.time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan

import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from th_system_msgs.msg import (RouteInfo, RouteStatus, StateEffect,
                                StateEvent, SystemState)
from th_system_msgs.srv import OpenMapSession

# WS-9L: 再生の経路選択で地図を読み直すための共通ヘルパー。
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
    _safe_id, can_replay_route, finalized_path, owns_route_status, polyline_length,
    route_from_dict)
from th_planning.route_replay_core import (
    ReplayParams, advance_index, align_path_to_current, cross_track_error,
    pure_pursuit, ramp_toward, reverse_points, rotate_toward,
    scale_replay_params,
)
from th_planning.localize_core import (
    LOCALIZE_DEFAULTS, judge_localize, load_pgm_map, search as localize_search,
)


# W-01 P2: 探索スレッドの待ち上限。P0 の実測は最大 ~8s（i7-1165G7。実機値は P6）。
# executor を止めないよう別スレッドで走らせ、ここを超えたら打ち切って
# timed_out として扱う。ROS パラメータ化はしない（registry 登録は P5）。
_LOCALIZE_SEARCH_TIMEOUT_S = 30.0
# 探索開始時に最新の /scan を待つ上限（LOCALIZE 中は機体が止まっている前提）。
_LOCALIZE_SCAN_WAIT_S = 3.0


def _yaw_from_quat(q) -> float:
    """クォータニオン (w,x,y,z フィールドを持つ) から yaw [rad] を返す。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _laser_pose_to_base(laser_pose, offset):
    """laser フレームの姿勢 (x, y, yaw) を base_link の姿勢に直す。

    offset は laser 基準の base_link 姿勢 (x, y, yaw)（_laser_to_base_offset）。
    合成: base_map = laser_map ⊕ offset（回転を畳み込む）。
    """
    lx, ly, lyaw = laser_pose
    ox, oy, oyaw = offset
    c, s = math.cos(lyaw), math.sin(lyaw)
    yaw = (lyaw + oyaw + math.pi) % (2 * math.pi) - math.pi
    return (lx + ox * c - oy * s, ly + ox * s + oy * c, yaw)


class ReplayRunner(Node):
    def __init__(self):
        super().__init__('replay_runner')

        # ── パラメータ ──────────────────────────────────────
        # W-03 解除: 挙動値（周期・先読み・速度・加減速・到着判定・待ち時間・
        # 鮮度）は registry.yaml 駆動。launch が渡す生成 yaml が上書きし、
        # そちらが正になる。既定値は起動単体のフォールバックとして残す。
        # 移さないもの: routes_dir（パス）・use_map_frame（配線フラグ）・
        # map_frame/base_frame（フレーム名）・map_session_id（ID）・
        # odom_topic/odom_filtered_topic（トピック名）。
        # 調整値ではなく配線設定のため registry の対象外
        # （th_config_manager/tunable_targets.py と同じ考え方）。
        self.declare_parameter('routes_dir', '/root/th_data/routes')
        self.declare_parameter('control_period_ms', 50)   # 20Hz
        self.declare_parameter('status_period_ms', 500)
        # WS-9F: 再生側の点列は load_route 後、走行中は一切変わらない。それを 2Hz で
        # 毎回まるごと送るのは無駄なので、点列が変わったとき＋このハートビート間隔
        # だけプレビューを配信する（既定 5000ms）。なお target_index で点の添字を
        # 参照するため、再生側は間引かない（全点のまま回数を減らす）。
        self.declare_parameter('preview_heartbeat_ms', 5000)
        # 経路プレビューはロボット中心・固定倍率なので、この pose が画面全体の
        # 位置を決める。2Hz では旋回 1 ステップ 14°＝外周 47px のジャンプになり
        # 「ガクガク」になる（2026-09-02 実機報告）。status から分離して 10Hz。
        # 詳細は route_recorder.py の同名パラメータのコメント参照。
        self.declare_parameter('pose_period_ms', 100)
        # 2026-09-05: 高速再生（cruise_speed_mps 最大端）でふらつきが実機で
        # 確認された。pure_pursuit は実質的に lookahead_m/cruise_speed_mps
        # （先読み時間）で damping が決まり、固定 lookahead のままだと cruise が
        # 上がるほど先読み時間が短くなって応答が過敏になる（低速 0.40/0.15≈2.7s
        # に対し旧デフォルトの高速は 0.40/0.45≈0.9s）。lookahead_m も
        # cruise_speed_mps と一緒にスケールするようにし（scale_replay_params）、
        # この値は「速度スケール最大端」（高速側）の先読み距離として引き上げた
        # （0.40→0.60。0.60/0.45≈1.33s、既存の中速 0.40/0.33≈1.2s 相当に近づける
        # 狙い）。低速側は replay_lookahead_min_m（既存の 0.40 のまま）で従来の
        # 挙動を保つ。実機未検証の見積もりなので、高速再生で実測しながら調整すること
        # （Spec.md §5 SD-7: 測らないと分からない値に事前の目標を置かない）。
        self.declare_parameter('lookahead_m', 0.60)
        # 2026-09-01: 実機で DRIVE_RUNAWAY を踏んだため巡航・旋回を下げてランプ化した（#2）。
        # WS-9T: これは「速度スケール最大端」。実効値は /replay/speed_scale の比率で
        # replay_cruise_min_mps〜この値 の間に補間される（scale_replay_params）。
        self.declare_parameter('cruise_speed_mps', 0.45)
        self.declare_parameter('max_yaw_rate_rps', 0.9)
        # WS-9T: 速度スケール最小端 ＋ WebUI 未接続時の既定比率。
        self.declare_parameter('replay_cruise_min_mps', 0.12)
        self.declare_parameter('replay_yaw_min_rps', 0.4)
        # 2026-09-05: lookahead の速度スケール最小端。既存の低速再生の挙動
        # （lookahead_m の旧既定値そのもの）をそのまま保つため 0.40 とする。
        self.declare_parameter('replay_lookahead_min_m', 0.40)
        self.declare_parameter('replay_speed_default_ratio', 0.6)
        self.declare_parameter('arrive_dist_m', 0.20)
        self.declare_parameter('yaw_tol_rad', 0.10)
        self.declare_parameter('linear_accel_mps2', 0.5)
        self.declare_parameter('angular_accel_rps2', 1.5)
        # WS-8B: use_map_frame（launch が enable_route_slam のとき true）でだけ
        # map→base_link TF を使う。既定 false ＝ 従来どおり /odom のみ・TF リスナも
        # 作らない（通常起動で挙動を一切変えない）。
        self.declare_parameter('use_map_frame', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # WS-9K-B: 地図セッション ID。bringup.launch.py が 1 回だけ生成し、
        # route_recorder と replay_runner の両方へ同じ値を渡す。load_route の経路が
        # map フレームなら、このセッション ID と一致したときだけ再生を進める。
        self.declare_parameter('map_session_id', '')
        self.declare_parameter('localize_wait_s', 5.0)
        # W-01 P2: 全域ローカライズの確度閾値。既定値は LOCALIZE_DEFAULTS
        # （P0 の測定記録が根拠）。registry への登録は P5 なのでやらない。
        # DetailedDesign-names.md §7 に新規名として予約済み。
        self.declare_parameter(
            'localize_match_low', float(LOCALIZE_DEFAULTS['localize_match_low']))
        self.declare_parameter(
            'localize_margin_low', float(LOCALIZE_DEFAULTS['localize_margin_low']))
        self.declare_parameter(
            'localize_margin_min', float(LOCALIZE_DEFAULTS['localize_margin_min']))
        # 死角（laser 基準・度・[start, end] の平坦配列）。lidar_filter と同じ
        # 取り方（registry.yaml が出所・空は死角なし）。空配列 override を
        # 受けられるよう dynamic_typing=True（CLAUDE.md「環境の癖」参照）。
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter(
            'blind_angle_ranges', [],
            ParameterDescriptor(dynamic_typing=True))
        # WS-9D: 自己位置源の EKF 出力 (/odometry/filtered) を優先し、古ければ
        # 生 /odom にフォールバックする。両ノードで同名・同既定にすること。
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_filtered_topic', '/odometry/filtered')
        self.declare_parameter('odom_stale_ms', 500)
        self._use_map_frame = bool(self.get_parameter('use_map_frame').value)
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._map_session_id = self.get_parameter('map_session_id').value
        self._localize_wait_s = float(self.get_parameter('localize_wait_s').value)
        self._localize_match_low = float(
            self.get_parameter('localize_match_low').value)
        self._localize_margin_low = float(
            self.get_parameter('localize_margin_low').value)
        self._localize_margin_min = float(
            self.get_parameter('localize_margin_min').value)
        self._odom_topic = self.get_parameter('odom_topic').value
        self._odom_filtered_topic = self.get_parameter('odom_filtered_topic').value
        self._odom_stale_ms = int(self.get_parameter('odom_stale_ms').value)
        self._preview_heartbeat_ms = int(self.get_parameter('preview_heartbeat_ms').value)
        self._routes_dir = self.get_parameter('routes_dir').value
        control_ms = self.get_parameter('control_period_ms').value
        status_ms = self.get_parameter('status_period_ms').value
        dt = control_ms / 1000.0
        self._max_dv = self.get_parameter('linear_accel_mps2').value * dt
        self._max_dw = self.get_parameter('angular_accel_rps2').value * dt

        # ── 純コアのパラメータ束 ──
        # WS-9T: _base_params は「速度スケール最大端」。実際に pure-pursuit へ渡す
        # self._params は speed_ratio で cruise/yaw を最小端との間に補間したもの
        # （scale_replay_params）。/replay/speed_scale の受信（_on_speed_scale）で作り直す。
        self._base_params = ReplayParams(
            lookahead_m=self.get_parameter('lookahead_m').value,
            cruise_speed_mps=self.get_parameter('cruise_speed_mps').value,
            max_yaw_rate_rps=self.get_parameter('max_yaw_rate_rps').value,
            arrive_dist_m=self.get_parameter('arrive_dist_m').value,
            yaw_tol_rad=self.get_parameter('yaw_tol_rad').value)
        self._cruise_min = float(self.get_parameter('replay_cruise_min_mps').value)
        self._yaw_min = float(self.get_parameter('replay_yaw_min_rps').value)
        self._lookahead_min = float(self.get_parameter('replay_lookahead_min_m').value)
        self._speed_ratio = float(
            self.get_parameter('replay_speed_default_ratio').value)
        self._params = scale_replay_params(
            self._base_params, self._speed_ratio, self._cruise_min, self._yaw_min,
            self._lookahead_min)

        # ── 状態 ────────────────────────────────────────────
        self._route = None
        self._route_id = None           # W-01 P2: 探索・reload が参照する現経路
        self._points = []
        self._start_yaw = 0.0
        self._from_index = 0
        self._need_rotate = False
        self._rotated = False
        self._arrived_sent = False
        self._preview_dirty = False          # WS-9F: 点列が変わった（load_route で差し替え）
        self._last_preview_ms = 0.0          # WS-9F: 直前に preview を publish した時刻
        self._was_moving = False
        self._target_index = -1
        # S-14「経路からのずれ」表示用（Spec-webui.md §3.7 / Spec-transit.md §4.4）。
        # いまのずれ [m] とこの再生での最大値 [m]。PAUSE では保ち（何もしない）、
        # 次の再生（load_route）で 0 に戻す。
        self._cross_track_m = 0.0
        self._cross_track_max_m = 0.0
        self._cur_v = 0.0   # ランプ後の実際の publish 値
        self._cur_w = 0.0
        self._pose = None                 # /odom 由来 (x, y, yaw)
        self._route_frame = 'odom'        # 読み込んだ経路のフレーム
        self._localize_pending = False    # map TF 待ち（W-01 縮小）
        self._localize_deadline = 0.0
        self._localize_pending_arg = '{}'  # TF 待ちの末に evt.localize_done へ載せる arg_json
        # W-01 P2: 確度表示（/route/status）。load_route で ''/0/0 に戻し、
        # READY・RUN・PAUSE 中も最後の値を出し続ける。
        self._localize_quality = ''
        self._localize_score = 0.0
        self._localize_margin = 0.0
        # W-01 P2: 探索スレッドの管理。executor を数秒止めないよう別スレッドで
        # 走らせ、結果を _poll_search（_control_timer から呼ぶ）で拾う。
        # 探索中も /route/status は出し続ける（_status_timer は独立）。
        self._search_lock = threading.Lock()
        self._search_gen = 0              # 世代。load_route で進めて旧探索を無効化
        self._search_result = None        # (gen, kind, route_id, result, base_pose)
        self._last_best_laser = None      # widen の中心にする直近の最良（laser 姿勢）
        self._last_scan = None            # 最新の /scan（探索スレッドが読む）
        self._mode = ""
        self._state = ""
        # WS-9D: 自己位置源の選択（生 odom / EKF 出力）。(pose, stamp_ms) の組。
        # stamp は受信時刻（_now_ms）を使い、ノード時計と同一基準にする。
        self._raw_sample: tuple[tuple[float, float, float], int] | None = None
        self._filtered_sample: tuple[tuple[float, float, float], int] | None = None
        self._current_source: str | None = None   # 直近の選択出所（切替ログ用）

        self._tf_buffer = None
        self._tf_listener = None
        if self._use_map_frame:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Subscribers ────────────────────────────────────
        # WS-9L: /system/effect は callback 内から別サービス (/map_session/open
        # reload) を同期的に呼ぶ。単一スレッド executor だと応答コールバックが動けず
        # デッドロックするので、ReentrantCallbackGroup に乗せ、main() 側で
        # MultiThreadedExecutor を使う（slam_control.py と同じ構成）。
        sub_cbg = ReentrantCallbackGroup()
        effect_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(StateEffect, '/system/effect', self._on_effect,
                                 effect_qos, callback_group=sub_cbg)

        state_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(SystemState, '/system/state', self._on_state, state_qos)

        # WS-9T: WebUI（S-14）から再生速度の比率（0..1）が latched で届く。
        # 走行中に変わっても次ティックから効く（_params を作り直すだけ）。
        speed_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(
            Float32, '/replay/speed_scale', self._on_speed_scale, speed_qos)

        # WS-9L: 再生の経路選択で地図を読み直す。client は既定グループに置き、
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

        # W-01 P2: 粗探索の入力。センサストリームは必ず BEST_EFFORT で購読する
        # （lidar_filter と同じ。既定 QoS だと実機の WiFi で届かない）。
        # 最新の 1 件だけ保持し、探索スレッドが読みに来る。
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        # ── Publishers（速度は /cmd_vel_behavior のみ）──────
        cmd_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel_behavior', cmd_qos)
        self._pub_event = self.create_publisher(StateEvent, '/system/event', cmd_qos)
        self._pub_status = self.create_publisher(RouteStatus, '/route/status', cmd_qos)
        self._pub_preview = self.create_publisher(Path, '/route/preview', cmd_qos)
        # WS-8B: WebUI が地図上にロボットを描くための pose（map or odom フレーム）。
        self._pub_robot_pose = self.create_publisher(
            PoseStamped, '/route/robot_pose', cmd_qos)

        # ── Timers ─────────────────────────────────────────
        self.create_timer(control_ms / 1000.0, self._control_timer)
        self.create_timer(status_ms / 1000.0, self._status_timer)
        # pose は表示の滑らかさに直結するので status とは別周期で回す。
        self.create_timer(
            self.get_parameter('pose_period_ms').value / 1000.0,
            self._publish_robot_pose)

        self.get_logger().info('replay_runner 起動')

    # ── 購読コールバック ──────────────────────────────────
    def _on_effect(self, msg: StateEffect):
        name = msg.name
        if name == 'load_route':
            args = json.loads(msg.args_json or '{}')
            route_id = args.get('route_id')
            rev_arg = args.get('reverse')
            if isinstance(rev_arg, str):
                reverse = rev_arg.lower() in ('true', '1', 'yes')
            else:
                reverse = bool(rev_arg)
            # 保存側 (finalized_path) と同じ規則でファイル名を作る。route_id に `/`
            # や `\` が入る名前 (例: 9/3_koushakukou) でも保存先と読み込み先が
            # 必ず一致するようにする。
            path = finalized_path(self._routes_dir, route_id)
            try:
                with open(path, encoding='utf-8') as f:
                    self._route = route_from_dict(json.load(f))
            except Exception as e:
                self.get_logger().error(f'経路を読み込めない: {path}: {e}')
                return
            # use_map_frame オフ（通常起動）なら map フレーム経路も odom 扱いにし、
            # 従来どおり align_path_to_current で現在地を始点とみなす。
            file_frame = self._route.frame_id or 'odom'
            map_route = self._use_map_frame and file_frame == self._map_frame
            self._route_frame = self._map_frame if map_route else 'odom'
            # WS-9K-B / WS-9U: map フレーム経路は自身の .posegraph を読み直すので
            # 別セッションでも再生してよい（WS-9S で経路選択のたび slam_toolbox を
            # 作り直してその経路の地図を deserialize する）。保存地図が無い
            # （map_session_id が空 ＝ 古い記録 or 教示の「保存」未完）経路だけ弾く。
            # 実ファイルの有無は slam_control._handle_map_reload が確認する。
            # odom フレーム経路は地図に依存しないので常に再生してよい。
            route_session = self._route.map_session_id or ''
            if not can_replay_route(
                    file_frame, route_session, self._map_session_id, self._map_frame):
                self.get_logger().error(
                    'この経路には保存された地図が無い（教示の「保存」が完了して'
                    'いない可能性）。再生できない。教示からやり直すこと')
                self._route = None
                return
            pts = list(self._route.points)
            if reverse:
                pts = reverse_points(pts)
            rec_start_yaw = pts[0][2] if reverse else self._route.start_yaw
            aligned = False
            if map_route:
                # WS-8B: 経路が map フレーム。slam_toolbox が同じ map を再構築して
                # おり、始点マーク運用なら記録座標がそのまま有効 → 剛体変換しない。
                pass
            elif pts:
                # WAIVER(demo): W-01（縮小）— odom フォールバック経路のみ。
                # 「今いる場所を記録の始点とみなす」2D 剛体変換で経路の形を辿る。
                # 基点は WS-9D の自己位置源選択結果を使う（取れなければスキップ）。
                cur = self._odom_pose()
                if cur is not None:
                    recorded_start = (pts[0][0], pts[0][1], rec_start_yaw)
                    pts = align_path_to_current(pts, recorded_start, cur)
                    aligned = True
            self._points = pts
            self._preview_dirty = True   # WS-9F: 点列が変わった → 次ティックで preview 配信
            self._start_yaw = pts[0][2] if pts else rec_start_yaw
            self._from_index = 0
            self._target_index = -1
            # 次の再生を始めたらずれ・最大値を 0 から数え直す（S-14 の仕様）。
            self._cross_track_m = 0.0
            self._cross_track_max_m = 0.0
            # W-01 P2: 確度表示も '' に戻す（READY・RUN・PAUSE 中は最後の値を保つ）。
            # 進行中の探索・TF 待ちはこの load_route が上書きする（旧世代は無効）。
            self._search_gen += 1
            with self._search_lock:
                self._search_result = None
            self._localize_pending = False
            self._localize_pending_arg = '{}'
            self._localize_quality = ''
            self._localize_score = 0.0
            self._localize_margin = 0.0
            self._last_best_laser = None
            self._need_rotate = False
            self._rotated = False
            self._arrived_sent = False
            self._route_id = route_id
            if map_route:
                pgm_base = self._route_pgm_base(route_id)
                if pgm_base is not None:
                    # W-01 P2: 保存 pgm 上で経路始点の周辺を探索し、最良候補を
                    # 初期姿勢に reload する（始点決め打ちをやめる）。探索は
                    # 別スレッド（executor を数秒止めない）。結果は _poll_search
                    # で拾い、成立したら reload → TF 待ち → evt.localize_done。
                    # s 不足なら evt.localize_low → FSM が widen_search を返す。
                    self._localize_quality = 'searching'
                    center = ((float(self._points[0][0]), float(self._points[0][1]))
                              if self._points else None)
                    self._start_search(
                        route_id, kind='initial', center=center,
                        radius=float(LOCALIZE_DEFAULTS['search_radius_m']))
                    self.get_logger().info(
                        f'load_route: id={route_id} reverse={reverse} 点={len(pts)} '
                        f'frame=map（保存地図上で探索中。終わり次第 reload）')
                else:
                    # pgm が無い旧経路: 現行どおり始点決め打ちで reload し、
                    # localize_quality='unknown' とする。
                    self._localize_quality = 'unknown'
                    # WS-9L: 経路が map フレーム。手で機体を始点へ戻したうえで、教示の
                    # 地図を読み直す（deserialize。自己位置が地図の最初のノード＝経路の
                    # 始点に一致する）。成功したときだけ evt.localize_done へ進める。
                    # 失敗したら LOCALIZE から進まない（error ログのみ）。
                    err = self._reload_map(route_id)
                    if err is None:
                        self._localize_pending = True
                        self._localize_deadline = (
                            self.get_clock().now().nanoseconds / 1e9
                            + self._localize_wait_s)
                        self.get_logger().info(
                            f'load_route: id={route_id} reverse={reverse} 点={len(pts)} '
                            f'frame=map（pgm 無しのため始点決め打ち。地図を読み直しました。'
                            f'map→base_link TF を最大 '
                            f'{self._localize_wait_s:.0f}s 待つ）')
                    else:
                        self.get_logger().error(
                            f'地図を読み直せなかったため LOCALIZE から進めない: '
                            f'id={route_id} ({err})')
            else:
                self.get_logger().info(
                    f'load_route: id={route_id} reverse={reverse} 点={len(pts)} '
                    f'frame=odom 現在地合わせ={"あり" if aligned else "なし(odom 未受信)"} '
                    '(WAIVER(demo): W-01 により即 evt.localize_done)')
                self._emit_event('evt.localize_done')
        elif name == 'rotate_to_start_yaw':
            self._need_rotate = True
            self._rotated = False
        elif name == 'resume_path':
            # _from_index から追従を続ける（index はそのままなので途中から再開する）。
            # WS-9P: 到着ラッチを解除する。終端に達したあとに再生を押した場合、
            # 解除しないと evt.arrived を出し直せず RUN のまま機体が動かない状態で
            # 固まる。解除しておけば「もう終端だ」と即座に判定して PAUSE へ戻る。
            self._arrived_sent = False
            self.get_logger().info('resume_path: 追従を再開（index は保持）')
        elif name == 'widen_search':
            # W-01 P2: 直前の探索の最良候補の周辺の広い窓で再探索する。
            # それでも不成立なら evt を出さず LOCALIZE に留まる（failed）。
            self._start_followup_search('widen')
        elif name == 'global_localize':
            # W-01 P2: 地図全域で探索する。不成立なら evt.localize_done を
            # 出さず LOCALIZE に留まる（failed）。no-op の即 done はやめた。
            self._start_followup_search('global')
        else:
            self.get_logger().debug(f'無視する effect: {name}')

    def _on_state(self, msg: SystemState):
        self._mode = msg.mode
        self._state = msg.state

    def _on_speed_scale(self, msg: Float32):
        """WS-9T: 再生速度の比率（0..1）を受けて pure-pursuit パラメータを作り直す。"""
        ratio = 0.0 if msg.data < 0.0 else 1.0 if msg.data > 1.0 else float(msg.data)
        if ratio == self._speed_ratio:
            return
        self._speed_ratio = ratio
        self._params = scale_replay_params(
            self._base_params, ratio, self._cruise_min, self._yaw_min,
            self._lookahead_min)
        self.get_logger().info(
            f'再生速度スケール {ratio:.2f} → cruise {self._params.cruise_speed_mps:.2f} m/s / '
            f'yaw {self._params.max_yaw_rate_rps:.2f} rad/s / '
            f'lookahead {self._params.lookahead_m:.2f} m')

    def _reload_map(self, route_id: str, init_pose=None) -> "str | None":
        """/map_session/open reload で教示の地図を読み直す。エラー文字列 or None。

        init_pose: (x, y, yaw) の初期姿勢（map フレーム・base_link）。省略時は
        従来どおり経路始点（self._points[0] と self._start_yaw）。W-01 P2 では
        探索の最良候補を渡す（始点決め打ちをやめる）。
        deserialize_map(match_type=3 LOCALIZE_AT_POSE) で自己位置を初期姿勢に合わせる。
        再生中は地図を凍結したまま自己位置推定のみ継続する。

        WS-9S: slam_control 側は読み直しのたび slam_toolbox を respawn してから
        deserialize する（deserialize の反復呼びでグラフが上乗せされ地図が壊れる
        バグの対策）。respawn 待ち（最大 ~45s）＋ 6.9MB〜20MB の deserialize I/O
        （最大 30s）を含むので、タイムアウトは 90s と長めに取る。
        """
        if not self._map_session_client.wait_for_service(timeout_sec=1.0):
            return '/map_session/open に接続できません'
        # 送る側で正規化する（route_record_core._safe_id と同じ規則）。経路 JSON の
        # 保存名（finalized_path が内部で _safe_id する）と地図ファイル名を一致させる
        # ため。受ける側（slam_control_logic）は未正規化 id を拒否するだけ。
        session_id = _safe_id(route_id)
        if init_pose is not None:
            init_x, init_y, init_yaw = (float(init_pose[0]), float(init_pose[1]),
                                        float(init_pose[2]))
            has_init = True
        else:
            has_init = bool(self._points)
            init_x = float(self._points[0][0]) if has_init else 0.0
            init_y = float(self._points[0][1]) if has_init else 0.0
            init_yaw = float(self._start_yaw) if has_init else 0.0
        req = OpenMapSession.Request(
            slot='ROUTE',
            session_id=session_id,
            mode='reload',
            has_initial_pose=has_init,
            initial_x=init_x,
            initial_y=init_y,
            initial_yaw=init_yaw)
        resp, err = call_and_wait(self, self._map_session_client, req, 90.0)
        if err:
            return f'地図の読み直しに失敗: {err}'
        if resp is not None and not resp.success:
            msg = resp.message if resp.message else 'success=False'
            return f'地図の読み直しに失敗: {msg}'
        return None

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, _yaw_from_quat(msg.pose.pose.orientation))
        self._raw_sample = (self._pose, self._now_ms())

    def _on_odom_filtered(self, msg: Odometry):
        p = msg.pose.pose.position
        self._filtered_sample = (
            (p.x, p.y, _yaw_from_quat(msg.pose.pose.orientation)), self._now_ms())

    def _on_scan(self, msg: LaserScan):
        # W-01 P2: 最新の 1 件だけ保持する（探索スレッドが読みに来る）。
        self._last_scan = msg

    # ── 全域ローカライズの探索（W-01 P2）─────────────────────────
    def _route_pgm_base(self, route_id: str):
        """経路 id に対応する保存地図の base 名（拡張子なし）。pgm＋yaml が
        揃っていなければ None（pgm 無しの旧経路＝始点決め打ち・'unknown'）。"""
        base = os.path.join(self._routes_dir, _safe_id(route_id))
        if os.path.exists(base + '.pgm') and os.path.exists(base + '.yaml'):
            return base
        return None

    def _start_search(self, route_id: str, kind: str, center, radius):
        """探索スレッドを起こす（kind: initial／widen／global）。

        ROS 側で読む入力（死角・確度閾値）はここでスナップショットし、重い探索本体は
        別スレッド（_run_search）で走らせる。結果は _poll_search が拾う。
        1 回の探索につき reload は最良候補の 1 回だけ（WS-9S の対策を保つ）。
        """
        self._search_gen += 1
        gen = self._search_gen
        blind = list(self.get_parameter('blind_angle_ranges').value or [])
        thresholds = (self._localize_match_low, self._localize_margin_low,
                      self._localize_margin_min)
        t = threading.Thread(
            target=self._run_search,
            args=(gen, route_id, kind, center, radius, blind, thresholds),
            daemon=True)
        t.start()
        self.get_logger().info(
            f'localize 探索開始 kind={kind} id={route_id} '
            f'center={center} radius={radius}')

    def _start_followup_search(self, kind: str) -> bool:
        """widen_search／global_localize effect の受け口。探索を起こせたら True。

        起こせない（経路なし・odom 経路・pgm なし）ときは警告ログだけで
        False（evt は出さず LOCALIZE に留まる）。
        """
        route_id = self._route_id
        if self._route is None or route_id is None or self._route_frame != self._map_frame:
            self.get_logger().warn(
                f'{kind}: 地図経路が読み込まれていないため探索できない')
            return False
        if self._route_pgm_base(route_id) is None:
            self.get_logger().warn(
                f'{kind}: 保存地図（pgm）が無いため探索できない id={route_id}')
            return False
        if kind == 'widen':
            # 最良候補の周辺の広い窓で再探索する。直前の最良が無ければ経路始点。
            if self._last_best_laser is not None:
                center = (self._last_best_laser[0], self._last_best_laser[1])
            else:
                center = ((float(self._points[0][0]), float(self._points[0][1]))
                          if self._points else None)
            radius = float(LOCALIZE_DEFAULTS['widen_radius_m'])
        else:
            center, radius = None, None
        self._localize_quality = 'searching'
        self._start_search(route_id, kind=kind, center=center, radius=radius)
        return True

    def _run_search(self, gen: int, route_id: str, kind: str, center, radius,
                    blind, thresholds):
        """探索スレッド本体（executor を止めない）。結果だけを置いて終わる。

        reload（/map_session/open の同期呼び出し）もここで行う。call_and_wait
        は worker スレッド上でポーリングする設計（service_call.py）で、ROS
        コールバック（_control_timer 等）の中から呼ぶと、同じコールバック
        グループの応答とデッドロックするため。
        """
        match_low, margin_low, margin_min = thresholds
        pgm_base = self._route_pgm_base(route_id)
        if pgm_base is None:
            self._store_search_result(
                gen, kind, route_id, 'failed', 0.0, 0.0, None, None, None)
            return
        try:
            fld = load_pgm_map(pgm_base + '.pgm', pgm_base + '.yaml')
        except Exception as e:
            self.get_logger().warn(f'localize 探索: 地図を読めない: {e}')
            fld = None
        scan = self._wait_scan()
        if fld is None or scan is None:
            if scan is None:
                self.get_logger().warn('localize 探索: /scan が来ないため探索できない')
            self._store_search_result(
                gen, kind, route_id, 'failed', 0.0, 0.0, None, None, None)
            return
        off = self._laser_to_base_offset(scan.header.frame_id)
        try:
            result = localize_search(
                fld, list(scan.ranges), scan.angle_min,
                scan.angle_increment, blind,
                center=center, radius=radius,
                timeout_s=_LOCALIZE_SEARCH_TIMEOUT_S)
        except Exception as e:
            self.get_logger().warn(f'localize 探索: 探索が例外で終わった: {e}')
            result = None
        if result is None:
            self._store_search_result(
                gen, kind, route_id, 'failed', 0.0, 0.0, None, None, None)
            return
        if result.timed_out:
            self.get_logger().warn(
                f'localize 探索: 時間切れ（{result.elapsed_s:.1f}s）')
        s, m = float(result.s), float(result.m)
        quality = judge_localize(s, m, match_low, margin_low, margin_min)
        self.get_logger().info(
            f'localize 探索結果 kind={kind} s={s:.3f} m={m:.3f} → {quality}')
        base_pose, laser_pose, reload_err = None, None, None
        if quality in ('high', 'low_margin') and result.best is not None:
            laser_pose = (result.best.x, result.best.y, result.best.yaw)
            base_pose = _laser_pose_to_base(laser_pose, off)
            # 1 回の探索につき reload は最良候補の 1 回だけ（WS-9S を保つ）。
            reload_err = self._reload_map(route_id, init_pose=base_pose)
            if reload_err is not None:
                self.get_logger().error(
                    f'地図を読み直せなかったため LOCALIZE から進めない: '
                    f'id={route_id} ({reload_err})')
        self._store_search_result(
            gen, kind, route_id, quality, s, m, base_pose, laser_pose,
            reload_err)

    def _store_search_result(self, gen, kind, route_id, quality, s, m,
                             base_pose, laser_pose, reload_err):
        with self._search_lock:
            # 新しい load_route が来ていたら世代が進んでいるので置かない
            # （古い探索が新しい結果を消さない）。
            if gen == self._search_gen:
                self._search_result = (gen, kind, route_id, quality, s, m,
                                       base_pose, laser_pose, reload_err)

    def _wait_scan(self):
        """最新の /scan を待つ（_LOCALIZE_SCAN_WAIT_S まで）。無ければ None。"""
        deadline = time.monotonic() + _LOCALIZE_SCAN_WAIT_S
        while time.monotonic() < deadline:
            scan = self._last_scan
            if scan is not None and len(scan.ranges) > 0:
                return scan
            time.sleep(0.1)
        return None

    def _laser_to_base_offset(self, laser_frame: str):
        """laser フレーム基準の base_link 姿勢 (x, y, yaw)。TF が無ければ原点。"""
        if self._tf_buffer is None or not laser_frame:
            return (0.0, 0.0, 0.0)
        try:
            tf = self._tf_buffer.lookup_transform(
                laser_frame, self._base_frame, rclpy.time.Time())
        except Exception:
            self.get_logger().warn(
                f'localize 探索: {laser_frame}→{self._base_frame} TF が無いため'
                f'オフセット無しで探索する')
            return (0.0, 0.0, 0.0)
        t = tf.transform.translation
        return (t.x, t.y, _yaw_from_quat(tf.transform.rotation))

    def _poll_search(self):
        """探索スレッドの結果を拾う（_control_timer から毎 tick 呼ぶ）。

        ここでは重い呼び出し（reload 等）をしない。TF 待ちの開始
        （_localize_pending）は代入だけなので安全。
        """
        with self._search_lock:
            item = self._search_result
            self._search_result = None
        if item is None:
            return
        gen, kind, route_id, quality, s, m, base_pose, laser_pose, reload_err = item
        if gen != self._search_gen:
            return
        if laser_pose is not None:
            self._last_best_laser = laser_pose
        if quality in ('high', 'low_margin'):
            if reload_err is not None or base_pose is None:
                self._localize_quality = 'failed'
                self._localize_score = s
                self._localize_margin = m
                return
            self._localize_quality = quality
            self._localize_score = s
            self._localize_margin = m
            self._localize_pending = True
            self._localize_deadline = (
                self.get_clock().now().nanoseconds / 1e9
                + self._localize_wait_s)
            self._localize_pending_arg = json.dumps(
                {'score': s, 'margin': m})
            self.get_logger().info(
                f'localize 探索: 最良候補を初期姿勢に reload 済み '
                f'({base_pose[0]:.2f}, {base_pose[1]:.2f}). '
                f'map→base_link TF を最大 {self._localize_wait_s:.0f}s 待つ')
        elif quality == 'low' and kind == 'initial':
            # s 不足 → evt.localize_low → FSM が widen_search を返す。
            # widen／global の low はここで止める（failed に落とす）。
            self._localize_quality = 'searching'
            self._localize_score = s
            self._localize_margin = m
            self._emit_event(
                'evt.localize_low', json.dumps({'score': s, 'margin': m}))
        else:
            self._localize_quality = 'failed'
            self._localize_score = s
            self._localize_margin = m
            self.get_logger().warn(
                f'localize 探索: 不成立のため LOCALIZE に留まる（{quality}）')

    # ── フレーム解決 ──────────────────────────────────────
    def _map_pose(self):
        """map→base_link TF から (x, y, yaw) を返す。取れなければ None。

        **非ブロッキング**。timeout を渡すと単一スレッド executor 上で TF 到着
        コールバックが動けず必ずタイムアウトまで固まる。バッファにあるものだけ読む。
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

    def _get_pose(self):
        """追従に使うロボット pose を経路フレームで返す。取れなければ None。"""
        if self._use_map_frame and self._route_frame == self._map_frame:
            return self._map_pose()
        # WS-9D: odom フレーム経路は選択結果（EKF 出力優先、生 odom フォールバック）。
        # map フレーム経路（上の TF 経路）はここを通らず既存のまま。
        return self._odom_pose()

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

    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def _publish_robot_pose(self):
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

    def _tick_localize_pending(self):
        """map フレーム経路の LOCALIZE 待ち: TF が来たら / 期限切れで localize_done。"""
        if not self._localize_pending:
            return
        arg = self._localize_pending_arg
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self._map_pose() is not None:
            self._localize_pending = False
            self._localize_pending_arg = '{}'
            self.get_logger().info('map→base_link TF 取得 → evt.localize_done')
            self._emit_event('evt.localize_done', arg)
        elif now_s >= self._localize_deadline:
            self._localize_pending = False
            self._localize_pending_arg = '{}'
            self.get_logger().warn(
                'map→base_link TF が来ないまま期限切れ → evt.localize_done'
                '（slam_toolbox 未起動の可能性。デモ継続優先）')
            self._emit_event('evt.localize_done', arg)

    def _publish_ramped(self, target_v: float, target_w: float):
        """目標 (v, w) へランプで近づけてから publish（#2: DRIVE_RUNAWAY 対策）。"""
        self._cur_v = ramp_toward(self._cur_v, target_v, self._max_dv)
        self._cur_w = ramp_toward(self._cur_w, target_w, self._max_dw)
        t = Twist()
        t.linear.x = self._cur_v
        t.angular.z = self._cur_w
        self._pub_cmd.publish(t)
        self._was_moving = abs(self._cur_v) > 1e-6 or abs(self._cur_w) > 1e-6

    # ── 制御ループ（20Hz）────────────────────────────────
    def _control_timer(self):
        self._tick_localize_pending()
        self._poll_search()  # W-01 P2: 探索スレッドの結果を拾う（非ブロッキング）
        pose = self._get_pose()
        # 走行条件が揃っていない → ランプで 0 へ戻す（急な 0 もステップになる）
        if self._mode != 'REPLAY' or self._state != 'RUN' or \
                self._route is None or pose is None:
            if self._was_moving:
                self._publish_ramped(0.0, 0.0)
            return

        if self._need_rotate and not self._rotated:
            w, done = rotate_toward(pose[2], self._start_yaw, self._params)
            self._publish_ramped(0.0, w)
            if done:
                self._rotated = True
            return

        # 追従フェーズ
        # WS-8B: map フレーム経路なら slam_toolbox の map→odom 連続補正が効く
        # （W-02 縮小）。odom 経路は従来どおり補正なし。
        # 先に通過済みの点まで index を進めてから、同じ index で pure-pursuit する。
        self._from_index = advance_index(pose, self._points, self._from_index, self._params)
        # 走行中（RUN）に毎 tick 測り、最大値を保持する。PAUSE ではこの節に
        # 来ないので最大値は保たれる（S-14 の仕様。Spec-webui.md §3.7）。
        self._cross_track_m = cross_track_error(
            pose, self._points, self._from_index)
        if self._cross_track_m > self._cross_track_max_m:
            self._cross_track_max_m = self._cross_track_m
        cmd = pure_pursuit(pose, self._points, self._params, self._from_index)
        self._target_index = cmd.target_index
        if cmd.arrived:
            self._publish_ramped(0.0, 0.0)
            if not self._arrived_sent:
                self._emit_event('evt.arrived')
                self._arrived_sent = True
            return
        self._publish_ramped(cmd.v, cmd.w)
        self._was_moving = True

    # ── イベント発行 ─────────────────────────────────────
    def _emit_event(self, name: str, arg_json: str = '{}'):
        ev = StateEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.event = name
        ev.source_node = 'replay_runner'
        ev.arg_json = arg_json
        self._pub_event.publish(ev)
        self.get_logger().info(f'/system/event 発行: {name} {arg_json}')

    # ── ステータス publish ───────────────────────────────
    def _status_timer(self):
        # WS-9Q: 自分のモードのときだけ出す。両方が無条件に出していたため、再生中も
        # route_recorder が points=0 を交互に流し、state_manager の route_loaded が
        # 4Hz で往復していた。T-REPLAY-07 のガードがそれを見るので「再生」が
        # 押したタイミング次第で拒否されていた（実機 2026-09-04）。
        if not owns_route_status(self._mode, recorder=False):
            return
        stamp = self.get_clock().now().to_msg()
        # pose は専用の 10Hz タイマ（pose_period_ms）で出す。ここでは出さない。
        msg = RouteStatus()
        msg.header.stamp = stamp
        msg.state = self._state or 'NONE'
        msg.points = len(self._points)
        msg.recorded_m = float(polyline_length(self._points))
        msg.elapsed_sec = 0.0  # 再生では未使用
        msg.target_index = int(self._target_index)
        # WS-9P: 一時停止の理由を UI に伝える。PAUSE は終端到達だけでなくフォルト・
        # ジョグ介入・停止ボタンでも起きるので、状態名だけでは区別できない。
        msg.arrived = bool(self._arrived_sent)
        # S-14「経路からのずれ」表示用。PAUSE 中も最大値は残ったまま出る。
        msg.cross_track_m = float(self._cross_track_m)
        msg.cross_track_max_m = float(self._cross_track_max_m)
        # W-01 P2: 全域ローカライズの確度。READY・RUN・PAUSE 中も最後の値を出す。
        msg.localize_quality = self._localize_quality
        msg.localize_score = float(self._localize_score)
        msg.localize_margin = float(self._localize_margin)
        if self._route is not None:
            info = RouteInfo()
            info.id = self._route.id
            info.name = self._route.name
            info.point_count = len(self._points)
            msg.current = info
        self._pub_status.publish(msg)
        # 追従中の点列を経路フレーム（map or odom）の Path でプレビュー配信する。
        # #4: 経路を読んでいるときだけ出す（空 Path を出すと、記録側と交互配信になり
        # WebUI のプレビューが点滅する）。
        # WS-9F: 走行中は点列が変わらないため毎 2Hz で送るのは無駄。点列が変わった
        # （_preview_dirty / load_route で立てる）とき＋ハートビート間隔だけ配信する。
        # ここは間引かない（RoutePreview が preview[targetIndex] で添字参照するため）。
        if self._route is not None and self._points:
            now_ms = self._now_ms()
            changed = self._preview_dirty
            heartbeat_due = (now_ms - self._last_preview_ms) >= self._preview_heartbeat_ms
            if changed or heartbeat_due:
                self._pub_preview.publish(_points_to_path(
                    self._points, frame_id=self._route_frame, stamp=stamp))
                self._preview_dirty = False
                self._last_preview_ms = now_ms


def main(args=None):
    rclpy.init(args=args)
    node = ReplayRunner()
    # WS-9L: /system/effect コールバック内から /map_session/open を同期的に呼ぶため
    # MultiThreadedExecutor + ReentrantCallbackGroup を使う（slam_control.py と同じ
    # 構成。単一スレッド executor だと応答コールバックが動けず経路選択で固まる）。
    # WS-9U: call_and_wait はポーリング中に worker スレッドを塞ぐので下限 4 本。
    executor = MultiThreadedExecutor(num_threads=4)
    node._executor = executor
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
