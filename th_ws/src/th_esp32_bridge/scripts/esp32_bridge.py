#!/usr/bin/env python3
"""
esp32_bridge.py — ROS2 ノード本体
====================================
ws_protocol.py のバイナリプロトコルで ESP32 と WebSocket 通信する。
esp32_bridge がサーバー、ESP32 がクライアントとして接続しにくる(6.1節参照)。

担当:
  1. /cmd_vel (Twist) → 差動駆動変換 → WHEEL_CMD フレーム送信
  2. WHEEL_FEEDBACK フレーム受信 → /odom + TF 計算
  3. ESTOP_HW フレーム受信 → /safety/estop_hw へ中継、flags を
     /safety/firmware_flags へ中継 (判定はしない。WP-ESP32-01)
  4. /esp32/wheel_feedback (WheelFeedback) 発行 (他ノード用)
"""
import asyncio
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, UInt8
from th_system_msgs.msg import WheelFeedback
import tf2_ros

import websockets

from th_esp32_bridge.keepalive_core import Twist2, keepalive_value
from th_esp32_bridge.odom_core import (
    Pose2D, integrate_pose, resolve_dt, resync_stamp, yaw_to_quaternion_zw,
)
from th_esp32_bridge.rx_backpressure import BoundedFrameQueue
from th_esp32_bridge.send_coalescer import INITIAL_STATE as _COALESCER_INITIAL_STATE
from th_esp32_bridge.send_coalescer import complete as coalescer_complete
from th_esp32_bridge.send_coalescer import offer as coalescer_offer
from th_esp32_bridge.ws_protocol import (
    ProtocolError, WHEEL_FEEDBACK, ESTOP_HW, IMU_DATA,
    pack_wheel_cmd, unpack_wheel_feedback, unpack_estop_hw_flags, unpack_imu_data, peek_type,
)


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('wheel_base', 0.39)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('feedback_timeout_ms', 500)
        # ESP32 の制御周期 (esp32/src/config.h の CTRL_PERIOD_MS と一致させること)。
        # WHEEL_FEEDBACK 1 フレームが表す走行時間そのもので、オドメトリ積分の dt になる。
        self.declare_parameter('feedback_period_ms', 100)
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)
        # DEBT-4 対処 (WP-SAFE-02): /cmd_vel がこの時間 [ms] 途絶したら、
        # キープアライブの参照値をゼロへ書き換える (esp32_bridge.py 側で
        # 先に止める。A7: esp32_watchdog_ms=600ms より短いこと)。
        # 通常運用では generated/esp32_bridge.yaml (registry.yaml 由来、
        # 既定 400ms) で上書きされる。ここでの既定値は launch を介さず
        # 直接ノードを立てた場合のフォールバック。
        self.declare_parameter('cmd_vel_stale_ms', 400)
        # D-2 (受信ギャップ切り分け作業): 受信キュー (_rx_queue) の上限件数。
        # 溢れたら古い方を捨てる (rx_backpressure.py 参照)。WHEEL_FEEDBACK と
        # IMU_DATA はどちらも ESP32 の制御周期 (既定 CTRL_PERIOD_MS=100ms)
        # ごとに1フレームずつ、あわせて公称 20Hz で届く。既定値 200 は
        # この公称レートで 10 秒分に相当し、実機計測で観測された最悪ギャップ
        # (4106ms) を大きく上回る余裕を持たせつつ、詰まりが際限なく伸びない
        # よう上限を設けている。
        self.declare_parameter('rx_queue_maxsize', 200)
        # 1回の drain (_drain_rx_queue、50Hz) で処理するフレーム数の上限。
        # 上限に達したら残りは次周期 (20ms 後) に持ち越す。バックログが
        # あっても 1 回のタイマーコールバックの処理時間を有界にし、
        # 「詰まりの自己増幅」(1回のコールバックが長時間 executor を
        # 占有し、次の詰まりを招く)を防ぐ。既定値 50 は rx_queue_maxsize
        # (既定200) を 4 周期 (80ms) で掃き出せる値。
        self.declare_parameter('rx_queue_drain_max_per_cycle', 50)

        self._wheel_base = self.get_parameter('wheel_base').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._ws_host = self.get_parameter('ws_host').value
        self._ws_port = self.get_parameter('ws_port').value
        self._cmd_vel_stale_ms = self.get_parameter('cmd_vel_stale_ms').value
        self._rx_queue_drain_max_per_cycle = \
            self.get_parameter('rx_queue_drain_max_per_cycle').value

        # wheel_base はキャリブレーション後にランタイムで再設定されるため、
        # 変更を即座に _cb_cmd_vel / _on_wheel_feedback へ反映する。
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # ── Publishers ─────────────────────────────────────
        self._pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self._pub_wheel_feedback = self.create_publisher(
            WheelFeedback, '/esp32/wheel_feedback', 10)
        # /cmd_vel を差動駆動変換した左右目標速度 (WHEEL_CMD で ESP32 へ送る値と同じ)。
        # WheelFeedback 型を指令値側にも再利用する(フィールド形状が同一のため)。
        # WebUI で /esp32/wheel_feedback (実測) と重ねて表示し、PID の追従遅れ・
        # 定常偏差を目視で確認できるようにする。
        self._pub_wheel_cmd = self.create_publisher(
            WheelFeedback, '/esp32/wheel_cmd_speed', 10)
        self._pub_estop_hw = self.create_publisher(Bool, '/safety/estop_hw', 10)
        # ファーム構成フラグ (ESTOP_HW の flags をそのまま流すだけ。判定は
        # safety_monitor が行う。WP-SAFE-01)。transient_local: 後から立ち上がる
        # 購読側(safety_monitor)が起動順序に関わらず最新値を受け取れるように。
        firmware_flags_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_firmware_flags = self.create_publisher(
            UInt8, '/safety/firmware_flags', firmware_flags_qos)
        # 直近に publish した値。変化時だけ publish するための比較用。
        # ESP32 が(再)接続するたびに None へ戻し、次に届いた ESTOP_HW フレームで
        # 必ず publish させる(値が変化していなくても、接続直後は改めて知らせる)。
        self._last_firmware_flags = None
        self._pub_imu = self.create_publisher(Imu, '/esp32/imu_data', 10)
        self._pub_imu_calib = self.create_publisher(UInt8, '/esp32/imu_calib_status', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)

        # twist_mux はロック作動中、無出力(silent)になり得る実装のため、
        # /cmd_vel の途絶だけに頼らずここでも直接ロック状態を購読し、
        # ロック中は _last_cmd_vel の内容によらず強制的にゼロを送る
        # (VISION.md §5 参照。2026-08-06 手動ジョグ中の E-Stop で発覚)。
        #
        # ロックトピック自体が途絶した場合 (safety_monitor のクラッシュ等) も
        # twist_mux.yaml の locks.*.timeout(0.5s) と同じ考え方でフェイルセーフに
        # ロック扱いにする。単純な bool ラッチだけだと safety_monitor が
        # 死んだ際に「ロックされていない」状態のまま固定されてしまい、上と同じ
        # 事象(古い非ゼロ指令の再送)が別経路で再発するため
        # (twist_mux.yaml の timeout と同値の 0.5s を使用)。
        self._LOCK_TIMEOUT_SEC = 0.5
        self._estop_active = False
        self._fault_lock_active = False
        self._last_estop_msg_time = None
        self._last_fault_lock_msg_time = None
        self.create_subscription(Bool, '/safety/estop', self._cb_estop, 10)
        self.create_subscription(Bool, '/safety/fault_lock', self._cb_fault_lock, 10)

        # 最新の /cmd_vel を一定周期で再送するキープアライブ。ESP32側の
        # WHEEL_CMD ウォッチドッグ(300ms, config.h)は「WebSocketリンクが
        # 生きているか」を測るためのものだが、/cmd_vel のコールバック駆動
        # 送信だけだと上流の発行間隔(WiFi平常時ジッタで0.5〜1.2秒、
        # docs/network.md 参照)がそのままウォッチドッグの入力になってしまい、
        # リンクは健全なのに誤発動していた(2026-08-05 実機ログで確認)。
        # このタイマーで実際のリンク死活とは無関係な発行間隔のばらつきを
        # 吸収する。ROS2/本ノードが本当にクラッシュすればこのタイマーごと
        # 止まるため、ESP32側の最終保証(300ms以内に停止)は変わらない。
        self._last_cmd_vel = Twist()
        # 未受信 (起動直後) は「無限に stale」として扱う (_send_wheel_cmd 参照)。
        self._last_cmd_vel_time: "rclpy.time.Time | None" = None
        self.create_timer(0.05, self._cb_cmd_vel_keepalive)  # 20Hz

        self._odom_x = self._odom_y = self._odom_yaw = 0.0
        self._feedback_period_sec = \
            self.get_parameter('feedback_period_ms').value / 1000.0
        # 受信ギャップの警告閾値。公称周期の 5 倍 (既定 0.5 s) を超えたら WiFi 遅延を疑う。
        self._ARRIVAL_GAP_WARN_SEC = self._feedback_period_sec * 5.0
        # ESP32 から受けた dt の妥当範囲の上限。ファーム側も 1.0 を超えたら公称値へ
        # フォールバックする実装 (esp32/src/main.cpp) なので同じ値を使う。
        self._DT_MAX_SEC = 1.0
        # オドメトリのヘッダスタンプが実時間からこれ以上ずれたら貼り直す。
        #
        # 正常時はどのケースでもずれは 0 近傍に留まるため、これは異常時の安全網
        # でしかない: 定常時は到着間隔と dt がどちらも公称周期で進む。バースト
        # 配送でも、バースト開始時点の基準スタンプが既にギャップぶん古いので、
        # dt を積み上げると現在時刻に着地する (これがこの方式の狙い)。ESP32 の
        # ループ停滞で dt が伸びた場合も、到着間隔が同じだけ伸びるので釣り合う。
        # したがって閾値は「dt の累積が実時間から乖離し続けている」異常だけを
        # 捉えればよく、docs/network.md 記載の最大ギャップ(1.2s)では発火しない
        # 幅を取る。
        self._STAMP_RESYNC_SEC = 2.0
        # この機体の最大ヨーレートは 1 rad/s 程度。これを大きく超える角速度が
        # 届いたらジャイロ単位の取り違えを疑う (_on_imu_data 参照)。
        self._IMU_WZ_IMPLAUSIBLE_RAD_S = 10.0
        self._odom_stamp_sec: "float | None" = None
        # 受信した WHEEL_FEEDBACK が新形式(dt 付き)かどうか。None = 未受信。
        # 変化したときだけログに出す (_on_wheel_feedback 参照)。
        self._feedback_has_dt = None
        self._last_odom_time = self.get_clock().now()
        self._last_feedback_time = self.get_clock().now()
        self._esp32_alive = False

        # ── フィードバック死活監視タイマー ─────────────────
        timeout_ms = self.get_parameter('feedback_timeout_ms').value
        self.create_timer(timeout_ms / 1000.0, self._cb_watchdog)

        # 受信フレームは asyncio スレッドからロック付きキューへ積み、rclpy 側の
        # タイマーで drain する(rclpy publisher は任意スレッドからの同時呼び出し
        # が保証されないため)。D-2: 有界キュー (rx_backpressure.py 参照)。
        self._rx_queue = BoundedFrameQueue(
            maxsize=self.get_parameter('rx_queue_maxsize').value)
        self.create_timer(0.02, self._drain_rx_queue)  # 50Hz

        self._loop: "asyncio.AbstractEventLoop | None" = None
        self._ws_conn_lock = threading.Lock()
        self._ws_conn = None  # 現在接続中の ESP32 (単一クライアント想定)

        # D-1: WHEEL_CMD 送信コアレッシング (send_coalescer.py 参照)。
        # in-flight 中の送信要求はキューせず保留フレームの上書きで束ねる。
        # rclpy タイマー(_send_wheel_cmd)と asyncio スレッド(_send_and_continue
        # の完了時)の双方から読み書きするため、専用ロックで守る。
        self._coalescer_lock = threading.Lock()
        self._coalescer_state = _COALESCER_INITIAL_STATE

        self._ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f"esp32_bridge 起動 wheel_base={self._wheel_base:.3f} "
            f"ws={self._ws_host}:{self._ws_port}")

    # ── パラメータ変更をランタイムで反映 (キャリブレーション用) ────────
    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'wheel_base':
                self._wheel_base = p.value
                self.get_logger().info(f"wheel_base 更新: {self._wheel_base:.6f} m")
        return SetParametersResult(successful=True)

    # ── WebSocket サーバー (別スレッドの asyncio イベントループ) ──────
    def _run_ws_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
        except BaseException as e:
            # bind 失敗 (ポート使用中等) やイベントループの想定外クラッシュ。
            # WS サーバー無しでノードだけ生き続けると「起動しているのに
            # ESP32 が繋がらない」状態に陥るため、即座にプロセスごと落とす
            # (launch の respawn や再起動で復帰させる方が安全)。
            self.get_logger().fatal(
                f"WebSocket サーバー停止: {type(e).__name__}: {e} — ノードを終了します "
                f"(ポート使用中の場合は別の esp32_bridge が動いていないか確認)")
            import os
            os._exit(1)

    async def _serve(self):
        async with websockets.serve(self._handle_client, self._ws_host, self._ws_port):
            await asyncio.Future()  # run forever

    async def _handle_client(self, websocket):
        with self._ws_conn_lock:
            self._ws_conn = websocket
        # 接続のたびにリセットする: 次に届く ESTOP_HW フレームで
        # /safety/firmware_flags を必ず publish させる(接続時に改めて知らせる。
        # WP-ESP32-01 §3.1「変化時＋接続時」)。rclpy 側スレッドの
        # _handle_frame からしか _last_firmware_flags を読み書きしないため
        # (drain は同一タイマーコールバック内)、ここでの代入だけロックなしで安全。
        self._last_firmware_flags = None
        self.get_logger().info(f"ESP32 接続: {websocket.remote_address}")
        try:
            async for message in websocket:
                if not isinstance(message, (bytes, bytearray)):
                    continue
                self._rx_queue.push(bytes(message))
        finally:
            with self._ws_conn_lock:
                if self._ws_conn is websocket:
                    self._ws_conn = None
            self.get_logger().warn("ESP32 切断")

    # ── /cmd_vel → WHEEL_CMD 送信 ─────────────────────────────────
    def _cb_cmd_vel(self, msg: Twist):
        self._last_cmd_vel = msg
        self._last_cmd_vel_time = self.get_clock().now()
        self._send_wheel_cmd()

    # ── ロック状態購読 ─────────────────────────────────────────
    def _cb_estop(self, msg: Bool):
        self._estop_active = msg.data
        self._last_estop_msg_time = self.get_clock().now()

    def _cb_fault_lock(self, msg: Bool):
        self._fault_lock_active = msg.data
        self._last_fault_lock_msg_time = self.get_clock().now()

    def _is_locked(self) -> bool:
        if self._estop_active or self._fault_lock_active:
            return True
        return self._is_stale(self._last_estop_msg_time) or \
            self._is_stale(self._last_fault_lock_msg_time)

    def _is_stale(self, last_msg_time) -> bool:
        # 未受信 (起動直後) も安全側にロック扱い。safety_monitor は起動後
        # 即座に 10Hz で送り始めるため、正常時にここで待たされるのは一瞬。
        if last_msg_time is None:
            return True
        elapsed = (self.get_clock().now() - last_msg_time).nanoseconds / 1e9
        return elapsed > self._LOCK_TIMEOUT_SEC

    # ── キープアライブ: 最新の /cmd_vel を一定周期で再送 ────────────
    # DEBT-4 対処 (WP-SAFE-02): 20Hz の送信そのものは止めない (K-1)。
    # ロック中 or /cmd_vel が cmd_vel_stale_ms 途絶していれば、送る内容
    # (参照値) をゼロへ書き換える。「送るのをやめる」のではなく
    # 「ゼロを送る」——止めるとウォッチドッグ経由(最大 esp32_watchdog_ms
    # =600ms)になり、PC 側の対処(cmd_vel_stale_ms=400ms)より遅れる。
    def _cb_cmd_vel_keepalive(self):
        self._send_wheel_cmd()

    def _send_wheel_cmd(self):
        locked = self._is_locked()
        if locked:
            # twist_mux が無出力(silent)のままでも取り残された非ゼロ指令を
            # 再送し続けないよう、ここで強制的にゼロにする。_last_cmd_vel
            # 自体もここでゼロ化しておかないと、ロック解除の瞬間に
            # キープアライブがロック前の古い非ゼロ指令をそのまま再送して
            # しまう(2026-08-06 E-Stop 解除後に前進指令が復活する事象で発覚)。
            self._last_cmd_vel = Twist()
            self._last_cmd_vel_time = None

        now_ms = self.get_clock().now().nanoseconds / 1e6
        last_cmd_ms = (float('-inf') if self._last_cmd_vel_time is None
                       else self._last_cmd_vel_time.nanoseconds / 1e6)
        result = keepalive_value(
            last_cmd_ms=last_cmd_ms,
            now_ms=now_ms,
            last_cmd=Twist2(self._last_cmd_vel.linear.x, self._last_cmd_vel.angular.z),
            stale_ms=self._cmd_vel_stale_ms,
            locked=locked)

        v, w = result.linear, result.angular
        v_right = v + (w * self._wheel_base / 2.0)
        v_left = v - (w * self._wheel_base / 2.0)
        frame = pack_wheel_cmd(v_left, v_right)

        cmd_fb = WheelFeedback()
        cmd_fb.header.stamp = self.get_clock().now().to_msg()
        cmd_fb.header.frame_id = self._base_frame
        cmd_fb.left_speed = v_left
        cmd_fb.right_speed = v_right
        self._pub_wheel_cmd.publish(cmd_fb)

        with self._ws_conn_lock:
            conn = self._ws_conn
            loop = self._loop
        if conn is None or loop is None:
            return  # ESP32 未接続 — ESP32 側もローカルウォッチドッグで安全側に倒れる

        # D-1: 送信コアレッシング。in-flight でなければ即座に送信を投げ、
        # in-flight 中なら保留フレームを上書きするだけでコルーチンは追加しない
        # (send_coalescer.py の docstring 参照)。
        with self._coalescer_lock:
            coalesce_result = coalescer_offer(self._coalescer_state, frame)
            self._coalescer_state = coalesce_result.state
        if coalesce_result.frame_to_send is not None:
            asyncio.run_coroutine_threadsafe(
                self._send_and_continue(coalesce_result.frame_to_send), loop)

    async def _send_and_continue(self, frame: bytes):
        """1フレーム送信し、完了時に保留フレームがあれば続けて送る。

        conn は送信のたびに読み直す (ESP32 の再接続を跨いで古い conn オブジェクト
        へ送り続けないため)。
        """
        with self._ws_conn_lock:
            conn = self._ws_conn
        if conn is not None:
            await self._send_safe(conn, frame)
        with self._coalescer_lock:
            result = coalescer_complete(self._coalescer_state)
            self._coalescer_state = result.state
        if result.frame_to_send is not None:
            await self._send_and_continue(result.frame_to_send)

    @staticmethod
    async def _send_safe(conn, frame: bytes):
        try:
            await conn.send(frame)
        except Exception:
            pass  # 切断済み等 — 次周期の cmd_vel で再送されるため無視してよい

    # ── 受信キューの drain (rclpy スレッド側) ─────────────────────
    def _drain_rx_queue(self):
        # D-2: 1回の呼び出しで処理するフレーム数を有界にする (バックログが
        # あっても、このタイマーコールバックが際限なく executor を占有しない
        # ようにするため。rx_backpressure.py 参照)。
        frames = self._rx_queue.drain(self._rx_queue_drain_max_per_cycle)
        for data in frames:
            self._handle_frame(data)

    def _handle_frame(self, data: bytes):
        try:
            msg_type = peek_type(data)
            if msg_type == WHEEL_FEEDBACK:
                left, right, dt = unpack_wheel_feedback(data)
                self._on_wheel_feedback(left, right, dt)
            elif msg_type == ESTOP_HW:
                estop, flags = unpack_estop_hw_flags(data)
                self._pub_estop_hw.publish(Bool(data=estop))
                # 判定はしない(safety_monitor が WP-SAFE-01 で行う)。ここは
                # 変化時 ＋ 接続時(_handle_client で _last_firmware_flags を
                # None に戻す)だけ publish する。
                if flags != self._last_firmware_flags:
                    self._last_firmware_flags = flags
                    self._pub_firmware_flags.publish(UInt8(data=flags))
            elif msg_type == IMU_DATA:
                self._on_imu_data(*unpack_imu_data(data))
            else:
                self.get_logger().warn(f"未知の type tag: 0x{msg_type:02x}")
        except ProtocolError as e:
            self.get_logger().warn(f"不正なフレームを受信: {e}")

    # ── WHEEL_FEEDBACK → /odom + TF ───────────────────────────────
    def _on_wheel_feedback(self, v_left: float, v_right: float,
                            dt_from_esp32: "float | None" = None):
        now = self.get_clock().now()
        self._last_feedback_time = now
        self._esp32_alive = True

        # ── 積分区間は「到着間隔」ではなく ESP32 が速度算出に使った dt ──────
        # ESP32 は velL = counts * distPerCount / dt (esp32/src/main.cpp) で
        # 速度を出しているので、そのフレームが表す走行時間はまさにこの dt。
        # フレームに載せて送ってもらい、そのまま積分区間として使う。
        #
        # 以前は到着時刻の差分を dt にしていたが、これは「車輪が回った時刻」ではなく
        # 「WiFi/TCP がパケットを配送した時刻」を測っている。docs/network.md 記載の
        # 0.5〜1.2 秒の受信ギャップがそのまま積分誤差になり、さらに dt>1.0 の
        # 場合は更新ごと return で破棄していたため、ギャップ中の回転が丸ごと
        # 消えていた (旋回時の yaw ドリフト。2026-08-06 修正)。
        #
        # 旧ファームウェア (dt なし) からは None が来るので公称周期で代替する。
        # 壊れたフレームの異常値をそのまま yaw に積むのは元の不具合より悪いため、
        # 範囲外の値も公称周期へ落とす。
        # ファームウェアの世代をログに出す。後方互換にしてある都合上、旧形式でも
        # 黙って動いてしまうため、「書き込んだつもり」の取り違えに気づけない。
        # dt の有無とジャイロ単位修正は同じビルドに入るので、旧形式が届いている
        # ことは「ジャイロもまだ dps」を意味する。
        has_dt = dt_from_esp32 is not None
        if has_dt != self._feedback_has_dt:
            self._feedback_has_dt = has_dt
            if has_dt:
                self.get_logger().info(
                    "WHEEL_FEEDBACK: 新形式 (dt 付き) を受信。ESP32 側の制御周期で"
                    "オドメトリを積分します")
            else:
                self.get_logger().error(
                    "WHEEL_FEEDBACK: 旧形式 (dt なし) を受信 — ESP32 ファームウェアが "
                    "2026-08-06 の更新前です。オドメトリの積分区間は公称 "
                    f"{self._feedback_period_sec * 1000:.0f} ms で代替します。"
                    "同じ更新にジャイロの dps→rad/s 修正が入っているため、"
                    "この状態で imu_enabled:=true にすると EKF が 57.3 倍の"
                    "ヨーレートを信じてオドメトリが壊れます")

        dt, used_esp32_dt = resolve_dt(
            dt_from_esp32, self._feedback_period_sec, self._DT_MAX_SEC)
        if not used_esp32_dt and dt_from_esp32 is not None:
            # フィールド自体は届いたが値が範囲外 (has_dt=True のケース)。
            # フィールド自体が無かった (旧形式) 場合は上の has_dt ブロックで
            # 既に error ログを出しているので、ここでは二重に警告しない。
            self.get_logger().warn(
                f"ESP32 から不正な dt={dt_from_esp32} を受信。公称周期で代替します",
                throttle_duration_sec=5.0)

        # ── ヘッダスタンプも dt で進める ────────────────────────────
        # robot_localization は /odom の pose を使わず twist とヘッダスタンプの
        # 差分で積分する (ekf_params.yaml の odom0_config は vx/vyaw のみ true)。
        # ここで到着時刻を貼ると、遅延後にまとめて届いたフレーム群がほぼ同一の
        # スタンプを持ち、EKF 側では積分区間ゼロになって内部の姿勢がいくら
        # 正しくても TF に反映されない。dt ぶんずつ進む単調なスタンプを使う。
        #
        # ただし実時間から離れすぎると、slam_toolbox がスキャン時刻で TF を
        # 引けなくなる (slam_params.yaml の transform_timeout 参照)。
        # ずれが許容量を超えたら現在時刻へ貼り直す。
        now_sec = now.nanoseconds / 1e9
        resync = resync_stamp(self._odom_stamp_sec, now_sec, dt, self._STAMP_RESYNC_SEC)
        if resync.resynced:
            skew = abs(now_sec - (self._odom_stamp_sec + dt))
            self.get_logger().warn(
                f"オドメトリのスタンプが実時間から {skew:.2f} s ずれたため貼り直します",
                throttle_duration_sec=5.0)
        self._odom_stamp_sec = resync.stamp_sec
        t = Time(nanoseconds=round(self._odom_stamp_sec * 1e9), clock_type=now.clock_type)

        # 到着間隔そのものは診断情報として監視する (積分には使わない)。
        arrival_dt = (now - self._last_odom_time).nanoseconds / 1e9
        self._last_odom_time = now
        if arrival_dt > self._ARRIVAL_GAP_WARN_SEC:
            self.get_logger().warn(
                f"wheel_feedback の受信ギャップ {arrival_dt:.2f} s "
                f"(ESP32 側 dt {dt:.3f} s)。WiFi 遅延の可能性",
                throttle_duration_sec=5.0)

        # ── 差動駆動オドメトリ更新 ──────────────────────────
        new_pose, v_center, omega = integrate_pose(
            Pose2D(self._odom_x, self._odom_y, self._odom_yaw),
            v_left, v_right, self._wheel_base, dt)
        self._odom_x, self._odom_y, self._odom_yaw = new_pose.x, new_pose.y, new_pose.yaw

        # yaw のみの姿勢なのでクォータニオンは手計算で十分 (roll=pitch=0)
        qz, qw = yaw_to_quaternion_zw(self._odom_yaw)

        # ── /odom 発行 ───────────────────────────────────────
        odom = Odometry()
        odom.header.stamp = t.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_center
        odom.twist.twist.angular.z = omega

        # 共分散 (暫定値 — 実機キャリブ後に調整)
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.05

        self._pub_odom.publish(odom)

        # ── TF: odom → base_link ─────────────────────────────
        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = t.to_msg()
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._odom_x
            tf.transform.translation.y = self._odom_y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf)

        # ── /esp32/wheel_feedback (カスタム型) 発行 ────────────
        fb = WheelFeedback()
        fb.header.stamp = t.to_msg()
        fb.header.frame_id = self._base_frame
        fb.left_speed = v_left
        fb.right_speed = v_right
        self._pub_wheel_feedback.publish(fb)

    # ── IMU_DATA → /esp32/imu_data + /esp32/imu_calib_status ──────
    def _on_imu_data(self, qw, qx, qy, qz, wx, wy, wz, ax, ay, az, calib_status):
        # ジャイロ単位の取り違えを実機で検知する。sensor_msgs/Imu は rad/s 規定だが、
        # 2026-08-06 より前のファームウェアは Adafruit_BNO055 の戻り値(dps)を
        # そのまま送ってくるため 57.3 倍になる。この機体の最大ヨーレートは
        # 1 rad/s 程度(planning_params.yaml の w_max 系)なので、それを大きく
        # 超える値が続くなら単位の取り違えを疑う。
        # 低速域では dps の値も閾値を下回るため、これは「気づける」ための警告で
        # あって保証ではない。EKF への投入可否は imu_enabled で明示的に管理する。
        if abs(wz) > self._IMU_WZ_IMPLAUSIBLE_RAD_S:
            self.get_logger().error(
                f"IMU の角速度が非現実的です (wz={wz:.1f} rad/s)。"
                f"ジャイロ単位が dps のままの可能性があります "
                f"(ESP32 ファームウェアの再書き込みが必要)。"
                f"この状態で imu_enabled:=true にするとオドメトリが壊れます",
                throttle_duration_sec=10.0)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        imu.orientation.w = qw
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.angular_velocity.x = wx
        imu.angular_velocity.y = wy
        imu.angular_velocity.z = wz
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        # 共分散 (暫定値 — 実機キャリブ後に調整。ekf_params.yaml の imu0_config は
        # yaw/vyaw のみ使用するため roll/pitch/加速度側は EKF に影響しない)
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.01
        imu.angular_velocity_covariance[0] = 0.0025
        imu.angular_velocity_covariance[4] = 0.0025
        imu.angular_velocity_covariance[8] = 0.0025
        imu.linear_acceleration_covariance[0] = 0.04
        imu.linear_acceleration_covariance[4] = 0.04
        imu.linear_acceleration_covariance[8] = 0.04

        self._pub_imu.publish(imu)
        self._pub_imu_calib.publish(UInt8(data=calib_status))

    # ── フィードバック死活監視 ────────────────────────────────────
    def _cb_watchdog(self):
        elapsed = (self.get_clock().now() - self._last_feedback_time).nanoseconds / 1e9
        timeout = self.get_parameter('feedback_timeout_ms').value / 1000.0
        if self._esp32_alive and elapsed > timeout:
            self.get_logger().warn(f"ESP32 wheel_feedback タイムアウト ({elapsed:.1f} s)")
            self._esp32_alive = False
            # safety_monitor が /esp32/wheel_feedback の途絶を別途検知するため
            # ここでは警告のみ(二重処理を避ける)


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
