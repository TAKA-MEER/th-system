#!/usr/bin/env python3
"""
esp32_bridge.py — ROS2 ノード本体
====================================
ws_protocol.py のバイナリプロトコルで ESP32 と WebSocket 通信する。
esp32_bridge がサーバー、ESP32 がクライアントとして接続しにくる(6.1節参照)。

担当:
  1. /cmd_vel (Twist) → 差動駆動変換 → WHEEL_CMD フレーム送信
  2. WHEEL_FEEDBACK フレーム受信 → /odom + TF 計算
  3. ESTOP_HW フレーム受信 → /safety/estop_hw へ中継
  4. /esp32/wheel_feedback (WheelFeedback) 発行 (他ノード用)
"""
import asyncio
import math
import queue
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from th_system_msgs.msg import WheelFeedback
import tf2_ros

import websockets

from th_esp32_bridge.ws_protocol import (
    ProtocolError, WHEEL_FEEDBACK, ESTOP_HW,
    pack_wheel_cmd, unpack_wheel_feedback, unpack_estop_hw, peek_type,
)


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('wheel_radius', 0.0391)
        self.declare_parameter('wheel_base', 0.39)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('feedback_timeout_ms', 500)
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)

        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._wheel_base = self.get_parameter('wheel_base').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._ws_host = self.get_parameter('ws_host').value
        self._ws_port = self.get_parameter('ws_port').value

        # ── Publishers ─────────────────────────────────────
        self._pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self._pub_wheel_feedback = self.create_publisher(
            WheelFeedback, '/esp32/wheel_feedback', 10)
        self._pub_estop_hw = self.create_publisher(Bool, '/safety/estop_hw', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)

        self._odom_x = self._odom_y = self._odom_yaw = 0.0
        self._last_odom_time = self.get_clock().now()
        self._last_feedback_time = self.get_clock().now()
        self._esp32_alive = False

        # ── フィードバック死活監視タイマー ─────────────────
        timeout_ms = self.get_parameter('feedback_timeout_ms').value
        self.create_timer(timeout_ms / 1000.0, self._cb_watchdog)

        # 受信フレームは asyncio スレッドからロック付きキューへ積み、rclpy 側の
        # タイマーで drain する(rclpy publisher は任意スレッドからの同時呼び出し
        # が保証されないため)。
        self._rx_queue: queue.Queue = queue.Queue()
        self.create_timer(0.02, self._drain_rx_queue)  # 50Hz

        self._loop: "asyncio.AbstractEventLoop | None" = None
        self._ws_conn_lock = threading.Lock()
        self._ws_conn = None  # 現在接続中の ESP32 (単一クライアント想定)

        self._ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f"esp32_bridge 起動 wheel_radius={self._wheel_radius:.4f} "
            f"wheel_base={self._wheel_base:.3f} ws={self._ws_host}:{self._ws_port}")

    # ── WebSocket サーバー (別スレッドの asyncio イベントループ) ──────
    def _run_ws_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self):
        async with websockets.serve(self._handle_client, self._ws_host, self._ws_port):
            await asyncio.Future()  # run forever

    async def _handle_client(self, websocket):
        with self._ws_conn_lock:
            self._ws_conn = websocket
        self.get_logger().info(f"ESP32 接続: {websocket.remote_address}")
        try:
            async for message in websocket:
                if not isinstance(message, (bytes, bytearray)):
                    continue
                self._rx_queue.put(bytes(message))
        finally:
            with self._ws_conn_lock:
                if self._ws_conn is websocket:
                    self._ws_conn = None
            self.get_logger().warn("ESP32 切断")

    # ── /cmd_vel → WHEEL_CMD 送信 ─────────────────────────────────
    def _cb_cmd_vel(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        v_right = v + (w * self._wheel_base / 2.0)
        v_left = v - (w * self._wheel_base / 2.0)
        frame = pack_wheel_cmd(v_left, v_right)

        with self._ws_conn_lock:
            conn = self._ws_conn
            loop = self._loop
        if conn is None or loop is None:
            return  # ESP32 未接続 — ESP32 側もローカルウォッチドッグで安全側に倒れる
        asyncio.run_coroutine_threadsafe(self._send_safe(conn, frame), loop)

    @staticmethod
    async def _send_safe(conn, frame: bytes):
        try:
            await conn.send(frame)
        except Exception:
            pass  # 切断済み等 — 次周期の cmd_vel で再送されるため無視してよい

    # ── 受信キューの drain (rclpy スレッド側) ─────────────────────
    def _drain_rx_queue(self):
        while True:
            try:
                data = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_frame(data)

    def _handle_frame(self, data: bytes):
        try:
            msg_type = peek_type(data)
            if msg_type == WHEEL_FEEDBACK:
                left, right = unpack_wheel_feedback(data)
                self._on_wheel_feedback(left, right)
            elif msg_type == ESTOP_HW:
                estop = unpack_estop_hw(data)
                self._pub_estop_hw.publish(Bool(data=estop))
            else:
                self.get_logger().warn(f"未知の type tag: 0x{msg_type:02x}")
        except ProtocolError as e:
            self.get_logger().warn(f"不正なフレームを受信: {e}")

    # ── WHEEL_FEEDBACK → /odom + TF ───────────────────────────────
    def _on_wheel_feedback(self, v_left: float, v_right: float):
        self._last_feedback_time = self.get_clock().now()
        self._esp32_alive = True

        t = self.get_clock().now()
        dt = (t - self._last_odom_time).nanoseconds / 1e9
        self._last_odom_time = t
        if dt <= 0.0 or dt > 1.0:
            return

        # ── 差動駆動オドメトリ更新 ──────────────────────────
        v_center = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self._wheel_base
        dtheta = omega * dt

        self._odom_x += v_center * math.cos(self._odom_yaw + dtheta / 2.0) * dt
        self._odom_y += v_center * math.sin(self._odom_yaw + dtheta / 2.0) * dt
        self._odom_yaw += dtheta

        # yaw のみの姿勢なのでクォータニオンは手計算で十分 (roll=pitch=0)
        qz = math.sin(self._odom_yaw / 2.0)
        qw = math.cos(self._odom_yaw / 2.0)

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
