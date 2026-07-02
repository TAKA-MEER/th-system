#!/usr/bin/env python3
"""
ws_test_server.py - ESP32単体(開発ボードのみ)向け WebSocket 通信テストツール
================================================================================
esp32_bridge(rclpy ノード)や colcon/ROS2/Docker を一切使わず、素の Python
(`pip install websockets` のみ)でESP32ファームウェアの書き込み・WiFi/WS通信を
確認するための最小ツール。モーター・エンコーダ・E-Stopスイッチが未配線の
開発ボード単体でも、以下を確認できる:

  1. ファームウェア書き込み確認: ESP32のシリアルモニタ(`pio device monitor`)に
     起動バナーが出ること。
  2. 通信確認: 本ツールを実行した状態でESP32が起動すると、ここに
     "ESP32 接続" ログが出て、以降 ESTOP_HW フレーム(毎周期送信)が届き続ける
     ことで ESP32→PC 方向の通信を確認できる。
  3. (--send-test-cmd 指定時) PC→ESP32 方向のテストとして WHEEL_CMD を周期送信し、
     ESP32側シリアルモニタに [WHEEL_CMD] ログが出ることを確認する
     (モーター未配線でも安全。開発ボード単体でのみ使うこと)。

使い方:
  pip install websockets
  python ws_test_server.py                        # 受信のみ(安全なデフォルト)
  python ws_test_server.py --send-test-cmd         # WHEEL_CMD も周期送信してPC→ESP32を確認
  python ws_test_server.py --host 0.0.0.0 --port 8765

esp32_bridge の config/params.yaml の ws_host/ws_port、および ESP32側の
wifi_credentials.h の WS_SERVER_HOST/WS_SERVER_PORT と同じ値を使うこと。
"""
import argparse
import asyncio
import os
import sys

# Windows のデフォルトコンソール(cp932)では一部の記号でUnicodeEncodeErrorに
# なりうるため、標準出力をUTF-8に固定する
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding='utf-8')
    except (AttributeError, ValueError):
        pass

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'src', 'th_esp32_bridge', 'th_esp32_bridge'))

from ws_protocol import (  # noqa: E402
    WHEEL_FEEDBACK, ESTOP_HW,
    ProtocolError,
    pack_wheel_cmd, unpack_wheel_feedback, unpack_estop_hw, peek_type,
)


async def handle_client(websocket, send_test_cmd: bool):
    print(f"[接続] ESP32 接続: {websocket.remote_address}")
    sender_task = None
    if send_test_cmd:
        sender_task = asyncio.create_task(_send_test_cmds(websocket))
    try:
        async for message in websocket:
            if not isinstance(message, (bytes, bytearray)):
                continue
            _log_frame(bytes(message))
    finally:
        if sender_task is not None:
            sender_task.cancel()
        print("[切断] ESP32 切断")


async def _send_test_cmds(websocket):
    """PC→ESP32方向のテスト。左右交互に小さな速度指令を送る(モーター未配線前提)。"""
    toggle = False
    try:
        while True:
            speed = 0.10 if toggle else 0.0
            toggle = not toggle
            frame = pack_wheel_cmd(speed, speed)
            await websocket.send(frame)
            print(f"[送信] WHEEL_CMD left={speed:.2f} right={speed:.2f}")
            await asyncio.sleep(2.0)
    except asyncio.CancelledError:
        pass


def _log_frame(data: bytes):
    try:
        msg_type = peek_type(data)
        if msg_type == WHEEL_FEEDBACK:
            left, right = unpack_wheel_feedback(data)
            print(f"[受信] WHEEL_FEEDBACK left={left:.3f} right={right:.3f}")
        elif msg_type == ESTOP_HW:
            active = unpack_estop_hw(data)
            print(f"[受信] ESTOP_HW active={active}")
        else:
            print(f"[受信] 未知の type tag: 0x{msg_type:02x}")
    except ProtocolError as e:
        print(f"[エラー] 不正なフレーム: {e}")


async def main_async(host: str, port: int, send_test_cmd: bool):
    import websockets

    async def handler(websocket):
        await handle_client(websocket, send_test_cmd)

    print(f"WebSocket テストサーバー起動: {host}:{port}")
    print("ESP32 の起動を待っています... (Ctrl+C で終了)")
    async with websockets.serve(handler, host, port):
        await asyncio.Future()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0', help='bind アドレス')
    parser.add_argument('--port', type=int, default=8765, help='bind ポート')
    parser.add_argument('--send-test-cmd', action='store_true',
                         help='WHEEL_CMDを周期送信してPC→ESP32方向も確認する'
                              '(モーター未配線の開発ボード単体でのみ使うこと)')
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args.host, args.port, args.send_test_cmd))
    except KeyboardInterrupt:
        print("\n終了しました")


if __name__ == '__main__':
    main()
