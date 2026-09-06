#!/usr/bin/env python3
"""
serial_test.py — ESP32単体でのシリアル通信確認 (ROS2/pi_serial_relay 不要)
====================================================================
モーター等を未配線のボード単体で、PCへ直接USB接続したまま書き込みとシリアル
プロトコル疎通を確認する（旧 ws_test_server.py の置き換え。2026-09-05
シリアル化に伴い、WebSocketサーバーではなくPCから直接シリアルポートを開いて
確認する形に変わった。pi_serial_relay を経由しない単体テストなので、
最終的な Pi 経由の疎通確認は docs/network.md の手順で別途行うこと）。

使い方:
  1. ESP32 をPCにUSB接続し、書き込む:
       cd th_ws/esp32 && pio run --target upload
  2. このスクリプトを実行する (pyserial が要る: pip install pyserial):
       python3 th_ws/esp32/tools/serial_test.py /dev/ttyUSB0 --send-test-cmd

確認できること:
  - 書き込み確認: 起動バナー(`TH System ESP32 Firmware (Serial)` + ビルド日時)
    をそのまま画面に流す(ASCIIノイズとして無視されるはずのバナーそのもの)。
  - 通信確認: ESP32→PC 方向の WHEEL_FEEDBACK / ESTOP_HW / IMU_DATA フレームを
    デコードして表示する。
  - `--send-test-cmd` を付けると PC→ESP32 方向(WHEEL_CMD)も確認できる
    (motor が未配線でも、速度 0 を送るだけなので安全)。
"""
import argparse
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "src" / "th_esp32_bridge" / "th_esp32_bridge"))
import serial_framer   # noqa: E402
import ws_protocol     # noqa: E402


def describe(payload: bytes) -> str:
    tag = ws_protocol.peek_type(payload)
    try:
        if tag == ws_protocol.WHEEL_FEEDBACK:
            left, right, dt = ws_protocol.unpack_wheel_feedback(payload)
            return f"WHEEL_FEEDBACK left={left:.3f} right={right:.3f} dt={dt}"
        if tag == ws_protocol.ESTOP_HW:
            active, flags = ws_protocol.unpack_estop_hw_flags(payload)
            return f"ESTOP_HW active={active} flags=0x{flags:02x}"
        if tag == ws_protocol.IMU_DATA:
            vals = ws_protocol.unpack_imu_data(payload)
            return f"IMU_DATA calib={vals[-1]}"
    except ws_protocol.ProtocolError as e:
        return f"デコード失敗: {e}"
    return f"未知の tag=0x{tag:02x} len={len(payload)}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="例: /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--send-test-cmd", action="store_true",
                         help="起動2秒後に WHEEL_CMD(left=0, right=0) を送る")
    args = parser.parse_args()

    # pi_serial_relay.open_serial() と違い、ここでは意図的に DTR/RTS 対策を
    # しない: 単体テストなので ESP32 が open() のたびに再起動して起動バナー
    # から流れ直すことを期待している(本番のpi_serial_relayでは逆にこれを
    # 避ける。docs/network.md参照)。
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    decoder = serial_framer.SerialFrameDecoder()
    sent_test_cmd = False
    start = time.monotonic()
    print(f"[serial_test] {args.port} @ {args.baud} を監視します (Ctrl+C で終了)")
    try:
        while True:
            data = ser.read(4096)
            if data:
                for frame in decoder.feed(data):
                    print(f"[受信] {describe(frame)}")
            if args.send_test_cmd and not sent_test_cmd and time.monotonic() - start > 2.0:
                ser.write(serial_framer.encode_frame(ws_protocol.pack_wheel_cmd(0.0, 0.0)))
                print("[送信] WHEEL_CMD left=0.0 right=0.0")
                sent_test_cmd = True
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
