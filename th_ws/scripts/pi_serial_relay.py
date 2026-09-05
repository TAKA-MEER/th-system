#!/usr/bin/env python3
"""
pi_serial_relay.py — ラズパイ上で動く ESP32(シリアル) ⇔ esp32_bridge(WebSocket) 中継
====================================================================
2026-09-05: ESP32↔PC間の無線WebSocketを廃止し、ESP32はUSB-UARTでラズパイに
直結、ラズパイ上のこのプロセスがPC側の esp32_bridge(WebSocketサーバー、
8766番、無変更)へクライアントとして接続する構成に変更した(VISION.md
「ESP32の無線化をやめ、ラズパイ経由のシリアル接続にする」参照)。

役割:
  ESP32 --USB/UART(エンベロープ付き)--> [このプロセス] --WS client--> PC:8766 esp32_bridge

このプロセス自身は ws_protocol フレームの中身(WHEEL_CMD/WHEEL_FEEDBACK/...)を
一切解釈しない。エンベロープを剥がした/被せたバイト列をそのまま右から左へ
流すだけで、中身の意味付けは PC 側の esp32_bridge に委ねる。エンベロープの
符号化/復号だけを serial_framer.py に丸投げする。同ファイルは pure Python
(rclpy 非依存) なので、colcon ビルド無しでこのスクリプトと同じディレクトリに
置くだけで動く(th_ws/src/th_esp32_bridge/th_esp32_bridge/serial_framer.py の
コピー。中身は変更しないこと。配置方法は docs/network.md 参照)。

デプロイ上の重要な注意 (docs/network.md にも記載):
  - シリアルポートは必ず /dev/serial/by-id/... で指定すること。RPLIDAR も同じ
    ラズパイの USB-UART なので、/dev/ttyUSB0 のような列挙順に依存した指定は
    ある日 LiDAR と ESP32 を取り違える(VISION.md 参照)。
  - ポートを開く際に DTR/RTS を上げると、ESP32 の自動リセット回路が誤発動して
    再起動してしまう(シリアルモニタの既知の注意点と同じ原因)。このスクリプトは
    open() 前に dtr/rts を明示的に落としてから開くことでこれを避けている。
    close 時の HUPCL による再アサートは stty 側で無効化する
    (systemd unit の ExecStartPre 参照)。
"""
import argparse
import asyncio
import logging
import sys
import time
from pathlib import Path

import serial
import websockets
import websockets.exceptions  # websockets>=13 は遅延importのため明示的にimportが必要

sys.path.insert(0, str(Path(__file__).resolve().parent))
import serial_framer          # noqa: E402  (同ディレクトリに配置する想定)

LOG = logging.getLogger("pi_serial_relay")

RECONNECT_DELAY_S = 2.0


def open_serial(port: str, baud: int) -> serial.Serial:
    """DTR/RTSを上げずにポートを開く(ESP32の自動リセット回路対策)。"""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    # timeout=None (無期限ブロック) にすること。ser.read(size) は「size バイト
    # 溜まるか timeout が経過するまで」待つため、size>1 に有限timeoutを
    # 組み合わせると「timeout の粒度で必ず足止めされる」固定ポーリングになる
    # (実測: timeout=0.05 だと WHEEL_FEEDBACK の到着間隔が 50ms 周期でビート
    # を起こし、教示再生時のふらつきの原因になった。2026-09-05)。
    # None にして「1バイト来るまで無期限に待つ」→「その後は溜まっている分だけ
    # 即座に読む」の2段構えにすることで、カーネルのselect起床にほぼ一致した
    # 低遅延・低ジッタな受信になる(serial_to_ws参照)。
    ser.timeout = None
    ser.dsrdtr = False
    ser.rtscts = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def _blocking_read_batch(ser: serial.Serial) -> bytes:
    """1バイト来るまで無期限に待ち、そこから溜まっている分をまとめて読む。

    executorスレッドの中で呼ぶ前提(ここでブロックしてもイベントループは
    止まらない)。timeout=None の ser.read(1) はデータが来るまでCPUを使わず
    カーネル側で寝る(ビジーポーリングにならない)ため、待機コストは実質ゼロ。
    """
    first = ser.read(1)
    if not first:
        return b""  # timeout=Noneなら通常起きない。ポート断でSerialExceptionが飛ぶ
    rest = ser.read(ser.in_waiting) if ser.in_waiting else b""
    return first + rest


async def serial_to_ws(ser: serial.Serial, ws, decoder: "serial_framer.SerialFrameDecoder",
                        loop: asyncio.AbstractEventLoop) -> None:
    """シリアルから読めたバイトをデコードし、フレーム単位でPCへ転送する。"""
    while True:
        data = await loop.run_in_executor(None, _blocking_read_batch, ser)
        if data:
            for frame in decoder.feed(data):
                await ws.send(frame)


async def ws_to_serial(ws, ser: serial.Serial, loop: asyncio.AbstractEventLoop) -> None:
    """PCから届いたフレーム(WHEEL_CMD等)をエンベロープに包んでシリアルへ書く。

    WSが正常にcloseされて `async for` が抜けたときも例外を投げて呼び出し元の
    gatherを終わらせる。そうしないと(ESP32が繋がっていない等で)serial_to_ws
    側が何も送信せず、gatherが永久に片肺のまま止まる。
    """
    async for message in ws:
        if not isinstance(message, (bytes, bytearray)):
            continue
        envelope = serial_framer.encode_frame(bytes(message))
        await loop.run_in_executor(None, ser.write, envelope)
    raise RuntimeError("WS接続が閉じられました(ws_to_serial側で検知)")


async def relay_once(ser: serial.Serial, ws_uri: str) -> None:
    decoder = serial_framer.SerialFrameDecoder()
    loop = asyncio.get_running_loop()
    LOG.info("esp32_bridge へ接続します: %s", ws_uri)
    async with websockets.connect(ws_uri) as ws:
        # WS が切れていた間もシリアルポート自体は開いたまま(DTR/RTSを動かさない
        # ため)なので、カーネルのtty受信バッファに数秒分のフレームが溜まって
        # いることがある。再接続直後にそれをそのまま流すと、古いWHEEL_FEEDBACK
        # が一気に届いてオドメトリが瞬間的に破綻するため、接続直後に読み捨てる。
        ser.reset_input_buffer()
        LOG.info("接続しました: %s", ws_uri)
        tasks = [
            asyncio.ensure_future(serial_to_ws(ser, ws, decoder, loop)),
            asyncio.ensure_future(ws_to_serial(ws, ser, loop)),
        ]
        try:
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
            for task in done:
                task.result()  # 例外があれば再送出し、呼び出し元の再接続ループへ落とす
        finally:
            for task in tasks:
                task.cancel()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial-port", required=True,
                         help="/dev/serial/by-id/... を指定すること(/dev/ttyUSB* は不可)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--ws-host", default="192.168.5.50")
    parser.add_argument("--ws-port", type=int, default=8766)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                         format="%(asctime)s [%(levelname)s] %(message)s")

    if "by-id" not in args.serial_port and "by-path" not in args.serial_port:
        LOG.warning("serial-port が /dev/serial/by-id ではありません (%s)。"
                     "USB列挙順が変わるとLiDARと衝突する恐れがあります。",
                     args.serial_port)

    ws_uri = f"ws://{args.ws_host}:{args.ws_port}"

    ser = open_serial(args.serial_port, args.baud)
    try:
        while True:
            try:
                asyncio.run(relay_once(ser, ws_uri))
            except serial.SerialException as e:
                # ser.read()/ser.write() がここから投げるのは「ポート自体が
                # 死んだ」場合(ESP32抜線・USB切断等)。serial.SerialException は
                # OSError のサブクラスなので、下のWS用exceptより先に捕まえる
                # 必要がある(順番を変えるとここに来ずWS再接続扱いになり、
                # 死んだfdへの再試行を無限に続けてしまう)。
                # ポートの再オープンはこのプロセスの責務にせず、systemdの
                # Restart=always に委ねて素直に終了する。
                LOG.error("シリアルポートが切れました: %r。終了します(systemdが再起動)", e)
                sys.exit(1)
            except (OSError, websockets.exceptions.WebSocketException, RuntimeError) as e:
                LOG.warning("esp32_bridge との接続が切れました: %r。%.1fs後に再接続します",
                            e, RECONNECT_DELAY_S)
                time.sleep(RECONNECT_DELAY_S)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
