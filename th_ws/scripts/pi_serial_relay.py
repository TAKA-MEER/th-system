#!/usr/bin/env python3
"""
pi_serial_relay.py — ラズパイ上で動く ESP32(シリアル) ⇔ esp32_bridge(WebSocket) 中継
====================================================================
2026-09-05: ESP32↔PC間の無線WebSocketを廃止し、ESP32はUSB-UARTでラズパイに
直結、ラズパイ上のこのプロセスがPC側の esp32_bridge(WebSocketサーバー、
8766番、無変更)へクライアントとして接続する構成に変更した(Spec.md §6.1
「ESP32の無線化をやめ、ラズパイ経由のシリアル接続にする」参照)。

2026-11-19 (Spec-safety.md §6.4): 接続の向きを逆にした。このプロセスが
WebSocket **サーバー**として待ち受け、esp32_bridge（PC）がクライアントとして
接続しに来る。理由は「決まった 1 台の PC でなくても制御できるようにする」
（PC のアドレスを決め打ちにしない）ためで、その代わりに**同時に制御する PC は
先着 1 台に限る**（relay_gate.FirstComeGate）。

役割:
  ESP32 --USB/UART(エンベロープ付き)--> [このプロセス] --WS server--> PC:8766 esp32_bridge

このプロセス自身は ws_protocol フレームの中身(WHEEL_CMD/WHEEL_FEEDBACK/...)を
一切解釈しない。エンベロープを剥がした/被せたバイト列をそのまま右から左へ
流すだけで、中身の意味付けは PC 側の esp32_bridge に委ねる。エンベロープの
符号化/復号だけを serial_framer.py に丸投げする。同ファイルは pure Python
(rclpy 非依存) なので、colcon ビルド無しでこのスクリプトと同じディレクトリに
置くだけで動く(th_ws/src/th_esp32_bridge/th_esp32_bridge/serial_framer.py の
コピー。中身は変更しないこと。配置方法は docs/network.md 参照)。
「先着 1 台」の判定は relay_gate.py（一緒に置く。pure Python）。

デプロイ上の重要な注意 (docs/network.md にも記載):
  - シリアルポートは必ず /dev/serial/by-id/... で指定すること。RPLIDAR も同じ
    ラズパイの USB-UART なので、/dev/ttyUSB0 のような列挙順に依存した指定は
    ある日 LiDAR と ESP32 を取り違える(spec/ 参照)。
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
import relay_gate              # noqa: E402  (同ディレクトリに配置する想定)
import serial_framer          # noqa: E402  (同ディレクトリに配置する想定)

LOG = logging.getLogger("pi_serial_relay")

# 無通信で経路を手放すまでの既定値（ms）。Spec-safety.md §6.4「手放すまでの時間は
# 短くする」。PC 側（esp32_bridge）は止まっていても 20Hz で送り続けるので、
# 1.5 秒分の無通信は PC か通信路が死んだときだけに起こる。ESP32 のウォッチドッグ
# （600 ms）がこの間、駆動を止め続ける。短すぎると Wi-Fi のジッタで誤って手放し、
# 長すぎると死んだ PC の後任が待たされるため 1.5 s にする。
# registry.yaml の esp32_relay_idle_timeout_ms と同じ値（ラズパイ側は CLI の
# --idle-timeout-ms が正。ユニットの ExecStart は既定のまま）。
DEFAULT_IDLE_TIMEOUT_MS = 1500


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


async def ws_to_serial(ws, ser: serial.Serial, gate: relay_gate.FirstComeGate,
                        loop: asyncio.AbstractEventLoop) -> None:
    """PCから届いたフレーム(WHEEL_CMD等)をエンベロープに包んでシリアルへ書く。

    届くたびにゲートへ記録する（無通信の手放し判定の入力）。
    WSが正常にcloseされて `async for` が抜けたときも例外を投げて呼び出し元の
    gatherを終わらせる。そうしないと(ESP32が繋がっていない等で)serial_to_ws
    側が何も送信せず、gatherが永久に片肺のまま止まる。
    """
    async for message in ws:
        if not isinstance(message, (bytes, bytearray)):
            continue
        gate.note_frame(time.monotonic())
        envelope = serial_framer.encode_frame(bytes(message))
        await loop.run_in_executor(None, ser.write, envelope)
    raise RuntimeError("WS接続が閉じられました(ws_to_serial側で検知)")


async def _idle_watchdog(ws, gate: relay_gate.FirstComeGate, timeout_s: float) -> None:
    """無通信が timeout_s を超えたら接続を閉じる（ゲートが空く）。

    半開きの TCP（Wi-Fi で FIN が届かない。esp32_bridge.py の 2026-09-02 の注記）
    では ws_to_serial の `async for` が永久に復帰しないため、TCP の切断検知に
    頼らず「受信時刻」でこちらから閉じる。
    """
    while True:
        await asyncio.sleep(0.1)
        if gate.check_release_if_idle(time.monotonic(), timeout_s):
            LOG.warning("制御 PC から %.1fs フレームが届かないため経路を手放します",
                        timeout_s)
            try:
                await ws.close()
            except Exception:  # noqa: BLE001 — close 失敗は次の周で再試行
                pass
            return


def _peer_address(ws) -> str:
    """接続相手の表示用アドレス（IP のみ。ポートは変動するので出さない）。"""
    remote = getattr(ws, "remote_address", None)
    if remote:
        return str(remote[0])
    return "unknown"


def _conflict_response(owner: str):
    """先着の制御 PC が居るときの 409 応答（握手の HTTP エラー）。

    websockets の版で Response の組み立て方が違う:
      - 新しい asyncio 実装 (>=13): Response(status_code, reason_phrase, headers, body)
      - 古い legacy 実装 (<=12):  Response(status, reason, headers, text_body=...)
    ラズパイの版は未固定（wheel の取得時に未指定のため最新が入る見込みだが、
    入れ替わっても動くよう両対応にし、どちらが効いたかはテストで確かめる）。
    本文と X-Controlling-PC 見出しの両方に制御中の PC のアドレスを入れる。
    """
    from websockets.http11 import Response

    body_text = f"another PC ({owner}) is already controlling this robot"
    try:
        from websockets.datastructures import Headers
        headers = Headers([("X-Controlling-PC", owner),
                           ("Content-Type", "text/plain; charset=utf-8")])
    except Exception:  # noqa: BLE001
        headers = {"X-Controlling-PC": owner,
                   "Content-Type": "text/plain; charset=utf-8"}
    try:
        return Response(409, "Conflict", headers, body_text.encode("utf-8"))
    except TypeError:
        return Response(409, "Conflict", headers, text_body=body_text)


async def serve_once(ser: serial.Serial, bind_host: str, port: int,
                     timeout_s: float) -> None:
    """サーバーを 1 回起動する（通常は起動しっぱなし）。

    ラズパイの古い websockets でも動くよう、ハンドラは (ws, *args) で受け、
    process_request は引数を可変で受ける（版で渡される引数が違う）。
    """
    gate = relay_gate.FirstComeGate()
    decoder = serial_framer.SerialFrameDecoder()
    loop = asyncio.get_running_loop()

    def _process_request(*args) -> object | None:
        owner = gate.owner
        if owner is not None:
            LOG.info("制御中の PC (%s) が居るため接続を断ります", owner)
            return _conflict_response(owner)
        return None

    async def _handler(ws, *args) -> None:
        owner = _peer_address(ws)
        if not gate.try_acquire(owner, time.monotonic()):
            # process_request をすり抜けた競合（稀）。静かに閉じる。
            LOG.warn("競合のため接続を断ります: %s", owner)
            await ws.close(code=1008, reason="another PC is controlling")
            return
        # WS が切れていた間もシリアルポート自体は開いたまま(DTR/RTSを動かさない
        # ため)なので、カーネルのtty受信バッファに数秒分のフレームが溜まって
        # いることがある。再接続直後にそれをそのまま流すと、古いWHEEL_FEEDBACK
        # が一気に届いてオドメトリが瞬間的に破綻するため、接続直後に読み捨てる。
        ser.reset_input_buffer()
        LOG.info("制御 PC が接続しました: %s", owner)
        tasks = [
            asyncio.ensure_future(serial_to_ws(ser, ws, decoder, loop)),
            asyncio.ensure_future(ws_to_serial(ws, ser, gate, loop)),
            asyncio.ensure_future(_idle_watchdog(ws, gate, timeout_s)),
        ]
        try:
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
            for task in done:
                if not task.cancelled() and task.exception() is not None:
                    raise task.exception()
        finally:
            for task in tasks:
                task.cancel()
            gate.release()
            LOG.info("制御 PC の接続が終わりました: %s", owner)

    LOG.info("WebSocket サーバーを待ち受けます: %s:%d（先着 1 台）", bind_host, port)
    async with websockets.serve(_handler, bind_host, port,
                                 process_request=_process_request):
        # 正常に抜けることは無い（サーバーは起動しっぱなし）。シリアル断は
        # serial_to_ws の executor 内で起き、その例外はこの serve の外側で
        # 捕まえて終了する（systemd が再起動する）。
        await asyncio.Future()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial-port", required=True,
                         help="/dev/serial/by-id/... を指定すること(/dev/ttyUSB* は不可)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--ws-host", default="0.0.0.0",
                         help="WebSocket サーバーの bind アドレス（既定 0.0.0.0＝全I/F）。"
                              "接続しに来るのは PC 側 esp32_bridge（--ws-host は接続先ではなく待ち受け側）")
    parser.add_argument("--ws-port", type=int, default=8766)
    parser.add_argument("--idle-timeout-ms", type=int, default=DEFAULT_IDLE_TIMEOUT_MS,
                         help="制御 PC からこの間フレームが届かなかったら経路を手放す（既定 1500）")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                         format="%(asctime)s [%(levelname)s] %(message)s")

    if "by-id" not in args.serial_port and "by-path" not in args.serial_port:
        LOG.warning("serial-port が /dev/serial/by-id ではありません (%s)。"
                     "USB列挙順が変わるとLiDARと衝突する恐れがあります。",
                     args.serial_port)

    ser = open_serial(args.serial_port, args.baud)
    try:
        asyncio.run(serve_once(ser, args.ws_host, args.ws_port,
                               args.idle_timeout_ms / 1000.0))
    except serial.SerialException as e:
        # ser.read()/ser.write() が投げるのは「ポート自体が死んだ」場合
        # (ESP32抜線・USB切断等)。ポートの再オープンはこのプロセスの責務にせず、
        # systemdの Restart=always に委ねて素直に終了する。
        LOG.error("シリアルポートが切れました: %r。終了します(systemdが再起動)", e)
        sys.exit(1)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
