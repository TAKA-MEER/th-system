"""test_pi_serial_relay_socket.py — pi_serial_relay を実サーバーで起動し、
実際のソケットで「先着 1 台」「2 本目を 409 で断る」「無通信で空く」を確かめる。

文字列検査ではなく、本物の pi_serial_relay.py のサーバーを subprocess で立て、
本物の websockets クライアントで接続する。シリアルの代わりに PTY 対を使う
（ESP32 側のエンベロープ符号化も本物の serial_framer で確かめる）。

ROS2 は要らないが websockets / pyserial が要る。
ホストの素の python3 には無いので、CLAUDE.md「環境の癖」の除外一覧に載せ、
Docker（th_ws/Dockerfile が websockets を入れる）で回す。
"""
import asyncio
import os
import select
import socket
import subprocess
import sys
import threading
import time

import pytest

pytest.importorskip("websockets")
pytest.importorskip("serial")

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__),  "..", "..", "..", "scripts"))
sys.path.insert(0, _SCRIPTS)

import serial_framer  # noqa: E402

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_FRAMER = os.path.join(_SRC, "th_esp32_bridge", "th_esp32_bridge", "serial_framer.py")

IDLE_MS = 800  # 試験を速くするため既定より短くする


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class _Relay:
    """pi_serial_relay の実サーバー（subprocess）と PTY 対。"""

    def __init__(self):
        self.master, self.slave = os.openpty()
        self.slave_name = os.ttyname(self.slave)
        self.port = _free_port()
        # RPi と同じ配置にする: 本体と serial_framer.py / relay_gate.py を
        # 同じディレクトリに並べる（serial_framer は元リポジトリの正本のベタコピー）。
        import shutil
        import tempfile
        self._stage = tempfile.mkdtemp(prefix="pi_relay_")
        for name in ("pi_serial_relay.py", "relay_gate.py"):
            shutil.copy(os.path.join(_SCRIPTS, name), self._stage)
        shutil.copy(_FRAMER, self._stage)
        self.proc = subprocess.Popen(
            [sys.executable, os.path.join(self._stage, "pi_serial_relay.py"),
             "--serial-port", self.slave_name,
             "--ws-port", str(self.port),
             "--idle-timeout-ms", str(IDLE_MS)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self._decoder = serial_framer.SerialFrameDecoder()
        self._rx_on_serial: list[bytes] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_serial, daemon=True)
        self._reader.start()
        self._wait_listening()

    def _wait_listening(self):
        deadline = time.time() + 15
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise AssertionError(
                    f"pi_serial_relay が起動直後に終了した:\n{self.proc.stdout.read() if self.proc.stdout else ''}")
            try:
                with socket.create_connection(("127.0.0.1", self.port), timeout=0.3):
                    return
            except OSError:
                time.sleep(0.1)
        raise AssertionError("pi_serial_relay が待ち受けを開始しなかった")

    def _read_serial(self):
        while not self._stop.is_set():
            r, _, _ = select.select([self.master], [], [], 0.2)
            if not r:
                continue
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            if data:
                with self._lock:
                    self._rx_on_serial.extend(self._decoder.feed(data))

    def wait_serial_frames(self, n: int, timeout: float = 5.0) -> list[bytes]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if len(self._rx_on_serial) >= n:
                    return list(self._rx_on_serial)
            time.sleep(0.05)
        with self._lock:
            return list(self._rx_on_serial)

    def send_from_esp32(self, frame: bytes):
        os.write(self.master, serial_framer.encode_frame(frame))

    def stop(self):
        self._stop.set()
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        os.close(self.master)
        os.close(self.slave)


async def _recv_one(ws, timeout: float = 5.0):
    return await asyncio.wait_for(ws.recv(), timeout=timeout)


def test_second_connection_is_refused_with_owner_and_released_on_idle():
    """1 本目が制御中は 2 本目を 409 で断り、無通信で空いたら 2 本目が入れる。"""
    import websockets

    relay = _Relay()
    try:
        async def main():
            uri = f"ws://127.0.0.1:{relay.port}"
            # 1 本目が接続してフレームを送る（制御 PC になる）
            async with websockets.connect(uri) as ws1:
                await ws1.send(b"cmd-1")
                # シリアルへエンベロープ付きで届くこと（向きが本物であることの証明）
                frames = relay.wait_serial_frames(1)
                assert frames and frames[0] == b"cmd-1", frames

                # 2 本目は 409 で断られる。制御中の PC のアドレスが
                # 見出しか本文のどちらかに入っていること
                with pytest.raises(Exception) as ei:
                    await websockets.connect(uri)
                resp = getattr(ei.value, "response", None)
                assert resp is not None, f"握手の応答が無い: {ei.value!r}"
                status = getattr(resp, "status_code", None) or getattr(resp, "status", None)
                assert status == 409, f"状態 {status} (期待 409)"
                headers = getattr(resp, "headers", {}) or {}
                body = getattr(resp, "body", b"") or b""
                text_body = getattr(resp, "text_body", None) or getattr(resp, "text", None)
                owner_in_header = "127.0.0.1" in str(headers)
                owner_in_body = ("127.0.0.1" in body.decode("utf-8", "ignore")
                                 or (text_body is not None and "127.0.0.1" in str(text_body)))
                assert owner_in_header or owner_in_body, (
                    f"所有者のアドレスが応答に無い headers={headers!r} body={body!r}")

                # ESP32 側からフレームを送ると ws1 に届く（受信方向も本物）
                relay.send_from_esp32(b"fb-1")
                got = await _recv_one(ws1)
                assert got == b"fb-1", got

                # ws1 は送り続けるのを止める（閉じない＝半開きの再現）。
                # idle 時間を過ぎるとサーバー側が閉じて空けること。
                closed_by_server = False
                deadline_idle = time.time() + (IDLE_MS / 1000.0 + 4.0)
                while time.time() < deadline_idle:
                    try:
                        await asyncio.wait_for(ws1.recv(), timeout=0.3)
                    except (asyncio.TimeoutError, Exception):
                        # TimeoutError はまだ生きている。ConnectionClosed は空いた。
                        import websockets.exceptions as _e  # noqa: F401
                        # サーバーが close したかは次の受信で判る。少し待つ
                        pass
                    # サーバーが close したら recv が ConnectionClosed を投げる
                    # （上の except では TimeoutError と識別できないため別経路で見る）
                    if ws1.close_code is not None:
                        closed_by_server = True
                        break
                    await asyncio.sleep(0.1)
                assert closed_by_server, "無通信でもサーバーが接続を閉じなかった"
                deadline = time.time() + (IDLE_MS / 1000.0 + 4.0)
                last_err = None
                while time.time() < deadline:
                    try:
                        async with websockets.connect(uri) as ws2:
                            await ws2.send(b"cmd-2")
                            frames2 = relay.wait_serial_frames(2)
                            assert frames2[-1] == b"cmd-2", frames2
                            return
                    except Exception as e:  # noqa: BLE001 — 空くまでリトライ
                        last_err = e
                        await asyncio.sleep(0.2)
                raise AssertionError(f"無通信のあと 2 本目が入れなかった: {last_err!r}")

        asyncio.run(main())
    finally:
        relay.stop()
