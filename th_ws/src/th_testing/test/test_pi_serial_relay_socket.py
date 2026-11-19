"""test_pi_serial_relay_socket.py — pi_serial_relay を実サーバーで起動し、
実際のソケットで「先着 1 台」「2 本目を 409 で断る」「無通信で空く」を確かめる。

文字列検査ではなく、本物の pi_serial_relay.py のサーバーを subprocess で立て、
本物の websockets クライアントで接続する。シリアルの代わりに PTY 対を使う
（ESP32 側のエンベロープ符号化も本物の serial_framer で確かめる）。

レビュー指摘（2026-11-19）の回帰も縛る:
  (1) 1 台目が無通信で手放され 2 台目が取った後、1 台目の後片付けが済んでも
      3 台目が断られる（古い接続の release が新しい所有を消さない）。
  (2) 手放した 1 台目が後から送ったフレームがシリアルに書かれない。
  (3) シリアルの読み取りが例外を出したら serve が終わる（プロセス終了）。

ROS2 は要らない。websockets は要る（Docker のイメージに入っている 16.1.1。
無ければ import で落とす。importorskip で黙って飛ばさない）。
pyserial は使わない: pi_serial_relay が使う Serial / SerialException の部分は
試験内の偽 serial モジュール（同梱の _FAKE_SERIAL_SOURCE。PTY の fd で実動作する）
に差し替える。ラズパイの実物は不要で、Docker でもホストでも走る。
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

import websockets  # noqa: F401  (無ければ import で落とす。黙って飛ばさない)

_SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "scripts"))
sys.path.insert(0, _SCRIPTS)

import serial_framer  # noqa: E402

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_FRAMER = os.path.join(_SRC, "th_esp32_bridge", "th_esp32_bridge", "serial_framer.py")

IDLE_MS = 800  # 試験を速くするため既定より短くする

# pi_serial_relay が使う pyserial の部分（Serial / SerialException）の偽物。
# PTY の fd をそのまま使うので、シリアルの実データは本物として流れる。
# ラズパイの実物は不要。Docker のイメージには pyserial が無いため、
# 本物の互換実装を試験が用意する（中継と同じディレクトリに置く）。
_FAKE_SERIAL_SOURCE = '''
"""偽 serial（試験用）。PTY の fd で実動作する。"""
import fcntl
import os
import struct
import termios


class SerialException(Exception):
    pass


class Serial:
    def __init__(self):
        self.port = None
        self.baudrate = 9600
        self.timeout = None
        self.dsrdtr = False
        self.rtscts = False
        self.dtr = False
        self.rts = False
        self._fd = None

    def open(self):
        if self.port is None:
            raise SerialException("port is not set")
        self._fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY)
        try:
            raw = termios.tcgetattr(self._fd)
            raw[0] = 0  # iflag
            raw[1] = 0  # oflag
            raw[3] = 0  # lflag
            raw[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
            termios.tcsetattr(self._fd, termios.TCSANOW, raw)
        except Exception:
            pass

    @property
    def in_waiting(self):
        if self._fd is None:
            return 0
        buf = fcntl.ioctl(self._fd, 0x541B, struct.pack("i", 0))  # FIONREAD
        return struct.unpack("i", buf)[0]

    def read(self, size=1):
        if self._fd is None:
            raise SerialException("port not open")
        try:
            data = os.read(self._fd, size)
        except OSError as e:
            raise SerialException(str(e)) from e
        if not data:
            raise SerialException("device reports readiness to read but returned no data")
        return data

    def write(self, data):
        if self._fd is None:
            raise SerialException("port not open")
        try:
            return os.write(self._fd, bytes(data))
        except OSError as e:
            raise SerialException(str(e)) from e

    def reset_input_buffer(self):
        if self._fd is not None:
            try:
                fcntl.ioctl(self._fd, termios.TIOCFLUSH, termios.TCIFLUSH)
            except Exception:
                pass

    def close(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            finally:
                self._fd = None
'''


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
        with open(os.path.join(self._stage, "serial.py"), "w") as f:
            f.write(_FAKE_SERIAL_SOURCE)
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

    def serial_frames(self) -> list[bytes]:
        with self._lock:
            return list(self._rx_on_serial)

    def send_from_esp32(self, frame: bytes):
        os.write(self.master, serial_framer.encode_frame(frame))

    def kill_serial(self):
        """PTY の master を閉じてシリアルの読み取りに例外を出させる。"""
        try:
            os.close(self.master)
        except OSError:
            pass

    def stop(self):
        self._stop.set()
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


async def _recv_one(ws, timeout: float = 5.0):
    return await asyncio.wait_for(ws.recv(), timeout=timeout)


async def _assert_refused(uri: str, owner: str = "127.0.0.1"):
    """2 本目の接続が 409 で断られること（所有者のアドレスが応答に入っている）。"""
    import websockets
    with pytest.raises(Exception) as ei:
        await websockets.connect(uri)
    resp = getattr(ei.value, "response", None)
    assert resp is not None, f"握手の応答が無い: {ei.value!r}"
    status = getattr(resp, "status_code", None) or getattr(resp, "status", None)
    assert status == 409, f"状態 {status} (期待 409)"
    headers = getattr(resp, "headers", {}) or {}
    body = getattr(resp, "body", b"") or b""
    assert owner in str(headers) or owner in body.decode("utf-8", "ignore"), (
        f"所有者のアドレスが応答に無い headers={headers!r} body={body!r}")


async def _wait_connected(uri: str, timeout: float = 6.0):
    import websockets
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            return await websockets.connect(uri)
        except Exception as e:  # noqa: BLE001 — 空くまでリトライ
            last = e
            await asyncio.sleep(0.2)
    raise AssertionError(f"接続できなかった: {last!r}")


def test_second_connection_is_refused_with_owner_and_released_on_idle():
    """1 本目が制御中は 2 本目を 409 で断り、無通信で空いたら 2 本目が入れる。"""
    import websockets

    relay = _Relay()
    try:
        async def main():
            uri = f"ws://127.0.0.1:{relay.port}"
            async with websockets.connect(uri) as ws1:
                await ws1.send(b"cmd-1")
                frames = relay.wait_serial_frames(1)
                assert frames and frames[0] == b"cmd-1", frames

                await _assert_refused(uri)

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
                    except Exception:  # noqa: BLE001 — TimeoutError はまだ生きている
                        pass
                    if ws1.close_code is not None:
                        closed_by_server = True
                        break
                    await asyncio.sleep(0.1)
                assert closed_by_server, "無通信でもサーバーが接続を閉じなかった"

                async with await _wait_connected(uri) as ws2:
                    await ws2.send(b"cmd-2")
                    frames2 = relay.wait_serial_frames(2)
                    assert frames2[-1] == b"cmd-2", frames2

        asyncio.run(main())
    finally:
        relay.stop()


def test_stale_owner_cannot_evict_new_owner_and_cannot_write():
    """(1) 2 台目が取った後、1 台目の後片付けが済んでも 3 台目が断られる。
    (2) 手放した 1 台目が後から送ったフレームがシリアルに書かれない。"""
    import websockets

    relay = _Relay()
    try:
        async def main():
            uri = f"ws://127.0.0.1:{relay.port}"
            ws1 = await websockets.connect(uri)
            await ws1.send(b"cmd-1")
            assert relay.wait_serial_frames(1)[0] == b"cmd-1"

            # ws1 の送信を止める（半開き）。idle で手放され、2 台目が取る。
            ws2 = await _wait_idle_takeover(uri)
            await ws2.send(b"cmd-2")
            assert relay.wait_serial_frames(2)[-1] == b"cmd-2"

            # (2) 手放した 1 台目が後から送ってもシリアルに書かれない
            # （ws2 は所有を保つため送り続ける）
            base = len(relay.serial_frames())
            try:
                await ws1.send(b"late-frame-from-stale-owner")
            except Exception:  # noqa: BLE001 — サーバーが既に閉じていてもよい
                pass
            for _ in range(4):
                await ws2.send(b"keepalive")
                await asyncio.sleep(0.3)
            later = relay.serial_frames()[base:]
            assert b"late-frame-from-stale-owner" not in later, later

            # (1) 1 台目の後片付け（close 完了）が済んだ後も 3 台目は断られる
            await ws2.send(b"keepalive")
            await _assert_refused(uri)
            await ws2.close()
            await ws1.close()

        asyncio.run(main())
    finally:
        relay.stop()


async def _wait_idle_takeover(uri: str):
    """ws1 を送らずに放置し、idle で空くのを待って接続する（半開きの再現）。"""
    import websockets
    deadline = time.time() + IDLE_MS / 1000.0 + 5.0
    while time.time() < deadline:
        try:
            return await websockets.connect(uri)
        except Exception:  # noqa: BLE001 — 空くまでリトライ
            await asyncio.sleep(0.2)
    raise AssertionError("idle で空かなかった")


def test_serial_death_ends_the_process():
    """(3) シリアルの読み取りが例外を出したら serve が終わり、プロセスが終了する
    （systemd が立て直す。Spec-safety §3.5.6）。"""
    relay = _Relay()
    try:
        assert relay.proc.poll() is None
        relay.kill_serial()
        deadline = time.time() + 10.0
        while relay.proc.poll() is None and time.time() < deadline:
            time.sleep(0.1)
        assert relay.proc.poll() is not None, "シリアルが死んでもプロセスが終わらなかった"
        assert relay.proc.returncode != 0, relay.proc.returncode
    finally:
        relay.stop()
