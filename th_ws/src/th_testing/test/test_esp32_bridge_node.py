"""
test_esp32_bridge_node.py
============================
WP-SAFE-02 の統合テスト。esp32_bridge ノードを実際に起動し、ESP32 の代わりに
WS クライアント (esp32_bridge がサーバー側であることに注意) を接続して、
/cmd_vel が途絶したときに WHEEL_CMD フレームの参照値がゼロへ書き換わること
(DEBT-4 の解除) と、その間も 20Hz の送信自体は止まらないこと (K-1) を検証する。

DetailedDesign-safety.md §10 の故障注入 12「/cmd_vel の途絶」に対応する。
満たす仕様: docs/plan/detailed/DetailedDesign-wp2.md WP-SAFE-02 §4.3 (K-1)。
"""
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# conftest.py が th_esp32_bridge/th_esp32_bridge (ws_protocol.py の置き場所) を
# sys.path に追加済み。
from ws_protocol import WHEEL_CMD, peek_type, unpack_wheel_cmd

# 実機・他テストの esp32_bridge (既定 8766) と衝突しないポートを使う。
_WS_PORT = 18765
_STALE_MS = 150


@pytest.mark.launch_test
def generate_test_description():
    esp32_bridge = launch_ros.actions.Node(
        package='th_esp32_bridge',
        executable='esp32_bridge.py',
        name='esp32_bridge',
        parameters=[{
            'ws_host': '127.0.0.1',
            'ws_port': _WS_PORT,
            'cmd_vel_stale_ms': _STALE_MS,
            # フィードバック死活監視のタイムアウト警告でログが汚れないよう緩める
            # (WHEEL_FEEDBACK を送るモックではないため常に鳴るが、テストの
            # 合否には関与しない)。
            'feedback_timeout_ms': 60000,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        esp32_bridge,
        launch_testing.actions.ReadyToTest(),
    ]), {'esp32_bridge': esp32_bridge}


class _WsMockEsp32:
    """esp32_bridge の WS サーバーへ ESP32 の代わりに接続し、受信した
    WHEEL_CMD フレームを記録するモッククライアント。

    接続は別スレッドの asyncio イベントループで非同期に確立する
    (esp32_bridge 自身の実装と同じ構造)。呼び出し側は wait_connected() で
    接続完了を明示的に待ってから publish を始めること — 待たずに
    publish を始めると、接続がまだ確立していない間のフレームを
    取りこぼして偽陰性になる (レース条件)。
    """

    def __init__(self, uri: str):
        self._uri = uri
        self.frames: "list[tuple[float, float, float]]" = []  # (recv_time, left, right)
        self._lock = threading.Lock()
        self._loop = None
        self._ws = None
        self._connected = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def wait_connected(self, timeout: float):
        """接続確立を待つバリア。タイムアウトすれば理由が分かる形で失敗させる。"""
        if not self._connected.wait(timeout):
            raise AssertionError(
                f'ESP32 モッククライアントが {timeout}s 以内に esp32_bridge の '
                f'WS サーバー ({self._uri}) へ接続できなかった')

    def stop(self):
        """接続を明示的に閉じる。次のテストの接続とすれ違わせないため
        tearDown で必ず呼ぶ (呼ばずに放置すると、ノード終了時に
        ConnectionClosedError が別スレッドで飛んでログが汚れる)。"""
        if self._loop is not None and self._ws is not None:
            import asyncio

            async def _close():
                await self._ws.close()
            try:
                fut = asyncio.run_coroutine_threadsafe(_close(), self._loop)
                fut.result(timeout=2.0)
            except Exception:
                pass  # 既に切断済み等 — テスト後始末なので握りつぶしてよい

    def _run(self):
        import asyncio
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._connect_and_recv())
        except Exception:
            pass  # バックグラウンドスレッドの後始末。テストの合否は wait_connected() 等で判定する

    async def _connect_and_recv(self):
        import asyncio
        import websockets

        deadline = asyncio.get_event_loop().time() + 15.0
        while asyncio.get_event_loop().time() < deadline:
            try:
                async with websockets.connect(self._uri) as ws:
                    self._ws = ws
                    self._connected.set()
                    try:
                        async for message in ws:
                            if not isinstance(message, (bytes, bytearray)):
                                continue
                            data = bytes(message)
                            if peek_type(data) == WHEEL_CMD:
                                left, right = unpack_wheel_cmd(data)
                                with self._lock:
                                    self.frames.append((time.time(), left, right))
                    except websockets.exceptions.ConnectionClosed:
                        # stop() での明示的な close、またはノード終了時の
                        # 突然の切断。テスト後始末の一部として想定内。
                        pass
                return
            except (OSError, ConnectionRefusedError):
                await asyncio.sleep(0.3)

    def snapshot(self):
        with self._lock:
            return list(self.frames)

    def clear(self):
        with self._lock:
            self.frames.clear()


class TestEsp32BridgeCmdVelStaleTimeout(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_esp32_bridge_stale')
        self.pub_cmd_vel = self.node.create_publisher(Twist, '/cmd_vel', 10)
        # esp32_bridge の独立ロック層 (K-2) は /safety/estop・/safety/fault_lock を
        # 一度も受信していない間 (このテストでは safety_monitor が居ない) を
        # フェイルセーフで「ロック中」として扱う。ここで false を送り続けない
        # と常時ロック扱いになり、/cmd_vel の内容によらず常にゼロになって
        # しまい、stale タイムアウトの検証にならない (test_twist_mux_priority.py
        # と同じ手法で safety_monitor の代わりを務める)。
        self.pub_estop = self.node.create_publisher(Bool, '/safety/estop', 10)
        self.pub_fault_lock = self.node.create_publisher(Bool, '/safety/fault_lock', 10)

        self.client = _WsMockEsp32(f'ws://127.0.0.1:{_WS_PORT}')
        self.client.start()
        # ① 接続確立そのものを待つ (publish を接続前に始めるとフレームを
        #    取りこぼして偽陰性になる)。
        self.client.wait_connected(15.0)
        # ② ロック解除状態が伝わり、独立ロック層が解除されるまで待つ
        #    (lock_stale_ms=0.5s より十分長く)。接続直後はキープアライブの
        #    次周期 (最大 50ms) まで何も来ないので、最初のフレームが届く
        #    のもここで併せて待つ。
        self._spin(0.6)
        self._wait_for_frames(min_count=1, timeout=2.0)
        self.client.clear()

    def tearDown(self):
        self.client.stop()
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, sec: float):
        deadline = time.time() + sec
        while time.time() < deadline:
            # safety_monitor 相当: ロック解除状態を送り続ける (上の setUp 参照)。
            self.pub_estop.publish(Bool(data=False))
            self.pub_fault_lock.publish(Bool(data=False))
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _wait_for_frames(self, min_count: int, timeout: float):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if len(self.client.snapshot()) >= min_count:
                return
            self._spin(0.05)
        self.fail(f'{min_count} 件のフレームを {timeout}s 以内に受信できなかった'
                   ' (esp32_bridge の WS サーバーに接続できていない可能性)')

    # ════════════════════════════════════════════════════════
    def test_still_publishes_at_20hz_when_stale(self):
        """K-1: /cmd_vel が途絶しても 20Hz の送信そのものは止めない（沈黙しない）"""
        cmd = Twist()
        cmd.linear.x = 0.2
        for _ in range(5):
            self.pub_cmd_vel.publish(cmd)
            self._spin(0.05)

        # /cmd_vel の発行を止めて cmd_vel_stale_ms を十分に超えて待つ。
        self.client.clear()
        self._wait_for_frames(min_count=1, timeout=2.0)
        count_before = len(self.client.snapshot())

        self._spin((_STALE_MS + 300) / 1000.0)
        count_after = len(self.client.snapshot())

        self.assertGreater(
            count_after, count_before,
            'stale 判定後もキープアライブが 20Hz で送信され続けていること (K-1)。'
            f'stale 前 {count_before} 件 → 待機後 {count_after} 件')

    def test_stale_zeroes_wheel_cmd(self):
        """DEBT-4: /cmd_vel が cmd_vel_stale_ms 途絶したら WHEEL_CMD の参照値がゼロになる"""
        cmd = Twist()
        cmd.linear.x = 0.2
        for _ in range(5):
            self.pub_cmd_vel.publish(cmd)
            self._spin(0.05)

        frames = self.client.snapshot()
        self.assertTrue(
            any(abs(left) > 0.01 or abs(right) > 0.01 for _, left, right in frames),
            '前提: 非ゼロ指令が一度も送信されなかった')

        # /cmd_vel の発行を止めて stale_ms を超えるまで待つ。
        self.client.clear()
        self._spin((_STALE_MS + 300) / 1000.0)

        frames = self.client.snapshot()
        self.assertTrue(len(frames) > 0, 'stale 後もフレームが届いていること (K-1 の前提)')
        _, last_left, last_right = frames[-1]
        self.assertAlmostEqual(last_left, 0.0, places=3,
                                msg=f'stale 後の WHEEL_CMD.left が非ゼロ: {last_left}')
        self.assertAlmostEqual(last_right, 0.0, places=3,
                                msg=f'stale 後の WHEEL_CMD.right が非ゼロ: {last_right}')

    def test_fresh_cmd_vel_passes_through(self):
        """途絶していない間は指令がそのまま (ゼロ化されずに) 通ること（回帰確認）"""
        cmd = Twist()
        cmd.linear.x = 0.15
        self.client.clear()
        for _ in range(8):
            self.pub_cmd_vel.publish(cmd)
            self._spin(0.05)

        frames = self.client.snapshot()
        self.assertTrue(len(frames) > 0)
        _, last_left, last_right = frames[-1]
        # 直進指令なので left == right ≈ 0.15
        self.assertGreater(last_left, 0.05)
        self.assertGreater(last_right, 0.05)
