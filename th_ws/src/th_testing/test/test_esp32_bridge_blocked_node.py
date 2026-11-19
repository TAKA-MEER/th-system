"""
test_esp32_bridge_blocked_node.py
============================
Spec-safety.md §6.4「断られた PC の表示」: ラズパイの中継が「別の PC が制御中」
で 409 拒否したとき、esp32_bridge（クライアント）が /esp32/bridge_blocked へ
制御中の PC のアドレスとともに「別の PC が制御中」を出すこと。

偽の中継サーバー（pi_serial_relay の代わり）は握手で必ず 409 を返す。
本物の esp32_bridge ノードを launch_testing で起動して確かめる
（実際のソケットを使い、文字列検査はしない）。
"""
import json
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy

_WS_PORT = 18768
_OWNER = "192.168.5.51"


@pytest.mark.launch_test
def generate_test_description():
    esp32_bridge = launch_ros.actions.Node(
        package='th_esp32_bridge',
        executable='esp32_bridge.py',
        name='esp32_bridge',
        parameters=[{
            'esp32_ws_host': '127.0.0.1',
            'esp32_ws_port': _WS_PORT,
            'feedback_timeout_ms': 60000,
        }],
        output='screen',
    )
    test = launch_testing.actions.ReadyToTest()
    return launch.LaunchDescription([esp32_bridge, test]), {'esp32_bridge': esp32_bridge}


class _RejectingRelay:
    """握手を必ず 409 で断る偽の中継。X-Controlling-PC に所有者を入れる。"""

    def __init__(self, port: int, owner: str):
        self._port = port
        self._owner = owner
        self._loop = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        import asyncio
        import websockets
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        def _process_request(*args):
            from websockets.http11 import Response
            body = f"another PC ({self._owner}) is already controlling this robot"
            try:
                from websockets.datastructures import Headers
                headers = Headers([("X-Controlling-PC", self._owner)])
            except Exception:  # noqa: BLE001
                headers = {"X-Controlling-PC": self._owner}
            try:
                return Response(409, "Conflict", headers, body.encode())
            except TypeError:
                return Response(409, "Conflict", headers, text_body=body)

        async def _serve():
            async with websockets.serve(lambda ws, *a: asyncio.Future(),
                                         '127.0.0.1', self._port,
                                         process_request=_process_request):
                await asyncio.Future()

        try:
            self._loop.run_until_complete(_serve())
        except Exception:
            pass


class TestEsp32BridgeBlocked(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_blocked_topic_carries_owner(self):
        from std_msgs.msg import String

        relay = _RejectingRelay(_WS_PORT, _OWNER)
        relay.start()

        node = rclpy.create_node('test_esp32_bridge_blocked')
        got = []
        sub = node.create_subscription(String, '/esp32/bridge_blocked',
                                       lambda m: got.append(m.data), 10)
        try:
            deadline = time.time() + 20.0
            while time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if got:
                    obj = json.loads(got[-1])
                    if obj.get("blocked") is True and obj.get("owner") == _OWNER:
                        return
            self.fail(f'/esp32/bridge_blocked へ blocked=true/owner={_OWNER} が '
                      f'20 秒以内に届かなかった。受信: {got!r}')
        finally:
            node.destroy_subscription(sub)
            node.destroy_node()
