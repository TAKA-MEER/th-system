"""
test_esp32_scale.py
====================
WP-ESP32-02 (§4.2〜4.3) の統合テスト。esp32_bridge ノードを実際に起動し、
ESP32 の代わりに WS クライアント（esp32_bridge がサーバー側であることに注意）を
接続して、wheel_radius_scale k の両方向適用と A10 の強制を検証する。

満たす仕様: docs/plan/detailed/DetailedDesign-maintenance.md §4.2 (O-d10)・§4.3 (A10)
  | 方向 | 変換 |
  | --- | --- |
  | 指令（PC → ESP32） | v_send = v_desired / k |
  | 実測（ESP32 → PC） | v_actual = v_report × k |
  A10: |wheel_radius_scale − 1| ≤ wheel_radius_scale_max_dev。超えたら起動を拒否。

お手本: test_esp32_bridge_node.py（WS モックの流儀・_spin・ロック解除パブリッシュ）。
加えて偽 ESP32 が WHEEL_FEEDBACK を送り返せるようにし、往復で元の速度に
戻ることを /esp32/wheel_feedback と /odom の両方で確認する（設計の完了条件）。

トピックは /esp32_scale/* へ remap して、他テスト（同 /cmd_vel 等を使う
esp32_bridge 系）との混線を避ける。A10 違反（k=1.2）のノードを同一
LaunchDescription に載せ、起動直後に exit code 1 で終わることを
proc_info / assertExitCodes で確認する。
"""
import asyncio
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertExitCodes
import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Bool
from th_system_msgs.msg import WheelFeedback

# conftest.py が th_esp32_bridge/th_esp32_bridge (ws_protocol.py の置き場所) を
# sys.path に追加済み。
from ws_protocol import (WHEEL_CMD, pack_wheel_feedback, peek_type,
                         unpack_wheel_cmd)

# 実機・他テストの esp32_bridge (既定 8766) と、test_esp32_bridge_node.py
# (18765) と衝突しないポートを使う。
_WS_PORT_GOOD = 18766
_WS_PORT_BAD = 18767

# 良いノードのスケール（1.0 以外）。|1.05 − 1| = 0.05 ≤ 0.10 なので A10 合格。
_GOOD_SCALE = 1.05
# A10 違反。|1.2 − 1| = 0.2 > 0.10。起動を拒否されるべき値。
_BAD_SCALE = 1.2
# パラメータは float32 で WS を往復し、ノード側も float64 で変換している。
# 誤差は < 1e-6 なので 1e-3 の許容で十分（将来のパック精度変更に揺れない）。
_TOL = 1e-3
_DESIRED = 0.3

# 指令・実測の往復を「偽 ESP32 が受け取った値」そのままで回すための待ち。
# proc_output の A10 ログも substring で確認する。


def _has_a10_startup(output: str) -> bool:
    return 'A10 違反により起動を中断します' in output


@pytest.mark.launch_test
def generate_test_description():
    good_node = launch_ros.actions.Node(
        package='th_esp32_bridge',
        executable='esp32_bridge.py',
        name='esp32_bridge_scale',
        parameters=[{
            'ws_host': '127.0.0.1',
            'ws_port': _WS_PORT_GOOD,
            'wheel_radius_scale': _GOOD_SCALE,
            'wheel_radius_scale_max_dev': 0.10,
            # フィードバック死活監視のタイムアウト警告でログが汚れないよう緩める。
            'feedback_timeout_ms': 60000,
        }],
        # 他テスト (test_esp32_bridge_node.py 等) と /cmd_vel・/esp32/* を
        # 共有しないよう、このノードのやり取り先を /esp32_scale/* へ remap する
        # (colcon test の並列実行で他テストの指令・実測が混ざるのを防ぐ)。
        remappings=[
            ('/cmd_vel', '/esp32_scale/cmd_vel'),
            ('/safety/estop', '/esp32_scale/safety/estop'),
            ('/safety/fault_lock', '/esp32_scale/safety/fault_lock'),
            ('/esp32/wheel_feedback', '/esp32_scale/wheel_feedback'),
            ('/esp32/wheel_cmd_speed', '/esp32_scale/wheel_cmd_speed'),
            ('/odom', '/esp32_scale/odom'),
        ],
        output='screen',
    )
    # A10 違反の値で起動した場合、esp32_bridge は早期にその旨をログして
    # exit code 1 で終了する (WS サーバーを立てる前に os._exit(1))。
    # こちらは該当挙動の確認専用ノード。
    bad_node = launch_ros.actions.Node(
        package='th_esp32_bridge',
        executable='esp32_bridge.py',
        name='esp32_bridge_scale_bad',
        parameters=[{
            'ws_host': '127.0.0.1',
            'ws_port': _WS_PORT_BAD,
            'wheel_radius_scale': _BAD_SCALE,
            'wheel_radius_scale_max_dev': 0.10,
            'feedback_timeout_ms': 60000,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        good_node,
        bad_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'good': good_node, 'bad': bad_node}


class _WsScaleMockEsp32:
    """esp32_bridge の WS サーバーへ ESP32 の代わりに接続し、WHEEL_CMD を
    受信して記録する。加えて WHEEL_FEEDBACK を送り返せる点が
    test_esp32_bridge_node.py のモックと異なる (両方向変換の往復検証用)。

    接続は別スレッドの asyncio イベントループで非同期に確立する。呼び出し側は
    wait_connected() で接続完了を明示的に待ってから使い始めること。
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
        if not self._connected.wait(timeout):
            raise AssertionError(
                f'ESP32 モッククライアントが {timeout}s 以内に esp32_bridge の '
                f'WS サーバー ({self._uri}) へ接続できなかった')

    def stop(self):
        """接続を明示的に閉じる。次のテストの接続とすれ違わせないため
        tearDown で必ず呼ぶ。"""
        if self._loop is not None and self._ws is not None:
            async def _close():
                await self._ws.close()
            try:
                fut = asyncio.run_coroutine_threadsafe(_close(), self._loop)
                fut.result(timeout=2.0)
            except Exception:
                pass  # 既に切断済み等 — テスト後始末なので握りつぶしてよい

    def send_feedback(self, left: float, right: float, dt: float = 0.1):
        """受信した指令値をそのまま実測として送り返す (往復検証用)。"""
        if self._loop is None or self._ws is None:
            raise AssertionError('ESP32 モックが未接続のため実測を送れません')
        frame = pack_wheel_feedback(left, right, dt)

        async def _send():
            try:
                await self._ws.send(frame)
            except Exception:
                pass  # 切断済み等 — 合否は呼び出し側の受信待ちで判定する

        fut = asyncio.run_coroutine_threadsafe(_send(), self._loop)
        fut.result(timeout=2.0)

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._connect_and_recv())
        except Exception:
            pass  # バックグラウンドスレッドの後始末。合否は wait_connected() 等で判定する

    async def _connect_and_recv(self):
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


class TestWheelRadiusScale(unittest.TestCase):
    """良いノード (k=1.05) が両方向に k を適用することの振る舞い検証。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_esp32_scale')
        self.pub_cmd_vel = self.node.create_publisher(
            Twist, '/esp32_scale/cmd_vel', 10)
        # esp32_bridge の独立ロック層 (K-2) は /safety/estop・/fault_lock を
        # 一度も受信していない間をフェイルセーフで「ロック中」として扱う。
        # false を送り続けないと常にゼロになり、スケール検証にならない
        # (test_esp32_bridge_node.py と同じ手法)。
        self.pub_estop = self.node.create_publisher(
            Bool, '/esp32_scale/safety/estop', 10)
        self.pub_fault_lock = self.node.create_publisher(
            Bool, '/esp32_scale/safety/fault_lock', 10)

        self._fb: "list[WheelFeedback]" = []
        self._cmd_fb: "list[WheelFeedback]" = []
        self._odom: "list[Odometry]" = []
        self.node.create_subscription(
            WheelFeedback, '/esp32_scale/wheel_feedback', self._fb.append, 10)
        self.node.create_subscription(
            WheelFeedback, '/esp32_scale/wheel_cmd_speed', self._cmd_fb.append, 10)
        self.node.create_subscription(
            Odometry, '/esp32_scale/odom', self._odom.append, 10)

        # 実行中の set_parameters 検査用 (ros2 param set 相当)。
        self._set_cli = self.node.create_client(
            SetParameters, '/esp32_bridge_scale/set_parameters')
        self._get_cli = self.node.create_client(
            GetParameters, '/esp32_bridge_scale/get_parameters')

        self.client = _WsScaleMockEsp32(f'ws://127.0.0.1:{_WS_PORT_GOOD}')
        self.client.start()
        self.client.wait_connected(15.0)
        self._cmd_x = 0.0
        # ① ロック解除状態が伝わり、独立ロック層が解除されるまで待つ
        #    (lock_stale_ms=0.5s より十分長く)。
        self._spin(0.6)
        self._wait_cmd_value(0.0, timeout=2.0)  # ゼロ指令の初回フレームを貰う
        self.client.clear()

    def tearDown(self):
        self.client.stop()
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, sec: float, cmd_x=None):
        """cmd_x を指定すれば毎周期 /cmd_vel に (キープアライブの stale 化防止)、
        ロック解除状態を常に送る (safety_monitor 相当)。"""
        if cmd_x is not None:
            self._cmd_x = cmd_x
        deadline = time.time() + sec
        while time.time() < deadline:
            cmd = Twist()
            cmd.linear.x = self._cmd_x
            self.pub_cmd_vel.publish(cmd)
            self.pub_estop.publish(Bool(data=False))
            self.pub_fault_lock.publish(Bool(data=False))
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _wait_cmd_value(self, expected_sent: float, timeout: float):
        """偽 ESP32 が受信した直近の WHEEL_CMD が expected_sent±_TOL になるまで
        待ち、その (left, right) を返す。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            frames = self.client.snapshot()
            if frames:
                _, last_left, last_right = frames[-1]
                if abs(last_left - expected_sent) < _TOL \
                        and abs(last_right - expected_sent) < _TOL:
                    return last_left, last_right
            self._spin(0.05, cmd_x=_DESIRED if self._cmd_x != 0.0 else 0.0)
        self.fail(
            f'偽 ESP32 が受信した WHEEL_CMD が {expected_sent:_}±{_TOL} にならない。'
            f'直近: {frames[-1] if frames else "無し"} (ws={_WS_PORT_GOOD})')

    def _call_service(self, cli, req, timeout: float = 5.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            self.fail(f'service {cli.srv_type.__name__} が利用できない')
        deadline = time.time() + timeout
        while time.time() < deadline:
            future = cli.call_async(req)
            while not future.done():
                if time.time() > deadline:
                    break
                rclpy.spin_once(self.node, timeout_sec=0.02)
            if future.done() and future.result() is not None:
                return future.result()
        self.fail(f'service {cli.srv_type.__name__} 呼び出しが {timeout}s 以内に完了しない')

    def _set_param(self, name: str, type_: int, value):
        req = SetParameters.Request()
        p = Parameter()
        p.name = name
        p.value = ParameterValue(type=type_)
        if type_ == ParameterType.PARAMETER_DOUBLE:
            p.value.double_value = float(value)
        elif type_ == ParameterType.PARAMETER_STRING:
            p.value.string_value = str(value)
        req.parameters = [p]
        res = self._call_service(self._set_cli, req)
        return bool(res.results[0].successful)

    def _get_param_double(self, name: str) -> float:
        req = GetParameters.Request()
        req.names = [name]
        res = self._call_service(self._get_cli, req)
        return float(res.values[0].double_value)

    def _clear_buffers(self):
        self._fb.clear()
        self._cmd_fb.clear()
        self._odom.clear()

    def _wait_until(self, desc, predicate, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return
            self._spin(0.05)
        self.fail(f'{desc}: {timeout}s 以内に条件を満たさなかった')

    # ════════════════════════════════════════════════════════
    def test_a_cmd_is_divided_by_k(self):
        """指令側: /cmd_vel 0.3 を送ると、偽 ESP32 が受ける WHEEL_CMD は
        0.3 / 1.05 になっている。/esp32/wheel_cmd_speed は変換前 (0.3) のまま。"""
        self.client.clear()
        self._clear_buffers()
        self._spin(0.3, cmd_x=_DESIRED)

        sent_left, sent_right = self._wait_cmd_value(
            _DESIRED / _GOOD_SCALE, timeout=3.0)
        self.assertAlmostEqual(sent_left, _DESIRED / _GOOD_SCALE, delta=_TOL)
        self.assertAlmostEqual(sent_right, _DESIRED / _GOOD_SCALE, delta=_TOL)

        self._wait_until(
            '/esp32/wheel_cmd_speed が変換前の値 (0.3) を出す',
            lambda: any(abs(m.left_speed - _DESIRED) < _TOL for m in self._cmd_fb),
            timeout=3.0)

    def test_b_feedback_roundtrip_restores_original(self):
        """実測側: 偽 ESP32 が受けた値 (0.3/1.05) をそのまま WHEEL_FEEDBACK で
        送り返すと、/esp32/wheel_feedback と /odom の速度が ×1.05 で元の 0.3 に
        戻る (設計の完了条件「往復で元に戻る」。DRIVE_RUNAWAY 用に両方真の単位)。"""
        self.client.clear()
        self._clear_buffers()
        self._spin(0.3, cmd_x=_DESIRED)
        sent_left, sent_right = self._wait_cmd_value(
            _DESIRED / _GOOD_SCALE, timeout=3.0)

        self._clear_buffers()
        self.client.send_feedback(sent_left, sent_right, dt=0.1)

        self._wait_until(
            '/esp32/wheel_feedback が 0.3 に戻る',
            lambda: any(abs(m.left_speed - _DESIRED) < _TOL for m in self._fb),
            timeout=3.0)
        self._wait_until(
            '/odom の速度が 0.3 に戻る',
            lambda: any(abs(m.twist.twist.linear.x - _DESIRED) < _TOL for m in self._odom),
            timeout=3.0)

    def test_c_runtime_rejections_keep_current(self):
        """実行中の set_parameters: A10 違反 (1.2) と非数値 (文字列) は拒否され、
        現行値 (1.05) が維持される。"""
        self.assertTrue(
            not self._set_param(
                'wheel_radius_scale', ParameterType.PARAMETER_DOUBLE, _BAD_SCALE),
            'A10 違反 (1.2) が実行中 set_parameters で受理されてはいけない')
        self.assertTrue(
            not self._set_param(
                'wheel_radius_scale', ParameterType.PARAMETER_STRING, 'oops'),
            '非数値が実行中 set_parameters で受理されてはいけない')
        self.assertAlmostEqual(
            self._get_param_double('wheel_radius_scale'), _GOOD_SCALE, delta=_TOL,
            msg='拒否後も wheel_radius_scale が 1.05 のままであること')

    def test_d_runtime_boundary_change_applies(self):
        """境界ちょうど (1.0 ± 0.10 = 1.1) の実行中変更は受理され、以降の指令に
        新しい k が効く。/esp32/wheel_cmd_speed は変換前のまま。終わったら
        k を 1.05 に戻す (後に続く検査を汚さない)。"""
        self.assertTrue(
            self._set_param(
                'wheel_radius_scale', ParameterType.PARAMETER_DOUBLE, 1.1),
            '境界ちょうど 1.1 は実行中に受理されるべき (A10: |1.1-1|=0.10 ≤ 0.10)')
        self.client.clear()
        self._clear_buffers()
        self._spin(0.3, cmd_x=_DESIRED)
        self._wait_cmd_value(_DESIRED / 1.1, timeout=3.0)
        self._wait_until(
            '/esp32/wheel_cmd_speed が k 変更後も変換前の値 (0.3)',
            lambda: any(abs(m.left_speed - _DESIRED) < _TOL for m in self._cmd_fb),
            timeout=3.0)
        self.assertTrue(
            self._set_param(
                'wheel_radius_scale', ParameterType.PARAMETER_DOUBLE, _GOOD_SCALE),
            '戻しの k=1.05 は受理されるべき')


class TestBadScaleNodeRejectsStartup(unittest.TestCase):
    """A10 違反 (k=1.2) で起動したノードが、エラーを出して exit code 1 で
    終了すること。"""

    def test_startup_rejects_a10_violation(self, bad, proc_info, proc_output):
        # A10 違反の旨をログしてから終了していることを、出力で確認する
        # (別要因のクラッシュ (exit 1) と取り違えないため)。
        proc_output.assertWaitFor(
            'A10 違反により起動を中断します',
            process=bad,
            stream='stderr',
            output_filter=_has_a10_startup,
            timeout=30.0,
        )
        proc_info.assertWaitForShutdown(bad, timeout=30.0)
        assertExitCodes(proc_info, [1], process=bad)


if __name__ == '__main__':
    unittest.main()