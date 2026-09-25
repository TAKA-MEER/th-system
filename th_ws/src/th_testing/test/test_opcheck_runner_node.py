"""
test_opcheck_runner_node.py
============================
WP-MAINT-01 §4-2 — opcheck_runner ノードの振る舞い試験（Docker launch テスト）。

`th_maintenance/check_core.py` の純粋関数は test_opcheck_core.py が縛るが、
ここでは「本番の経路」（/system/state を見て OPCHECK のときだけ動く・
MOTOR のデッドマン・/system/event への evt.check_result 発行）が実際に
配線されているかを、ノードを実際に起動して確かめる（お手本は
test_runaway_freshness_node.py / test_localization_mode_gate_node.py。
ソースの文字列検査だけでは配線の欠落がすり抜けるため）。

state_manager は起動しない。opcheck_runner は `/system/state` を見て
mode==OPCHECK のときだけ動く「ノード側でも二重に確認」の実装なので、
テスト側が `/system/state`（TRANSIENT_LOCAL・state_manager と同じ QoS）を
直接出せば FSM 無しで全項目を操作できる（brief §3 の要件そのもの）。

検証する4点（brief §4-2）:
  a. OPCHECK 以外のモードでは /opcheck/run_item を受け付けない・
     /cmd_vel_behavior に何も出さない
  b. MOTOR: 「押している」を送っている間は /cmd_vel_behavior に v_check の
     指令が出て、送るのをやめたら opcheck_deadman_timeout_s 以内に 0 になる
  c. MOTOR: 偽の /esp32/wheel_cmd_speed・/esp32/wheel_feedback を流し、
     符号逆で evt.check_result が NG になる
  d. OPCHECK を抜けたら（/system/state を IDLE に）即座に指令が 0 になる

変異チェック（WP-MAINT-01 §4-3。実装管理担当が別途 opcheck_runner.py を
一時的に壊して実行する）:
  ① デッドマンの途絶判定を消す → test_b_motor_hold_and_deadman_release が赤
  ② OPCHECK 以外でも動くようにする → test_a_rejects_outside_opcheck が赤
  ③ judge_motor の符号判定を消す → test_c_motor_sign_mismatch_emits_ng が赤
"""
import json
import time
import unittest

import pytest
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from th_system_msgs.msg import CheckStatus, StateEvent, SystemState, WheelFeedback
from th_system_msgs.srv import RunCheck


# テストを速く安定させるための短縮値。既定値（registry.yaml）とは別に
# launch の parameters= で上書きする（test_state_manager_node.py の
# JOG_LEASE_MS と同じやり方）。
DEADMAN_S = 0.4
V_CHECK = 0.05
MOTOR_DEADBAND = 0.02
MOTOR_FOLLOW_MIN_RATIO = 0.5

# state_manager.py の state_qos と同じ（depth 1・RELIABLE・TRANSIENT_LOCAL・
# KEEP_LAST）。opcheck_runner の /system/state 購読もこれに合わせてある
# （合わせないと1件も届かない。CLAUDE.md の既知の罠）。
_STATE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


@pytest.mark.launch_test
def generate_test_description():
    opcheck_runner = launch_ros.actions.Node(
        package='th_maintenance',
        executable='opcheck_runner.py',
        name='opcheck_runner',
        parameters=[{
            'opcheck_deadman_timeout_s': DEADMAN_S,
            'v_check': V_CHECK,
            'motor_deadband_mps': MOTOR_DEADBAND,
            'motor_follow_min_ratio': MOTOR_FOLLOW_MIN_RATIO,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        opcheck_runner,
        launch_testing.actions.ReadyToTest(),
    ]), {'opcheck_runner': opcheck_runner}


class TestOpcheckRunnerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_opcheck_runner_client')

        self._cmd: list[Twist] = []
        self.node.create_subscription(
            Twist, '/cmd_vel_behavior', self._cmd.append, 10)
        self._status: list[CheckStatus] = []
        self.node.create_subscription(
            CheckStatus, '/opcheck/status', self._status.append, 10)
        self._events: list[StateEvent] = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._events.append, 10)

        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', _STATE_QOS)
        self.pub_hold = self.node.create_publisher(
            String, '/opcheck/motor_hold', 10)
        self.pub_wheel_cmd = self.node.create_publisher(
            WheelFeedback, '/esp32/wheel_cmd_speed', 10)
        self.pub_wheel_fb = self.node.create_publisher(
            WheelFeedback, '/esp32/wheel_feedback', 10)

        self.cli_run = self.node.create_client(RunCheck, '/opcheck/run_item')
        assert self.cli_run.wait_for_service(timeout_sec=5.0), \
            'opcheck_runner が起動していない'

        # 前のテストの残留状態を消す（mode を OPCHECK 以外にすると opcheck_runner
        # は _on_state() で実行中の項目を自動中断・停止する。test_state_manager_node
        # の _reset_to_idle() と同じ考え方）。
        self._set_mode('IDLE', 'NONE')
        self._publish_hold('NONE')
        self._cmd.clear()
        self._status.clear()
        self._events.clear()

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _set_mode(self, mode: str, state: str = 'LIST'):
        msg = SystemState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = mode
        msg.state = state
        self.pub_state.publish(msg)
        self._spin(0.2)

    def _run_item(self, item: str):
        req = RunCheck.Request()
        req.item = item
        future = self.cli_run.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        return future.result()

    def _publish_hold(self, value: str):
        self.pub_hold.publish(String(data=value))
        self._spin(0.1)

    def _publish_wheel(self, cmd_l: float, cmd_r: float, meas_l: float, meas_r: float):
        cmd = WheelFeedback()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.left_speed = cmd_l
        cmd.right_speed = cmd_r
        self.pub_wheel_cmd.publish(cmd)
        fb = WheelFeedback()
        fb.header.stamp = self.node.get_clock().now().to_msg()
        fb.left_speed = meas_l
        fb.right_speed = meas_r
        self.pub_wheel_fb.publish(fb)
        self._spin(0.05)

    def _wait_for_event(self, event: str, timeout: float = 3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.05)
            hit = [e for e in self._events if e.event == event]
            if hit:
                return hit
        return []

    # ════════════════════════════════════════════════════════
    # a. OPCHECK 以外のモードでは動かない
    # ════════════════════════════════════════════════════════
    def test_a_rejects_outside_opcheck(self):
        self._set_mode('IDLE', 'NONE')
        res = self._run_item('MOTOR')
        assert res.started is False, \
            'OPCHECK 以外のモードで /opcheck/run_item が受理された（②の標的）'

        self._cmd.clear()
        self._publish_hold('FORWARD')
        self._spin(DEADMAN_S + 0.2)
        assert not self._cmd, \
            'OPCHECK 以外のモードで /cmd_vel_behavior に指令が出た（②の標的）'
        self._publish_hold('NONE')

    # ════════════════════════════════════════════════════════
    # b. MOTOR: 押している間だけ動く。デッドマンで自動停止する
    # ════════════════════════════════════════════════════════
    def test_b_motor_hold_and_deadman_release(self):
        self._set_mode('OPCHECK', 'LIST')
        res = self._run_item('MOTOR')
        assert res.started, res.message

        self._cmd.clear()
        self._publish_hold('FORWARD')
        # デッドマン窓の内側（押し続けている状態を模擬）: v_check の指令が出る。
        self._spin(DEADMAN_S * 0.5)
        assert self._cmd, '押下中に /cmd_vel_behavior へ何も出なかった'
        last = self._cmd[-1]
        assert abs(last.linear.x - V_CHECK) < 1e-6, \
            f'FORWARD 中の指令が v_check と違う: {last.linear.x}'

        # 送るのをやめる（画面が離れた/途絶した想定）。deadman_timeout を
        # 超えた時点で 0 になっているはず（①の標的）。
        self._cmd.clear()
        self._spin(DEADMAN_S + 0.3)
        assert self._cmd, 'デッドマン経過後に /cmd_vel_behavior へ停止指令が出なかった'
        assert abs(self._cmd[-1].linear.x) < 1e-9 and abs(self._cmd[-1].angular.z) < 1e-9, \
            f'デッドマン経過後も指令が 0 になっていない: {self._cmd[-1]}'

    # ════════════════════════════════════════════════════════
    # c. MOTOR: 符号逆で evt.check_result が NG になる
    # ════════════════════════════════════════════════════════
    def test_c_motor_sign_mismatch_emits_ng(self):
        self._set_mode('OPCHECK', 'LIST')
        res = self._run_item('MOTOR')
        assert res.started, res.message

        self._events.clear()
        self._publish_hold('FORWARD')
        # 指令は前進（cmd_l/cmd_r 正）なのに、左だけ実測が逆符号（③の標的:
        # judge_motor の符号判定が無いと OK になってしまう）。
        self._publish_wheel(cmd_l=V_CHECK, cmd_r=V_CHECK,
                            meas_l=-V_CHECK, meas_r=V_CHECK)
        self._publish_hold('NONE')  # 離した扱いで即時に判定を確定させる

        hits = self._wait_for_event('evt.check_result', timeout=3.0)
        assert hits, 'evt.check_result が一度も出なかった'
        args = json.loads(hits[-1].arg_json)
        assert args.get('item') == 'MOTOR'
        assert args.get('result') == 'NG', \
            f'符号逆なのに NG にならなかった: {args}（③の標的）'

    # ════════════════════════════════════════════════════════
    # d. OPCHECK を抜けたら即座に指令が 0 になる
    # ════════════════════════════════════════════════════════
    def test_d_leaving_opcheck_halts_immediately(self):
        self._set_mode('OPCHECK', 'LIST')
        res = self._run_item('MOTOR')
        assert res.started, res.message

        self._publish_hold('FORWARD')
        self._spin(DEADMAN_S * 0.3)
        assert self._cmd and abs(self._cmd[-1].linear.x - V_CHECK) < 1e-6, \
            '押下中に v_check の指令が出ていない（前提が崩れている）'

        self._cmd.clear()
        self._set_mode('IDLE', 'NONE')
        assert self._cmd, 'OPCHECK を抜けても /cmd_vel_behavior に停止指令が出なかった'
        assert abs(self._cmd[-1].linear.x) < 1e-9 and abs(self._cmd[-1].angular.z) < 1e-9, \
            f'OPCHECK を抜けた直後の指令が 0 になっていない: {self._cmd[-1]}'
        self._publish_hold('NONE')


if __name__ == '__main__':
    unittest.main()
