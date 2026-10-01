"""
test_calib_runner_node.py
=========================
WP-MAINT-02 — calib_runner **単体**の振る舞い試験（launch 試験。state_manager は起動しない）。

`/system/state` と `/system/effect` をテストが直接出す（お手本 test_opcheck_runner_node.py）。
state_manager 込みの通し・配線は test_calib_wiring_node.py。ここでは、FSM に頼らず
ノード自身が守るべき安全を縛る。

  a. CALIB 以外では effect もサービスも受け付けず、/cmd_vel_behavior に何も出さない
  b. **CALIB を抜けたら（IDLE・ESTOP 等へ）走行が即座に止まる**（走り続けない）
  c. /system/state が途絶したら走行を止める
  d. /odom が無ければ走り始めない（自分の位置が分からないまま走らせない）
  e. **検証に合格していない補正値は確定しない**（commit_calib を直接送られても書かない）
  f. FSM のガードをすり抜けて apply_and_verify が来ても、sane でなければ適用しない
  g. IMU: 走らせず、/esp32/imu_calib_status が全部 3 になったら進み、確定できる
  h. 出力は /cmd_vel_behavior だけ（/cmd_vel へは出さない）

変異チェック（一覧は報告に記載）:
  ② _in_calib() を True 固定 → test_a_* が赤
  ⑥ _on_state() の _abort_all("left_calib") を消す → test_b_leaving_calib_stops_motion が赤
"""
import json
import math
import os
import shutil
import tempfile
import time
import unittest

import pytest
import rclpy
import yaml
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from th_system_msgs.msg import StateEffect, StateEvent, SystemState
from th_system_msgs.srv import ApplyCalib, RollbackCalib, StartCalib, SubmitCalib

CALIB_DIR = tempfile.mkdtemp(prefix='calib_runner_node_')
DIST_M = 1.0          # 走り終わらないよう長めに取る（試験は途中で止める）
V_CALIB = 0.20

_STATE_QOS = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST)


@pytest.mark.launch_test
def generate_test_description():
    calib_runner = launch_ros.actions.Node(
        package='th_maintenance', executable='calib_runner.py', name='calib_runner',
        parameters=[{
            'calib_dir': CALIB_DIR,
            'params_digest_path': os.path.join(CALIB_DIR, 'none.json'),
            'calib_linear_distance_m': DIST_M,
            'v_calib': V_CALIB,
            'calib_linear_tolerance_ratio': 0.05,
            'calib_run_timeout_s': 30.0,
        }],
        output='screen')
    return launch.LaunchDescription([
        calib_runner, launch_testing.actions.ReadyToTest(),
    ]), {}


class TestCalibRunnerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        shutil.rmtree(CALIB_DIR, ignore_errors=True)

    def setUp(self):
        for name in os.listdir(CALIB_DIR):
            path = os.path.join(CALIB_DIR, name)
            shutil.rmtree(path) if os.path.isdir(path) else os.remove(path)
        self.node = rclpy.create_node('test_calib_runner_client')
        self.cmds = []
        self.cmd_vel_direct = []
        self.events = []
        self.node.create_subscription(Twist, '/cmd_vel_behavior', self.cmds.append, 10)
        self.node.create_subscription(Twist, '/cmd_vel', self.cmd_vel_direct.append, 10)
        self.node.create_subscription(StateEvent, '/system/event', self.events.append, 10)
        self.pub_state = self.node.create_publisher(SystemState, '/system/state', _STATE_QOS)
        self.pub_effect = self.node.create_publisher(StateEffect, '/system/effect', 10)
        self.pub_odom = self.node.create_publisher(Odometry, '/odom', 10)
        self.pub_imu = self.node.create_publisher(UInt8, '/esp32/imu_calib_status', 10)
        self.cli_start = self.node.create_client(StartCalib, '/calib/start')
        self.cli_submit = self.node.create_client(SubmitCalib, '/calib/submit')
        self.cli_apply = self.node.create_client(ApplyCalib, '/calib/apply')
        self.cli_rollback = self.node.create_client(RollbackCalib, '/calib/rollback')
        assert self.cli_start.wait_for_service(timeout_sec=10.0), 'calib_runner 未起動'

        self.odom_x = 0.0
        self._cmd_t = 0.0
        self._last = Twist()
        self.node.create_subscription(Twist, '/cmd_vel_behavior', self._remember, 10)
        self.state = ('IDLE', 'NONE')
        self.send_state = True
        self.send_odom = True
        self._last_state_pub = 0.0
        self._set_state('IDLE', 'NONE')
        self._spin(0.3)
        self.cmds.clear()
        self.events.clear()

    def tearDown(self):
        self._set_state('IDLE', 'NONE')
        self._spin(0.3)
        self.node.destroy_node()

    def _remember(self, msg):
        self._last = msg
        self._cmd_t = time.time()

    def _set_state(self, mode, state):
        self.state = (mode, state)
        self._publish_state()

    def _publish_state(self):
        msg = SystemState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode, msg.state = self.state
        self.pub_state.publish(msg)
        self._last_state_pub = time.time()

    def _spin(self, duration=0.2):
        deadline = time.time() + duration
        last = time.time()
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            now = time.time()
            if self.send_state and now - self._last_state_pub >= 0.1:
                self._publish_state()          # state_manager と同じ 10Hz の再送
            if now - last >= 0.04:
                dt = now - last
                last = now
                cmd = self._last if now - self._cmd_t < 0.3 else Twist()
                self.odom_x += cmd.linear.x * dt
                if self.send_odom:
                    odom = Odometry()
                    odom.header.stamp = self.node.get_clock().now().to_msg()
                    odom.pose.pose.position.x = self.odom_x
                    odom.pose.pose.orientation.w = 1.0
                    self.pub_odom.publish(odom)

    def _wait(self, cond, timeout=6.0, what=''):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.05)
            if cond():
                return True
        self.fail(f'タイムアウト: {what}')

    def _effect(self, name, item='LINEAR', args=None):
        eff = StateEffect()
        eff.header.stamp = self.node.get_clock().now().to_msg()
        eff.name = name
        eff.dest = 'calib_runner'
        eff.args_json = json.dumps(args if args is not None else {'item': item})
        self.pub_effect.publish(eff)
        self._spin(0.2)

    def _call(self, client, req, timeout=4.0):
        future = client.call_async(req)
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            self._spin(0.02)
        return future.result()

    def _moving(self):
        return any(abs(c.linear.x) > 1e-9 or abs(c.angular.z) > 1e-9 for c in self.cmds)

    def _stopped_now(self):
        self._spin(0.5)
        # 最後の指令が 0（止めたあとは publish をやめるので、最後の 1 件を見る）
        return bool(self.cmds) and abs(self.cmds[-1].linear.x) < 1e-9 \
            and abs(self.cmds[-1].angular.z) < 1e-9

    def _start_running(self, item='LINEAR'):
        self._set_state('CALIB', 'S1')
        self._effect('begin_wizard', item=item)
        self._set_state('CALIB', 'S2')
        self._effect('run_measurement', item=item)
        self._wait(self._moving, what='走り出す')

    # ── a ────────────────────────────────────────────────────
    def test_a_ignores_everything_outside_calib(self):
        self._set_state('IDLE', 'NONE')
        # state を再送しない（再送が続くと、入口のゲートが壊れていても
        # 「CALIB でない」state 受信による中断が後から効いて、ゲートの欠陥が隠れる）
        self.send_state = False
        self._effect('begin_wizard')
        self._effect('run_measurement')
        self._spin(1.0)
        assert not self._moving(), 'CALIB 以外で effect を受けて走った（②の標的）'
        req = StartCalib.Request()
        req.item = 'LINEAR'
        assert not self._call(self.cli_start, req).started
        sub = SubmitCalib.Request()
        sub.item = 'LINEAR'
        sub.measured = 1.0
        assert not self._call(self.cli_submit, sub).success
        rb = RollbackCalib.Request()
        rb.item = 'LINEAR'
        rb.generation = 1
        assert not self._call(self.cli_rollback, rb).success
        ap = ApplyCalib.Request()
        ap.item = 'LINEAR'
        assert not self._call(self.cli_apply, ap).success

    # ── b ────────────────────────────────────────────────────
    def test_b_leaving_calib_stops_motion(self):
        self._start_running()
        x_before = self.odom_x
        assert x_before >= 0.0
        # 非常停止・強制遷移などで CALIB を抜ける（IDLE へ）
        self._set_state('IDLE', 'NONE')
        assert self._stopped_now(), 'CALIB を抜けたのに /cmd_vel_behavior が 0 にならない（⑥の標的）'
        x_stop = self.odom_x
        self._spin(1.0)
        assert self.odom_x - x_stop < 0.01, 'CALIB を抜けたあとも走り続けた'
        assert not self._call_started_ok(), '抜けたあとにサービスで再開できた'

    def _call_started_ok(self):
        req = StartCalib.Request()
        req.item = 'LINEAR'
        return self._call(self.cli_start, req).started

    def test_b2_estop_mode_stops_motion(self):
        self._start_running()
        self._set_state('ESTOP', 'NONE')
        assert self._stopped_now()
        x_stop = self.odom_x
        self._spin(0.8)
        assert self.odom_x - x_stop < 0.01

    # ── c ────────────────────────────────────────────────────
    def test_c_state_silence_stops_motion(self):
        self._start_running()
        self.send_state = False           # /system/state を送るのをやめる
        self._wait(lambda: self._stopped_now(), timeout=8.0, what='/system/state 途絶で停止')
        x_stop = self.odom_x
        self._spin(1.0)
        assert self.odom_x - x_stop < 0.01

    # ── i: 走行中に /odom が途絶したら止める（自分の位置が分からないまま走らせない）──
    def _assert_stops_on_odom_silence(self, item):
        self._start_running(item)
        self.events.clear()
        self.send_odom = False           # ここで /odom を止める
        # _ODOM_STALE_S(0.5s) + 余裕の内に 0 になる
        self._wait(lambda: bool(self.cmds) and abs(self.cmds[-1].linear.x) < 1e-9
                   and abs(self.cmds[-1].angular.z) < 1e-9,
                   timeout=1.5, what=f'{item}: /odom 途絶で /cmd_vel_behavior が 0 になる（⑦の標的）')
        n = len(self.cmds)
        self._spin(1.0)
        later = self.cmds[n:]
        assert all(abs(c.linear.x) < 1e-9 and abs(c.angular.z) < 1e-9 for c in later), \
            '/odom 途絶のあとにまた動かした'
        assert not any(e.event == 'evt.calib_step_done' for e in self.events), \
            '/odom 途絶で測定走行が「完了」扱いになった'
        assert not os.path.exists(os.path.join(CALIB_DIR, 'current.yaml')), \
            '/odom 途絶で補正値が確定された'
        # /odom が戻っても勝手に再開しない（再走行は /calib/start だけ）
        self.send_odom = True
        self._spin(1.0)
        assert not self._moving_since(n), '/odom が戻ったら勝手に走り出した'

    def _moving_since(self, n):
        return any(abs(c.linear.x) > 1e-9 or abs(c.angular.z) > 1e-9 for c in self.cmds[n:])

    def test_i_odom_silence_stops_linear(self):
        self._assert_stops_on_odom_silence('LINEAR')

    def test_i_odom_silence_stops_rotation(self):
        self._assert_stops_on_odom_silence('ROTATION')

    # ── d ────────────────────────────────────────────────────
    def test_d_no_odom_does_not_start(self):
        self.send_odom = False
        self._spin(1.0)          # 前のテストの /odom を古くする
        self._set_state('CALIB', 'S1')
        self._effect('begin_wizard')
        self._set_state('CALIB', 'S2')
        self._effect('run_measurement')
        self._spin(1.0)
        assert not self._moving(), '/odom が無いのに走り出した'

    # ── e ────────────────────────────────────────────────────
    def test_e_commit_without_verification_writes_nothing(self):
        self._set_state('CALIB', 'S1')
        self._effect('begin_wizard')
        self._set_state('CALIB', 'LIST')
        self._effect('commit_calib')
        self._spin(0.3)
        assert not os.path.exists(os.path.join(CALIB_DIR, 'current.yaml')), \
            '検証に合格していないのに補正値が確定された'

    # ── f ────────────────────────────────────────────────────
    def test_f_apply_without_sane_preview_is_refused(self):
        self._set_state('CALIB', 'S1')
        self._effect('begin_wizard')
        self._set_state('CALIB', 'S4')
        self._effect('apply_and_verify')
        self._wait(lambda: any(e.event == 'evt.calib_verify_ng' for e in self.events),
                   what='sane でない適用要求の拒否（evt.calib_verify_ng）')
        ev = [e for e in self.events if e.event == 'evt.calib_verify_ng'][0]
        assert json.loads(ev.arg_json).get('reason') == 'preview_not_sane'
        assert not self._moving()

    # ── g ────────────────────────────────────────────────────
    def test_g_imu_flow_and_commit(self):
        self._set_state('CALIB', 'S1')
        self._effect('begin_wizard', item='IMU')
        self._set_state('CALIB', 'S2')
        self._effect('run_measurement', item='IMU')
        self.pub_imu.publish(UInt8(data=0x7F))       # sys=1 …（まだ全部 3 ではない）
        self._spin(0.4)
        assert not any(e.event == 'evt.calib_step_done' for e in self.events)
        self.pub_imu.publish(UInt8(data=0xFF))
        self._wait(lambda: any(e.event == 'evt.calib_step_done' for e in self.events),
                   what='IMU 完了 → evt.calib_step_done')
        assert not self._moving(), 'IMU 校正で機体が動いた'
        self._set_state('CALIB', 'S3')
        self.events.clear()
        self._effect('apply_and_verify', item='IMU')
        self._wait(lambda: any(e.event == 'evt.calib_step_done' for e in self.events),
                   what='IMU の S4 完了')
        self._set_state('CALIB', 'LIST')
        self._effect('commit_calib', item='IMU')
        self._wait(lambda: os.path.exists(os.path.join(CALIB_DIR, 'current.yaml')),
                   what='IMU の確定')
        with open(os.path.join(CALIB_DIR, 'current.yaml'), encoding='utf-8') as f:
            entry = yaml.safe_load(f)['items']['IMU']
        assert int(entry['values']['calib_status']) == 0xFF

    # ── h ────────────────────────────────────────────────────
    def test_h_output_only_on_cmd_vel_behavior(self):
        self._start_running()
        self._spin(0.5)
        assert self.cmds, '/cmd_vel_behavior に出ていない'
        assert not self.cmd_vel_direct, '/cmd_vel へ直接 publish した（不変ルール違反）'
        v = max(abs(c.linear.x) for c in self.cmds)
        assert v <= V_CALIB + 1e-6, f'v_calib を超える速度を出した: {v}'


if __name__ == '__main__':
    unittest.main()
