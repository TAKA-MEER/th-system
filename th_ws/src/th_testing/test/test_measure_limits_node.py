"""
test_measure_limits_node.py
===========================
8-g P5c — `meas.set_limits` の速度・加速度の上限を本物の
`state_manager`・`measure_link`・`twist_mux`・`obstacle_limiter` で縛る試験。
正本は docs/plan/detailed/DetailedDesign-measure.md §6.6 と
docs/plan/spec/Spec-measure.md §3・§5.1。

起動は「生成 yaml を先に渡す」本番の経路。指令は本番の経路
`/cmd_vel_nav` → `twist_mux` → `obstacle_limiter` → `/cmd_vel` で流す。

  a. AUTO_MEASURE/WAIT で v_max 0.05 → DONE 1 回・状態不変・
     /measure/status.v_limit_mps が 0.05、/cmd_vel が 0.05 を超えない
     （比べとして指定なしでは超える）
  b. a_max を小さく → /cmd_vel の立ち上がりの傾きが指定を超えない
  c. 指定したまま UI 非常停止 → /cmd_vel が次周期で 0（ランプで鈍らない）。
     解除して戻ったあとも指定が残る（/system/state・/measure/status）
  d. 自律測定を抜ける（meas.finish）→ 4 つとも 0 に戻る
  e. IDLE・AT_PANEL/IDLE_P では state_not_accepting、負は bad_argument。
     AT_PANEL/WORKING では受け、meas.done で解けると 0 に戻る

注意（launch_testing の挙動）: ノードはファイル単位で共有されるため、
メソッド同士の状態に頼らない（各メソッドが自分の前提を作る）。
指示を送る前に生存確認が戻ったことを待つ（test_adjust_node の流儀）。
"""
import json
import math
import os
import sys
import tempfile
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Pose, Twist
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from th_system_msgs.msg import (ActiveScreen, MeasureHeartbeat,
                                MeasureResult, MeasureStatus, Pin, PinList,
                                StateEvent, SystemState)
from th_system_msgs.srv import MeasureCommand, UiTrigger

_THIS_DIR = os.path.dirname(__file__)
_LAUNCH_DIR = os.path.abspath(os.path.join(
    _THIS_DIR, '..', '..', 'th_bringup', 'launch'))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
import params_generation as pg  # noqa: E402

_GEN_DIR = None


def _generated_yaml(name):
    global _GEN_DIR
    if _GEN_DIR is None:
        _GEN_DIR = tempfile.mkdtemp(prefix='measure_limits_test_gen_')
        out_dir = os.path.join(_GEN_DIR, 'generated')
        pg.run_generation(
            stage=4, sim=False, nodes=list(pg.REGISTRY_NODES), out_dir=out_dir,
            registry_path=os.path.join(
                _THIS_DIR, '..', '..', 'th_params', 'config', 'registry.yaml'),
            calib_dir=os.path.join(_GEN_DIR, 'no_calib'),
            overrides_path=os.path.join(_GEN_DIR, 'no_overrides.yaml'))
    return os.path.join(_GEN_DIR, 'generated', name)


V_SLOW = 0.35
V_JOG_PANEL = 0.12
W_MAX = 0.6
FAR_M = 30.0
N_BEAMS = 360
PUB_HZ = 20.0

_LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


@pytest.mark.launch_test
def generate_test_description():
    tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher', name='laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen')
    state_manager = launch_ros.actions.Node(
        package='th_state', executable='state_manager.py', name='state_manager',
        parameters=[_generated_yaml('state_manager.yaml')], output='screen')
    measure_link = launch_ros.actions.Node(
        package='th_measure', executable='measure_link.py', name='measure_link',
        parameters=[_generated_yaml('measure_link.yaml')], output='screen')
    limiter = launch_ros.actions.Node(
        package='th_safety', executable='obstacle_limiter', name='obstacle_limiter',
        parameters=[{
            'v_max': 1.0, 'v_slow': V_SLOW, 'v_jog_panel': V_JOG_PANEL,
            'v_reverse': 0.25, 'obstacle_min_points': 1, 'w_max': W_MAX,
            'normal_accel_mps2': 0.0, 'normal_jerk_mps3': 0.0,
            'normal_angular_accel_rps2': 0.0,
        }], output='screen')
    twist_mux = launch_ros.actions.Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[{
            'topics': {
                'behavior': {'topic': '/cmd_vel_behavior', 'timeout': 0.5,
                             'priority': 20},
                'nav': {'topic': '/cmd_vel_nav', 'timeout': 0.5, 'priority': 10},
                'manual_joy': {'topic': '/cmd_vel_manual', 'timeout': 1.0,
                               'priority': 30},
            },
            'locks': {
                'estop': {'topic': '/safety/estop', 'timeout': 0.5,
                          'priority': 255},
                'fault': {'topic': '/safety/fault_lock', 'timeout': 0.5,
                          'priority': 254},
            },
        }],
        remappings=[('cmd_vel_out', '/cmd_vel_muxed')], output='screen')
    return launch.LaunchDescription([
        tf, state_manager, measure_link, limiter, twist_mux,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestMeasureLimitsNode(unittest.TestCase):
    _cid = 0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        n = self.node = rclpy.create_node('test_measure_limits_client')
        self._states = []
        self._statuses = []
        self._results = []
        self._cmd_vel = []          # (t, linear.x)
        self._cmd_states = []
        n.create_subscription(SystemState, '/system/state', self._states.append, _LATCHED)
        n.create_subscription(MeasureStatus, '/measure/status', self._statuses.append, _LATCHED)
        n.create_subscription(MeasureResult, '/measure/result', self._results.append, 20)
        n.create_subscription(String, '/measure/command_state', self._cmd_states.append, _LATCHED)
        n.create_subscription(
            Twist, '/cmd_vel',
            lambda m: self._cmd_vel.append((time.monotonic(), float(m.linear.x))), 10)
        self.pub_event = n.create_publisher(StateEvent, '/system/event', 10)
        self.pub_pins = n.create_publisher(PinList, '/onsite/pins', _LATCHED)
        self.pub_screen = n.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.pub_estop = n.create_publisher(Bool, '/safety/estop', 1)
        self.pub_lock = n.create_publisher(Bool, '/safety/fault_lock', 1)
        self.pub_estop_ui = n.create_publisher(Bool, '/safety/estop_ui', 10)
        self.pub_estop_hw = n.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_scan = n.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_nav = n.create_publisher(Twist, '/cmd_vel_nav', 1)
        self.pub_hb = n.create_publisher(MeasureHeartbeat, '/measure/heartbeat', 10)
        self._hb_version = "1"
        self._hb_seq = 0
        self._hb_timer = n.create_timer(0.2, self._publish_hb)
        self.cli_measure = n.create_client(MeasureCommand, '/measure/command')
        self.cli_trigger = n.create_client(UiTrigger, '/system/trigger')
        assert self.cli_measure.wait_for_service(timeout_sec=10.0)
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0)
        self._nav_on = False
        self._nav_linear = 0.3
        self._estop_ui_on = False
        self._pub_timer = n.create_timer(1.0 / PUB_HZ, self._publish)
        self._spin(0.5)
        self._drive_to_idle()

    def tearDown(self):
        self._hb_version = None
        self._nav_on = False
        self._estop_ui_on = False
        for timer in (self._hb_timer, self._pub_timer):
            if timer is not None:
                timer.cancel()
                self.node.destroy_timer(timer)
        self._hb_timer = self._pub_timer = None
        self.node.destroy_node()

    # ── 試験側の出版（実機と同じトピック・型） ──
    def _publish(self):
        # safety_monitor の代役: /safety/estop は UI 非常停止に追随する
        self.pub_estop.publish(Bool(data=bool(self._estop_ui_on)))
        self.pub_lock.publish(Bool(data=False))
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / N_BEAMS
        scan.angle_max = scan.angle_min + scan.angle_increment * (N_BEAMS - 1)
        scan.range_min = 0.05
        scan.range_max = 40.0
        scan.ranges = [FAR_M] * N_BEAMS
        self.pub_scan.publish(scan)
        # 本物の画面（S-60）を開いている端末が居る＝画面由来の上限は v_slow
        self.pub_screen.publish(ActiveScreen(
            screen_id='S-60', client_id='fk-measure-limits-test',
            interacting=True, last_input=self.node.get_clock().now().to_msg()))
        if self._nav_on:
            cmd = Twist()
            cmd.linear.x = float(self._nav_linear)
            self.pub_nav.publish(cmd)
        self.pub_estop_ui.publish(Bool(data=bool(self._estop_ui_on)))

    def _publish_hb(self):
        if self._hb_version is None:
            return
        self._hb_seq += 1
        msg = MeasureHeartbeat()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.interface_version = self._hb_version
        msg.sender = 'fk-system'
        msg.seq = self._hb_seq
        self.pub_hb.publish(msg)

    def _spin(self, duration=0.2):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _drive_to_idle(self):
        self.pub_estop_hw.publish(Bool(data=False))
        self.pub_estop_ui.publish(Bool(data=False))
        self._spin(0.3)
        if not self._states or self._states[-1].mode != 'INIT':
            return
        deadline = time.time() + 8.0
        while time.time() < deadline:
            msg = StateEvent()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.event = 'evt.link_ok'
            msg.source_node = 'connectivity_checker'
            msg.arg_json = '{}'
            self.pub_event.publish(msg)
            self._spin(0.3)
            if self._states and self._states[-1].mode == 'IDLE':
                return
        assert self._states and self._states[-1].mode == 'IDLE', \
            (self._states[-1].mode, self._states[-1].state) if self._states else 'no state'

    def _event(self, name, arg='{}'):
        msg = StateEvent()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.event = name
        msg.source_node = 'test'
        msg.arg_json = arg
        self.pub_event.publish(msg)

    def _wait_link_ok(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self._statuses and self._statuses[-1].link_ok:
                return
        raise AssertionError('/measure/status の link_ok が立たない')

    def _ensure_idle(self):
        st = self._latest_state()
        if (st.mode, st.state) == ('IDLE', 'NONE'):
            return
        if st.mode == 'ESTOP':
            self._estop_ui_on = False
            self.pub_estop_ui.publish(Bool(data=False))
            self._spin(0.3)
            self._call_trigger('ui.resume_no')
        elif st.mode == 'CARRY':
            self.pub_estop_hw.publish(Bool(data=False))
            self._spin(0.3)
            self._call_trigger('ui.finish')
        else:
            self._call_trigger('ui.finish')
        assert self._wait_mode_state('IDLE', 'NONE'), \
            (self._states[-1].mode, self._states[-1].state)

    def _ensure_auto_wait(self):
        self._wait_link_ok()
        st = self._latest_state()
        if (st.mode, st.state) == ('AUTO_MEASURE', 'WAIT'):
            return
        self._ensure_idle()
        self._wait_link_ok()
        res = self._call_trigger('ui.goto', json.dumps({"kind": "AUTO_MEASURE"}))
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT') is True
        # 前の試験の上限を消す
        res = self._call_measure('meas.set_limits')
        assert res.accepted is True, res.reject_reason
        self._spin(0.3)

    def _go_at_panel(self):
        self._ensure_idle()
        res = self._call_trigger('ui.goto', {'kind': 'HOME'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('HOME_NAV', 'NAV')
        self._event('evt.arrived')
        assert self._wait_mode_state('AT_HOME', 'IDLE_H')
        pin = Pin(id='p1', name='panel', kind='PANEL', pose=Pose())
        self.pub_pins.publish(PinList(pins=[pin]))
        self._spin(0.4)
        res = self._call_trigger('ui.goto', {'kind': 'PANEL', 'pin_id': 'p1'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('PANEL_NAV', 'NAV')
        self._event('evt.arrived')
        assert self._wait_mode_state('PANEL_NAV', 'ALIGN')
        self._event('evt.align_done')
        assert self._wait_mode_state('AT_PANEL', 'IDLE_P')

    # ── ヘルパー ──
    def _latest_state(self):
        self._spin(0.2)
        assert self._states, '/system/state を1件も受信していない'
        return self._states[-1]

    def _latest_status(self):
        self._spin(0.2)
        assert self._statuses, '/measure/status を1件も受信していない'
        return self._statuses[-1]

    def _next_cid(self):
        type(self)._cid += 1
        return f"ml-{type(self)._cid}"

    def _call_measure(self, command, command_id=None, v_max=0.0, w_max=0.0,
                      a_max=0.0, alpha_max=0.0):
        req = MeasureCommand.Request()
        req.command_id = self._next_cid() if command_id is None else command_id
        req.command = command
        req.v_max_mps = float(v_max)
        req.w_max_radps = float(w_max)
        req.a_max_mps2 = float(a_max)
        req.alpha_max_radps2 = float(alpha_max)
        fut = self.cli_measure.call_async(req)
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert fut.done(), f'/measure/command が返らない: {command}'
        return fut.result()

    def _call_trigger(self, trigger, arg=None):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg is not None else '{}'
        req.requester = 'test'
        fut = self.cli_trigger.call_async(req)
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert fut.done(), f'/system/trigger が返らない: {trigger}'
        return fut.result()

    def _wait_mode_state(self, mode, state, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._states and (self._states[-1].mode, self._states[-1].state) == (mode, state):
                return True
        return False

    def _wait_status_v_limit(self, value, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            if self._statuses and abs(self._statuses[-1].v_limit_mps - value) < 1e-6:
                return True
        return False

    def _collect_cmd_vel(self, duration=1.2):
        start = len(self._cmd_vel)
        self._spin(duration)
        return self._cmd_vel[start:]

    # ═══════════
    def test_a_v_cap_clamps_cmd_vel(self):
        """a. v_cap 0.05 で /cmd_vel が 0.05 を超えず、指定なしでは超える。"""
        self._ensure_auto_wait()
        # 比べ: 指定なしでは 0.05 を超える
        self._cmd_vel.clear()
        self._nav_on = True
        self._nav_linear = 0.3
        samples = self._collect_cmd_vel(1.2)
        self._nav_on = False
        self._spin(0.6)
        assert samples and max(v for _, v in samples) > 0.05, \
            [v for _, v in samples][-10:]
        # 指定
        cid = self._next_cid()
        res = self._call_measure('meas.set_limits', command_id=cid, v_max=0.05)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AUTO_MEASURE', 'WAIT'), (res.mode, res.state)
        self._spin(0.5)
        done = [r for r in self._results if r.command_id == cid]
        assert len(done) == 1 and done[0].result == 'DONE', done
        assert self._wait_status_v_limit(0.05) is True, self._latest_status().v_limit_mps
        st = self._latest_state()
        assert st.measure_v_cap_mps == 0.05
        self._cmd_vel.clear()
        self._nav_on = True
        samples = self._collect_cmd_vel(1.2)
        self._nav_on = False
        assert samples, 'cmd_vel が来ていない'
        assert max(v for _, v in samples) <= 0.05 + 1e-6, \
            [v for _, v in samples][-10:]

    def test_b_a_cap_limits_slope(self):
        """b. a_max が小さいと立ち上がりの傾きが指定を超えない。"""
        self._ensure_auto_wait()
        a_cap = 0.05
        res = self._call_measure('meas.set_limits', a_max=a_cap)
        assert res.accepted is True, res.reject_reason
        self._spin(0.3)
        self._cmd_vel.clear()
        self._nav_on = True
        self._nav_linear = 0.3
        samples = self._collect_cmd_vel(2.0)
        self._nav_on = False
        assert len(samples) >= 10, len(samples)
        # 傾き（連続サンプルの差分 / dt）が a_cap を超えない（余裕を持たせる）
        worst = 0.0
        for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
            dt = t1 - t0
            if dt > 0:
                worst = max(worst, abs(v1 - v0) / dt)
        assert worst <= a_cap + 0.05, (worst, [v for _, v in samples][:10])
        assert max(v for _, v in samples) < 0.3, [v for _, v in samples][-5:]

    def test_c_estop_stops_immediately_and_caps_remain(self):
        """c. UI 非常停止で次周期に 0、解除して戻ったあとも指定が残る。"""
        self._ensure_auto_wait()
        res = self._call_measure('meas.set_limits', v_max=0.05)
        assert res.accepted is True, res.reject_reason
        assert self._wait_status_v_limit(0.05) is True
        self._cmd_vel.clear()
        self._nav_on = True
        samples = self._collect_cmd_vel(1.0)
        assert samples and max(v for _, v in samples) > 0.03
        t_estop = time.monotonic()
        self._estop_ui_on = True
        assert self._wait_mode_state('ESTOP', 'NONE') is True
        self._spin(0.5)
        after = [v for t, v in self._cmd_vel if t > t_estop]
        assert after and all(abs(v) < 1e-9 for v in after[-5:]), after[-10:]
        self._nav_on = False
        self._estop_ui_on = False
        self.pub_estop_ui.publish(Bool(data=False))
        self._spin(0.3)
        res = self._call_trigger('ui.resume_yes')
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT') is True
        st = self._latest_state()
        assert st.measure_v_cap_mps == 0.05, st.measure_v_cap_mps
        assert self._wait_status_v_limit(0.05) is True

    def test_d_finish_clears_caps(self):
        """d. meas.finish で 4 つとも 0 に戻る。"""
        self._ensure_auto_wait()
        res = self._call_measure('meas.set_limits', v_max=0.05, w_max=0.1,
                                 a_max=0.1, alpha_max=0.1)
        assert res.accepted is True, res.reject_reason
        st = self._latest_state()
        assert st.measure_v_cap_mps == 0.05
        assert st.measure_w_cap_radps == 0.1
        assert st.measure_a_cap_mps2 == 0.1
        assert st.measure_alpha_cap_radps2 == 0.1
        res = self._call_measure('meas.finish')
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('IDLE', 'NONE') is True
        st = self._latest_state()
        assert st.measure_v_cap_mps == 0.0
        assert st.measure_w_cap_radps == 0.0
        assert st.measure_a_cap_mps2 == 0.0
        assert st.measure_alpha_cap_radps2 == 0.0

    def test_e_rejecting_states_and_bad_argument(self):
        """e. IDLE・AT_PANEL/IDLE_P は state_not_accepting、負は bad_argument。
        AT_PANEL/WORKING では受け、meas.done で解けると 0 に戻る。"""
        self._ensure_idle()
        res = self._call_measure('meas.set_limits', v_max=0.05)
        assert res.accepted is False
        assert res.reject_reason == 'state_not_accepting', res.reject_reason
        # AT_PANEL / IDLE_P
        self._go_at_panel()
        res = self._call_measure('meas.set_limits', v_max=0.05)
        assert res.accepted is False
        assert res.reject_reason == 'state_not_accepting', res.reject_reason
        # WORKING
        res = self._call_trigger('ui.working')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'WORKING') is True
        self._wait_link_ok()
        res = self._call_measure('meas.set_limits', v_max=-0.1)
        assert res.accepted is False
        assert res.reject_reason == 'bad_argument', res.reject_reason
        res = self._call_measure('meas.set_limits', v_max=0.05)
        assert res.accepted is True, res.reject_reason
        st = self._latest_state()
        assert st.measure_v_cap_mps == 0.05
        res = self._call_measure('meas.done')
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AT_PANEL', 'IDLE_P') is True
        st = self._latest_state()
        assert st.measure_v_cap_mps == 0.0
