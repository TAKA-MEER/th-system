"""
test_working_stop_node.py
=========================
8-g P2 — お立会試験の停止命令（`AT_PANEL` の `WORKING`）を本物の
`state_manager`＋`measure_link`＋`obstacle_limiter` で縛る試験。
正本は docs/plan/spec/Spec-modes.md（SM-3.1.2-062・-063・-134・-139・-140）と
docs/plan/detailed/DetailedDesign-measure.md §6.3（P2 の表）。

起動は「生成 yaml を先に渡す」本番の経路（`state_manager`・`measure_link` は
P1/P5a と同じ生成 yaml。`obstacle_limiter` は既定と違う上限値にして取り違えを
数値で分かるようにした。test_athome_jog_speed_node.py と同じ流儀）。

  a. `ui.working` on で `WORKING`、`meas.done` → `IDLE_P`、
     `/measure/result` に `DONE` がちょうど 1 回、`/measure/status.working`
     が true → false
  b. `WORKING` でジョグ（本物の画面と同じ `/ui/active_screen` S-21・
     `/ui/jog_lease`・`/cmd_vel_muxed` 等）→ 拒否 `working_in_progress`、
     `/cmd_vel` が 0 のまま。対照として `IDLE_P` では同じ入力で動く
  c. `WORKING` で回復フォルト（`/safety/fault` に safety_monitor と同じ形）→
     `WORKING` のまま、そのあとジョグしても `/cmd_vel` 0。解消後の
     `ui.resume_ack` で `WORKING` のまま
  d. `WORKING` で UI 非常停止 → `/system/state.working` が true のまま →
     解除 → W-1 が送る `ui.resume_yes` で `AT_PANEL`/`WORKING` へ。
     `IDLE_P` から同じ手順なら `IDLE_P` へ
  e. `WORKING` で手押し（`CARRY`）→ 解除で `WORKING`
  f. `WORKING` で生存確認を止める → `WORKING` のまま
  g. `WORKING` で「停止を解く」（`ui.working` off）→ `IDLE_P`、行き先が選べる

注意（launch_testing の挙動）: ノードはファイル単位で共有されるため、
メソッド同士の状態に頼らない（各メソッドが _ensure_working で自分の状態を作る）。
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
from geometry_msgs.msg import Pose, Twist
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import (ActiveScreen, FaultStatus, LimiterStatus,
                                MeasureHeartbeat, MeasureResult, MeasureStatus,
                                Pin, PinList, StateEvent, SystemState)
from th_system_msgs.srv import MeasureCommand, UiTrigger

# params_generation の取り込み（test_auto_measure_node.py と同じ流儀）。
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
        _GEN_DIR = tempfile.mkdtemp(prefix='working_stop_test_gen_')
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
CMD_LINEAR_X = 0.8
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
        parameters=[_generated_yaml('state_manager.yaml')],
        output='screen')
    measure_link = launch_ros.actions.Node(
        package='th_measure', executable='measure_link.py', name='measure_link',
        parameters=[_generated_yaml('measure_link.yaml')],
        output='screen')
    limiter = launch_ros.actions.Node(
        package='th_safety', executable='obstacle_limiter', name='obstacle_limiter',
        parameters=[{
            'v_max': 1.0, 'v_slow': V_SLOW, 'v_jog_panel': V_JOG_PANEL,
            'v_reverse': 0.25, 'obstacle_min_points': 1,
        }],
        output='screen')
    return launch.LaunchDescription([
        tf, state_manager, measure_link, limiter,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestWorkingStopNode(unittest.TestCase):

    _cid = 0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        n = self.node = rclpy.create_node('test_working_stop_client')
        self._states = []
        self._statuses = []
        self._results = []
        self._cmd_vel = []
        self.status = None
        n.create_subscription(SystemState, '/system/state', self._states.append, _LATCHED)
        n.create_subscription(MeasureStatus, '/measure/status',
                              self._statuses.append, _LATCHED)
        n.create_subscription(MeasureResult, '/measure/result',
                              self._results.append, 20)
        n.create_subscription(Twist, '/cmd_vel', self._cmd_vel.append, 10)
        n.create_subscription(LimiterStatus, '/safety/limiter_status',
                              lambda m: setattr(self, 'status', m),
                              qos_profile_sensor_data)
        self.pub_event = n.create_publisher(StateEvent, '/system/event', 10)
        self.pub_pins = n.create_publisher(PinList, '/onsite/pins', _LATCHED)
        self.pub_fault = n.create_publisher(FaultStatus, '/safety/fault', 5)
        self.pub_estop_ui = n.create_publisher(Bool, '/safety/estop_ui', 10)
        self.pub_estop_hw = n.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_screen = n.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.pub_jog = n.create_publisher(String, '/ui/jog_lease', 1)
        self.pub_panel = n.create_publisher(String, '/ui/manual_panel', 1)
        self.pub_muxed = n.create_publisher(Twist, '/cmd_vel_muxed', 1)
        self.pub_manual = n.create_publisher(Twist, '/cmd_vel_manual', 1)
        self.pub_scan = n.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_estop = n.create_publisher(Bool, '/safety/estop', 1)
        self.pub_lock = n.create_publisher(Bool, '/safety/fault_lock', 1)
        self.pub_hb = n.create_publisher(MeasureHeartbeat, '/measure/heartbeat', 10)
        self._hb_version = "1"
        self._hb_seq = 0
        self._hb_timer = n.create_timer(0.2, self._publish_hb)
        self.cli_measure = n.create_client(MeasureCommand, '/measure/command')
        self.cli_trigger = n.create_client(UiTrigger, '/system/trigger')
        assert self.cli_measure.wait_for_service(timeout_sec=10.0)
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0)
        self.jog = False
        self._pub_timer = n.create_timer(1.0 / PUB_HZ, self._publish)
        self._spin(0.5)
        self._drive_to_idle()

    def tearDown(self):
        self._hb_version = None
        for timer in (self._hb_timer, self._pub_timer):
            if timer is not None:
                timer.cancel()
                self.node.destroy_timer(timer)
        self._hb_timer = self._pub_timer = None
        self.node.destroy_node()

    # ── 入力（実機と同じトピック・型）──────────────────────────
    def _publish(self):
        cmd = Twist()
        cmd.linear.x = CMD_LINEAR_X
        self.pub_muxed.publish(cmd)
        self.pub_manual.publish(cmd)
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
        self.pub_estop.publish(Bool(data=False))
        self.pub_lock.publish(Bool(data=False))
        # 本物の画面（S-21）を開いている端末が居る＝画面由来の上限は v_slow。
        self.pub_screen.publish(ActiveScreen(
            screen_id='S-21', client_id='tablet-working-stop-test',
            interacting=True,
            last_input=self.node.get_clock().now().to_msg()))
        if self.jog:
            self.pub_jog.publish(String(data='working-stop-test'))
            self.pub_panel.publish(String(data='working-stop-test'))

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

    def _publish_fault(self, active, severity, fault_type='LIDAR_LOST'):
        msg = FaultStatus()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.active = bool(active)
        msg.fault_type = fault_type
        msg.description = 'test'
        msg.severity = severity
        self.pub_fault.publish(msg)

    # ── ヘルパー ──
    def _spin(self, duration=0.2):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _call_trigger(self, trigger, arg=''):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = 'test'
        fut = self.cli_trigger.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        assert fut.done(), f'/system/trigger が返らない: {trigger}'
        return fut.result()

    def _next_cid(self):
        type(self)._cid += 1
        return f"ws-{type(self)._cid}"

    def _call_measure(self, command, command_id=None):
        req = MeasureCommand.Request()
        req.command_id = self._next_cid() if command_id is None else command_id
        req.command = command
        fut = self.cli_measure.call_async(req)
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert fut.done(), f'/measure/command が返らない: {command}'
        return fut.result()

    def _latest_state(self):
        self._spin(0.2)
        assert self._states, '/system/state を受信していない'
        return self._states[-1]

    def _latest_status(self):
        self._spin(0.2)
        assert self._statuses, '/measure/status を受信していない'
        return self._statuses[-1]

    def _wait_mode_state(self, mode, state, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self._states and (self._states[-1].mode, self._states[-1].state) == (mode, state):
                return True
        return False

    def _wait_status_working(self, want, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self._statuses and self._statuses[-1].working == want:
                return True
        return False

    def _wait_status_link_ok(self, want, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self._statuses and self._statuses[-1].link_ok == want:
                return True
        return False

    def _drive_to_idle(self):
        """起動直後の INIT/CHECK から IDLE へ出す（connectivity_checker の代役）。"""
        self._publish_fault(False, 'RECOVERABLE')
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

    def _event(self, name):
        msg = StateEvent()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.event = name
        msg.source_node = 'test'
        msg.arg_json = '{}'
        self.pub_event.publish(msg)

    def _ensure_idle(self):
        """前の試験の残りを IDLE に戻す（ESTOP・CARRY を含む）。"""
        self.jog = False
        self._publish_fault(False, 'RECOVERABLE')
        self._spin(0.3)
        st = self._latest_state()
        if (st.mode, st.state) == ('IDLE', 'NONE'):
            return
        if st.mode == 'ESTOP':
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

    def _go_at_panel(self):
        """IDLE → HOME_NAV → AT_HOME → PANEL_NAV → AT_PANEL / IDLE_P。"""
        self._ensure_idle()
        self._hb_version = "1"
        assert self._wait_status_link_ok(True) is True, 'link_ok にならない'
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

    def _ensure_working(self):
        """自分の前提を作る: AT_PANEL/WORKING（停止命令の間）。"""
        st = self._latest_state()
        if (st.mode, st.state) == ('AT_PANEL', 'WORKING'):
            return
        self._go_at_panel()
        res = self._call_trigger('ui.working')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'WORKING')
        assert self._latest_state().working is True

    # ── 試験 ──
    def test_a_meas_done_to_idle_p_with_done_once(self):
        """a. meas.done → IDLE_P、/measure/result に DONE がちょうど 1 回、
        /measure/status.working が true → false。"""
        self._ensure_working()
        assert self._wait_status_working(True) is True
        cid = self._next_cid()
        res = self._call_measure('meas.done', command_id=cid)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AT_PANEL', 'IDLE_P'), (res.mode, res.state)
        assert self._wait_mode_state('AT_PANEL', 'IDLE_P') is True
        assert self._wait_status_working(False) is True
        self._spin(0.5)
        done = [r for r in self._results if r.command_id == cid]
        assert len(done) == 1 and done[0].result == 'DONE', done
        assert self._latest_state().working is False

    def test_b_jog_rejected_in_working_but_moves_in_idle_p(self):
        """b. WORKING のジョグは working_in_progress で拒否、/cmd_vel は 0。
        対照として IDLE_P では同じ入力で /cmd_vel が動く。"""
        self._ensure_working()
        self._cmd_vel.clear()
        self.jog = True
        self._spin(1.5)
        self.jog = False
        st = self._latest_state()
        assert (st.mode, st.state) == ('AT_PANEL', 'WORKING'), (st.mode, st.state)
        assert st.last_reject_reason == 'working_in_progress', st.last_reject_reason
        assert st.jog_active is False
        assert st.speed_limit == 'stop', st.speed_limit
        assert self._cmd_vel and all(
            abs(m.linear.x) < 1e-9 and abs(m.angular.z) < 1e-9
            for m in self._cmd_vel), [m.linear.x for m in self._cmd_vel]
        assert self.status is not None and self.status.out_linear == 0.0, \
            self.status.out_linear if self.status else None

        # 対照: IDLE_P では同じ入力でジョグが通り、/cmd_vel が動く。
        res = self._call_trigger('ui.working')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'IDLE_P') is True
        self._cmd_vel.clear()
        self.jog = True
        assert self._wait_mode_state('AT_PANEL', 'PAUSE') is True
        deadline = time.time() + 5.0
        moved = False
        while time.time() < deadline:
            self._spin(0.1)
            if self._cmd_vel and any(abs(m.linear.x) > 1e-3 for m in self._cmd_vel):
                moved = True
                break
        assert moved, '/cmd_vel が動かない（IDLE_P のジョグが通っていない）'
        st = self._latest_state()
        assert st.speed_limit == 'v_jog_panel', st.speed_limit
        self.jog = False
        assert self._wait_mode_state('AT_PANEL', 'IDLE_P', timeout=10.0) is True

    def test_c_recoverable_fault_stays_in_working_then_ack(self):
        """c. 回復フォルトでも WORKING のまま、そのあとジョグしても /cmd_vel 0。
        解消後の ui.resume_ack で WORKING のまま。"""
        self._ensure_working()
        # safety_monitor が出すのと同じ形（/safety/fault に active + severity）。
        self._publish_fault(True, 'RECOVERABLE')
        self._spin(1.0)
        st = self._latest_state()
        assert (st.mode, st.state) == ('AT_PANEL', 'WORKING'), (st.mode, st.state)
        # そのあとジョグしても動かない（停止命令のまま）。
        self._cmd_vel.clear()
        self.jog = True
        self._spin(1.2)
        self.jog = False
        st = self._latest_state()
        assert (st.mode, st.state) == ('AT_PANEL', 'WORKING'), (st.mode, st.state)
        assert st.jog_active is False
        assert self._cmd_vel and all(abs(m.linear.x) < 1e-9 for m in self._cmd_vel)
        # フォルトを解消してから W-1 の「確認」を送る（WORKING のまま）。
        self._publish_fault(False, 'RECOVERABLE')
        self._spin(0.5)
        res = self._call_trigger('ui.resume_ack')
        assert res.accepted is True, res.reject_reason_key
        st = self._latest_state()
        assert (st.mode, st.state) == ('AT_PANEL', 'WORKING'), (st.mode, st.state)
        assert st.working is True

    def test_d_ui_estop_returns_to_working_and_idle_p_contrast(self):
        """d. WORKING で UI 非常停止 → working が true のまま → 解除 →
        W-1 の ui.resume_yes で WORKING へ。IDLE_P から同じ手順なら IDLE_P へ。"""
        self._ensure_working()
        self.pub_estop_ui.publish(Bool(data=True))
        assert self._wait_mode_state('ESTOP', 'NONE') is True
        self._spin(0.5)
        st = self._latest_state()
        assert st.working is True, 'ESTOP の間 working が外れている'
        assert self._wait_status_working(True) is True
        self.pub_estop_ui.publish(Bool(data=False))
        self._spin(0.5)
        res = self._call_trigger('ui.resume_yes')
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'WORKING') is True
        assert self._latest_state().working is True

        # 対照: IDLE_P から同じ手順 → IDLE_P。
        res = self._call_trigger('ui.working')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'IDLE_P') is True
        assert self._latest_state().working is False
        self.pub_estop_ui.publish(Bool(data=True))
        assert self._wait_mode_state('ESTOP', 'NONE') is True
        self._spin(0.5)
        assert self._latest_state().working is False
        self.pub_estop_ui.publish(Bool(data=False))
        self._spin(0.5)
        res = self._call_trigger('ui.resume_yes')
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'IDLE_P') is True

    def test_e_carry_returns_to_working(self):
        """e. WORKING で手押し（CARRY）→ 解除で WORKING へ戻る。"""
        self._ensure_working()
        self.pub_estop_hw.publish(Bool(data=True))
        assert self._wait_mode_state('CARRY', 'NONE') is True
        self._spin(0.5)
        assert self._latest_state().working is True, 'CARRY の間 working が外れている'
        self.pub_estop_hw.publish(Bool(data=False))
        self._spin(0.5)
        res = self._call_trigger('ui.carry_resume')
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'WORKING') is True
        assert self._latest_state().working is True

    def test_f_link_loss_stays_in_working(self):
        """f. WORKING で生存確認を止める → WORKING のまま（-139 は遷移しない）。"""
        self._ensure_working()
        self._hb_version = None
        assert self._wait_status_link_ok(False) is True, 'link_ok が落ちない'
        self._spin(2.0)
        st = self._latest_state()
        assert (st.mode, st.state) == ('AT_PANEL', 'WORKING'), (st.mode, st.state)
        assert st.working is True

    def test_g_working_off_to_idle_p_and_goto_allowed(self):
        """g. 「停止を解く」（ui.working off）→ IDLE_P、行き先が選べる。"""
        self._ensure_working()
        res = self._call_trigger('ui.working')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'IDLE_P') is True
        assert self._latest_state().working is False
        res = self._call_trigger('ui.goto', {'kind': 'HOME'})
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('HOME_NAV', 'NAV') is True


if __name__ == '__main__':
    unittest.main()
