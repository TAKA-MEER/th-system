"""
test_athome_jog_speed_node.py
=============================
Spec-modes.md §6（2026-10-13）: ジョグ中の速度上限は AT_HOME が `v_slow`、AT_PANEL が
`v_jog_panel`。本物の state_manager（`/system/state.speed_limit` を決める）と本物の
obstacle_limiter（それを数値に引いて `/cmd_vel` を絞る）を同じ launch に並べ、
実機と同じ入力（`/ui/active_screen` S-21・`/ui/jog_lease`・`/scan`・
`/cmd_vel_muxed`・`/cmd_vel_manual`）だけを外から与えて、出力側（SystemState と
LimiterStatus.applied_limit_mps）で見る。

純関数（test_zones.py）が固くても、attributes.yaml の行から zones.mode_speed_limit() を
経て state_manager が /system/state に載せ、limiter が受ける、までの数行は別に縛る。

  a. AT_HOME・ジョグ中 : speed_limit == v_slow、applied_limit == V_SLOW
  b. AT_HOME・ジョグなし: speed_limit == stop（従来どおり止まる）
  c. AT_PANEL・ジョグ中 : speed_limit == v_jog_panel、applied_limit == V_JOG_PANEL

limiter の v_slow / v_jog_panel はあえて既定と違う値（0.35 / 0.12）にして、
取り違えが数値で分かるようにする。
"""
import json
import math
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

from th_system_msgs.msg import (ActiveScreen, FaultStatus, LimiterStatus, Pin, PinList,
                                StateEvent, SystemState)
from th_system_msgs.srv import UiTrigger

V_SLOW = 0.35
V_JOG_PANEL = 0.12
CMD_LINEAR_X = 0.8
FAR_M = 30.0
N_BEAMS = 360
PUB_HZ = 20.0
JOG_LEASE_MS = 600

_LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
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
        parameters=[{
            'jog_lease_ms': JOG_LEASE_MS,
            'link_wait_timeout_ms': 60_000,
            'ui_active_window_s': 5,
            'screen_stale_ms': 60_000,
        }],
        output='screen')
    limiter = launch_ros.actions.Node(
        package='th_safety', executable='obstacle_limiter', name='obstacle_limiter',
        parameters=[{
            'v_max': 1.0, 'v_slow': V_SLOW, 'v_jog_panel': V_JOG_PANEL,
            'v_reverse': 0.25, 'obstacle_min_points': 1,
        }],
        output='screen')
    return launch.LaunchDescription([
        tf, state_manager, limiter, launch_testing.actions.ReadyToTest(),
    ]), {}


class TestAtHomeJogSpeedNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        n = self.node = rclpy.create_node('test_athome_jog_speed_client')
        self.hist = []
        self.status = None
        n.create_subscription(SystemState, '/system/state', self.hist.append, _LATCHED)
        n.create_subscription(LimiterStatus, '/safety/limiter_status',
                              lambda m: setattr(self, 'status', m), qos_profile_sensor_data)
        self.pub_event = n.create_publisher(StateEvent, '/system/event', 10)
        self.pub_pins = n.create_publisher(PinList, '/onsite/pins', _LATCHED)
        self.pub_fault = n.create_publisher(FaultStatus, '/safety/fault', 5)
        self.pub_hw = n.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_screen = n.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.pub_jog = n.create_publisher(String, '/ui/jog_lease', 1)
        self.pub_muxed = n.create_publisher(Twist, '/cmd_vel_muxed', 1)
        self.pub_manual = n.create_publisher(Twist, '/cmd_vel_manual', 1)
        self.pub_scan = n.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_estop = n.create_publisher(Bool, '/safety/estop', 1)
        self.pub_lock = n.create_publisher(Bool, '/safety/fault_lock', 1)
        self.cli_trigger = n.create_client(UiTrigger, '/system/trigger')
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0), 'state_manager が起動していない'
        self.jog = False
        self.timer = n.create_timer(1.0 / PUB_HZ, self._publish)
        self._spin(0.5)
        self.pub_fault.publish(FaultStatus(active=False, fault_type='', severity=''))
        self.pub_hw.publish(Bool(data=False))
        self._spin(0.3)
        self._trigger('ui.finish')
        self._wait_mode('IDLE')

    def tearDown(self):
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
        # 場内の画面（S-21）を開いている端末が居る＝画面由来の上限は v_slow。
        self.pub_screen.publish(ActiveScreen(
            screen_id='S-21', client_id='tablet-athome-test', interacting=True,
            last_input=self.node.get_clock().now().to_msg()))
        if self.jog:
            self.pub_jog.publish(String(data='athome-test'))

    def _spin(self, duration=0.2):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _trigger(self, trigger, arg=None):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = json.dumps(arg) if arg else ''
        req.requester = 'test'
        fut = self.cli_trigger.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        return fut.result()

    def _wait_mode(self, mode, state=None, timeout=4.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self.hist and self.hist[-1].mode == mode and (
                    state is None or self.hist[-1].state == state):
                return True
        return False

    def _event(self, name):
        self.pub_event.publish(StateEvent(event=name, source_node='test', arg_json=''))

    def _go_at_home(self):
        res = self._trigger('ui.goto', {'kind': 'HOME'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('HOME_NAV')
        self._event('evt.arrived')
        assert self._wait_mode('AT_HOME', 'IDLE_H')

    def _go_at_panel(self):
        self._go_at_home()
        self.pub_pins.publish(PinList(pins=[Pin(id='p1', name='panel', kind='PANEL', pose=Pose())]))
        self._spin(0.4)
        res = self._trigger('ui.goto', {'kind': 'PANEL', 'pin_id': 'p1'})
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode('PANEL_NAV')
        self._event('evt.arrived')
        assert self._wait_mode('PANEL_NAV', 'ALIGN')
        self._event('evt.align_done')
        assert self._wait_mode('AT_PANEL', 'IDLE_P')

    def _jog_on(self, mode):
        self.jog = True
        assert self._wait_mode(mode, 'PAUSE'), 'ジョグで PAUSE に入らない'
        assert self.hist[-1].jog_active

    def _applied(self, expected, timeout=5.0):
        """limiter が返す applied_limit_mps が expected で落ち着くまで見る。"""
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            s = self.status
            if s is not None and s.action != 'STOP' and abs(s.applied_limit_mps - expected) < 1e-3:
                self._spin(0.5)
                return abs(self.status.applied_limit_mps - expected) < 1e-3
        return False

    # ── 試験 ──
    def test_a_at_home_jog_is_v_slow(self):
        self._go_at_home()
        self._jog_on('AT_HOME')
        self._spin(0.5)
        assert self.hist[-1].speed_limit == 'v_slow', self.hist[-1].speed_limit
        assert self._applied(V_SLOW), (
            f'AT_HOME のジョグ上限が v_slow({V_SLOW}) にならない: '
            f'{self.status.applied_limit_mps if self.status else None}')
        assert abs(self.status.out_linear - V_SLOW) < 1e-2, self.status.out_linear

    def test_b_at_home_without_jog_stays_stop(self):
        self._go_at_home()
        self.jog = False
        self._spin(1.0)
        snap = self.hist[-1]
        assert (snap.mode, snap.state) == ('AT_HOME', 'IDLE_H')
        assert snap.speed_limit == 'stop', snap.speed_limit
        assert self._applied(0.0)

    def test_c_at_panel_jog_is_v_jog_panel(self):
        self._go_at_panel()
        self._jog_on('AT_PANEL')
        self._spin(0.5)
        assert self.hist[-1].speed_limit == 'v_jog_panel', self.hist[-1].speed_limit
        assert self._applied(V_JOG_PANEL), (
            f'AT_PANEL のジョグ上限が v_jog_panel({V_JOG_PANEL}) にならない: '
            f'{self.status.applied_limit_mps if self.status else None}')
        assert abs(self.status.out_linear - V_JOG_PANEL) < 1e-2, self.status.out_linear


if __name__ == '__main__':
    unittest.main()
