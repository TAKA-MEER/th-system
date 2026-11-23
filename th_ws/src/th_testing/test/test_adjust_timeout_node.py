"""
test_adjust_timeout_node.py
===========================
8-g P3 — `measure_adjuster` を起動しない構成で、締め切り（adjust_timeout）の
試験。正本は docs/plan/spec/Spec-modes.md SM-3.1.2-137 と
docs/plan/detailed/DetailedDesign-measure.md §6.4。

  i. meas.turn → ADJUST に入るが何もしなくて、adjust_timeout_ms（state_manager
     に渡した値）後に WORKING へ戻り CANCELLED（adjust_timeout）。

起動は本物の `state_manager`・`measure_link`・`measure_clearance`・
`twist_mux`・`obstacle_limiter`（生成 yaml 先渡し）。`measure_adjuster` は
起動しない（締め切りで戻れることだけが目的なので、停止命令だけを縛る）。
"""
import json
import math
import os
import sys
import tempfile
import threading
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
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import (ActiveScreen, MeasureHeartbeat, MeasureResult,
                                MeasureStatus, Pin, PinList, StateEvent,
                                SystemState)
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
        _GEN_DIR = tempfile.mkdtemp(prefix='adjust_timeout_test_gen_')
        out_dir = os.path.join(_GEN_DIR, 'generated')
        pg.run_generation(
            stage=4, sim=False, nodes=list(pg.REGISTRY_NODES), out_dir=out_dir,
            registry_path=os.path.join(
                _THIS_DIR, '..', '..', 'th_params', 'config', 'registry.yaml'),
            calib_dir=os.path.join(_GEN_DIR, 'no_calib'),
            overrides_path=os.path.join(_GEN_DIR, 'no_overrides.yaml'))
    return os.path.join(_GEN_DIR, 'generated', name)


ADJUST_TIMEOUT_MS = 3000

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
        parameters=[_generated_yaml('state_manager.yaml'),
                    {'adjust_timeout_ms': ADJUST_TIMEOUT_MS}],
        output='screen')
    measure_link = launch_ros.actions.Node(
        package='th_measure', executable='measure_link.py', name='measure_link',
        parameters=[_generated_yaml('measure_link.yaml')],
        output='screen')
    measure_clearance = launch_ros.actions.Node(
        package='th_measure', executable='measure_clearance.py',
        name='measure_clearance',
        parameters=[_generated_yaml('measure_clearance.yaml')],
        output='screen')
    lidar_filter = launch_ros.actions.Node(
        package='th_perception', executable='lidar_filter.py', name='lidar_filter',
        parameters=[os.path.join(
            _LAUNCH_DIR, 'config', 'perception_params.yaml'),
            _generated_yaml('lidar_filter.yaml')],
        output='screen')
    limiter = launch_ros.actions.Node(
        package='th_safety', executable='obstacle_limiter', name='obstacle_limiter',
        parameters=[{
            'v_max': 1.0, 'v_slow': 0.35, 'v_jog_panel': 0.12,
            'v_reverse': 0.25, 'obstacle_min_points': 1, 'w_max': 0.6,
        }],
        output='screen')
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
        remappings=[('cmd_vel_out', '/cmd_vel_muxed')],
        output='screen')
    return launch.LaunchDescription([
        tf, lidar_filter, state_manager, measure_link, measure_clearance,
        limiter, twist_mux,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestAdjustTimeoutNode(unittest.TestCase):
    _cid = 0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        n = self.node = rclpy.create_node('test_adjust_timeout_client')
        self._states = []
        self._results = []
        n.create_subscription(SystemState, '/system/state',
                              self._states.append, _LATCHED)
        n.create_subscription(MeasureResult, '/measure/result',
                              self._results.append, 20)
        self.pub_event = n.create_publisher(StateEvent, '/system/event', 10)
        self.pub_pins = n.create_publisher(PinList, '/onsite/pins', _LATCHED)
        self.pub_screen = n.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.pub_estop = n.create_publisher(Bool, '/safety/estop', 1)
        self.pub_lock = n.create_publisher(Bool, '/safety/fault_lock', 1)
        self.pub_estop_hw = n.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_estop_ui = n.create_publisher(Bool, '/safety/estop_ui', 10)
        self.pub_scan = n.create_publisher(LaserScan, '/scan',
                                           qos_profile_sensor_data)
        from nav_msgs.msg import OccupancyGrid
        self.pub_costmap = n.create_publisher(
            OccupancyGrid, '/local_costmap/costmap', _LATCHED)
        self.pub_tf = n.create_publisher(TFMessage, '/tf', 10)
        self.pub_hb = n.create_publisher(MeasureHeartbeat, '/measure/heartbeat', 10)
        self._hb_version = "1"
        self._hb_seq = 0
        self._hb_timer = n.create_timer(0.2, self._publish_hb)
        self.cli_measure = n.create_client(MeasureCommand, '/measure/command')
        self.cli_trigger = n.create_client(UiTrigger, '/system/trigger')
        assert self.cli_measure.wait_for_service(timeout_sec=10.0)
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0)
        self._pub_timer = n.create_timer(0.05, self._publish)
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

    def _publish(self):
        self.pub_estop.publish(Bool(data=False))
        self.pub_lock.publish(Bool(data=False))
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / 360
        scan.angle_max = scan.angle_min + scan.angle_increment * 359
        scan.range_min = 0.05
        scan.range_max = 40.0
        scan.ranges = [30.0] * 360
        self.pub_scan.publish(scan)
        from nav_msgs.msg import OccupancyGrid
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution = 0.05
        m.info.width = 40
        m.info.height = 40
        m.info.origin.position.x = -1.0
        m.info.origin.position.y = -1.0
        m.info.origin.orientation.w = 1.0
        m.data = [0] * (40 * 40)
        self.pub_costmap.publish(m)
        self.pub_screen.publish(ActiveScreen(
            screen_id='S-21', client_id='tablet-adjust-timeout-test',
            interacting=True,
            last_input=self.node.get_clock().now().to_msg()))
        from geometry_msgs.msg import TransformStamped
        # map→odom→base_link の連鎖で出す（base_link に親を 2 つ付けない）。
        transforms = []
        for parent, child in (('map', 'odom'), ('odom', 'base_link')):
            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.pub_tf.publish(TFMessage(transforms=transforms))

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
        return f"adt-{type(self)._cid}"

    def _call_measure(self, command, command_id=None, angle=0.0):
        req = MeasureCommand.Request()
        req.command_id = self._next_cid() if command_id is None else command_id
        req.command = command
        req.angle_rad = float(angle)
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

    def _wait_mode_state(self, mode, state, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self._states and (self._states[-1].mode, self._states[-1].state) == (mode, state):
                return True
        return False

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

    def _event(self, name):
        msg = StateEvent()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.event = name
        msg.source_node = 'test'
        msg.arg_json = '{}'
        self.pub_event.publish(msg)

    def test_i_adjust_timeout_returns_to_working(self):
        """i. ADJUST に入るが何もしない。adjust_timeout_ms 後に WORKING へ戻り
        CANCELLED（adjust_timeout）が出る。"""
        # AT_PANEL/WORKING に出す（test_adjust_node と同じ経路）。
        st = self._latest_state()
        if (st.mode, st.state) != ('AT_PANEL', 'WORKING'):
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
            res = self._call_trigger('ui.working')
            assert res.accepted, res.reject_reason_key
            assert self._wait_mode_state('AT_PANEL', 'WORKING')
        cid = self._next_cid()
        res = self._call_measure('meas.turn', command_id=cid, angle=1.0)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AT_PANEL', 'ADJUST'), (res.mode, res.state)
        start = time.monotonic()
        assert self._wait_mode_state('AT_PANEL', 'WORKING', timeout=20.0) is True
        elapsed = (time.monotonic() - start) * 1000.0
        # 締め切り（3000 ms）の後に戻っていること。戻り先は WORKING。
        assert elapsed + 500.0 >= ADJUST_TIMEOUT_MS, elapsed
        assert self._latest_state().working is True
        end = time.time() + 5.0
        while time.time() < end:
            self._spin(0.1)
            if any(r.command_id == cid for r in self._results):
                break
        cancelled = [r for r in self._results if r.command_id == cid]
        assert len(cancelled) == 1, cancelled
        assert cancelled[0].result == 'CANCELLED', cancelled
        assert cancelled[0].reason == 'adjust_timeout', cancelled


if __name__ == '__main__':
    unittest.main()
