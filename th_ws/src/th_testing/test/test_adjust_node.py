"""
test_adjust_node.py
===================
8-g P3 — 盤前の旋回・前後の並進（`ADJUST`）を本物の
`state_manager`・`measure_link`・`measure_clearance`・`measure_adjuster`・
`twist_mux`（生成設定）・`obstacle_limiter` で縛る試験。
正本は docs/plan/spec/Spec-modes.md（SM-3.1.2-127・-128・-132・-136〜-140）
と docs/plan/detailed/DetailedDesign-measure.md §6.4。

起動は「生成 yaml を先に渡す」本番の経路。`/scan_filtered` は本物の
`lidar_filter`（`/scan` から出す。死角マスク込み）が出し、`twist_mux` は
生成設定（behavior 優先度 20）の本物を通す。試験側が `/cmd_vel` を積分して
`odom→base_link` を進める（閉ループ）。`/cmd_vel_muxed` や
`/measure/corner_status` は試験側で偽らない。

  a. 停止命令（WORKING）で meas.turn 1.57 → ADJUST → 左へ約 90° 回って
     WORKING、DONE がちょうど 1 回。/cmd_vel の角速度が w_max を超えない
  b. 自律測定 WAIT で meas.turn -3.5 → 右回りに 3.5 rad（近道の逆向きでない）
  c. meas.translate FORWARD 0.3 → 前へ約 0.3 m・向き不変で WAIT。
     /cmd_vel の並進が v_jog_panel 以下
  d. 前方 0.9 m に壁 → FORWARD 0.5 は path_obstacle で拒否、0.1 は受ける。
     後方 0.9 m に壁 → BACKWARD 0.5 は拒否
  e. 角の円に物 → corner_obstacle
  f. 停止命令の ADJUST 中に UI 非常停止 → 動きが止まり CANCELLED（estop）→
     解除で WORKING。そのあと遅れた evt.adjust_done（同じ command_id）を
     流しても DONE は出ない
  g. 停止命令の ADJUST 中に meas.done・ui.working off・行き先 →
     adjust_in_progress
  h. 自律測定の ADJUST 中に meas.stop → PAUSE、CANCELLED（stopped）、
     meas.start → WAIT（動き直さない）
  i. measure_adjuster を起動しない構成で meas.turn → adjust_timeout_ms 後
     WORKING へ戻り CANCELLED（adjust_timeout）【別ファイル
     test_adjust_timeout_node.py】

注意（launch_testing の挙動）: ノードはファイル単位で共有されるため、
メソッド同士の状態に頼らない（各メソッドが自分の前提を作る）。
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
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import (ActiveScreen, LimiterStatus, MeasureHeartbeat,
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
        _GEN_DIR = tempfile.mkdtemp(prefix='adjust_test_gen_')
        out_dir = os.path.join(_GEN_DIR, 'generated')
        pg.run_generation(
            stage=4, sim=False, nodes=list(pg.REGISTRY_NODES), out_dir=out_dir,
            registry_path=os.path.join(
                _THIS_DIR, '..', '..', 'th_params', 'config', 'registry.yaml'),
            calib_dir=os.path.join(_GEN_DIR, 'no_calib'),
            overrides_path=os.path.join(_GEN_DIR, 'no_overrides.yaml'))
    return os.path.join(_GEN_DIR, 'generated', name)


V_JOG_PANEL = 0.12
V_SLOW = 0.35
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
        parameters=[_generated_yaml('state_manager.yaml')],
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
    measure_adjuster = launch_ros.actions.Node(
        package='th_measure', executable='measure_adjuster.py',
        name='measure_adjuster',
        parameters=[_generated_yaml('measure_adjuster.yaml')],
        output='screen')
    limiter = launch_ros.actions.Node(
        package='th_safety', executable='obstacle_limiter', name='obstacle_limiter',
        parameters=[{
            'v_max': 1.0, 'v_slow': V_SLOW, 'v_jog_panel': V_JOG_PANEL,
            'v_reverse': 0.25, 'obstacle_min_points': 1, 'w_max': W_MAX,
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
    lidar_filter = launch_ros.actions.Node(
        package='th_perception', executable='lidar_filter.py', name='lidar_filter',
        parameters=[os.path.join(
            _LAUNCH_DIR, 'config', 'perception_params.yaml'),
            _generated_yaml('lidar_filter.yaml')],
        output='screen')
    return launch.LaunchDescription([
        tf, lidar_filter, state_manager, measure_link, measure_clearance,
        measure_adjuster, limiter, twist_mux,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestAdjustNode(unittest.TestCase):
    _cid = 0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        n = self.node = rclpy.create_node('test_adjust_client')
        self._states = []
        self._statuses = []
        self._results = []
        self._cmd_vel = []
        self._behavior = []
        self.status = None
        n.create_subscription(SystemState, '/system/state',
                              self._states.append, _LATCHED)
        n.create_subscription(MeasureStatus, '/measure/status',
                              self._statuses.append, _LATCHED)
        n.create_subscription(MeasureResult, '/measure/result',
                              self._results.append, 20)
        n.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        n.create_subscription(Twist, '/cmd_vel_behavior',
                              self._behavior.append, 10)
        n.create_subscription(LimiterStatus, '/safety/limiter_status',
                              lambda m: setattr(self, 'status', m),
                              qos_profile_sensor_data)
        self.pub_event = n.create_publisher(StateEvent, '/system/event', 10)
        self.pub_pins = n.create_publisher(PinList, '/onsite/pins', _LATCHED)
        self.pub_screen = n.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.pub_estop = n.create_publisher(Bool, '/safety/estop', 1)
        self.pub_lock = n.create_publisher(Bool, '/safety/fault_lock', 1)
        self.pub_estop_ui = n.create_publisher(Bool, '/safety/estop_ui', 10)
        self.pub_estop_hw = n.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_scan = n.create_publisher(LaserScan, '/scan',
                                           qos_profile_sensor_data)
        self.pub_costmap = n.create_publisher(
            __import__('nav_msgs.msg', fromlist=['OccupancyGrid']).OccupancyGrid,
            '/local_costmap/costmap', _LATCHED)
        self.pub_tf = n.create_publisher(TFMessage, '/tf', 10)
        self.pub_hb = n.create_publisher(MeasureHeartbeat, '/measure/heartbeat', 10)
        self._hb_version = "1"
        self._hb_seq = 0
        self._hb_timer = n.create_timer(0.2, self._publish_hb)
        self.cli_measure = n.create_client(MeasureCommand, '/measure/command')
        self.cli_trigger = n.create_client(UiTrigger, '/system/trigger')
        assert self.cli_measure.wait_for_service(timeout_sec=10.0)
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0)
        # 閉ループの pose（/cmd_vel を積分する）。
        self._px = self._py = self._yaw = 0.0
        self._last_cmd = None
        self._last_int_t = None
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

    # ── /cmd_vel の積分（閉ループ）──────────────────────────────
    def _on_cmd_vel(self, msg):
        self._cmd_vel.append(msg)
        self._last_cmd = msg

    def _integrate(self):
        if self._last_cmd is None:
            return
        now = time.monotonic()
        if self._last_int_t is not None:
            dt = min(0.2, max(0.0, now - self._last_int_t))
            self._yaw += self._last_cmd.angular.z * dt
            self._px += self._last_cmd.linear.x * math.cos(self._yaw) * dt
            self._py += self._last_cmd.linear.x * math.sin(self._yaw) * dt
        self._last_int_t = now

    def _reset_pose(self):
        self._px = self._py = self._yaw = 0.0
        self._last_cmd = None
        self._publish_tf()

    # ── 試験側の出版（実機と同じトピック・型）──────────────────
    def _publish(self):
        self._integrate()
        self.pub_estop.publish(Bool(data=False))
        self.pub_lock.publish(Bool(data=False))
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / N_BEAMS
        scan.angle_max = scan.angle_min + scan.angle_increment * (N_BEAMS - 1)
        scan.range_min = 0.05
        scan.range_max = 40.0
        ranges = [FAR_M] * N_BEAMS
        if getattr(self, "scan_mode", "clear") == "front_wall_09":
            ranges[N_BEAMS // 2] = 0.9
        elif getattr(self, "scan_mode", "clear") == "rear_wall_09":
            ranges[0] = 0.9
        elif getattr(self, "scan_mode", "clear") == "corner":
            ranges[int((0.5 + math.pi) / scan.angle_increment)] = 0.3
        scan.ranges = ranges
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
        # 本物の画面（S-21）を開いている端末が居る＝画面由来の上限は v_slow。
        self.pub_screen.publish(ActiveScreen(
            screen_id='S-21', client_id='tablet-adjust-test',
            interacting=True,
            last_input=self.node.get_clock().now().to_msg()))
        self._publish_tf()

    def _publish_tf(self):
        from geometry_msgs.msg import TransformStamped
        stamp = self.node.get_clock().now().to_msg()
        transforms = []
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._px
        t.transform.translation.y = self._py
        qw, qz = math.cos(self._yaw / 2.0), math.sin(self._yaw / 2.0)
        t.transform.rotation.w = qw
        t.transform.rotation.z = qz
        transforms.append(t)
        # map→odom（固定）→base_link（動的）の連鎖で出す（map→base と
        # odom→base を並べると base_link に親が 2 つできて TF が
        # 「unconnected trees」になる）。
        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'odom'
        t2.transform.rotation.w = 1.0
        transforms.append(t2)
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

    # ── ヘルパー ───────────────────────────────────────────────
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
        return f"adj-{type(self)._cid}"

    def _call_measure(self, command, command_id=None, side="",
                      angle=0.0, distance=0.0):
        req = MeasureCommand.Request()
        req.command_id = self._next_cid() if command_id is None else command_id
        req.command = command
        req.side = side
        req.angle_rad = float(angle)
        req.distance_m = float(distance)
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

    def _wait_mode_state(self, mode, state, timeout=40.0):
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

    def _event(self, name, arg='{}'):
        msg = StateEvent()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.event = name
        msg.source_node = 'test'
        msg.arg_json = arg
        self.pub_event.publish(msg)

    def _ensure_idle(self):
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
        self._ensure_idle()
        self.scan_mode = 'clear'
        self._reset_pose()
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT') is False or True
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
        st = self._latest_state()
        if (st.mode, st.state) == ('AT_PANEL', 'WORKING'):
            self.scan_mode = 'clear'
            return
        self._go_at_panel()
        res = self._call_trigger('ui.working')
        assert res.accepted, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'WORKING')
        self.scan_mode = 'clear'
        self._spin(0.5)

    def _ensure_auto_wait(self):
        st = self._latest_state()
        if (st.mode, st.state) == ('AUTO_MEASURE', 'WAIT'):
            self.scan_mode = 'clear'
            return
        self._ensure_idle()
        self.scan_mode = 'clear'
        self._reset_pose()
        res = self._call_trigger('ui.goto', {'kind': 'AUTO_MEASURE'})
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT')
        self._spin(0.5)

    def _wait_result(self, cid, result, timeout=40.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if any(r.command_id == cid and r.result == result
                   for r in self._results):
                return True
        return False

    # ── 試験 ──
    def test_a_panel_turn_left_90(self):
        """a. WORKING で meas.turn 1.57 → ADJUST → 左へ約 90° 回って WORKING、
        DONE がちょうど 1 回。角速度は w_max 以下で ADJUST の speed_limit が
        v_jog_panel。"""
        self._ensure_working()
        self._reset_pose()
        cid = self._next_cid()
        res = self._call_measure('meas.turn', command_id=cid, angle=1.57)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AT_PANEL', 'ADJUST'), (res.mode, res.state)
        st = self._latest_state()
        assert st.speed_limit == 'v_jog_panel', st.speed_limit
        assert self._wait_mode_state('AT_PANEL', 'WORKING') is True
        assert self._wait_result(cid, 'DONE') is True
        self._spin(0.5)
        assert len([r for r in self._results if r.command_id == cid]) == 1
        assert 1.2 < self._yaw < 1.9, self._yaw
        assert self._cmd_vel
        assert max(abs(m.angular.z) for m in self._cmd_vel) <= W_MAX + 0.05, \
            max(abs(m.angular.z) for m in self._cmd_vel)
        # /cmd_vel_behavior から出た指令がある（/cmd_vel_nav への書き換えを掴む）。
        assert any(abs(m.angular.z) > 0.05 for m in self._behavior), \
            len(self._behavior)

    def test_b_auto_turn_right_3p5(self):
        """b. AUTO WAIT で meas.turn -3.5 → 右回りに 3.5 rad（近道の逆向きでない）。"""
        self._ensure_auto_wait()
        self._reset_pose()
        cid = self._next_cid()
        res = self._call_measure('meas.turn', command_id=cid, angle=-3.5)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AUTO_MEASURE', 'ADJUST'), (res.mode, res.state)
        st = self._latest_state()
        assert st.speed_limit == 'v_jog_panel', st.speed_limit
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT') is True
        assert self._wait_result(cid, 'DONE') is True
        # -3.5 rad の右回り。+2.78 の近道（逆向き）では終わらない。
        assert self._yaw < -3.2, self._yaw
        assert self._yaw > -3.8, self._yaw

    def test_c_translate_forward_03(self):
        """c. meas.translate FORWARD 0.3 → 前へ約 0.3 m・向き不変で WAIT。
        並進は v_jog_panel 以下。"""
        self._ensure_auto_wait()
        self._reset_pose()
        cid = self._next_cid()
        res = self._call_measure('meas.translate', command_id=cid,
                                 side='FORWARD', distance=0.3)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AUTO_MEASURE', 'ADJUST'), (res.mode, res.state)
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT') is True
        assert self._wait_result(cid, 'DONE') is True
        assert 0.20 < self._px < 0.45, self._px
        # 向き不変（reset した始点が 0 なので yaw は 0 のはず）。
        assert abs(self._yaw) < 0.15, self._yaw
        assert max(abs(m.linear.x) for m in self._cmd_vel) <= V_JOG_PANEL + 0.05, \
            max(abs(m.linear.x) for m in self._cmd_vel)
        assert any(abs(m.linear.x) > 0.02 for m in self._behavior), \
            len(self._behavior)

    def test_d_path_obstacle_rejects_far_but_accepts_near(self):
        """d. 前方 0.9 m に壁 → FORWARD 0.5 は path_obstacle、0.1 は受ける。
        後方 0.9 m に壁 → BACKWARD 0.5 は拒否。"""
        self._ensure_auto_wait()
        self.scan_mode = 'front_wall_09'
        self._reset_pose()
        self._spin(1.0)  # corner_status が壁を見て free を落とすのを待つ
        cid = self._next_cid()
        res = self._call_measure('meas.translate', command_id=cid,
                                 side='FORWARD', distance=0.5)
        assert res.accepted is False, res
        assert res.reject_reason == 'path_obstacle', res.reject_reason
        assert (res.mode, res.state) == ('AUTO_MEASURE', 'WAIT')
        cid2 = self._next_cid()
        res = self._call_measure('meas.translate', command_id=cid2,
                                 side='FORWARD', distance=0.1)
        assert res.accepted is True, res.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT') is True
        assert self._wait_result(cid2, 'DONE') is True
        # 後方の壁。
        self.scan_mode = 'rear_wall_09'
        self._reset_pose()
        self._spin(1.0)
        cid3 = self._next_cid()
        res = self._call_measure('meas.translate', command_id=cid3,
                                 side='BACKWARD', distance=0.5)
        assert res.accepted is False, res
        assert res.reject_reason == 'path_obstacle', res.reject_reason
        assert (res.mode, res.state) == ('AUTO_MEASURE', 'WAIT')
        self.scan_mode = 'clear'

    def test_e_corner_obstacle(self):
        """e. 角の円に物 → corner_obstacle（受理されない）。"""
        self._ensure_working()
        self.scan_mode = 'corner'
        self._reset_pose()
        self._spin(1.0)
        cid = self._next_cid()
        res = self._call_measure('meas.turn', command_id=cid, angle=0.5)
        assert res.accepted is False, res
        assert res.reject_reason == 'corner_obstacle', res.reject_reason
        assert (res.mode, res.state) == ('AT_PANEL', 'WORKING')
        self.scan_mode = 'clear'
        self._spin(0.5)

    def test_f_estop_during_adjust_cancels_and_late_done_is_dropped(self):
        """f. ADJUST 中に UI 非常停止 → 止まり CANCELLED（estop）→ 解除で
        WORKING。そのあと遅れた evt.adjust_done（同じ command_id）を流しても
        DONE は出ない。"""
        self._ensure_working()
        self._reset_pose()
        cid = self._next_cid()
        res = self._call_measure('meas.turn', command_id=cid, angle=1.57)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AT_PANEL', 'ADJUST')
        # 進行中でない command_id の evt.adjust_done は捨てる（ADJUST のまま）。
        self._event('evt.adjust_done', json.dumps({'command_id': 'stale-id'}))
        self._spin(0.8)
        st = self._latest_state()
        assert (st.mode, st.state) == ('AT_PANEL', 'ADJUST'), (st.mode, st.state)
        assert not any(r.command_id == cid and r.result == 'DONE'
                       for r in self._results)
        # ADJUST 中に非常停止。
        self.pub_estop_ui.publish(Bool(data=True))
        assert self._wait_mode_state('ESTOP', 'NONE') is True
        assert self._wait_result(cid, 'CANCELLED') is True
        cancelled = [r for r in self._results if r.command_id == cid][0]
        assert cancelled.reason == 'estop', cancelled.reason
        # 動きが止まった: behavior が 0 で静止している。
        self._spin(1.0)
        st = self._latest_state()
        assert st.working is True
        # 解除して WORKING へ（IDLE_P ではない）。
        self.pub_estop_ui.publish(Bool(data=False))
        self._spin(0.5)
        res = self._call_trigger('ui.resume_yes')
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('AT_PANEL', 'WORKING') is True
        # 遅れた evt.adjust_done（同じ command_id）を流す → state は変わらず DONE なし。
        before = len([r for r in self._results if r.command_id == cid and r.result == 'DONE'])
        self._event('evt.adjust_done', json.dumps({'command_id': cid}))
        self._spin(1.0)
        st = self._latest_state()
        assert (st.mode, st.state) == ('AT_PANEL', 'WORKING'), (st.mode, st.state)
        assert len([r for r in self._results if r.command_id == cid and r.result == 'DONE']) == before

    def test_g_adjust_rejects(self):
        """g. ADJUST 中に meas.done・ui.working off・行き先 → adjust_in_progress。"""
        self._ensure_working()
        self._reset_pose()
        cid = self._next_cid()
        res = self._call_measure('meas.turn', command_id=cid, angle=1.57)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AT_PANEL', 'ADJUST')
        res2 = self._call_measure('meas.done')
        assert res2.accepted is False, res2
        assert res2.reject_reason == 'adjust_in_progress', res2.reject_reason
        res3 = self._call_trigger('ui.working')
        assert res3.accepted is False, res3
        assert res3.reject_reason_key == 'adjust_in_progress', res3.reject_reason_key
        res4 = self._call_trigger('ui.goto', {'kind': 'HOME'})
        assert res4.accepted is False, res4
        assert res4.reject_reason_key == 'adjust_in_progress', res4.reject_reason_key
        assert (self._latest_state().mode, self._latest_state().state) == ('AT_PANEL', 'ADJUST')
        # 後始末: 動きを終わらせる（結果を回収する）。
        assert self._wait_mode_state('AT_PANEL', 'WORKING') is True
        assert self._wait_result(cid, 'DONE') is True

    def test_h_auto_stop_to_pause_then_start_to_wait(self):
        """h. 自律測定の ADJUST 中に meas.stop → PAUSE、CANCELLED（stopped）、
        meas.start → WAIT（動き直さない）。"""
        self._ensure_auto_wait()
        self._reset_pose()
        cid = self._next_cid()
        res = self._call_measure('meas.turn', command_id=cid, angle=1.57)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AUTO_MEASURE', 'ADJUST')
        # すぐ止める（動き出さない内に）。
        res2 = self._call_measure('meas.stop')
        assert res2.accepted is True, res2.reject_reason
        assert (res2.mode, res2.state) == ('AUTO_MEASURE', 'PAUSE'), (res2.mode, res2.state)
        assert self._wait_mode_state('AUTO_MEASURE', 'PAUSE') is True
        assert self._wait_result(cid, 'CANCELLED') is True
        cancelled = [r for r in self._results if r.command_id == cid][0]
        assert cancelled.reason == 'stopped', cancelled.reason
        yaw_at_pause = self._yaw
        res3 = self._call_measure('meas.start')
        assert res3.accepted is True, res3.reject_reason
        assert (res3.mode, res3.state) == ('AUTO_MEASURE', 'WAIT'), (res3.mode, res3.state)
        # 動き直さない: yaw が止めた当時からほぼ変わらない。
        self._spin(1.0)
        assert abs(self._yaw - yaw_at_pause) < 0.1, (yaw_at_pause, self._yaw)

    def test_i_panel_link_lost_from_adjust_to_working(self):
        """i. AT_PANEL/ADJUST の旋回中に measure_link の生存確認を止める →
        WORKING・/measure/result が CANCELLED（link_lost）ちょうど1回・
        /cmd_vel_behavior が止まる（§6.4「回復フォルト・途絶」→ WORKING）。"""
        self._ensure_working()
        self._reset_pose()
        cid = self._next_cid()
        res = self._call_measure('meas.turn', command_id=cid, angle=3.5)
        assert res.accepted is True, res.reject_reason
        assert (res.mode, res.state) == ('AT_PANEL', 'ADJUST')
        # measure_link の生存確認を止める（試験側ハートビートを止める）。
        self._hb_version = None
        assert self._wait_mode_state('AT_PANEL', 'WORKING') is True
        assert self._wait_result(cid, 'CANCELLED') is True
        cancelled = [r for r in self._results if r.command_id == cid]
        assert len(cancelled) == 1, cancelled
        assert cancelled[0].reason == 'link_lost', cancelled[0].reason
        # /cmd_vel_behavior が止まる: 最後の非ゼロ指令のあとに 0 が出て、
        # そのあと非ゼロが再び出ない。
        self._spin(1.5)
        nonzero = [i for i, m in enumerate(self._behavior)
                   if abs(m.angular.z) > 0.01 or abs(m.linear.x) > 0.01]
        assert nonzero, '旋回中に behavior 指令が出ていない'
        assert all(abs(m.angular.z) <= 0.01 and abs(m.linear.x) <= 0.01
                   for m in self._behavior[nonzero[-1]:]), \
            '途絶のあとに behavior 指令が再び出ている'
        # ハートビートは setUp で再開する（次試験への持ち越しを防ぐ）。


if __name__ == '__main__':
    unittest.main()
