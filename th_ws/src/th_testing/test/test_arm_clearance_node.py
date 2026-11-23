"""
test_arm_clearance_node.py
==========================
8-g P4 — 腕が振れる範囲の確認（`/measure/check_arm_clearance`）を本物の
`state_manager`・`measure_link`・`measure_clearance` で縛る試験。
正本は docs/plan/detailed/DetailedDesign-measure.md §3.5・§6.5、
docs/plan/spec/Spec-modes.md `SM-3.1.2-141`、Spec-measure.md §5.2。

起動は「生成 yaml を先に渡す」本番の経路（test_adjust_node.py と同じ）。
状態は偽の `/system/state` ではなく本物の `state_manager` を
`ui.goto`／`ui.working` で動かして作る。スキャンは `laser_link` で流し、
TF は `base_link`←`laser_link` の静的 TF（+ `odom`→`base_link` を試験側が
出す。test_adjust_node.py と同じ流儀）。

  a. `AT_PANEL`/`WORKING`、左（+y 側 0.6 m）に物 → `LEFT` `BLOCKED`・
     `RIGHT` `CLEAR`・`CURRENT` `CLEAR`（範囲は §3.5 の長方形
     x 0〜1.0・y ±0.275）。問い合わせの前後で `/system/state` が変わらない
  b. `AUTO_MEASURE`/`WAIT` で同じ問い合わせが受かる。
     `AT_PANEL`/`IDLE_P`・`AUTO_MEASURE`/`ADJUST` では `state_not_accepting`
  c. スキャンを止めて `arm_clearance_scan_max_age_ms` を過ぎる →
     全部 `UNKNOWN` `scan_stale`
  d. `blind_angle_ranges` を、右の範囲にだけ 0.65 m（`arm_clearance_blind_near_m`）
     より遠くでかかる値に変える → `RIGHT` `UNKNOWN` `blind_sector`・`LEFT` `CLEAR`
  e. 機体の内側（x 0.1・y 0）にだけ点 → `CURRENT` `CLEAR`
  f. `bad_argument`（頂点 2 つ・`sides` に `UP`）

注意（launch_testing の挙動）: ノードはファイル単位で共有されるため、
メソッド同士の状態に頼らない（各メソッドが自分の前提を作る）。

d の死角の変え方について: 1 ファイルで 1 つの `measure_clearance` を共有し、
生成 yaml に dict を後付けする起動は `a`（`RIGHT` `CLEAR`）と両立しない
（右だけ死角にすると `a` で `RIGHT` が `blind_sector` になる）ため、
`d` は生成 yaml を渡した同じノードの `blind_angle_ranges` を実行時に
書き換えてから問い合わせ、終わったら registry の値に戻す。
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

from geometry_msgs.msg import Point32, Polygon, Pose
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from th_system_msgs.msg import (ActiveScreen, MeasureHeartbeat, MeasureResult,
                                MeasureStatus, Pin, PinList, StateEvent,
                                SystemState)
from th_system_msgs.srv import (CheckArmClearance, MeasureCommand, UiTrigger)

_THIS_DIR = os.path.dirname(__file__)
_LAUNCH_DIR = os.path.abspath(os.path.join(
    _THIS_DIR, '..', '..', 'th_bringup', 'launch'))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
import params_generation as pg  # noqa: E402

_GEN_DIR = None

# registry の blind_angle_ranges（度。平坦配列）。d のあとに戻す値。
_REGISTRY_BLIND_DEG = [-132.8, -117.5, -59.5, -38.9, 45.9, 61.5, 130.3, 143.5]
# 右の範囲だけにかかる値（右は −y 方向 = −90° 付近）。
_RIGHT_BLIND_DEG = [-100.0, -80.0]

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


def _generated_yaml(name):
    global _GEN_DIR
    if _GEN_DIR is None:
        _GEN_DIR = tempfile.mkdtemp(prefix='arm_clearance_test_gen_')
        out_dir = os.path.join(_GEN_DIR, 'generated')
        pg.run_generation(
            stage=4, sim=False, nodes=list(pg.REGISTRY_NODES), out_dir=out_dir,
            registry_path=os.path.join(
                _THIS_DIR, '..', '..', 'th_params', 'config', 'registry.yaml'),
            calib_dir=os.path.join(_GEN_DIR, 'no_calib'),
            overrides_path=os.path.join(_GEN_DIR, 'no_overrides.yaml'))
    return os.path.join(_GEN_DIR, 'generated', name)


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
    limiter = launch_ros.actions.Node(
        package='th_safety', executable='obstacle_limiter', name='obstacle_limiter',
        parameters=[{
            'v_max': 1.0, 'v_slow': V_SLOW, 'v_jog_panel': V_JOG_PANEL,
            'v_reverse': 0.25, 'obstacle_min_points': 1, 'w_max': W_MAX,
        }],
        output='screen')
    return launch.LaunchDescription([
        tf, state_manager, measure_link, measure_clearance, limiter,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestArmClearanceNode(unittest.TestCase):
    _cid = 0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        n = self.node = rclpy.create_node('test_arm_clearance_client')
        self._states = []
        self._statuses = []
        self._results = []
        n.create_subscription(SystemState, '/system/state',
                              self._states.append, _LATCHED)
        n.create_subscription(MeasureStatus, '/measure/status',
                              self._statuses.append, _LATCHED)
        n.create_subscription(MeasureResult, '/measure/result',
                              self._results.append, 20)
        self.pub_event = n.create_publisher(StateEvent, '/system/event', 10)
        self.pub_pins = n.create_publisher(PinList, '/onsite/pins', _LATCHED)
        self.pub_screen = n.create_publisher(ActiveScreen, '/ui/active_screen', 5)
        self.pub_estop = n.create_publisher(Bool, '/safety/estop', 1)
        self.pub_lock = n.create_publisher(Bool, '/safety/fault_lock', 1)
        self.pub_estop_ui = n.create_publisher(Bool, '/safety/estop_ui', 10)
        self.pub_estop_hw = n.create_publisher(Bool, '/safety/estop_hw', 10)
        # /scan_filtered は measure_clearance（と obstacle_limiter）が読む。
        # laser_link 基準で流す（本番の lidar_filter の出し先と同じ）。
        self.pub_scanf = n.create_publisher(LaserScan, '/scan_filtered',
                                            qos_profile_sensor_data)
        self.pub_costmap = n.create_publisher(
            __import__('nav_msgs.msg', fromlist=['OccupancyGrid']).OccupancyGrid,
            '/local_costmap/costmap', _LATCHED)
        self.pub_tf = n.create_publisher(TFMessage, '/tf', 10)
        self.pub_hb = n.create_publisher(MeasureHeartbeat, '/measure/heartbeat', 10)
        self._hb_version = "1"
        self._hb_seq = 0
        self._hb_timer = n.create_timer(0.2, self._publish_hb)
        self.scan_mode = 'clear'
        self.scan_live = True
        self.cli_measure = n.create_client(MeasureCommand, '/measure/command')
        self.cli_trigger = n.create_client(UiTrigger, '/system/trigger')
        self.cli_arm = n.create_client(
            CheckArmClearance, '/measure/check_arm_clearance')
        self.cli_setparams = n.create_client(
            SetParameters, '/measure_clearance/set_parameters')
        assert self.cli_measure.wait_for_service(timeout_sec=10.0)
        assert self.cli_trigger.wait_for_service(timeout_sec=10.0)
        assert self.cli_arm.wait_for_service(timeout_sec=10.0)
        assert self.cli_setparams.wait_for_service(timeout_sec=10.0)
        self._pub_timer = n.create_timer(1.0 / PUB_HZ, self._publish)
        self._spin(0.5)
        self._drive_to_idle()

    def tearDown(self):
        try:
            self._set_blind(_REGISTRY_BLIND_DEG)
        except Exception:
            pass
        self._hb_version = None
        for timer in (self._hb_timer, self._pub_timer):
            if timer is not None:
                timer.cancel()
                self.node.destroy_timer(timer)
        self._hb_timer = self._pub_timer = None
        self.node.destroy_node()

    # ── 試験側の出版（実機と同じトピック・型）──────────────────
    def _publish(self):
        self.pub_estop.publish(Bool(data=False))
        self.pub_lock.publish(Bool(data=False))
        if self.scan_live:
            scan = LaserScan()
            scan.header.stamp = self.node.get_clock().now().to_msg()
            scan.header.frame_id = 'laser_link'
            scan.angle_min = -math.pi
            scan.angle_increment = 2.0 * math.pi / N_BEAMS
            scan.angle_max = scan.angle_min + scan.angle_increment * (N_BEAMS - 1)
            scan.range_min = 0.05
            scan.range_max = 40.0
            ranges = [FAR_M] * N_BEAMS
            mode = getattr(self, "scan_mode", "clear")
            if mode == "left_wall":
                # +y（左）0.6 m に物。角度 +90° のビーム。
                idx = int(round((math.pi / 2.0 + math.pi)
                                / scan.angle_increment))
                ranges[idx] = 0.6
            elif mode == "inside":
                # 機体の内側（x 0.1・y 0）にだけ点。角度 0° のビーム。
                ranges[N_BEAMS // 2] = 0.1
            scan.ranges = ranges
            self.pub_scanf.publish(scan)
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
            screen_id='S-21', client_id='tablet-arm-test',
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
        t.transform.rotation.w = 1.0
        transforms.append(t)
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
        return f"arm-{type(self)._cid}"

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

    def _call_arm(self, points, sides):
        req = CheckArmClearance.Request()
        region = Polygon()
        region.points = [Point32(x=float(x), y=float(y), z=0.0)
                         for x, y in points]
        req.region = region
        req.sides = list(sides)
        fut = self.cli_arm.call_async(req)
        deadline = time.time() + 10.0
        while not fut.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert fut.done(), '/measure/check_arm_clearance が返らない'
        return fut.result()

    def _set_blind(self, flat_deg):
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='blind_angle_ranges',
                      value=[float(v) for v in flat_deg]).to_parameter_msg()]
        fut = self.cli_setparams.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        assert fut.done() and fut.result().results[0].successful, \
            'blind_angle_ranges の書き換えに失敗'

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

    def _wait_link_ok(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if self._statuses and self._statuses[-1].link_ok:
                return
        raise AssertionError(
            '/measure/status の link_ok が立たない（最後=%s）' %
            (self._statuses[-1] if self._statuses else 'なし'))

    def _ensure_working(self):
        self._wait_link_ok()
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
        self._wait_link_ok()
        st = self._latest_state()
        if (st.mode, st.state) == ('AUTO_MEASURE', 'WAIT'):
            self.scan_mode = 'clear'
            return
        self._ensure_idle()
        self.scan_mode = 'clear'
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

    # §3.5 の範囲（回ったあとの base_link 基準で前に伸びる長方形）。
    _RECT = [(0.0, -0.275), (1.0, -0.275), (1.0, 0.275), (0.0, 0.275)]

    # ── 試験 ──
    def test_a_working_left_blocked(self):
        """a. WORKING、左 0.6 m に物 → LEFT BLOCKED・RIGHT CLEAR・CURRENT
        CLEAR。問い合わせの前後で /system/state が変わらない。"""
        self._ensure_working()
        self.scan_mode = 'left_wall'
        self._spin(1.0)  # スキャンが届くのを待つ
        before = (self._latest_state().mode, self._latest_state().state)
        res = self._call_arm(self._RECT, ['LEFT', 'RIGHT', 'CURRENT'])
        assert res.accepted is True, res.reject_reason
        assert res.sides == ['LEFT', 'RIGHT', 'CURRENT'], res.sides
        assert res.results == ['BLOCKED', 'CLEAR', 'CLEAR'], res.results
        assert res.unknown_reasons == ['', '', ''], res.unknown_reasons
        assert res.checked_at.sec > 0, res.checked_at
        after_st = self._latest_state()
        assert (after_st.mode, after_st.state) == before, (before, after_st.mode, after_st.state)
        self.scan_mode = 'clear'

    def test_b_auto_wait_accepted_and_others_rejected(self):
        """b. AUTO_MEASURE/WAIT で受かる。AT_PANEL/IDLE_P と
        AUTO_MEASURE/ADJUST では state_not_accepting。"""
        self._ensure_auto_wait()
        self.scan_mode = 'clear'
        self._spin(0.5)
        res = self._call_arm(self._RECT, ['LEFT'])
        assert res.accepted is True, res.reject_reason
        assert res.results == ['CLEAR'], res.results
        # AT_PANEL/IDLE_P では拒否（配列は空）。
        res = self._call_trigger('ui.finish')
        assert res.accepted is True, res.reject_reason_key
        assert self._wait_mode_state('IDLE', 'NONE')
        self._go_at_panel()
        res = self._call_arm(self._RECT, ['LEFT'])
        assert res.accepted is False, res
        assert res.reject_reason == 'state_not_accepting', res.reject_reason
        assert res.sides == [] and res.results == [] and res.unknown_reasons == []
        # AUTO_MEASURE/ADJUST では拒否。
        self._ensure_auto_wait()
        cid = self._next_cid()
        res2 = self._call_measure('meas.turn', command_id=cid, angle=1.57)
        assert res2.accepted is True, res2.reject_reason
        assert (res2.mode, res2.state) == ('AUTO_MEASURE', 'ADJUST'), (res2.mode, res2.state)
        res = self._call_arm(self._RECT, ['LEFT'])
        assert res.accepted is False, res
        assert res.reject_reason == 'state_not_accepting', res.reject_reason
        assert res.sides == [] and res.results == [] and res.unknown_reasons == []
        # 後始末: meas.stop → PAUSE、meas.abort → WAIT。
        res3 = self._call_measure('meas.stop')
        assert res3.accepted is True, res3.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'PAUSE') is True
        res4 = self._call_measure('meas.abort')
        assert res4.accepted is True, res4.reject_reason
        assert self._wait_mode_state('AUTO_MEASURE', 'WAIT') is True

    def test_c_scan_stale_all_unknown(self):
        """c. スキャンを止めて arm_clearance_scan_max_age_ms を過ぎる →
        全部 UNKNOWN scan_stale。"""
        self._ensure_working()
        self.scan_mode = 'clear'
        self.scan_live = False
        time.sleep(2.0)  # arm_clearance_scan_max_age_ms（500）の十分外
        res = self._call_arm(self._RECT, ['LEFT', 'RIGHT', 'CURRENT'])
        self.scan_live = True
        assert res.accepted is True, res.reject_reason
        assert res.results == ['UNKNOWN', 'UNKNOWN', 'UNKNOWN'], res.results
        assert res.unknown_reasons == ['scan_stale', 'scan_stale', 'scan_stale'], \
            res.unknown_reasons
        # checked_at は判定に使ったスキャンの時刻（最後の受信）。
        assert res.checked_at.sec > 0, res.checked_at

    def test_d_right_blind_sector(self):
        """d. 死角を右の範囲にだけ 0.65 m より遠くでかかる値に変える →
        RIGHT UNKNOWN blind_sector・LEFT CLEAR。"""
        self._ensure_working()
        self.scan_mode = 'clear'
        self._set_blind(_RIGHT_BLIND_DEG)
        self._spin(0.5)
        res = self._call_arm(self._RECT, ['RIGHT', 'LEFT'])
        self._set_blind(_REGISTRY_BLIND_DEG)
        assert res.accepted is True, res.reject_reason
        assert res.results == ['UNKNOWN', 'CLEAR'], res.results
        assert res.unknown_reasons == ['blind_sector', ''], res.unknown_reasons

    def test_e_inside_body_point_is_clear(self):
        """e. 機体の内側（x 0.1・y 0）にだけ点 → CURRENT CLEAR。"""
        self._ensure_working()
        self.scan_mode = 'inside'
        self._spin(1.0)
        res = self._call_arm(self._RECT, ['CURRENT'])
        self.scan_mode = 'clear'
        assert res.accepted is True, res.reject_reason
        assert res.results == ['CLEAR'], res.results
        assert res.unknown_reasons == [''], res.unknown_reasons

    def test_f_bad_argument(self):
        """f. bad_argument: 頂点 2 つ・sides に UP。拒否のとき配列は空。"""
        self._ensure_working()
        self.scan_mode = 'clear'
        res = self._call_arm([(0.0, 0.0), (1.0, 0.0)], ['LEFT'])
        assert res.accepted is False, res
        assert res.reject_reason == 'bad_argument', res.reject_reason
        assert res.sides == [] and res.results == [] and res.unknown_reasons == []
        res = self._call_arm(self._RECT, ['UP'])
        assert res.accepted is False, res
        assert res.reject_reason == 'bad_argument', res.reject_reason
        assert res.sides == [] and res.results == [] and res.unknown_reasons == []


if __name__ == '__main__':
    unittest.main()
