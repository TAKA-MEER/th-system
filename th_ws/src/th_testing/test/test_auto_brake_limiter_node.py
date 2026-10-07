"""
test_auto_brake_limiter_node.py
===============================
1b-15 SG-B10: 手動走行の自動ブレーキ ON/OFF を、obstacle_limiter（C++）の
ノード本体を実際に起動して縛る launch_testing。

純関数（gtest の ObstacleLimiterCoreAutoBrake）が固くても、/system/state.auto_brake の
購読・鮮度・LimiterStatus への出力は呼び出し側の数行で壊れうる。

前方 0.6 m に障害物を置き、手動走行（MANUAL・ジョイ新鮮）が 0.8 m/s を指令する。
  a. ON  → 減速する（v_allow）。接近警告は出ない
  b. OFF → 減速しない（0.8 のまま）。接近警告が出る
  c. OFF でも /system/state が途絶えたら止まる（ON に倒れる）。警告は出ない
  d. OFF でも非常停止（/safety/estop）は 0
  e. OFF でもフォルトロック（/safety/fault_lock）は 0
  f. OFF でも自律系（FOLLOW）は減速する・警告は出ない（無効化不可）
  g. OFF でもゾーン NA は減速する・警告は出ない
  h. OFF で障害物が遠いと警告は出ない

注意（CLAUDE.md の既知の癖）: 試験は TestCase のメソッドにする。
メソッドは名前順に走り、前の試験の状態を引き継ぐので、各メソッドは冒頭で自分の前提を作る。
"""
import math
import time
import unittest

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                        qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import LimiterStatus, SystemState


V_MAX = 1.0
CMD_LINEAR_X = 0.8
FLOOR_M = 0.4
BRAKE_ACCEL = 1.0
NEAR_M = 0.6
FAR_M = 30.0
V_ALLOW_NEAR = math.sqrt(2.0 * BRAKE_ACCEL * (NEAR_M - FLOOR_M))   # 約 0.632
PUB_HZ = 20.0
N_BEAMS = 360


@pytest.mark.launch_test
def generate_test_description():
    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen',
    )
    limiter = launch_ros.actions.Node(
        package='th_safety',
        executable='obstacle_limiter',
        name='obstacle_limiter',
        parameters=[{
            'obstacle_floor_distance_m': FLOOR_M,
            'hysteresis_band_m': 0.1,
            'brake_accel_mps2': BRAKE_ACCEL,
            'obstacle_min_points': 1,
            'v_max': V_MAX,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        tf, limiter,
        launch_testing.actions.ReadyToTest(),
    ]), {'obstacle_limiter': limiter}


class TestAutoBrakeLimiterNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_auto_brake_limiter_node')
        n = cls.node
        cls.status = None
        n.create_subscription(LimiterStatus, '/safety/limiter_status',
                              lambda m: setattr(cls, 'status', m), qos_profile_sensor_data)
        cls.pub_muxed = n.create_publisher(Twist, '/cmd_vel_muxed', 1)
        cls.pub_manual = n.create_publisher(Twist, '/cmd_vel_manual', 1)
        cls.pub_scan = n.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.pub_state = n.create_publisher(SystemState, '/system/state', latched)
        cls.pub_estop = n.create_publisher(Bool, '/safety/estop', 1)
        cls.pub_lock = n.create_publisher(Bool, '/safety/fault_lock', 1)

        cls._reset_inputs()
        cls._timer = n.create_timer(1.0 / PUB_HZ, cls._publish)

    @classmethod
    def _reset_inputs(cls):
        cls.mode = 'MANUAL'
        cls.zone = 'OUT'
        cls.auto_brake = True
        cls.obstacle_m = NEAR_M
        cls.estop = False
        cls.fault_lock = False
        cls.publish_state = True

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        type(self)._reset_inputs()

    # ── 送信 ──────────────────────────────────────────────
    @classmethod
    def _publish(cls):
        cmd = Twist()
        cmd.linear.x = CMD_LINEAR_X
        cls.pub_muxed.publish(cmd)
        cls.pub_manual.publish(cmd)

        scan = LaserScan()
        scan.header.stamp = cls.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / N_BEAMS
        scan.angle_max = scan.angle_min + scan.angle_increment * (N_BEAMS - 1)
        scan.range_min = 0.05
        scan.range_max = 40.0
        ranges = [FAR_M] * N_BEAMS
        # 正面（角度 0 ± 10 度）に障害物。
        center = int(round((0.0 - scan.angle_min) / scan.angle_increment))
        for i in range(center - 10, center + 11):
            ranges[i % N_BEAMS] = cls.obstacle_m
        scan.ranges = ranges
        cls.pub_scan.publish(scan)

        if cls.publish_state:
            st = SystemState()
            st.mode = cls.mode
            st.state = 'NONE'
            st.zone = cls.zone
            st.auto_brake = cls.auto_brake
            st.speed_limit = 'v_max'
            cls.pub_state.publish(st)
        cls.pub_estop.publish(Bool(data=cls.estop))
        cls.pub_lock.publish(Bool(data=cls.fault_lock))

    # ── 待ち ──────────────────────────────────────────────
    def _spin(self, duration):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait(self, pred, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.status is not None and pred(self.status):
                return True
        return False

    def _settle(self, pred, what):
        assert self._wait(lambda s: True, 20.0), 'limiter_status が来ない'
        assert self._wait(pred, 4.0), f'{what}: out={self.status.out_linear} ' \
            f'warn={self.status.approach_warning} class={self.status.source_class}'
        self._spin(0.5)  # 一瞬だけ満たした偶然を除く（続けて成り立つこと）
        assert pred(self.status), f'{what}（続かない）: out={self.status.out_linear} ' \
            f'warn={self.status.approach_warning}'

    # ── 試験 ──────────────────────────────────────────────
    def test_a_on_decelerates_without_warning(self):
        self._settle(lambda s: s.source_class == 'MANUAL'
                     and abs(s.out_linear - V_ALLOW_NEAR) < 1e-2
                     and not s.approach_warning,
                     'ON で減速（v_allow）しない／警告が出た')

    def test_b_off_keeps_speed_and_warns(self):
        type(self).auto_brake = False
        self._settle(lambda s: s.source_class == 'MANUAL'
                     and abs(s.out_linear - CMD_LINEAR_X) < 1e-3
                     and s.approach_warning,
                     'OFF で減速した／警告が出ない')

    def test_c_off_with_stale_state_falls_back(self):
        type(self).auto_brake = False
        self._settle(lambda s: abs(s.out_linear - CMD_LINEAR_X) < 1e-3, 'OFF の前提')
        type(self).publish_state = False   # /system/state の発行者が居なくなった
        assert self._wait(lambda s: s.out_linear == 0.0 and not s.approach_warning, 5.0), (
            f'/system/state が途絶えても OFF のまま動いた: {self.status.out_linear}')

    def test_d_off_estop_still_stops(self):
        type(self).auto_brake = False
        self._settle(lambda s: abs(s.out_linear - CMD_LINEAR_X) < 1e-3, 'OFF の前提')
        type(self).estop = True
        assert self._wait(lambda s: s.out_linear == 0.0 and s.action == 'STOP', 2.0), (
            f'OFF で非常停止が効かない: {self.status.out_linear}')

    def test_e_off_fault_lock_still_stops(self):
        type(self).auto_brake = False
        self._settle(lambda s: abs(s.out_linear - CMD_LINEAR_X) < 1e-3, 'OFF の前提')
        type(self).fault_lock = True
        assert self._wait(lambda s: s.out_linear == 0.0 and s.action == 'STOP', 2.0), (
            f'OFF でフォルトロックが効かない: {self.status.out_linear}')

    def test_f_off_does_not_relax_autonomous(self):
        type(self).auto_brake = False
        type(self).mode = 'FOLLOW'
        self._settle(lambda s: s.source_class == 'AUTO'
                     and abs(s.out_linear - V_ALLOW_NEAR) < 1e-2
                     and not s.approach_warning,
                     'AUTO が auto_brake=false で緩んだ')

    def test_g_off_does_not_relax_zone_na(self):
        type(self).auto_brake = False
        type(self).zone = 'NA'
        self._settle(lambda s: abs(s.out_linear - V_ALLOW_NEAR) < 1e-2
                     and not s.approach_warning,
                     'ゾーン NA が auto_brake=false で緩んだ')

    def test_h_off_far_obstacle_no_warning(self):
        type(self).auto_brake = False
        type(self).obstacle_m = FAR_M
        self._settle(lambda s: abs(s.out_linear - CMD_LINEAR_X) < 1e-3
                     and not s.approach_warning,
                     '遠いのに警告が出た')


if __name__ == '__main__':
    unittest.main()
