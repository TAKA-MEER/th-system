"""
test_blind_startup_limit_node.py
================================
obstacle_limiter の**起動時**の blind_angle_ranges も、実行中更新と同じ上限（1 区間 30°・総幅 90°・
8 区間）で検査され、違反ならマスクなしで起動することを、観測できる振る舞いで縛る。

死角マスクは「死角方向への速度上限を v_reverse に絞る」形で安全判定に効く（前方 0° がマスクに入ると
上限が v_slow → v_reverse）。上限違反の値を launch で直接与えた obstacle_limiter が
  - マスクを使っていれば前方の上限は v_reverse（0.25）
  - マスクなしで起動していれば v_slow（0.30）
になる。対照として、上限内の値で起動した obstacle_limiter は v_reverse に絞られる（観測できることの確認）。

変異チェック: obstacle_limiter.cpp の起動時検査 `if (!v.ok) {` を `if (false) {` にすると
test_over_limit_startup_value_is_not_used が赤。
"""
import time
import unittest

import pytest
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from th_system_msgs.msg import LimiterStatus, SystemState

V_SLOW = 0.30
V_REVERSE = 0.25


def _limiter(name, blind):
    return launch_ros.actions.Node(
        package='th_safety', executable='obstacle_limiter', name=name,
        parameters=[{'v_slow': V_SLOW, 'v_reverse': V_REVERSE,
                     'obstacle_floor_distance_m': 0.05,
                     'blind_angle_ranges': blind}],
        remappings=[('/safety/limiter_status', f'/test/{name}/status'),
                    ('/cmd_vel', f'/test/{name}/cmd_vel')],
        output='screen')


@pytest.mark.launch_test
def generate_test_description():
    tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher', name='laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'], output='screen')
    return launch.LaunchDescription([
        tf,
        _limiter('limiter_over', [-60.0, 60.0]),     # 120° 幅 = 1 区間 30° を超える
        _limiter('limiter_ok', [-12.0, 12.0]),       # 24°。上限内
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestBlindStartupLimit(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_blind_startup_limit')
        cls.caps = {}
        be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        for n in ('limiter_over', 'limiter_ok'):
            cls.node.create_subscription(
                LimiterStatus, f'/test/{n}/status',
                lambda m, n=n: cls.caps.__setitem__(n, m.applied_limit_mps), be)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.pub_state = cls.node.create_publisher(SystemState, '/system/state', latched)
        cls.pub_muxed = cls.node.create_publisher(Twist, '/cmd_vel_muxed', 1)
        cls.pub_scan = cls.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        cls.pub_estop = cls.node.create_publisher(Bool, '/safety/estop', 1)
        cls.pub_lock = cls.node.create_publisher(Bool, '/safety/fault_lock', 1)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _publish(self):
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - math.radians(1.0)
        scan.angle_increment = math.radians(1.0)
        scan.range_min = 0.05
        scan.range_max = 12.0
        scan.ranges = [3.0] * 360
        self.pub_scan.publish(scan)
        cmd = Twist()
        cmd.linear.x = 0.5
        self.pub_muxed.publish(cmd)
        st = SystemState()
        st.mode = 'MANUAL'
        st.state = 'NONE'
        st.zone = 'OUT'
        st.auto_brake = True
        st.speed_limit = 'v_slow'
        self.pub_state.publish(st)
        self.pub_estop.publish(Bool(data=False))
        self.pub_lock.publish(Bool(data=False))

    def _wait_caps(self, timeout=40.0):
        deadline = time.time() + timeout
        last = 0.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if time.time() - last >= 0.05:
                self._publish()
                last = time.time()
            if len(self.caps) == 2 and all(v > 0.0 for v in self.caps.values()):
                break
        # 値が落ち着くまで少し回す
        end = time.time() + 1.0
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if time.time() - last >= 0.05:
                self._publish()
                last = time.time()

    def test_over_limit_startup_value_is_not_used(self):
        self._wait_caps()
        assert set(self.caps) == {'limiter_over', 'limiter_ok'}, self.caps
        # 対照: 上限内の値で起動した方は、前方が死角に入るので v_reverse に絞られる
        assert abs(self.caps['limiter_ok'] - V_REVERSE) < 1e-3, self.caps
        # 上限違反の値で起動した方は、マスクなしで起動する（絞られず v_slow のまま）
        assert abs(self.caps['limiter_over'] - V_SLOW) < 1e-3, (
            f'上限違反の起動時マスクが使われている: {self.caps}')


if __name__ == '__main__':
    unittest.main()
