"""Verify the node reads normal_jerk_mps3 and uses it in its live update path."""
import math
import time
import unittest

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from th_system_msgs.msg import LimiterStatus, SystemState


@pytest.mark.launch_test
def generate_test_description():
    tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher', name='laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'], output='screen')
    limiter = launch_ros.actions.Node(
        package='th_safety', executable='obstacle_limiter', name='obstacle_limiter',
        parameters=[{'normal_accel_mps2': 0.4, 'normal_jerk_mps3': 0.5,
                     'obstacle_floor_distance_m': 0.0, 'v_max': 1.0,
                     'v_slow': 1.0, 'v_reverse': 1.0}], output='screen')
    return launch.LaunchDescription([tf, limiter, launch_testing.actions.ReadyToTest()]), {}


class TestLimiterSCurveNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_limiter_scurve_node')
        cls.samples = []
        cls.node.create_subscription(LimiterStatus, '/safety/limiter_status',
                                     lambda msg: cls.samples.append(msg.out_linear),
                                     qos_profile_sensor_data)
        cls.pub_cmd = cls.node.create_publisher(Twist, '/cmd_vel_muxed', 1)
        cls.pub_manual = cls.node.create_publisher(Twist, '/cmd_vel_manual', 1)
        cls.pub_scan = cls.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.pub_state = cls.node.create_publisher(SystemState, '/system/state', qos)
        cls.pub_estop = cls.node.create_publisher(Bool, '/safety/estop', 1)
        cls.pub_lock = cls.node.create_publisher(Bool, '/safety/fault_lock', 1)
        cls.timer = cls.node.create_timer(0.05, cls.publish_inputs)

    @classmethod
    def publish_inputs(cls):
        cmd = Twist()
        cmd.linear.x = 0.55
        cls.pub_cmd.publish(cmd)
        cls.pub_manual.publish(cmd)
        scan = LaserScan()
        scan.header.stamp = cls.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / 720
        scan.angle_max = scan.angle_min + scan.angle_increment * 719
        scan.range_min = 0.05
        scan.range_max = 40.0
        scan.ranges = [30.0] * 720
        cls.pub_scan.publish(scan)
        state = SystemState()
        state.mode = 'MANUAL'
        state.state = 'RUN'
        state.zone = 'OUT'
        state.auto_brake = False
        state.speed_limit = 'v_max'
        cls.pub_state.publish(state)
        cls.pub_estop.publish(Bool(data=False))
        cls.pub_lock.publish(Bool(data=False))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_live_output_starts_with_jerk_limited_acceleration(self):
        deadline = time.time() + 5.0
        while time.time() < deadline and len(self.samples) < 8:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertGreaterEqual(len(self.samples), 8, 'LimiterStatus not received')
        # At 20 Hz, J*dt=0.025 m/s² gives the first velocity delta
        # 0.5*(0+0.025)*0.05 = 0.000625 m/s. If the node ignores the
        # configured jerk (or substitutes zero), it jumps by 0.02 m/s.
        self.assertLess(max(self.samples[:4]), 0.01,
                        f'normal_jerk_mps3 did not shape initial output: {self.samples[:4]}')
        self.assertGreater(self.samples[-1], self.samples[0])
