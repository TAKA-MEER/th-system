"""
test_venue_dirty_node.py
========================
1b-6 SG-B6 — slam_control（本物）が /map_session/status の VENUE の dirty を
本当に出すことを、launch_testing で縛る。

これが無いと state_manager の「venue_map は未保存」判定は、試験の発行者が
dirty=True を流したときだけ働き、本番では一度も立たない（slam_control が
dirty=False 固定だった）。

- 起動直後（まっさらな地図）は dirty=False
- 画面から地図作成を始める（/slam_control/set_mapping true）と dirty=True
- 地図作成を止めても、保存していないあいだは dirty=True のまま
  （slam_toolbox が居ないので保存・破棄は成功しない。クリア側は
  test_slam_control_logic の呼び出し構造と実機確認に残す）

変異: set_mapping の dirty 立て行を消す → 赤。dirty を False 固定に戻す → 赤。
"""
import time
import unittest

import pytest
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from std_srvs.srv import SetBool
from th_system_msgs.msg import MapSessionStatus, SystemState

_LATCH = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)


@pytest.mark.launch_test
def generate_test_description():
    slam_control = launch_ros.actions.Node(
        package='th_config_manager', executable='slam_control.py', name='slam_control',
        output='screen')
    return launch.LaunchDescription([
        slam_control, launch_testing.actions.ReadyToTest(),
    ]), {}


class TestVenueDirty(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_venue_dirty_client')
        # slam_toolbox の代役（set_localization_mode が成功を返す）。
        self.node.create_service(
            SetBool, '/slam_toolbox/set_localization_mode', self._ok)
        self.pub_state = self.node.create_publisher(SystemState, '/system/state', _LATCH)
        self.dirty = None
        self.node.create_subscription(
            MapSessionStatus, '/map_session/status', self._on_status, _LATCH)
        self.cli = self.node.create_client(SetBool, '/slam_control/set_mapping')
        assert self.cli.wait_for_service(timeout_sec=15.0), 'slam_control 未起動'

    @staticmethod
    def _ok(req, resp):
        resp.success = True
        resp.message = ''
        return resp

    def _on_status(self, msg):
        if msg.slot == 'VENUE':
            self.dirty = bool(msg.dirty)

    def tearDown(self):
        self.node.destroy_node()

    def _spin(self, duration=0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _idle(self):
        msg = SystemState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = 'IDLE'
        msg.state = 'NONE'
        self.pub_state.publish(msg)

    def _set_mapping(self, on):
        req = SetBool.Request()
        req.data = on
        fut = self.cli.call_async(req)
        deadline = time.time() + 8.0
        while time.time() < deadline and not fut.done():
            self._idle()   # 鮮度ゲート（1.5 秒）を切らさない
            self._spin(0.1)
        self.assertTrue(fut.done(), 'set_mapping の応答が返らない')
        return fut.result()

    def _wait_dirty(self, want, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._idle()
            self._spin(0.2)
            if self.dirty == want:
                return True
        return False

    def test_a_fresh_map_is_not_dirty(self):
        self.assertTrue(self._wait_dirty(False), f'dirty={self.dirty}')

    def test_b_starting_mapping_makes_it_dirty_and_stays(self):
        self.assertTrue(self._wait_dirty(False))
        res = self._set_mapping(True)
        self.assertTrue(res.success, res.message)
        self.assertTrue(self._wait_dirty(True), f'dirty={self.dirty}')
        # 止めても保存していないあいだは未保存のまま。
        res = self._set_mapping(False)
        self.assertTrue(res.success, res.message)
        self._spin(2.5)
        self._idle()
        self.assertTrue(self._wait_dirty(True))
