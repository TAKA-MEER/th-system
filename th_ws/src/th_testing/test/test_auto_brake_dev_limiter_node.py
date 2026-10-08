"""
test_auto_brake_dev_limiter_node.py
===================================
開発モードの項目 auto_brake（Spec-safety.md §10。2026-10-08 改定）を、obstacle_limiter
（C++）のノード本体を実際に起動して縛る launch_testing。

純関数（gtest の policy_stops）が固くても、`/system/dev_mode` の購読・項目名・鮮度・
core への受け渡しは呼び出し側の数行で壊れうる（CLAUDE.md「エージェントの試験が本番の
経路を縛っているか」）。test_auto_brake_limiter_node.py の手動系の試験とは別に、
**自律系（AUTO）** の側を縛る。

前方 0.6 m に障害物を置き、自律系（mode=FOLLOW）が 0.8 m/s を指令する。
  a. 開発モードなし・auto_brake=false → 減速する（無効化不可のまま）
  b. effective.auto_brake=true・auto_brake=false → 減速しない（0.8）。接近警告が出る
  c. effective.auto_brake=true でも state.auto_brake=true（切り替えていない）→ 減速する
     （項目を選んだだけでは OFF にならない）
  d. effective.auto_brake=true・auto_brake=false でもゾーン NA（点検・校正）→ 減速する
  e. ignore（選択）だけ真で effective が偽 → 減速する
  f. dev_mode が false（マスタ OFF）で effective が真と書かれていても → 減速する
  g. effective.auto_brake=true で OFF のあと、/system/dev_mode が途絶えて 3 秒超 → 減速に戻る
  h. effective.auto_brake=true・auto_brake=false でも /system/state が途絶えたら 0（ON に倒れる）
  i. effective.auto_brake=true・auto_brake=false でも非常停止は 0
  j. effective.auto_brake=true・auto_brake=false でも障害物が遠ければ警告は出ない
  k. 別項目（scan_stop）だけ effective → 減速する（項目名を取り違えない）

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
import json
from std_msgs.msg import Bool, String

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


ITEMS = ('link', 'lidar_fault', 'scan_stop', 'battery', 'opcheck', 'auto_brake')


def _dev_json(dev_mode, ignore=(), effective=()):
    return json.dumps({
        'dev_mode': dev_mode,
        'ignore': {k: (k in ignore) for k in ITEMS},
        'effective': {k: (k in effective) for k in ITEMS},
        'estop_hw_known': False,
        'estop_hw_pressed': False,
    }, sort_keys=True)


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


class TestAutoBrakeDevLimiterNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_auto_brake_dev_limiter_node')
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
        cls.pub_dev = n.create_publisher(String, '/system/dev_mode', latched)

        cls._reset_inputs()
        cls._timer = n.create_timer(1.0 / PUB_HZ, cls._publish)
        cls._timer_dev = n.create_timer(1.0, cls._publish_dev)

    @classmethod
    def _reset_inputs(cls):
        cls.mode = 'FOLLOW'
        cls.dev_payload = None  # None の間は /system/dev_mode を出さない
        cls.zone = 'OUT'
        cls.auto_brake = False
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

    @classmethod
    def _publish_dev(cls):
        if cls.dev_payload is not None:
            cls.pub_dev.publish(String(data=cls.dev_payload))

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

    def _dev(self, payload):
        """/system/dev_mode を即時に 1 通出し、以後 1 Hz で出し続ける。"""
        type(self).dev_payload = payload
        if payload is not None:
            self.pub_dev.publish(String(data=payload))

    # ── 試験 ──────────────────────────────────────────────
    def test_a_no_dev_autonomous_always_brakes(self):
        self._settle(lambda s: s.source_class == 'AUTO'
                     and abs(s.out_linear - V_ALLOW_NEAR) < 1e-2
                     and not s.approach_warning,
                     '開発モードなしで AUTO が auto_brake=false に従った')

    def test_b_dev_item_autonomous_follows_off(self):
        self._dev(_dev_json(True, ('auto_brake',), ('auto_brake',)))
        self._settle(lambda s: s.source_class == 'AUTO'
                     and abs(s.out_linear - CMD_LINEAR_X) < 1e-3
                     and s.approach_warning,
                     '項目ありで AUTO が減速した／警告が出ない')

    def test_c_dev_item_without_toggle_still_brakes(self):
        type(self).auto_brake = True
        self._dev(_dev_json(True, ('auto_brake',), ('auto_brake',)))
        self._settle(lambda s: s.source_class == 'AUTO'
                     and abs(s.out_linear - V_ALLOW_NEAR) < 1e-2
                     and not s.approach_warning,
                     '項目を選んだだけで OFF になった')

    def test_d_dev_item_zone_na_still_brakes(self):
        type(self).zone = 'NA'
        self._dev(_dev_json(True, ('auto_brake',), ('auto_brake',)))
        self._settle(lambda s: abs(s.out_linear - V_ALLOW_NEAR) < 1e-2
                     and not s.approach_warning,
                     'ゾーン NA（点検・校正）で緩んだ')

    def test_e_ignore_only_not_effective_brakes(self):
        self._dev(_dev_json(True, ('auto_brake',), ()))
        self._settle(lambda s: s.source_class == 'AUTO'
                     and abs(s.out_linear - V_ALLOW_NEAR) < 1e-2,
                     'effective でないのに（選択だけで）緩んだ')

    def test_f_master_off_not_effective_brakes(self):
        self._dev(_dev_json(False, ('auto_brake',), ('auto_brake',)))
        self._settle(lambda s: s.source_class == 'AUTO'
                     and abs(s.out_linear - V_ALLOW_NEAR) < 1e-2,
                     'dev_mode が false なのに緩んだ')

    def test_g_dev_mode_expiry_returns_to_brake(self):
        self._dev(_dev_json(True, ('auto_brake',), ('auto_brake',)))
        self._settle(lambda s: abs(s.out_linear - CMD_LINEAR_X) < 1e-3, '項目ありの前提')
        self._dev(None)   # 発行者が居なくなった（transient_local の再配送も止める）
        assert self._wait(lambda s: abs(s.out_linear - V_ALLOW_NEAR) < 1e-2
                          and not s.approach_warning, 6.0), (
            f'/system/dev_mode が途絶えても緩んだまま: {self.status.out_linear}')

    def test_h_dev_item_stale_state_falls_back(self):
        self._dev(_dev_json(True, ('auto_brake',), ('auto_brake',)))
        self._settle(lambda s: abs(s.out_linear - CMD_LINEAR_X) < 1e-3, '項目ありの前提')
        type(self).publish_state = False
        assert self._wait(lambda s: s.out_linear == 0.0 and not s.approach_warning, 5.0), (
            f'/system/state が途絶えても OFF のまま動いた: {self.status.out_linear}')

    def test_i_dev_item_estop_still_stops(self):
        self._dev(_dev_json(True, ('auto_brake',), ('auto_brake',)))
        self._settle(lambda s: abs(s.out_linear - CMD_LINEAR_X) < 1e-3, '項目ありの前提')
        type(self).estop = True
        assert self._wait(lambda s: s.out_linear == 0.0 and s.action == 'STOP', 2.0), (
            f'項目ありで非常停止が効かない: {self.status.out_linear}')

    def test_j_dev_item_far_obstacle_no_warning(self):
        type(self).obstacle_m = FAR_M
        self._dev(_dev_json(True, ('auto_brake',), ('auto_brake',)))
        self._settle(lambda s: abs(s.out_linear - CMD_LINEAR_X) < 1e-3
                     and not s.approach_warning,
                     '遠いのに警告が出た')

    def test_k_other_item_does_not_relax(self):
        self._dev(_dev_json(True, ('scan_stop',), ('scan_stop',)))
        self._settle(lambda s: s.source_class == 'AUTO'
                     and abs(s.out_linear - V_ALLOW_NEAR) < 1e-2,
                     '別の項目で緩んだ（項目名の取り違え）')


if __name__ == '__main__':
    unittest.main()
