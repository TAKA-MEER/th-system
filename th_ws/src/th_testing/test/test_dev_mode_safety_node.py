"""
test_dev_mode_safety_node.py
============================
開発モードの項目（Spec-safety.md §10。2026-09-23 改定）を、safety_monitor と
obstacle_limiter の**ノード本体**で確かめる Docker launch テスト。
純関数（th_safety の test_dev_mode_core）が固くても、購読・鮮度・項目名の
配線は呼び出し側の数行で壊れうるため、ここで実際に起動して縛る
（CLAUDE.md「エージェントの試験が本番の経路を縛っているか」）。

LiDAR が無い状態（/scan を一度も出さない）で:
  obstacle_limiter（項目 scan_stop）
    a. 開発モードの状態が未受信 → MANUAL でも 0
    b. effective.scan_stop=true → MANUAL は動く。上限は通常と同じ
       （前進は speed_limit=v_slow まで出て v_reverse を超える。後退は v_reverse）
    c. 同じ状態で AUTO（mode=FOLLOW）→ 0
    d. ignore（選択）だけ真で effective が偽 → 0
    e. /system/dev_mode が途絶えて古くなる → 0 に戻る
  safety_monitor（項目 lidar_fault）
    f. 項目 OFF → LIDAR_LOST が出る
    g. 項目 ON → LIDAR_LOST が解除され、fault_lock が落ちる
    h. 項目 OFF に戻す → LIDAR_LOST が再び出る

safety_monitor の /safety/fault_lock・/safety/estop は /test/sm_* へ付け替え、
obstacle_limiter にはテスト側が「ロックなし」を流す（両者の試験を独立させる）。

注意（CLAUDE.md の既知の癖）: 試験は TestCase のメソッドにする。
メソッドは名前順に走り、前の試験の状態を引き継ぐ。
"""
import json
import time
import unittest

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.msg import FaultStatus, LimiterStatus, SystemState


V_SLOW = 0.30
V_REVERSE = 0.25
CMD_LINEAR_X = 0.5
PUB_HZ = 20.0
DEV_HZ = 1.0
STARTUP_GRACE_SEC = 1
STARTUP_DEADLINE_SEC = 1
LIDAR_TIMEOUT_MS = 500

ITEMS = ('link', 'lidar_fault', 'scan_stop', 'battery', 'opcheck', 'auto_brake')


def _dev_json(dev_mode: bool, ignore=(), effective=()):
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
        parameters=[{'v_slow': V_SLOW, 'v_reverse': V_REVERSE}],
        output='screen',
    )
    safety = launch_ros.actions.Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'check_period_ms': 50,
            'startup_grace_sec': STARTUP_GRACE_SEC,
            'startup_deadline_sec': STARTUP_DEADLINE_SEC,
            'lidar_timeout_ms': LIDAR_TIMEOUT_MS,
            'enabled_targets': ['lidar'],
        }],
        remappings=[('/safety/fault_lock', '/test/sm_fault_lock'),
                    ('/safety/estop', '/test/sm_estop')],
        output='screen',
    )
    return launch.LaunchDescription([
        tf, limiter, safety,
        launch_testing.actions.ReadyToTest(),
    ]), {'obstacle_limiter': limiter, 'safety_monitor': safety}


class TestDevModeSafetyNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_dev_mode_safety_node')
        n = cls.node
        cls.status = None
        cls.faults = []
        cls.sm_lock = None
        # limiter_status は best_effort で出る（reliable で購読すると QoS 不一致で届かない）。
        n.create_subscription(LimiterStatus, '/safety/limiter_status',
                              lambda m: setattr(cls, 'status', m),
                              QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        n.create_subscription(FaultStatus, '/safety/fault', cls.faults.append, 10)
        n.create_subscription(Bool, '/test/sm_fault_lock',
                              lambda m: setattr(cls, 'sm_lock', m.data), 10)

        cls.pub_muxed = n.create_publisher(Twist, '/cmd_vel_muxed', 1)
        cls.pub_manual = n.create_publisher(Twist, '/cmd_vel_manual', 1)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cls.pub_state = n.create_publisher(SystemState, '/system/state', latched)
        cls.pub_estop = n.create_publisher(Bool, '/safety/estop', 1)
        cls.pub_lock = n.create_publisher(Bool, '/safety/fault_lock', 1)
        cls.pub_dev = n.create_publisher(String, '/system/dev_mode', latched)

        cls.mode = 'MANUAL'
        cls.cmd_x = CMD_LINEAR_X
        cls.dev_payload = None  # None の間は /system/dev_mode を出さない
        cls._t_fast = n.create_timer(1.0 / PUB_HZ, cls._publish_fast)
        cls._t_dev = n.create_timer(1.0 / DEV_HZ, cls._publish_dev)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # ── 送信 ──────────────────────────────────────────────
    @classmethod
    def _publish_fast(cls):
        cmd = Twist()
        cmd.linear.x = cls.cmd_x
        cls.pub_muxed.publish(cmd)
        cls.pub_manual.publish(cmd)
        st = SystemState()
        st.mode = cls.mode
        st.state = 'NONE'
        st.zone = 'OUT'
        st.auto_brake = True
        st.speed_limit = 'v_slow'
        cls.pub_state.publish(st)
        cls.pub_estop.publish(Bool(data=False))
        cls.pub_lock.publish(Bool(data=False))

    @classmethod
    def _publish_dev(cls):
        if cls.dev_payload is not None:
            cls.pub_dev.publish(String(data=cls.dev_payload))

    def _set_dev(self, payload):
        type(self).dev_payload = payload
        self._publish_dev()

    # ── 待ち ──────────────────────────────────────────────
    def _spin(self, duration: float):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait(self, pred, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if pred():
                return True
        return False

    def _out(self) -> float:
        return float('nan') if self.status is None else self.status.out_linear

    def _lidar_lost_active(self) -> bool:
        """/safety/fault は変化時だけ出るので、最後の LIDAR_LOST 関連の edge を見る。"""
        last = None
        for f in self.faults:
            if f.active and f.fault_type == 'LIDAR_LOST':
                last = True
            elif not f.active and last:
                last = False
        return bool(last)

    # ════════════════════════════════════════════════════════
    # obstacle_limiter（項目 scan_stop）
    # ════════════════════════════════════════════════════════
    def test_a_limiter_without_dev_state_stops_manual(self):
        # TF 取得〜起動まで待つ（obstacle_limiter は最大 10 秒リトライする）。
        assert self._wait(lambda: self.status is not None, 20.0), 'limiter_status が来ない'
        self._spin(1.0)
        assert self.status.source_class == 'MANUAL', self.status.source_class
        assert self._out() == 0.0, f'開発モード未受信なのに動いた: {self._out()}'

    def test_b_limiter_scan_stop_uses_normal_limits(self):
        self._set_dev(_dev_json(True, ignore=('scan_stop',), effective=('scan_stop',)))
        assert self._wait(lambda: self._out() > 0.0, 3.0), (
            f'scan_stop が実効なのに MANUAL が動かない: {self._out()}')
        self._spin(0.5)
        # 前進は通常どおり speed_limit（v_slow）まで。未観測を理由に v_reverse へ絞らない。
        assert abs(self._out() - V_SLOW) < 1e-3, (
            f'前進の上限が通常（v_slow={V_SLOW}）と違う: {self._out()}')
        assert self.status.nearest_obstacle_m == -1.0, '未観測は -1（不明）のはず'
        # 後退は通常どおり v_reverse。
        type(self).cmd_x = -CMD_LINEAR_X
        try:
            assert self._wait(lambda: abs(self._out() + V_REVERSE) < 1e-3, 3.0), (
                f'後退の上限が通常（v_reverse={V_REVERSE}）と違う: {self._out()}')
        finally:
            type(self).cmd_x = CMD_LINEAR_X
        assert self._wait(lambda: self._out() > 0.0, 3.0)

    def test_c_limiter_scan_stop_still_stops_auto(self):
        type(self).mode = 'FOLLOW'
        try:
            assert self._wait(lambda: self.status.source_class == 'AUTO', 2.0)
            self._spin(0.5)
            assert self._out() == 0.0, f'AUTO が /scan 途絶で動いた: {self._out()}'
        finally:
            type(self).mode = 'MANUAL'
        assert self._wait(lambda: self._out() > 0.0, 3.0), 'MANUAL に戻しても動かない'

    def test_d_limiter_ignores_selection_that_is_not_effective(self):
        self._set_dev(_dev_json(True, ignore=('scan_stop',), effective=()))
        assert self._wait(lambda: self._out() == 0.0, 2.0), (
            f'effective が偽なのに動いた（ignore 側を読んでいる？）: {self._out()}')
        self._spin(0.5)
        assert self._out() == 0.0

    def test_e_limiter_falls_back_when_dev_state_goes_stale(self):
        self._set_dev(_dev_json(True, ignore=('scan_stop',), effective=('scan_stop',)))
        assert self._wait(lambda: self._out() > 0.0, 3.0)
        type(self).dev_payload = None   # 発行者が居なくなった
        assert self._wait(lambda: self._out() == 0.0, 6.0), (
            f'/system/dev_mode が途絶えても動き続けた: {self._out()}')

    # ════════════════════════════════════════════════════════
    # safety_monitor（項目 lidar_fault）
    # ════════════════════════════════════════════════════════
    def test_f_monitor_raises_lidar_lost_when_item_off(self):
        self._set_dev(_dev_json(True, ignore=(), effective=()))
        assert self._wait(self._lidar_lost_active, 5.0), 'LiDAR 無しで LIDAR_LOST が出ない'
        assert self._wait(lambda: self.sm_lock is True, 2.0), 'fault_lock が立たない'

    def test_g_monitor_clears_lidar_lost_when_item_on(self):
        self._set_dev(_dev_json(True, ignore=('lidar_fault',), effective=('lidar_fault',)))
        assert self._wait(lambda: not self._lidar_lost_active(), 3.0), (
            'lidar_fault を ON にしても LIDAR_LOST が解除されない')
        assert self._wait(lambda: self.sm_lock is False, 2.0), 'fault_lock が落ちない'
        self._spin(1.5)
        assert not self._lidar_lost_active(), 'ON のまま LIDAR_LOST が再発した'

    def test_h_monitor_raises_again_when_item_off(self):
        self._set_dev(_dev_json(True, ignore=(), effective=()))
        assert self._wait(self._lidar_lost_active, 3.0), 'OFF に戻しても LIDAR_LOST が出ない'


if __name__ == '__main__':
    unittest.main()
