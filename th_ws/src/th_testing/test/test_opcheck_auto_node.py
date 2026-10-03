"""
test_opcheck_auto_node.py
=========================
起動時の自動点検 `opcheck_auto` ノードの振る舞い試験（Docker launch テスト）。

`auto_check_core.py` の純粋関数は test_auto_check_core.py が縛るが、ここでは
「本番の経路」（購読→判定→`/opcheck/auto_status` 配信・開発モードの抑制・
駆動への出力が無いこと）が実際に配線されているかを、ノードを実際に起動して
確かめる（お手本は test_opcheck_runner_node.py）。

検証する6点:
  a. 起動直後（何も届いていない）→ overall CHECKING（判定中）
  b. 正常なセンサ入力（estop 解除・IMU・/scan）→ overall OK
  c. /scan が来ない → LIDAR が NG（no_data）で overall NG（LiDAR 未受信を
     NG にしない変異はここで赤になる＝変異チェック①の標的）
  d. IMU が来ない → IMU が NG で overall NG
  e. 非常停止が押されたまま → ESTOP が NG（pressed）で overall NG
  f. /system/dev_mode の effective に opcheck → suppressed が真。
     未受信・古い（3 秒超）・壊れた JSON では suppressed にならない
     （鮮度判定を外す変異はここで赤になる＝変異チェック②の標的）
  g. 自動点検の間、/cmd_vel_behavior に非ゼロが 1 件も出ない。
     構造的にも opcheck_auto は /cmd_vel_behavior の publisher を持たない
     （非ゼロを出す変異はここで赤になる＝変異チェック③の標的）
  h. `angle_min=-π` の /scan でも死角マスクと実角度で比べる。
     実角度どおりの帯なら LIDAR OK（ノード側で `angle_min` を渡し忘れる
     変異はここで赤になる）
"""
import json
import math
import time
import unittest

import pytest
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String, UInt8


_LATCH_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST)

# テストを速く安定させるための短縮値（launch の parameters= で上書き）。
SCAN_STALE_MS = 500
IMU_WINDOW_S = 0.5


@pytest.mark.launch_test
def generate_test_description():
    opcheck_auto = launch_ros.actions.Node(
        package='th_maintenance',
        executable='opcheck_auto.py',
        name='opcheck_auto',
        parameters=[{
            'scan_stale_ms': int(SCAN_STALE_MS),  # 宣言は整数（registry・生成 yaml と同じ型）
            'opcheck_imu_window_s': IMU_WINDOW_S,
            'imu_bias_max_rad_s': 0.2,
            'imu_wz_implausible_rad_s': 10.0,
            'opcheck_blind_tolerance_deg': 5.0,
            'opcheck_scan_coverage_gap_deg': 2.0,
            # h 用の死角マスク（実角度）。b〜g のフラットスキャンには帯が無い
            # ので影響しない（推定が空 → 比較不能 → OK のまま）
            'blind_angle_ranges': [10.0, 20.0],
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        opcheck_auto,
        launch_testing.actions.ReadyToTest(),
    ]), {'opcheck_auto': opcheck_auto}


def _flat_scan(node) -> LaserScan:
    msg = LaserScan()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'laser_link'
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = 2.0 * math.pi / 360.0
    msg.range_min = 0.05
    msg.range_max = 12.0
    msg.ranges = [5.0] * 360
    return msg


def _imu(node, wz: float = 0.01) -> Imu:
    msg = Imu()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.angular_velocity.z = wz
    return msg


def _parse_auto(raw: str) -> dict:
    return json.loads(raw)


class TestOpcheckAutoNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_opcheck_auto_client')

        self._auto: list[String] = []
        self.node.create_subscription(
            String, '/opcheck/auto_status', self._auto.append, _LATCH_QOS)
        # g: 駆動への出力の監視（opcheck_auto 自体は publisher を持たない）。
        self._cmd: list[Twist] = []
        self.node.create_subscription(
            Twist, '/cmd_vel_behavior',
            lambda m: self._cmd.append((m.linear.x, m.angular.z)), 10)

        self.pub_estop = self.node.create_publisher(Bool, '/safety/estop_hw', 10)
        self.pub_imu = self.node.create_publisher(Imu, '/esp32/imu_data', 10)
        self.pub_calib = self.node.create_publisher(
            UInt8, '/esp32/imu_calib_status', 10)
        self.pub_scan = self.node.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_dev = self.node.create_publisher(String, '/system/dev_mode', _LATCH_QOS)

        # b〜g の入力スイッチ。各テストが必要なものだけ ON にする。
        self._feed_estop = False
        self._estop_pressed = False
        self._feed_imu = False
        self._feed_scan = False
        self._scan_custom = None  # None 以外なら _flat_scan の代わりに流す関数
        self._feed_dev = None  # None=未受信 / str=JSON を流す
        # 周期配信スレッドの代わりに spin しながら流すためのタイマ記録
        self._last_feed = 0.0

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self._feed_once()

    def _feed_once(self):
        now = time.monotonic()
        if now - self._last_feed < 0.1:
            return
        self._last_feed = now
        if self._feed_estop:
            self.pub_estop.publish(Bool(data=self._estop_pressed))
        if self._feed_imu:
            self.pub_imu.publish(_imu(self.node))
            self.pub_calib.publish(UInt8(data=0xFF))
        if self._feed_scan:
            if self._scan_custom is not None:
                self.pub_scan.publish(self._scan_custom(self.node))
            else:
                self.pub_scan.publish(_flat_scan(self.node))
        if self._feed_dev is not None:
            self.pub_dev.publish(String(data=self._feed_dev))

    def _latest_auto(self):
        assert self._auto, '/opcheck/auto_status が一度も届いていない'
        return _parse_auto(self._auto[-1].data)

    def _wait_auto(self, predicate, timeout: float = 6.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.2)
            if self._auto and predicate(_parse_auto(self._auto[-1].data)):
                return _parse_auto(self._auto[-1].data)
        self.fail(f'/opcheck/auto_status が条件を満たさなかった: '
                  f'latest={self._auto[-1].data if self._auto else None}')

    # ── a: 起動直後は CHECKING ──
    def test_a_initial_is_checking(self):
        auto = self._wait_auto(lambda a: True, timeout=4.0)
        assert auto['overall'] == 'CHECKING', auto
        assert auto['suppressed'] is False

    # ── b: 正常なセンサ入力 → OK ──
    def test_b_healthy_inputs_is_ok(self):
        self._feed_estop = True
        self._feed_imu = True
        self._feed_scan = True
        auto = self._wait_auto(lambda a: a['overall'] == 'OK', timeout=8.0)
        assert auto['suppressed'] is False
        assert auto['items']['ESTOP']['result'] == 'OK'
        assert auto['items']['IMU']['result'] == 'OK'
        assert auto['items']['LIDAR']['result'] == 'OK'

    # ── c: /scan が来ない → LIDAR NG で overall NG ──
    def test_c_no_scan_is_ng(self):
        self._feed_estop = True
        self._feed_imu = True
        self._feed_scan = False
        auto = self._wait_auto(
            lambda a: a['overall'] == 'NG'
            and a['items']['LIDAR']['result'] == 'NG', timeout=8.0)
        assert auto['items']['LIDAR']['reason'] == 'no_data'

    # ── d: IMU が来ない → IMU NG で overall NG ──
    def test_d_no_imu_is_ng(self):
        self._feed_estop = True
        self._feed_imu = False
        self._feed_scan = True
        auto = self._wait_auto(
            lambda a: a['overall'] == 'NG'
            and a['items']['IMU']['result'] == 'NG', timeout=8.0)
        assert auto['items']['IMU']['reason'] == 'no_data'

    # ── e: 非常停止が押されたまま → ESTOP NG で overall NG ──
    def test_e_estop_pressed_is_ng(self):
        self._feed_estop = True
        self._estop_pressed = True
        self._feed_imu = True
        self._feed_scan = True
        auto = self._wait_auto(
            lambda a: a['overall'] == 'NG'
            and a['items']['ESTOP']['result'] == 'NG', timeout=8.0)
        assert auto['items']['ESTOP']['reason'] == 'pressed'

    # ── f: 開発モードの抑制 ──
    def test_f_dev_mode_suppression(self):
        self._feed_estop = True
        self._feed_imu = True
        self._feed_scan = True
        # 未受信では suppressed にならない
        self._wait_auto(lambda a: a['overall'] == 'OK', timeout=8.0)
        assert self._latest_auto()['suppressed'] is False

        # effective に opcheck → suppressed
        self._feed_dev = json.dumps(
            {"dev_mode": True, "effective": {"opcheck": True}})
        auto = self._wait_auto(lambda a: a['suppressed'] is True, timeout=6.0)
        assert auto['overall'] == 'OK', auto  # 判定そのものは止めない

        # 壊れた JSON → suppressed にならない
        self._feed_dev = 'broken-json{{{'
        auto = self._wait_auto(lambda a: a['suppressed'] is False, timeout=6.0)

        # 古い（3 秒超）effective → suppressed にならない
        self._feed_dev = json.dumps(
            {"dev_mode": True, "effective": {"opcheck": True}})
        self._wait_auto(lambda a: a['suppressed'] is True, timeout=6.0)
        self._feed_dev = None  # 配信停止
        auto = self._wait_auto(lambda a: a['suppressed'] is False, timeout=8.0)

    # ── g: 駆動への出力が無い ──
    def test_g_no_cmd_vel_behavior(self):
        self._feed_estop = True
        self._feed_imu = True
        self._feed_scan = True
        self._wait_auto(lambda a: a['overall'] == 'OK', timeout=8.0)
        # OK 到達後もしばらく回し、非ゼロが出ないことを見る
        self._cmd.clear()
        self._spin(2.0)
        nonzero = [(v, w) for v, w in self._cmd
                   if abs(v) > 1e-9 or abs(w) > 1e-9]
        assert nonzero == [], f'/cmd_vel_behavior に非ゼロが出た: {nonzero[:5]}'
        # 構造的にも publisher を持たない
        pubs = dict(self.node.get_publisher_names_and_types_by_node(
            'opcheck_auto', ''))
        assert '/cmd_vel_behavior' not in pubs, pubs

    # ── h: angle_min=-π でも実角度で比べる ──
    def test_h_blind_mask_compared_in_real_angles(self):
        # 実角度 10..20° に近距離帯（=`blind_angle_ranges` どおり）。
        # angle_min を見ずに比べる変異では推定帯が (190, 200) になり NG。
        def _band_scan(node):
            msg = _flat_scan(node)
            msg.ranges = list(msg.ranges)
            for i in range(190, 200):
                msg.ranges[i] = 0.3
            return msg

        self._feed_estop = True
        self._feed_imu = True
        self._feed_scan = True
        self._scan_custom = _band_scan
        # 注意: /opcheck/auto_status は TRANSIENT_LOCAL のため、購読直後に
        # 前テストの最終メッセージ（OK）が即届く。最新 1 件だけ見ると帯スキャン
        # を評価する前に述語が成立して空振りになるので、購読後に配信された
        # 新しいメッセージ（2 件目以降）で判定する。
        auto = self._wait_auto(
            lambda a: len(self._auto) >= 2 and a['overall'] == 'OK'
            and a['items']['LIDAR']['result'] == 'OK', timeout=8.0)
        assert auto['items']['LIDAR']['reason'] == '', auto
