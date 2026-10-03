"""
test_replay_localize_node.py
============================
W-01 P2（全域ローカライズの探索の実体）の Docker launch テスト。
replay_runner を実際に起動し、テスト側が /map_session/open の代役サービス・
/scan・既知の pgm を用意して次を確認する（純粋関数の試験は
test_localize_core.py が持ち、ここではノード本体の配線——探索結果で reload
する・イベントを出す・ quality を載せる——を見る。ソースの文字列検査だけ
だと「探索結果を捨てて始点を渡す」「不成立でも done を出す」変異が
すり抜けるため）:

  a. 正しい位置のスキャン → evt.localize_done が arg_json{score, margin} 付きで
     出る、かつ代役に渡された初期姿勢が探索結果（経路始点ではない）
  b. 地図と合わないスキャン → evt.localize_low が出る
  c. global_localize で不成立 → evt.localize_done が出ない・
     localize_quality=='failed'

地図は 6x5 m の部屋＋間仕切り（test_localize_core.py と同配置。真値の目安は
(2.0, 2.5, 0.3)）。経路始点は (0.5, 2.5) で真値から外しておく（変異②の検出用）。

お手本は test_replay_cross_track_node.py（起動方法・test_a_ 順序制御）。
"""
import json
import math
import os
import tempfile
import time
import unittest

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from th_system_msgs.msg import RouteStatus, StateEffect, StateEvent, SystemState
from th_system_msgs.srv import OpenMapSession


_ROUTE_ID = 'loc-test'
_SESSION_ID = 'test-sess'
# 真値（スキャンの撮影位置。laser 姿勢）と経路始点（わざと外す）
_TRUTH = (2.0, 2.5, 0.3)
_START = (0.5, 2.5, 0.0)
_N_BEAMS = 360
_ANGLE_MIN = -math.pi
_ANGLE_INC = 2.0 * math.pi / _N_BEAMS
_RES = 0.05


def _occ():
    """部屋＋間仕切り（test_localize_core.py の _pgm_occ と同配置）。"""
    occ = np.zeros((100, 120), dtype=bool)
    occ[0, :] = True
    occ[99, :] = True
    occ[:, 0] = True
    occ[:, 119] = True
    occ[20:61, 80] = True
    return occ


_OCC = _occ()


def _write_fixtures(routes_dir: str) -> None:
    img = np.full((100, 120), 254, dtype=np.uint8)
    img[_OCC] = 0
    base = os.path.join(routes_dir, _ROUTE_ID)
    with open(base + '.pgm', 'wb') as f:
        f.write(b'P5\n# test\n120 100\n255\n')
        f.write(img.tobytes())
    with open(base + '.yaml', 'w', encoding='utf-8') as f:
        f.write('image: %s.pgm\nresolution: 0.050000\n'
                'origin: [0.000000, 0.000000, 0.000000]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
                % _ROUTE_ID)
    points = [[0.5 + i * 0.5, 2.5, 0.0] for i in range(10)]
    route = {
        'id': _ROUTE_ID,
        'name': '全域ローカライズ試験用',
        'generation': 1,
        'length_m': 4.5,
        'point_count': len(points),
        'start_yaw': 0.0,
        'recorded_at_ms': 0,
        'frame_id': 'map',
        'map_session_id': _SESSION_ID,
        'points': points,
    }
    with open(base + '.json', 'w', encoding='utf-8') as f:
        json.dump(route, f)


def _raycast(x, y, yaw, max_range=40.0):
    """_OCC に対する模擬スキャン（当たらなければ inf）。"""
    H, W = _OCC.shape
    ang = _ANGLE_MIN + np.arange(_N_BEAMS) * _ANGLE_INC
    dx, dy = np.cos(ang + yaw), np.sin(ang + yaw)
    rng = np.full(_N_BEAMS, np.inf)
    alive = np.ones(_N_BEAMS, dtype=bool)
    d = 0.0
    step = _RES * 0.5
    while alive.any() and d < max_range:
        d += step
        gx = np.floor((x + dx * d) / _RES).astype(int)
        gy = np.floor((y + dy * d) / _RES).astype(int)
        oob = (gx < 0) | (gx >= W) | (gy < 0) | (gy >= H)
        hit = np.zeros(_N_BEAMS, dtype=bool)
        inb = alive & ~oob
        hit[inb] = _OCC[gy[inb], gx[inb]]
        rng[hit] = d
        alive &= ~hit & ~oob
    return rng


_MATCH_RANGES = _raycast(*_TRUTH)
_MISMATCH_RANGES = np.full(_N_BEAMS, np.inf)


_ROUTES_DIR = tempfile.mkdtemp(prefix='loc-routes-')
_write_fixtures(_ROUTES_DIR)


@pytest.mark.launch_test
def generate_test_description():
    runner = launch_ros.actions.Node(
        package='th_planning',
        executable='replay_runner.py',
        name='replay_runner',
        parameters=[{
            'routes_dir': _ROUTES_DIR,
            'use_map_frame': True,
            'map_session_id': _SESSION_ID,
            'localize_wait_s': 3.0,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        runner,
        launch_testing.actions.ReadyToTest(),
    ]), {'replay_runner': runner}


class TestReplayLocalizeNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_replay_localize_node')

        self._events = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._on_event, 10)
        self._statuses = []
        self.node.create_subscription(
            RouteStatus, '/route/status', self._statuses.append, 10)

        # /system/state は REPLAY/LOCALIZE を流し続ける（止めると status が
        # owns_route_status で出なくなる。本番の state_manager も transient_local）。
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._state_value = 'LOCALIZE'
        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', state_qos)
        self._state_timer = self.node.create_timer(0.1, self._publish_state)

        self.pub_effect = self.node.create_publisher(
            StateEffect, '/system/effect', 10)

        # /scan は sensor QoS（本番の LiDAR と同じ BEST_EFFORT）。
        self._scan_mode = 'match'
        self.pub_scan = self.node.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self._scan_timer = self.node.create_timer(0.1, self._publish_scan)

        # /map_session/open の代役。要求の初期姿勢を記録し、成功を返す。
        self._open_requests = []
        self.srv_open = self.node.create_service(
            OpenMapSession, '/map_session/open', self._on_open)

        # odom 偽装（control が pose 無しで止まらないよう。TF 経路が主だが保険）。
        self.pub_odom = self.node.create_publisher(Odometry, '/odom', 10)
        self._odom_timer = self.node.create_timer(0.05, self._publish_odom)

        self._spin(0.5)
        # /system/event は transient ではないため、discovery が済む前に出た
        # 単発イベントは二度と届かない。先に /route/status の到着（ノード→試験の
        # 疎通の証拠）を待ってから load_route を送る（同じ discovery で event 側も
        # 繋がる）。待たずに送ると、low の 1 発を取りこぼして 25s 空振りする。
        self._wait_status(lambda s: True, timeout=10.0, what='最初の /route/status')
        self._state_value = 'LOCALIZE'
        if not self._testMethodName.startswith('test_d_'):
            self._send_load_route()
            self._spin(1.0)

    def tearDown(self):
        for name in ('_state_timer', '_scan_timer', '_odom_timer'):
            timer = getattr(self, name, None)
            if timer is not None:
                timer.cancel()
                self.node.destroy_timer(timer)
                setattr(self, name, None)
        self.node.destroy_service(self.srv_open)
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _on_event(self, msg):
        self._events.append(msg)

    def _publish_state(self):
        msg = SystemState()
        msg.mode = 'REPLAY' if self._state_value == 'LOCALIZE' else 'IDLE'
        msg.state = self._state_value
        self.pub_state.publish(msg)

    def _publish_scan(self):
        msg = LaserScan()
        msg.header.frame_id = 'laser'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.angle_min = _ANGLE_MIN
        msg.angle_max = _ANGLE_MIN + (_N_BEAMS - 1) * _ANGLE_INC
        msg.angle_increment = _ANGLE_INC
        msg.range_min = 0.1
        msg.range_max = 40.0
        src = _MATCH_RANGES if self._scan_mode == 'match' else _MISMATCH_RANGES
        msg.ranges = [float(v) for v in src]
        self.pub_scan.publish(msg)

    def _publish_odom(self):
        msg = Odometry()
        msg.pose.pose.position.x = _TRUTH[0]
        msg.pose.pose.position.y = _TRUTH[1]
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pub_odom.publish(msg)

    def _on_open(self, request, response):
        self._open_requests.append(request)
        response.success = True
        response.message = 'test stub ok'
        return response

    def _send_load_route(self):
        msg = StateEffect()
        msg.name = 'load_route'
        msg.args_json = json.dumps({'route_id': _ROUTE_ID, 'reverse': False})
        self.pub_effect.publish(msg)

    def _send_effect(self, name):
        msg = StateEffect()
        msg.name = name
        msg.args_json = '{}'
        self.pub_effect.publish(msg)

    def _wait_event(self, pred, timeout: float, what: str):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            hit = [e for e in self._events if pred(e)]
            if hit:
                return hit[-1]
        raise AssertionError(
            f'{what} を満たす /system/event が {timeout}s 来ない '
            f'(受信 {len(self._events)} 件: '
            f'{[e.event for e in self._events]})')

    def _wait_status(self, pred, timeout: float, what: str) -> RouteStatus:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.1)
            hit = [s for s in self._statuses if pred(s)]
            if hit:
                return hit[-1]
        raise AssertionError(
            f'{what} を満たす /route/status が {timeout}s 来ない '
            f'(受信 {len(self._statuses)} 件)')

    # ════════════════════════════════════════════════════════
    def test_a_initial_search_done_with_arg_and_searched_pose(self):
        """正しい位置のスキャン → done が score/margin 付きで出る。

        代役に渡された初期姿勢が探索結果（経路始点ではない）であること。
        探索結果を捨てて始点を渡す変異ではここが赤くなる。
        """
        ev = self._wait_event(
            lambda e: e.event == 'evt.localize_done', timeout=25.0,
            what='evt.localize_done')
        arg = json.loads(ev.arg_json or '{}')
        assert 'score' in arg and 'margin' in arg, (
            f'done に score/margin が無い: {ev.arg_json!r}')
        assert 0.0 <= arg['score'] <= 1.0 and arg['margin'] >= 0.0
        assert self._open_requests, 'reload（/map_session/open）が呼ばれていない'
        req = self._open_requests[-1]
        assert req.has_initial_pose, '初期姿勢なしで reload している'
        got = (req.initial_x, req.initial_y)
        # 探索結果（真値 (2.0, 2.5) 付近）であって経路始点 (0.5, 2.5) ではない。
        assert math.hypot(got[0] - _TRUTH[0], got[1] - _TRUTH[1]) <= 0.6, (
            f'reload の初期姿勢が探索結果でない: {got}')
        assert abs(got[0] - _START[0]) > 0.5, (
            f'reload の初期姿勢が経路始点のまま（探索結果を捨てている）: {got}')
        st = self._wait_status(
            lambda s: s.localize_quality in ('high', 'low_margin'),
            timeout=10.0, what="quality high/low_margin")
        assert st.localize_score == pytest.approx(arg['score'], abs=1e-3)
        assert st.localize_margin == pytest.approx(arg['margin'], abs=1e-3)

    def test_b_mismatched_scan_emits_low(self):
        """地図と合わないスキャン → evt.localize_low が出る。"""
        self._scan_mode = 'mismatch'
        # 切替後のスキャンがノードの保持する最新になるまで待つ（直後に
        # load_route すると切替前の match スキャンで探索してしまう）。
        self._spin(0.5)
        self._events.clear()
        self._open_requests.clear()
        self._send_load_route()
        ev = self._wait_event(
            lambda e: e.event == 'evt.localize_low', timeout=25.0,
            what='evt.localize_low')
        arg = json.loads(ev.arg_json or '{}')
        assert 'score' in arg and 'margin' in arg, (
            f'low に score/margin が無い: {ev.arg_json!r}')

    def test_c_global_failure_stays_without_done(self):
        """global で不成立 → done が出ない・quality が failed。

        不成立でも done を出す変異ではここが赤くなる。
        """
        self._scan_mode = 'mismatch'
        self._spin(0.5)
        self._events.clear()
        self._statuses.clear()
        self._send_load_route()
        # 初期探索の low を待ってから（FSM の代わりに）global を送る。
        self._wait_event(
            lambda e: e.event == 'evt.localize_low', timeout=25.0,
            what='evt.localize_low（global 送信用の前提）')
        self._events.clear()
        self._statuses.clear()
        self._send_effect('global_localize')
        st = self._wait_status(
            lambda s: s.localize_quality == 'failed',
            timeout=25.0, what="quality failed")
        assert st.localize_score == pytest.approx(0.0, abs=1e-6)
        dones = [e for e in self._events if e.event == 'evt.localize_done']
        assert not dones, (
            f'不成立なのに evt.localize_done が出た（{len(dones)} 件）')

    def test_d_search_discarded_when_replay_left(self):
        """探索中に REPLAY を抜けたら、古い探索は reload も結果通知もしない。

        reload 直前のガード（世代・mode/state）を外す変異ではここが赤くなる
        （代役の /map_session/open が呼ばれてしまう）。
        """
        self._events.clear()
        self._open_requests.clear()
        self._scan_mode = 'match'
        self._spin(0.3)
        self._send_load_route()
        # 探索（pgm 読み込み＋粗探索）が走っている間に IDLE へ抜ける。
        self._state_value = 'IDLE'
        self._publish_state()
        self._spin(12.0)  # 探索の完了を十分に待つ
        assert not self._open_requests, (
            f'REPLAY を抜けた後に reload が呼ばれた（{len(self._open_requests)} 件）')
        bad = [e.event for e in self._events
               if e.event in ('evt.localize_done', 'evt.localize_low')]
        assert not bad, f'REPLAY を抜けた後に探索結果のイベントが出た: {bad}'


if __name__ == '__main__':
    unittest.main()
