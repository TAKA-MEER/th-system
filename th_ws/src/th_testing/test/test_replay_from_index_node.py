"""
test_replay_from_index_node.py
=============================
W-01 P4（経路の途中から再開する from_index＋向き合わせの再利用）の
Docker launch テスト。replay_runner を実際に起動し、テスト側が
/map_session/open の代役サービス・/scan・既知の pgm・map→base_link TF を
用意して次を確認する（再開 index の決定は test_route_replay_core.py が
持ち、ここではノード本体の配線——確定姿勢→_from_index→_start_yaw→
rotate／RUN／遠すぎたら留まる——を見る。本番の経路で縛る。
ソースの文字列検査だけだと「向きを始点のままにする」「離れすぎの判定を
外す」変異がすり抜けるため）:

  a. 経路途中の姿勢で確定 → rotate が再開点の向き（北）へ回る（w>0）。
     始点の向き（東）のままなら w<0 になる配置にしてある。
     その後 RUN で再開点から追従し、終端まで進んで evt.arrived が出る。
  b. 始点付近の姿勢で確定 → 今までどおり index 0 付近・始点の向きへ回る。
  c. 経路から遠い姿勢で確定（スキャンは合う） → evt.localize_done が出ない・
     localize_quality=='failed'（LOCALIZE に留まる）。
  d. 逆再生でも 1 本: 反転後の点列で確定 → 再開点の向き（南）へ回る（w<0）。

地図は 6x5 m の部屋＋間仕切り（test_replay_localize_node.py と同配置）。
経路は L 字: (0.5,1.0)→(2.5,1.0)→(2.5,4.0)。始点の向きは東 (0.0)、
北向き区間の点の yaw は pi/2。

お手本は test_replay_localize_node.py（探索まわり）と
test_replay_cross_track_node.py（RUN まわり）。
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
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import LaserScan
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from th_system_msgs.msg import RouteStatus, StateEffect, StateEvent, SystemState
from th_system_msgs.srv import OpenMapSession


_ROUTE_ID = 'from-index-test'
_SESSION_ID = 'test-sess'
# a/d の真値（laser 姿勢。北向き区間のそば。向きは東寄り 0.3）
_TRUTH = (2.5, 3.0, 0.3)
# d（逆再生）の真値（同じ位置。反転後の南向き区間に沿う向き）
_TRUTH_REV = (2.5, 3.0, -math.pi / 2 + 0.2)
# c（遠い。部屋の中だが経路から 2m 超。スキャンは合う場所）
_FAR = (5.0, 2.5, 0.0)
_N_BEAMS = 360
_ANGLE_MIN = -math.pi
_ANGLE_INC = 2.0 * math.pi / _N_BEAMS
_RES = 0.05


def _occ():
    """部屋＋間仕切り（test_replay_localize_node.py の _occ と同配置）。"""
    occ = np.zeros((100, 120), dtype=bool)
    occ[0, :] = True
    occ[99, :] = True
    occ[:, 0] = True
    occ[:, 119] = True
    occ[20:61, 80] = True
    return occ


_OCC = _occ()


def _route_points():
    """L 字: 東へ (0.5,1.0)→(2.5,1.0)、北へ (2.5,1.5)→(2.5,4.0)。"""
    pts = [[0.5 + i * 0.5, 1.0, 0.0] for i in range(5)]
    pts += [[2.5, 1.5 + j * 0.5, math.pi / 2] for j in range(6)]
    return pts


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
    points = _route_points()
    route = {
        'id': _ROUTE_ID,
        'name': '途中復帰試験用',
        'generation': 1,
        'length_m': 7.0,
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


_ROUTES_DIR = tempfile.mkdtemp(prefix='from-index-routes-')
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
            'resume_max_dist_m': 2.0,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        runner,
        launch_testing.actions.ReadyToTest(),
    ]), {'replay_runner': runner}


class TestReplayFromIndexNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_replay_from_index_node')

        self._events = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._on_event, 10)
        self._statuses = []
        self.node.create_subscription(
            RouteStatus, '/route/status', self._statuses.append, 10)
        self._cmds = []
        self.node.create_subscription(
            Twist, '/cmd_vel_behavior', self._cmds.append, 10)

        # /system/state は REPLAY を流し続ける（止めると status が出なくなる）。
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._fsm_state = 'LOCALIZE'
        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', state_qos)
        self._state_timer = self.node.create_timer(0.1, self._publish_state)

        self.pub_effect = self.node.create_publisher(
            StateEffect, '/system/effect', 10)

        # /scan は sensor QoS。撮影ポーズはテストごとに変える。
        self._scan_pose = _TRUTH
        self.pub_scan = self.node.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self._scan_timer = self.node.create_timer(0.1, self._publish_scan)

        # /map_session/open の代役。成功を返す（TF 到着で done へ進む）。
        self.srv_open = self.node.create_service(
            OpenMapSession, '/map_session/open', self._on_open)

        # map→base_link TF を流す（追従・旋回が読む現在 pose）。
        self._tf_pose = _TRUTH
        self._tf_broadcaster = TransformBroadcaster(self.node)
        self._tf_timer = self.node.create_timer(0.05, self._publish_tf)

        self._spin(0.5)
        # /system/event は非 transient のため、discovery が済む前に出た単発は
        # 届かない。先に /route/status の到着を待ってから load_route を送る。
        self._wait_status(lambda s: True, timeout=10.0, what='最初の /route/status')

    def tearDown(self):
        for name in ('_state_timer', '_scan_timer', '_tf_timer'):
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
        msg.mode = 'REPLAY'
        msg.state = self._fsm_state
        self.pub_state.publish(msg)

    def _publish_scan(self):
        ranges = _raycast(*self._scan_pose)
        msg = LaserScan()
        msg.header.frame_id = 'laser'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.angle_min = _ANGLE_MIN
        msg.angle_max = _ANGLE_MIN + (_N_BEAMS - 1) * _ANGLE_INC
        msg.angle_increment = _ANGLE_INC
        msg.range_min = 0.1
        msg.range_max = 40.0
        msg.ranges = [float(v) for v in ranges]
        self.pub_scan.publish(msg)

    def _publish_tf(self):
        x, y, yaw = self._tf_pose
        msg = TransformStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.transform.translation.x = float(x)
        msg.transform.translation.y = float(y)
        msg.transform.translation.z = 0.0
        msg.transform.rotation.z = math.sin(yaw / 2.0)
        msg.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_broadcaster.sendTransform(msg)

    def _on_open(self, request, response):
        response.success = True
        response.message = 'test stub ok'
        return response

    def _send_load_route(self, reverse=False):
        msg = StateEffect()
        msg.name = 'load_route'
        msg.args_json = json.dumps({'route_id': _ROUTE_ID, 'reverse': reverse})
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
    def test_a_mid_route_resume_rotates_to_resume_yaw(self):
        """経路途中で確定 → 再開点の向き（北）へ回り、再開点から終端へ進む。

        向きを始点のヨーのままにする変異では、最初の旋回が逆向き (w<0) になり
        ここが赤くなる。
        """
        self._scan_pose = _TRUTH
        self._tf_pose = _TRUTH
        self._spin(0.3)
        self._send_load_route(reverse=False)
        ev = self._wait_event(
            lambda e: e.event == 'evt.localize_done', timeout=60.0,
            what='evt.localize_done')
        arg = json.loads(ev.arg_json or '{}')
        assert 'score' in arg and 'margin' in arg
        self._send_effect('rotate_to_start_yaw')
        self._fsm_state = 'RUN'
        # 最初の指令を見る（古い status／cmd を捨ててから）。
        self._cmds.clear()
        self._spin(1.5)
        rots = [c for c in self._cmds]
        assert rots, '/cmd_vel_behavior が来ない（RUN 中の rotate が動いていない）'
        pos = [c for c in rots if c.angular.z > 0.01]
        assert len(pos) >= len(rots) // 2, (
            f'再開点の向き（北）へ回っていない（w>0 が {len(pos)}/{len(rots)} 件）。'
            f'始点の向きのままなら w<0 になるはず: '
            f'{[round(c.angular.z, 3) for c in rots[:8]]}')
        assert all(abs(c.linear.x) < 0.05 for c in rots), (
            '旋回中に前進している（超信地旋回でない）')
        # 回り切ったことにする（TF の向きを再開点の向きへ）。その後 RUN で
        # 再開点から追従する。最初の目標点は再開点 (8) 以降。
        self._tf_pose = (2.5, 3.0, math.pi / 2)
        self._spin(1.0)
        self._send_effect('resume_path')
        st = self._wait_status(
            lambda s: s.target_index >= 7, timeout=10.0,
            what='再開点以降の target_index')
        assert st.target_index >= 7
        # 終端まで進む（TF を終端近くへ歩める）と evt.arrived が出る。
        self._tf_pose = (2.5, 3.85, math.pi / 2)
        self._spin(0.5)
        self._wait_event(
            lambda e: e.event == 'evt.arrived', timeout=10.0,
            what='evt.arrived')

    def test_b_start_area_resume_is_unchanged(self):
        """始点付近で確定 → 今までどおり index 0 付近・始点の向きへ回る。"""
        self._scan_pose = (0.7, 1.0, 0.5)
        self._tf_pose = (0.7, 1.0, 0.5)
        self._spin(0.3)
        self._send_load_route(reverse=False)
        self._wait_event(
            lambda e: e.event == 'evt.localize_done', timeout=60.0,
            what='evt.localize_done（始点付近）')
        self._send_effect('rotate_to_start_yaw')
        self._fsm_state = 'RUN'
        self._cmds.clear()
        self._spin(1.5)
        rots = [c for c in self._cmds]
        assert rots, '/cmd_vel_behavior が来ない'
        neg = [c for c in rots if c.angular.z < -0.01]
        assert len(neg) >= len(rots) // 2, (
            f'始点の向き（東）へ回っていない（w<0 が {len(neg)}/{len(rots)} 件）: '
            f'{[round(c.angular.z, 3) for c in rots[:8]]}')

    def test_c_far_pose_stays_in_localize(self):
        """経路から遠い姿勢で確定 → done が出ない・quality が failed。

        離れすぎの判定を外す変異では done が出てここが赤くなる。
        """
        self._scan_pose = _FAR
        self._tf_pose = _FAR
        self._spin(0.3)
        self._send_load_route(reverse=False)
        st = self._wait_status(
            lambda s: s.localize_quality == 'failed',
            timeout=60.0, what="quality failed（遠すぎ）")
        assert st.localize_score >= 0.0
        dones = [e for e in self._events if e.event == 'evt.localize_done']
        assert not dones, (
            f'経路から遠いのに evt.localize_done が出た（{len(dones)} 件）')

    def test_d_reverse_resume_rotates_to_resume_yaw(self):
        """逆再生でも 1 本: 反転後の点列で確定 → 再開点の向き（南）へ回る。

        逆再生で反転前の点列を使う変異では、向きが北のまま (w>0) になり
        ここが赤くなる。
        """
        self._scan_pose = _TRUTH_REV
        self._tf_pose = _TRUTH_REV
        self._spin(0.3)
        self._send_load_route(reverse=True)
        self._wait_event(
            lambda e: e.event == 'evt.localize_done', timeout=60.0,
            what='evt.localize_done（逆再生）')
        self._send_effect('rotate_to_start_yaw')
        self._fsm_state = 'RUN'
        self._cmds.clear()
        self._spin(1.5)
        rots = [c for c in self._cmds]
        assert rots, '/cmd_vel_behavior が来ない'
        neg = [c for c in rots if c.angular.z < -0.01]
        assert len(neg) >= len(rots) // 2, (
            f'再開点の向き（南）へ回っていない（w<0 が {len(neg)}/{len(rots)} 件）: '
            f'{[round(c.angular.z, 3) for c in rots[:8]]}')


if __name__ == '__main__':
    unittest.main()
