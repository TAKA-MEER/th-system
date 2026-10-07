"""
test_replay_localize_registry_node.py
=====================================
W-01 P5（全域ローカライズの数値の registry 化）の Docker launch テスト。
replay_runner を**生成 yaml を先に渡して**実際に起動し、registry の値が
本番の経路（生成 yaml → ROS パラメータ）でノードへ届いていることを縛る
（dict で上書きして本番経路を迂回しない。お手本は
test_replay_localize_node.py。パラメータの読み戻しは
test_runaway_freshness_node.py の起動方法に倣う）:

  a. 生成 yaml の 6 値がノードのパラメータとして読める
  b. 途中復帰の上限（resume_max_dist_m）が振る舞いに効く
     （上限 0.0 では確定しても LOCALIZE に留まり done が出ない。
     ノードが値を読まず既定値 2.0 を使う変異では done が出て赤くなる）

本番の registry.yaml は placeholder の行を含むため、試験用の一時
registry（値だけを区別できる数値に置き換え。値は変えないのは本番の
registry の話で、試験用写しは risks を取らない）から生成する。
生成は `params_generation.run_generation()`（launch と同じ関数）。
"""
import copy
import json
import math
import os
import sys
import tempfile
import time
import unittest

import numpy as np
import pytest
import rclpy
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import LaserScan

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from th_system_msgs.msg import RouteStatus, StateEffect, StateEvent, SystemState
from th_system_msgs.srv import OpenMapSession


_REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_LAUNCH_DIR = os.path.join(_REPO_SRC, "th_bringup", "launch")
_PARAMS_SRC = os.path.join(_REPO_SRC, "th_params")

for _p in (_LAUNCH_DIR, _PARAMS_SRC):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402

REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")

# 試験用の値（本番の既定値と区別できる数値。届いたかどうかを見るため）。
# resume 0.0 は「確定しても必ず遠すぎ」→ failed に留まる。
_TEST_VALUES = {
    "localize_match_low": 0.97,
    "localize_margin_low": 0.91,
    "localize_margin_min": 0.01,
    "search_radius_m": 4.0,
    "widen_radius_m": 9.0,
    "resume_max_dist_m": 0.0,
}
_TEST_NAMES = sorted(_TEST_VALUES)

_ROUTE_ID = 'loc-reg-test'
_SESSION_ID = 'test-reg-sess'
_TRUTH = (2.0, 2.5, 0.3)
_START = (0.5, 2.5, 0.0)
_N_BEAMS = 360
_ANGLE_MIN = -math.pi
_ANGLE_INC = 2.0 * math.pi / _N_BEAMS
_RES = 0.05


def _occ():
    """部屋＋間仕切り（test_replay_localize_node.py と同配置）。"""
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
        'name': 'registry 経由試験用',
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


def _make_test_registry_and_generate():
    """試験用 registry 写し（6 値を区別できる数値に）から生成する。

    本番の registry.yaml は変えない。placeholder の 4 行は試験用の写しで
    given に読み替える（値の上書きの試験であって schema の試験ではない。
    schema 自体は test_w01p5_localize_registry.py が本番で縛る）。
    戻り値は (生成 replay_runner.yaml のパス, 期待値 dict)。
    """
    tmp = tempfile.mkdtemp(prefix='w01p5-reg-')
    with open(REGISTRY_YAML, encoding='utf-8') as f:
        rows = yaml.safe_load(f)
    for row in rows:
        if row.get('name') in _TEST_VALUES:
            row['class'] = 'b'
            row['status'] = 'given'
            row['value'] = float(_TEST_VALUES[row['name']])
            row.pop('blocking', None)
            row.pop('blocking_from_stage', None)
    reg_path = os.path.join(tmp, 'registry.yaml')
    with open(reg_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(rows, f, allow_unicode=True, sort_keys=True)
    out_dir = os.path.join(tmp, 'generated')
    env = dict(os.environ)
    env['PYTHONPATH'] = _PARAMS_SRC + os.pathsep + env.get('PYTHONPATH', '')
    pg.run_generation(stage=4, sim=False, nodes=list(pg.REGISTRY_NODES),
                      out_dir=out_dir, registry_path=reg_path, env=env,
                      calib_dir=os.path.join(tmp, 'no_calib'))
    gen_path = os.path.join(out_dir, 'replay_runner.yaml')
    with open(gen_path, encoding='utf-8') as f:
        params = yaml.safe_load(f)['replay_runner']['ros__parameters']
    for name, value in _TEST_VALUES.items():
        assert params.get(name) == value, (
            f"試験の生成物に {name}={value} が載っていない（{params.get(name)!r}）")
    return gen_path


_ROUTES_DIR = tempfile.mkdtemp(prefix='loc-reg-routes-')
_write_fixtures(_ROUTES_DIR)
_GENERATED_YAML = _make_test_registry_and_generate()


@pytest.mark.launch_test
def generate_test_description():
    runner = launch_ros.actions.Node(
        package='th_planning',
        executable='replay_runner.py',
        name='replay_runner',
        # registry 駆動の 6 値は生成 yaml からのみ渡す（dict で上書きしない）。
        # routes_dir 等は registry の対象外の配線設定のため dict で渡す
        # （replay_runner.py のコメント参照）。
        parameters=[_GENERATED_YAML, {
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


class TestReplayLocalizeRegistryNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_replay_localize_registry_node')

        self._events = []
        self.node.create_subscription(
            StateEvent, '/system/event', self._on_event, 10)
        self._statuses = []
        self.node.create_subscription(
            RouteStatus, '/route/status', self._statuses.append, 10)

        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', state_qos)
        self._state_timer = self.node.create_timer(0.1, self._publish_state)

        self.pub_effect = self.node.create_publisher(
            StateEffect, '/system/effect', 10)

        self.pub_scan = self.node.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self._scan_timer = self.node.create_timer(0.1, self._publish_scan)

        self._open_requests = []
        self.srv_open = self.node.create_service(
            OpenMapSession, '/map_session/open', self._on_open)

        self.pub_odom = self.node.create_publisher(Odometry, '/odom', 10)
        self._odom_timer = self.node.create_timer(0.05, self._publish_odom)

        self._spin(0.5)
        self._wait_status(lambda s: True, timeout=10.0, what='最初の /route/status')

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
        msg.mode = 'REPLAY'
        msg.state = 'LOCALIZE'
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
        msg.ranges = [float(v) for v in _MATCH_RANGES]
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

    def _read_node_params(self) -> dict:
        """起動中ノードのパラメータを本番の取得経路で読む（値＋型）。
        未宣言のパラメータは NOT_SET（double_value 0.0）で返るため、
        値だけでなく型（DOUBLE）も見る。resume の試験値が 0.0 なので、
        型を見ないと「宣言忘れ」が緑に化ける。"""
        from rclpy.parameter import Parameter
        cli = AsyncParameterClient(self.node, 'replay_runner')
        assert cli.wait_for_service(timeout_sec=10.0), (
            'replay_runner のパラメータサービスに繋がらない')
        fut = cli.get_parameter_types(_TEST_NAMES)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        assert fut.done() and fut.result() is not None
        for name, t in zip(_TEST_NAMES, fut.result().types):
            assert t == Parameter.Type.DOUBLE, (
                f'{name}: 型が DOUBLE でない（{t}）。INTEGER↔DOUBLE の罠')
        fut = cli.get_parameters(_TEST_NAMES)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        assert fut.done() and fut.result() is not None, (
            'パラメータの読み戻しが終わらない')
        return {name: p.double_value
                for name, p in zip(_TEST_NAMES, fut.result().values)}

    # ════════════════════════════════════════════════════════
    def test_a_params_arrive_via_generated_yaml(self):
        """生成 yaml の 6 値がノードのパラメータとして読める。
        registry→生成→ノードの本番経路。dict 上書きは使わない。"""
        got = self._read_node_params()
        for name in _TEST_NAMES:
            assert got[name] == _TEST_VALUES[name], (
                f'{name}: ノードの値 {got[name]!r} != 生成 yaml {_TEST_VALUES[name]!r}')

    def test_b_resume_limit_blocks_distant_start(self):
        """途中復帰の上限（resume_max_dist_m=0.0）が振る舞いに効く。
        探索は成立する（score が試験用の match_low 0.97 以上）が、再開点が
        見つからず LOCALIZE に留まり done が出ない。ノードが値を読まず
        既定値 2.0 を使う変異では done が出て赤くなる。"""
        self._send_load_route()
        st = self._wait_status(
            lambda s: s.localize_quality == 'failed',
            timeout=30.0, what="quality failed")
        assert st.localize_score >= 0.97, (
            f'探索自体が不成立（score={st.localize_score}）。試験用の match_low '
            f'0.97 を上回るはずの一致スキャンで low になった')
        dones = [e for e in self._events if e.event == 'evt.localize_done']
        assert not dones, (
            f'resume 上限 0.0 なのに evt.localize_done が出た（{len(dones)} 件）。'
            f'ノードが生成 yaml の値を読んでいない')


if __name__ == '__main__':
    unittest.main()
