"""
test_replay_cross_track_node.py
===============================
S-14「経路からのずれ」（Spec-webui.md §3.7 / Spec-transit.md §4.4）の
Docker launch テスト。replay_runner を実際に起動し、/odom を偽装して
次を確認する（純粋関数の試験は test_route_replay_core.py が持ち、ここでは
ノード本体の配線——毎 tick の計測・最大値の保持・load_route での
リセット——を見る。静的な配線検査だけだと「最大値を保持しない」
「リセットしない」変異がすり抜けるため）:
  a. 経路読み込み直後 → ずれ・最大値ともほぼ 0（現在地合わせで始点一致）
  b. /odom を横に 0.5m ずらす → ずれが増え、最大値が残る
  c. load_route をもう一度 → ずれ・最大値が 0 に戻る

お手本は test_runaway_freshness_node.py（起動方法・test_a_ 順序制御）。

注意（CLAUDE.md の既知の癖）: launch_testing の pytest プラグインが
ファイル全体を1つのアイテムとして収集するため、試験は TestCase の
メソッドにする。モジュール直下の素の関数は収集されない。
"""
import json
import os
import tempfile
import time
import unittest

import pytest
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from th_system_msgs.msg import RouteStatus, StateEffect, SystemState


_ROUTE_ID = 'xtrack-test'
_LATERAL_M = 0.5


def _write_route(routes_dir: str) -> None:
    """x=0..10 の直線経路（odom フレーム）を routes_dir に書く。"""
    points = [[i * 0.5, 0.0, 0.0] for i in range(21)]
    route = {
        'id': _ROUTE_ID,
        'name': '横ずれ試験用',
        'generation': 1,
        'length_m': 10.0,
        'point_count': len(points),
        'start_yaw': 0.0,
        'recorded_at_ms': 0,
        'frame_id': 'odom',
        'map_session_id': '',
        'points': points,
    }
    with open(os.path.join(routes_dir, _ROUTE_ID + '.json'),
              encoding='utf-8', mode='w') as f:
        json.dump(route, f)


_ROUTES_DIR = tempfile.mkdtemp(prefix='xtrack-routes-')
_write_route(_ROUTES_DIR)


@pytest.mark.launch_test
def generate_test_description():
    runner = launch_ros.actions.Node(
        package='th_planning',
        executable='replay_runner.py',
        name='replay_runner',
        parameters=[{
            'routes_dir': _ROUTES_DIR,
            'use_map_frame': False,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        runner,
        launch_testing.actions.ReadyToTest(),
    ]), {'replay_runner': runner}


class TestReplayCrossTrackNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_replay_cross_track_node')

        self._statuses: list[RouteStatus] = []
        self.node.create_subscription(
            RouteStatus, '/route/status', self._statuses.append, 10)

        # /system/state は REPLAY/RUN を流し続ける（止めると status が
        # owns_route_status で出なくなり、control も止まる）。
        # replay_runner 側の購読は transient_local のため、合わせないと
        # DURABILITY 不一致で届かない（本番の state_manager も transient_local）。
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_state = self.node.create_publisher(
            SystemState, '/system/state', state_qos)
        self._state_timer = self.node.create_timer(0.1, self._publish_state)

        self.pub_effect = self.node.create_publisher(
            StateEffect, '/system/effect', 10)

        # /odom 偽装。mutable な pose をタイマで流し続ける。
        self._odom_xy = [0.0, 0.0]
        self.pub_odom = self.node.create_publisher(Odometry, '/odom', 10)
        self._odom_timer = self.node.create_timer(0.05, self._publish_odom)

        # odom が溜まってから load_route（odom 未受信だと現在地合わせを
        # スキップして経路がずれたままになる）。
        self._spin(0.5)
        self._send_load_route()
        # load_route の処理＋最初の status を待つ。test_a が条件を縛る。
        self._spin(1.0)

    def tearDown(self):
        for name in ('_state_timer', '_odom_timer'):
            timer = getattr(self, name, None)
            if timer is not None:
                timer.cancel()
                self.node.destroy_timer(timer)
                setattr(self, name, None)
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.1):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_state(self):
        msg = SystemState()
        msg.mode = 'REPLAY'
        msg.state = 'RUN'
        self.pub_state.publish(msg)

    def _publish_odom(self):
        msg = Odometry()
        msg.pose.pose.position.x = self._odom_xy[0]
        msg.pose.pose.position.y = self._odom_xy[1]
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pub_odom.publish(msg)

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

    # ════════════════════════════════════════════════════════
    def test_a_loaded_routes_zero(self):
        """経路読み込み直後 → ずれ・最大値ともほぼ 0。"""
        st = self._wait_status(lambda s: s.points > 0, timeout=8.0,
                               what='points>0')
        assert st.cross_track_m == pytest.approx(0.0, abs=0.05), (
            f'読み込み直後のずれが 0 でない: {st.cross_track_m}')
        assert st.cross_track_max_m == pytest.approx(0.0, abs=0.05), (
            f'読み込み直後の最大値が 0 でない: {st.cross_track_max_m}')

    def test_b_lateral_offset_grows_and_latches_max(self):
        """odom を横に 0.5m → ずれが増え、最大値に残る。"""
        self._wait_status(lambda s: s.points > 0, timeout=8.0, what='points>0')
        self._odom_xy[1] = _LATERAL_M
        st = self._wait_status(
            lambda s: s.cross_track_m >= _LATERAL_M - 0.1,
            timeout=8.0, what=f'ずれ>={_LATERAL_M - 0.1}')
        assert st.cross_track_m == pytest.approx(_LATERAL_M, abs=0.1), (
            f'ずれが横ずれ量と合わない: {st.cross_track_m}')
        assert st.cross_track_max_m >= _LATERAL_M - 0.1, (
            f'最大値が残っていない: {st.cross_track_max_m}')

    def test_c_reload_resets_to_zero(self):
        """load_route をもう一度 → ずれ・最大値が 0 に戻る。"""
        self._wait_status(lambda s: s.points > 0, timeout=8.0, what='points>0')
        # まず最大値を育てる（test_b と同じ横ずれ。順序に依存しない）。
        self._odom_xy[1] = _LATERAL_M
        grown = self._wait_status(
            lambda s: s.cross_track_max_m >= _LATERAL_M - 0.1,
            timeout=8.0, what='最大値の育ち')
        assert grown.cross_track_max_m > 0.3
        # 次の再生を始めたら 0 から数え直す。読み直しで経路が現在地に
        # 合わせ直されるため、ずれ自体も 0 に戻る。
        self._statuses.clear()
        self._send_load_route()
        st = self._wait_status(
            lambda s: s.points > 0 and s.cross_track_max_m == 0.0,
            timeout=8.0, what='リセット後の最大値 0')
        assert st.cross_track_m == pytest.approx(0.0, abs=0.05), (
            f'リセット後のずれが 0 でない: {st.cross_track_m}')

if __name__ == '__main__':
    unittest.main()
