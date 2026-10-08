"""
test_transit_route_unusable_node.py
====================================
1b-11（SG-B19・SG-B14）— replay_runner を実際に起動し、経路が使えないときに
**無言で return せず原因別のイベントを出す**ことと、地図のある経路だけ
/route/status の has_map が真になることを縛る。

  a. 地図を持たない経路（map フレームなのに map_session_id が空）
     → evt.route_no_map が出る・has_map は偽・localize_quality=='failed'
  b. 読めない経路ファイル → evt.route_unreadable が出る・has_map は偽
  c. 地図のある経路 → 上記のどちらも出ない・has_map が真
     （state_manager の map_update_available の材料。偽に戻す変異で赤）

お手本は test_replay_localize_node.py（起動方法・疎通待ち）。
"""
import json
import os
import tempfile
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from th_system_msgs.msg import RouteStatus, StateEffect, StateEvent, SystemState
from th_system_msgs.srv import OpenMapSession

_SESSION_ID = 'test-sess'
_ROUTES_DIR = tempfile.mkdtemp(prefix='unusable-routes-')


def _route(rid, session):
    points = [[0.5 + i * 0.5, 2.5, 0.0] for i in range(10)]
    return {
        'id': rid, 'name': rid, 'generation': 1, 'length_m': 4.5,
        'point_count': len(points), 'start_yaw': 0.0, 'recorded_at_ms': 0,
        'frame_id': 'map', 'map_session_id': session, 'points': points,
    }


def _write(rid, body):
    with open(os.path.join(_ROUTES_DIR, rid + '.json'), 'w', encoding='utf-8') as f:
        f.write(body)


_write('nomap', json.dumps(_route('nomap', '')))
_write('broken', '{ this is not json')
_write('withmap', json.dumps(_route('withmap', _SESSION_ID)))


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


class TestTransitRouteUnusableNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_transit_route_unusable')
        self._events = []
        self.node.create_subscription(StateEvent, '/system/event', self._events.append, 10)
        self._statuses = []
        self.node.create_subscription(RouteStatus, '/route/status', self._statuses.append, 10)
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_state = self.node.create_publisher(SystemState, '/system/state', state_qos)
        self._timer = self.node.create_timer(0.1, self._publish_state)
        self.pub_effect = self.node.create_publisher(StateEffect, '/system/effect', 10)
        self.srv_open = self.node.create_service(
            OpenMapSession, '/map_session/open', self._on_open)
        self._spin(0.5)
        self._wait_status(lambda s: True, 10.0, '最初の /route/status')

    def tearDown(self):
        self._timer.cancel()
        self.node.destroy_timer(self._timer)
        self.node.destroy_service(self.srv_open)
        self.node.destroy_node()

    def _publish_state(self):
        msg = SystemState()
        msg.mode = 'REPLAY'
        msg.state = 'LOCALIZE'
        self.pub_state.publish(msg)

    def _on_open(self, request, response):
        response.success = True
        response.message = 'stub'
        return response

    def _spin(self, duration=0.1):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_status(self, pred, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            hit = [s for s in self._statuses if pred(s)]
            if hit:
                return hit[-1]
        raise AssertionError(f'{what} を満たす /route/status が来ない')

    def _load(self, rid):
        msg = StateEffect()
        msg.name = 'load_route'
        msg.args_json = json.dumps({'route_id': rid, 'reverse': False})
        self.pub_effect.publish(msg)

    def _names(self):
        return [e.event for e in self._events]

    def _wait_event(self, name, timeout=8.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if name in self._names():
                return True
        return False

    def test_a_route_without_map_emits_route_no_map(self):
        self._load('nomap')
        assert self._wait_event('evt.route_no_map'), (
            f'地図の無い経路で evt.route_no_map が出ない: {self._names()}')
        assert 'evt.route_unreadable' not in self._names()
        st = self._wait_status(lambda s: s.localize_quality == 'failed', 5.0, 'failed')
        assert st.has_map is False

    def test_b_unreadable_route_emits_route_unreadable(self):
        self._load('broken')
        assert self._wait_event('evt.route_unreadable'), (
            f'読めない経路で evt.route_unreadable が出ない: {self._names()}')
        assert 'evt.route_no_map' not in self._names()
        st = self._wait_status(lambda s: s.localize_quality == 'failed', 5.0, 'failed')
        assert st.has_map is False

    def test_c_route_with_map_reports_has_map(self):
        self._load('withmap')
        st = self._wait_status(
            lambda s: s.points > 0 and s.current.id == 'withmap' and s.has_map,
            10.0, 'has_map が真の /route/status')
        assert st.has_map is True
        assert 'evt.route_no_map' not in self._names()
        assert 'evt.route_unreadable' not in self._names()
