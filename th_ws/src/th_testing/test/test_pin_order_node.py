"""
test_pin_order_node.py — 配電盤の順番変更（R-5。D60・D64）の launch_testing 試験。

`/onsite/edit_pin` の `move`（1つずつ前後へ）と `order_ids`（まとめて決める）で
順番が変わり、pins.yaml に保存され、/onsite/pins の配列の並びに載ること。
再起動しなくても読み直しで順番が残る（起動時に pins.yaml を読む本番経路）。

本番ノード（`pin_registrar`）そのものを起動する。
"""

from __future__ import annotations

import os
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)

from th_system_msgs.msg import PinList
from th_system_msgs.srv import EditPin


_TH_TESTING_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
_WS_ROOT = os.path.abspath(os.path.join(_TH_TESTING_ROOT, '..', '..'))
_WORKTREE_ROOT = os.path.abspath(os.path.join(_WS_ROOT, '..'))


def _find_generated_dir() -> str:
    candidates = [
        os.environ.get('TH_GENERATED_DIR', ''),
        '/root/th_data/generated',
        os.path.join(_WS_ROOT, 'data', 'generated'),
    ]
    for path in candidates:
        if path and os.path.isdir(path):
            return path
    pytest.fail('生成パラメータ置き場が見つからない。')


_GENERATED = _find_generated_dir()
_VENUE_DIR = os.path.join(_WORKTREE_ROOT, '.briefs', 'tmp', 'order_venue')


def _prepare_venue_dir() -> None:
    os.makedirs(_VENUE_DIR, exist_ok=True)
    pins = {
        'pins': [
            {'id': 'home', 'name': '待機場所', 'kind': 'HOME',
             'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'registered_at': 0},
            {'id': 'p1', 'name': '盤1', 'kind': 'PANEL',
             'pose': {'x': 1.0, 'y': 0.0, 'yaw': 0.0}, 'registered_at': 0},
            {'id': 'p2', 'name': '盤2', 'kind': 'PANEL',
             'pose': {'x': 2.0, 'y': 0.0, 'yaw': 0.0}, 'registered_at': 0},
            {'id': 'p3', 'name': '盤3', 'kind': 'PANEL',
             'pose': {'x': 3.0, 'y': 0.0, 'yaw': 0.0}, 'registered_at': 0},
        ],
        'map_instance_id': '',
    }
    with open(os.path.join(_VENUE_DIR, 'pins.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(pins, f, allow_unicode=True, default_flow_style=False)


_prepare_venue_dir()


def _tl_qos() -> QoSProfile:
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST)


@pytest.mark.launch_test
def generate_test_description():
    registrar = launch_ros.actions.Node(
        package='th_onsite',
        executable='pin_registrar.py',
        name='pin_registrar',
        parameters=[os.path.join(_GENERATED, 'pin_registrar.yaml'),
                    {'venue_dir': _VENUE_DIR}],
        output='screen',
    )
    return launch.LaunchDescription([
        registrar,
        launch_testing.actions.ReadyToTest(),
    ]), {'pin_registrar': registrar}


class TestPinOrder(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        _prepare_venue_dir()
        self.node = rclpy.create_node('order_client')
        self._lock = threading.Lock()
        self.pins = []
        cbg = ReentrantCallbackGroup()
        self.node.create_subscription(
            PinList, '/onsite/pins', self._on_pins, _tl_qos(),
            callback_group=cbg)
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin,
                                             daemon=True)
        self._spin_thread.start()
        time.sleep(2.0)

    def tearDown(self):
        try:
            self._executor.shutdown()
        except Exception:
            pass
        self._spin_thread.join(timeout=5.0)
        self.node.destroy_node()

    def _on_pins(self, msg: PinList):
        with self._lock:
            self.pins = list(msg.pins)

    def _call_edit(self, **kwargs):
        cli = self.node.create_client(EditPin, '/onsite/edit_pin')
        try:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.fail('/onsite/edit_pin が 10s 待っても現れない')
            req = EditPin.Request()
            req.id = kwargs.get('id', '')
            req.new_name = kwargs.get('new_name', '')
            req.is_delete = kwargs.get('is_delete', False)
            req.update_pose = kwargs.get('update_pose', False)
            req.x = float(kwargs.get('x', 0.0))
            req.y = float(kwargs.get('y', 0.0))
            req.yaw = float(kwargs.get('yaw', 0.0))
            req.move = int(kwargs.get('move', 0))
            req.order_ids = list(kwargs.get('order_ids', []))
            fut = cli.call_async(req)
            deadline = time.monotonic() + 8.0
            while not fut.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not fut.done():
                self.fail('/onsite/edit_pin の応答が 8s 来ない')
            return fut.result()
        finally:
            self.node.destroy_client(cli)

    def _wait_ids(self, want, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                got = [p.id for p in self.pins]
            if got == want:
                return True
            time.sleep(0.05)
        return False

    def _read_file_ids(self):
        with open(os.path.join(_VENUE_DIR, 'pins.yaml'), encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return [p['id'] for p in data['pins']]

    def test_move_forward_back_persists_in_array_order(self):
        res = self._call_edit(id='p2', move=-1)
        self.assertTrue(res.success, res.message)
        self.assertTrue(self._wait_ids(['home', 'p2', 'p1', 'p3']),
                        '/onsite/pins の並びが前へ動かない')
        self.assertEqual(self._read_file_ids(), ['home', 'p2', 'p1', 'p3'],
                         'pins.yaml に順番が保存されていない')
        res = self._call_edit(id='p2', move=1)
        self.assertTrue(res.success, res.message)
        self.assertTrue(self._wait_ids(['home', 'p1', 'p2', 'p3']))

    def test_reorder_pressed_order_then_rest(self):
        res = self._call_edit(id='p1', order_ids=['p3', 'p1'])
        self.assertTrue(res.success, res.message)
        self.assertTrue(self._wait_ids(['home', 'p3', 'p1', 'p2']),
                        '押した順＋残り（現在の順）の並びにならない')
        self.assertEqual(self._read_file_ids(), ['home', 'p3', 'p1', 'p2'])

    def test_reorder_rejects_unknown_and_edge(self):
        # ノードのメモリ内は前試験の並びのままなので、現在の先頭の盤で測る。
        before = self._read_file_ids()
        res = self._call_edit(id='p1', order_ids=['p2', 'nope'])
        self.assertFalse(res.success)
        first_panel = None
        with self._lock:
            for p in self.pins:
                if p.kind == 'PANEL':
                    first_panel = p.id
                    break
        self.assertIsNotNone(first_panel)
        res = self._call_edit(id=first_panel, move=-1)
        self.assertFalse(res.success, '端の盤を外へ動かせた')
        res = self._call_edit(id='home', move=1)
        self.assertFalse(res.success, 'HOME の順番を変えられた')
        self.assertEqual(self._read_file_ids(), before,
                         '拒否したのに pins.yaml が変わった')


if __name__ == '__main__':
    unittest.main()
