"""
test_pin_reregister_node.py — ピンの再登録（SG-B16・1b-11②）の launch_testing 試験。

`/onsite/edit_pin` の `update_pose=true` で、同じ id・同じ名前のまま
位置と向きだけが変わること（Spec-onsite.md §2.3）。存在しない id は拒否。
再登録で新しい id を振る変異は赤くなる（id 不変を assert）。

本番ノード（`pin_registrar`）そのものを起動する。`/onsite/pins`
（transient_local）を読んで永続化・配信まで縛る（単なる応答の文字列検査にしない）。
"""

from __future__ import annotations

import math
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
_VENUE_DIR = os.path.join(_WORKTREE_ROOT, '.briefs', 'tmp', 'rereg_venue')
_PIN_ID = 'panel_01'
_PIN_NAME = '盤1'
_OLD_POSE = (1.0, 2.0, 0.5)
_NEW_POSE = (4.0, 5.0, -1.0)


def _prepare_venue_dir() -> None:
    os.makedirs(_VENUE_DIR, exist_ok=True)
    pins = {
        'pins': [{
            'id': _PIN_ID,
            'name': _PIN_NAME,
            'kind': 'PANEL',
            'pose': {'x': _OLD_POSE[0], 'y': _OLD_POSE[1], 'yaw': _OLD_POSE[2]},
            'registered_at': 0,
        }],
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


def _yaw_from_msg(pose) -> float:
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TestPinReregister(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        _prepare_venue_dir()  # 前試験の変更を消して seeded 状態に戻す
        self.node = rclpy.create_node('rereg_client')
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
            fut = cli.call_async(req)
            deadline = time.monotonic() + 8.0
            while not fut.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not fut.done():
                self.fail('/onsite/edit_pin の応答が 8s 来ない')
            return fut.result()
        finally:
            self.node.destroy_client(cli)

    def _wait_pin_pose(self, x, y, yaw, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                pins = list(self.pins)
            for p in pins:
                if p.id == _PIN_ID:
                    if (abs(p.pose.position.x - x) < 1e-6
                            and abs(p.pose.position.y - y) < 1e-6
                            and abs(_yaw_from_msg(p.pose) - yaw) < 1e-6):
                        return p
            time.sleep(0.05)
        return None

    def test_reregister_keeps_id_and_name(self):
        """再登録で id と名前は変わらず、位置・向きだけ変わる（/onsite/pins で確認）。"""
        res = self._call_edit(id=_PIN_ID, update_pose=True,
                              x=_NEW_POSE[0], y=_NEW_POSE[1], yaw=_NEW_POSE[2])
        self.assertTrue(res.success, f'resuccess のはず: {res.message}')
        pin = self._wait_pin_pose(*_NEW_POSE)
        self.assertIsNotNone(pin, '再登録後の pose が /onsite/pins に載らない')
        self.assertEqual(pin.id, _PIN_ID)  # 新しい id を振る変異はここで赤くなる
        self.assertEqual(pin.name, _PIN_NAME)

    def test_reregister_unknown_id_rejected(self):
        """存在しない id の再登録は拒否。"""
        res = self._call_edit(id='no_such_pin', update_pose=True,
                              x=0.0, y=0.0, yaw=0.0)
        self.assertFalse(res.success)
