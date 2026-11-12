"""
test_params_audit_broken_files_node.py
======================================
A-14 追補 — 壊れた `calib/current.yaml`・`overrides.yaml` を置いて params_audit を
起動しても、ノードは死なず `/params/get` が読めた範囲で応答し、overrides が壊れて
いるあいだの `/params/set` は理由つきで拒否して**壊れたファイルを上書きしない**ことを縛る。

（以前は `_apply_calib` が FileNotFoundError だけ、`_load_overrides` も同じで、
 YAMLError で起動時に落ちた。生成側 params_generation は警告で読み飛ばす。それに揃えた。）

【実行に関する注意】launch_testing（rclpy 要）。ホストの素の pytest では回さない。
"""
import json
import os
import tempfile
import time
import unittest

import pytest
import rclpy
import yaml

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from th_system_msgs.msg import SystemState
from th_system_msgs.srv import GetParams, SetParams

_ROWS = [
    {"name": "test_given_speed", "unit": "m/s", "class": "b", "status": "given",
     "value": 1.0, "consumers": ["params_audit"],
     "spec_ref": "test_params_audit_broken_files_node.py", "note": "given 行"},
]
_BROKEN = "{ this: is: not [valid yaml\n"

_TMP = tempfile.mkdtemp(prefix="th_params_audit_broken_")
_REGISTRY = os.path.join(_TMP, "registry.yaml")
_OVERRIDES = os.path.join(_TMP, "overrides.yaml")
_CALIB = os.path.join(_TMP, "calib_current.yaml")
with open(_REGISTRY, "w", encoding="utf-8") as f:
    yaml.safe_dump(_ROWS, f, allow_unicode=True)
with open(_OVERRIDES, "w", encoding="utf-8") as f:
    f.write(_BROKEN)
with open(_CALIB, "w", encoding="utf-8") as f:
    f.write(_BROKEN)


@pytest.mark.launch_test
def generate_test_description():
    params_audit = launch_ros.actions.Node(
        package='th_params', executable='params_audit.py', name='params_audit',
        parameters=[{
            'registry_path': _REGISTRY,
            'overrides_path': _OVERRIDES,
            'calib_path': _CALIB,
            'generated_dir': _TMP,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        params_audit,
        launch_testing.actions.ReadyToTest(),
    ]), {'params_audit': params_audit}


class TestParamsAuditBrokenFiles(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_params_audit_broken_client')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.node.create_publisher(SystemState, '/system/state', qos)
        self.get_client = self.node.create_client(GetParams, '/params/get')
        self.set_client = self.node.create_client(SetParams, '/params/set')

    def tearDown(self):
        self.node.destroy_node()

    def _spin(self, d=0.3):
        end = time.time() + d
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _get(self):
        self.assertTrue(self.get_client.wait_for_service(timeout_sec=20.0),
                        '壊れたファイルで起動した params_audit の /params/get が現れない（起動時に死んだ）')
        req = GetParams.Request()
        req.names = ['test_given_speed']
        fut = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        self.assertTrue(fut.done(), '/params/get の応答が返らない')
        return json.loads(fut.result().json)

    def test_get_responds_with_broken_calib_and_overrides(self):
        body = self._get()
        self.assertNotIn('error', body, body)
        self.assertEqual(body['effective']['test_given_speed'], 1.0)

    def test_set_rejected_and_broken_overrides_not_overwritten(self):
        self.assertTrue(self.set_client.wait_for_service(timeout_sec=20.0))
        msg = SystemState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = 'IDLE'
        self.pub_state.publish(msg)
        self._spin(0.5)
        req = SetParams.Request()
        req.json = json.dumps({'values': {'test_given_speed': 2.0},
                               'set_by': 't', 'reason': 'broken overrides'})
        fut = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        self.assertTrue(fut.done())
        self.assertFalse(fut.result().success, '壊れた overrides.yaml のまま保存が受理された')
        self.assertIn('変更できません', fut.result().message)
        with open(_OVERRIDES, encoding='utf-8') as f:
            self.assertEqual(f.read(), _BROKEN, '壊れた overrides.yaml が上書きされた')
        # 拒否のあとも生きている
        self.assertNotIn('error', self._get())
