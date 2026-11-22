"""
test_params_audit_launched_node.py
==================================
A-14（2026-11-12 実機 T-19 R-28）— `params_audit` が本番の launch 記述から
実際に起動し、`/params/get` に応答することを縛る。

事故: `params_audit.py` は install されノード単体の launch_testing も緑だったが、
`bringup.launch.py` にも `gazebo.launch.py` にも書かれておらず、実機では
S-50 の速度プリセット・自動ブレーキの既定が「現在値を取得できません」になった。

経路: `bringup.launch.py` の `generate_launch_description()` を実際に呼び、返った
記述の中から `params_audit` の Node アクションを取り出して、そのまま起動する
（Node 定義を試験側で書き写さない。bringup から行を外せばここで赤になる）。
`gazebo.launch.py` は記述が重い（Gazebo 系のアクションを組み立てる）ので、
`params_audit` の Node 定義が書かれていることを文字列で確かめる。

【実行に関する注意】launch_testing（rclpy・ビルド済み th_system_msgs が要る）。
ホストの素の pytest では回さない。
"""
import importlib.util
import json
import os
import re
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from th_system_msgs.srv import GetParams

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, '..', '..'))
_BRINGUP = os.path.join(_SRC, 'th_bringup', 'launch', 'bringup.launch.py')
_GAZEBO = os.path.join(_SRC, 'th_bringup', 'launch', 'gazebo.launch.py')


def _load_launch(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _find_params_audit(ld):
    return [e for e in ld.entities
            if isinstance(e, launch_ros.actions.Node)
            and getattr(e, '_Node__node_name', None) == 'params_audit']


# 本番の bringup の記述から Node を取り出す。無ければここで落ちる（収集エラー＝赤）。
_BRINGUP_LD = _load_launch(_BRINGUP, 'bringup_under_test').generate_launch_description()
_NODES = _find_params_audit(_BRINGUP_LD)
assert len(_NODES) == 1, (
    f'bringup.launch.py に params_audit の Node がちょうど 1 つ無い（{len(_NODES)} 個）。'
    'S-50 の /params/get が無応答になる（A-14）')


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        _NODES[0],
        launch_testing.actions.ReadyToTest(),
    ]), {'params_audit': _NODES[0]}


class TestParamsAuditLaunched(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_get_responds_from_bringup_node(self):
        """bringup の記述から起動した params_audit が /params/get に応答する。"""
        node = rclpy.create_node('test_params_audit_launched_client')
        try:
            client = node.create_client(GetParams, '/params/get')
            self.assertTrue(client.wait_for_service(timeout_sec=20.0),
                            'bringup から起動した params_audit の /params/get が現れない')
            req = GetParams.Request()
            req.names = ['speed_preset_low']
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            self.assertTrue(future.done(), '/params/get の応答が返らない')
            body = json.loads(future.result().json)
            self.assertIn('speed_preset_low', json.dumps(body))
        finally:
            node.destroy_node()

    def test_gazebo_launch_also_starts_params_audit(self):
        """シミュ（gazebo.launch.py）でも S-50 が使えるよう params_audit を起動する。"""
        text = open(_GAZEBO, encoding='utf-8').read()
        self.assertTrue(
            re.search(r"executable='params_audit\.py'[^)]*?name='params_audit'", text, re.S),
            'gazebo.launch.py に params_audit の Node が無い')
