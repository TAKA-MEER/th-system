"""
test_params_overrides_node.py
============================
SG-B8 — overrides.yaml に書いた値が「生成 → 生成 yaml → ノード起動」の
本番経路で実際にノードに届くことを、launch_testing で実際に起動して縛る。

経路:
  1. `export.main --overrides`（起動時の設定生成の正体）を回し、生成 yaml と
     params_provenance.json を書く。
  2. `params_audit` を、生成 yaml を `parameters=[...]` に載せた本番と同じ
     渡し方で起動する（registry/overrides/calib/generated_dir は一時パス注入）。
  3. /params/get が「いま効いている値」と「保存済みで次回から効く上書き値」を
     区別して返す。/params/set で書いた値は走行中の値に触れない
     （pending にだけ現れる）。
  4. 生成を回し直す（＝次回起動の再現）と、pending だった値が effective になる。

【実行に関する注意】このファイルはノードを起動する（`ros2 test` /
`colcon test` 経由）。ホストの素の pytest では回さない
（CLAUDE.md の除外一覧に入れること）。
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

from th_params import export

from th_system_msgs.msg import SystemState
from th_system_msgs.srv import GetParams, SetParams


# 試験専用の最小 registry.yaml（schema.py の必須項目: consumers / spec_ref を満たす）。
_TEST_REGISTRY_ROWS = [
    {
        "name": "test_preset_ratio",
        "unit": "ratio",
        "class": "b",
        "status": "given",
        "value": 0.5,
        "consumers": ["params_audit"],
        "spec_ref": "test_params_overrides_node.py",
        "note": "上書き対象の given 行",
    },
    {
        "name": "test_other_ratio",
        "unit": "ratio",
        "class": "b",
        "status": "given",
        "value": 0.3,
        "consumers": ["params_audit"],
        "spec_ref": "test_params_overrides_node.py",
        "note": "/params/set で後から書く given 行",
    },
    {
        "name": "test_derived_value",
        "unit": "m",
        "class": "b",
        "status": "derived",
        "value": None,
        "formula": "floor_distance",
        "derived_from": ["body_half_length_m", "floor_margin_m"],
        "consumers": ["params_audit"],
        "spec_ref": "test_params_overrides_node.py",
        "note": "derived 行（上書き対象外）",
    },
    {
        "name": "body_half_length_m",
        "unit": "m",
        "class": "b",
        "status": "given",
        "value": 0.3,
        "consumers": ["params_audit"],
        "spec_ref": "test_params_overrides_node.py",
        "note": "derived の依存行",
    },
    {
        "name": "floor_margin_m",
        "unit": "m",
        "class": "b",
        "status": "given",
        "value": 0.05,
        "consumers": ["params_audit"],
        "spec_ref": "test_params_overrides_node.py",
        "note": "derived の依存行",
    },
]


def _write(path: str, obj) -> None:
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(obj, f, allow_unicode=True, sort_keys=True)


def _read_yaml(path: str):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _setup_generation():
    """一時 registry＋一時 overrides で export.main を回す（起動時生成の再現）。

    戻り値は dict(registry_path, overrides_path, out_dir)。
    overrides には有効な上書き（test_preset_ratio → 0.6）と、適用されない
    上書き（derived 行・未知名）を混ぜる。"""
    tmp_dir = tempfile.mkdtemp(prefix="th_params_overrides_test_")
    registry_path = os.path.join(tmp_dir, "registry.yaml")
    _write(registry_path, _TEST_REGISTRY_ROWS)
    overrides_path = os.path.join(tmp_dir, "overrides.yaml")
    _write(overrides_path, {
        "test_preset_ratio": {"value": 0.6, "set_at": "t0", "set_by": "tester",
                              "reason": "setup"},
        "test_derived_value": {"value": 1.0, "set_at": "t0", "set_by": "tester",
                               "reason": "derived なので拒否されるはず"},
        "no_such_param_xyz": {"value": 1.0},
    })
    out_dir = os.path.join(tmp_dir, "generated")
    rc = export.main(["--registry", registry_path, "--out", out_dir,
                      "--stage", "8", "--overrides", overrides_path])
    assert rc == 0, "試験の前提が崩れている: 生成が失敗した"
    return {"tmp_dir": tmp_dir, "registry_path": registry_path,
            "overrides_path": overrides_path, "out_dir": out_dir}


_GEN = _setup_generation()
_GENERATED_NODE_YAML = os.path.join(_GEN["out_dir"], "params_audit.yaml")


@pytest.mark.launch_test
def generate_test_description():
    # 存在しない calib パス（FileNotFound → 校正なし扱い。二重の保険）。
    calib_path = os.path.join(_GEN["tmp_dir"], "calib_current.yaml")
    params_audit = launch_ros.actions.Node(
        package='th_params',
        executable='params_audit.py',
        name='params_audit',
        # 本番と同じ渡し方: 生成 yaml を parameters に載せる。
        parameters=[_GENERATED_NODE_YAML, {
            'registry_path': _GEN["registry_path"],
            'overrides_path': _GEN["overrides_path"],
            'calib_path': calib_path,
            'generated_dir': _GEN["out_dir"],
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        params_audit,
        launch_testing.actions.ReadyToTest(),
    ]), {'params_audit': params_audit}


class TestParamsOverridesNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_params_overrides_client')

        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.node.create_publisher(SystemState, '/system/state', state_qos)

        self.get_client = self.node.create_client(GetParams, '/params/get')
        self.set_client = self.node.create_client(SetParams, '/params/set')

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, duration: float = 0.3):
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_mode(self, mode: str):
        msg = SystemState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = mode
        self.pub_state.publish(msg)
        self._spin(0.3)

    def _call_get(self, names):
        assert self.get_client.wait_for_service(timeout_sec=5.0), \
            '/params/get に接続できない'
        req = GetParams.Request()
        req.names = list(names)
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        assert future.done(), '/params/get の応答が返らない'
        return json.loads(future.result().json)

    def _call_set(self, values: dict):
        assert self.set_client.wait_for_service(timeout_sec=5.0), \
            '/params/set に接続できない'
        req = SetParams.Request()
        req.json = json.dumps({'values': values, 'set_by': 'tester',
                               'reason': 'overrides node test'})
        future = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        assert future.done(), '/params/set の応答が返らない'
        return future.result()

    def _regenerate(self):
        """生成を回し直す（＝次回起動の再現）。"""
        rc = export.main(["--registry", _GEN["registry_path"],
                          "--out", _GEN["out_dir"],
                          "--stage", "8", "--overrides", _GEN["overrides_path"]])
        assert rc == 0, "生成の回し直しが失敗した"

    # ════════════════════════════════════════════════════════
    def test_a_generated_yaml_carries_override_and_provenance(self):
        """生成物に上書き値が乗り、出どころと rejected の理由が残る。
        通らない上書き（derived 行・未知名）は既定値のまま。"""
        gen = _read_yaml(_GENERATED_NODE_YAML)
        params = gen["params_audit"]["ros__parameters"]
        assert params["test_preset_ratio"] == 0.6, \
            "overrides.yaml の値が生成 yaml に届いていない"
        with open(os.path.join(_GEN["out_dir"], "params_provenance.json"),
                  encoding="utf-8") as f:
            prov = json.load(f)
        assert prov["origins"]["test_preset_ratio"]["set_by"] == "tester"
        assert "test_derived_value" in prov["rejected"]
        assert "no_such_param_xyz" in prov["rejected"]

    def test_b_get_distinguishes_effective_and_pending(self):
        """起動時に適用済みの値は effective に乗り、pending は空。
        restart_required は偽。"""
        payload = self._call_get(["test_preset_ratio", "test_other_ratio"])
        assert payload["effective"]["test_preset_ratio"] == 0.6
        assert payload["effective"]["test_other_ratio"] == 0.3
        assert payload["pending"] == {}
        assert payload["restart_required"] is False

    def test_c_set_then_get_shows_pending_without_touching_effective(self):
        """/params/set で書いた値は pending にだけ現れ、走行中の値
        （effective）に触れない。再起動が必要と出る。"""
        self._publish_mode('IDLE')
        result = self._call_set({"test_other_ratio": 0.9})
        assert result.success is True, f"/params/set が拒否された: {result.message}"

        payload = self._call_get(["test_other_ratio"])
        assert payload["effective"]["test_other_ratio"] == 0.3, \
            "走行中の値が変わっている（次の起動から効くはず）"
        assert payload["pending"]["test_other_ratio"] == 0.9
        assert payload["restart_required"] is True

    def test_d_regeneration_applies_pending_as_effective(self):
        """生成を回し直す（＝次回起動）と、pending だった値が effective になる。
        これが「次の起動から効く」の実体。"""
        self._publish_mode('IDLE')
        result = self._call_set({"test_other_ratio": 0.8})
        assert result.success is True, f"/params/set が拒否された: {result.message}"

        self._regenerate()

        payload = self._call_get(["test_other_ratio"])
        assert payload["effective"]["test_other_ratio"] == 0.8, \
            "生成の回し直し（次回起動相当）で値が効いていない"
        assert payload["pending"] == {}
        assert payload["restart_required"] is False


if __name__ == '__main__':
    unittest.main()
