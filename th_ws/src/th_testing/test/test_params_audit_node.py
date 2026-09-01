"""
test_params_audit_node.py
============================
DetailedDesign-wp1.md `WP-PARAM-02` §7 — params_audit ノードの単体試験。

【実行に関する注意】このファイルはノードを起動する（`ros2 test` / `colcon test` 経由）。
このセッションでは実機の無線リンク品質を計測中のため**実行しない**（指揮担当が
測定終了後に実行する）。`th_bringup/launch/params_generation.py` 自体の純粋関数の
検証は test_params_launch.py の対象（ROS2 非依存）。ここでは params_audit.py 自体の
責務（/system/params_status の publish、/params/set のモードゲート）だけを見る。

registry_path パラメータで、この試験専用の小さな registry.yaml（given な行を数個
だけ持つ）を注入する。実物の registry.yaml（97行超）に依存すると、他パケットの
placeholder 追加・削除でこのテストが無関係に壊れる。
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

from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)

from th_system_msgs.msg import ParamsStatus, SystemState
from th_system_msgs.srv import GetParams, SetParams


# 試験専用の最小 registry.yaml（schema.py の必須項目: consumers / spec_ref を満たす）。
# 「derived」行は export.py の _call_formula() が式名ごとに固定の依存名を要求する
# （汎用式は無い）ため、実在する式 "floor_distance"（derive.floor_distance の2引数
# body_half_length_m / floor_margin_m）をそのまま使う。
_TEST_REGISTRY_ROWS = [
    {
        "name": "test_given_speed",
        "unit": "m/s",
        "class": "b",
        "status": "given",
        "value": 1.0,
        "consumers": ["params_audit"],
        "spec_ref": "test_params_audit_node.py",
        "note": "調整可能な given 行（/params/set の対象）",
    },
    {
        "name": "body_half_length_m",
        "unit": "m",
        "class": "b",
        "status": "given",
        "value": 0.3,
        "consumers": ["params_audit"],
        "spec_ref": "test_params_audit_node.py",
        "note": "test_derived_value の依存行",
    },
    {
        "name": "floor_margin_m",
        "unit": "m",
        "class": "b",
        "status": "given",
        "value": 0.05,
        "consumers": ["params_audit"],
        "spec_ref": "test_params_audit_node.py",
        "note": "test_derived_value の依存行",
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
        "spec_ref": "test_params_audit_node.py",
        "note": "derived 行（/params/set では拒否されるべき）",
    },
    {
        "name": "test_placeholder_value",
        "unit": None,
        "class": "c",
        "status": "placeholder",
        "value": "TBD_MEASURE",
        # S1: class:c かつ status!=measured は blocking:true が必須。
        # blocking_from_stage は付けない（A8 は None なら常にスキップする＝
        # このテストでは起動やアサーションを妨げない、ただの未測定値として使う）。
        "blocking": True,
        "consumers": ["params_audit"],
        "spec_ref": "test_params_audit_node.py",
        "note": "未測定行。placeholder_count に数えられる",
    },
]


def _write_test_registry() -> str:
    tmp_dir = tempfile.mkdtemp(prefix="th_params_audit_test_")
    path = os.path.join(tmp_dir, "registry.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(_TEST_REGISTRY_ROWS, f, allow_unicode=True)
    return path


# D4: params_audit.py のモジュール定数 OVERRIDES_PATH / CALIB_PATH は
# `/root/th_data/...`（docker-compose.yml がホストの th_ws/data/ にバインドマウント
# する実データの置き場）を指す。テストが /params/set を叩くとそのまま実機の
# overrides.yaml へテスト用の値が書き込まれてしまうため、一時ディレクトリの
# パスを `overrides_path` / `calib_path` パラメータで注入する
# （registry_path と同じ流儀。既定値が空文字列ならモジュール定数を使う設計）。
def _temp_overrides_and_calib_paths() -> tuple[str, str]:
    tmp_dir = tempfile.mkdtemp(prefix="th_params_audit_test_data_")
    overrides_path = os.path.join(tmp_dir, "overrides.yaml")
    calib_path = os.path.join(tmp_dir, "calib_current.yaml")
    return overrides_path, calib_path


# 注意（CLAUDE.md の既知の癖）: @pytest.mark.launch_test を持つファイルは
# unittest.TestCase のメソッドだけが収集・実行される。素の pytest 関数は書かない。

@pytest.mark.launch_test
def generate_test_description():
    registry_path = _write_test_registry()
    overrides_path, calib_path = _temp_overrides_and_calib_paths()
    params_audit = launch_ros.actions.Node(
        package='th_params',
        executable='params_audit.py',
        name='params_audit',
        parameters=[{
            'registry_path': registry_path,
            'overrides_path': overrides_path,
            'calib_path': calib_path,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        params_audit,
        launch_testing.actions.ReadyToTest(),
    ]), {'params_audit': params_audit}


class TestParamsAuditNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_params_audit_client')

        status_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_history = []
        self.node.create_subscription(
            ParamsStatus, '/system/params_status', self._status_history.append, status_qos)

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

    def _call_set(self, values: dict, set_by: str = 'tester', reason: str = 'unit test'):
        assert self.set_client.wait_for_service(timeout_sec=5.0), \
            '/params/set に接続できない'
        req = SetParams.Request()
        req.json = json.dumps({'values': values, 'set_by': set_by, 'reason': reason})
        future = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        assert future.done(), '/params/set の応答が返らない'
        return future.result()

    # ════════════════════════════════════════════════════════
    def test_status_published(self):
        """§3.1: /system/params_status が publish される（transient_local なので
        起動後の遅い購読でも最新値が届く）。placeholder_count が
        test_placeholder_value の1件を数えていること。"""
        deadline = time.time() + 5.0
        while time.time() < deadline and not self._status_history:
            self._spin(0.2)
        assert self._status_history, '/system/params_status が一度も届かない'

        msg = self._status_history[-1]
        assert msg.placeholder_count >= 1
        assert 'test_placeholder_value' in list(msg.placeholder_names)
        assert msg.digest, 'digest が空'

    def test_set_rejected_outside_idle(self):
        """§3.2 / PT-3: IDLE/MANUAL 以外では /params/set が拒否される
        （accepted=false 相当。success=False で値は保持しない）。
        IDLE に切り替えれば（かつ given 行・妥当な値であれば）受理される。"""
        self._publish_mode('FOLLOWING')
        result = self._call_set({'test_given_speed': 2.0})
        assert result.success is False, \
            'FOLLOWING モード中に /params/set が受理された（PT-3 違反）'

        # モード不明（/system/state を一度も受け取っていない）のあいだも拒否される
        # ことは起動直後の状態と同じなので、モード切替の効果そのものを以下で確認する。
        self._publish_mode('IDLE')
        result_idle = self._call_set({'test_given_speed': 2.0})
        assert result_idle.success is True, (
            'IDLE モードで given 行への妥当な /params/set が拒否された: '
            f'{result_idle.message}')

        # derived 行への /params/set は拒否される（PT-2）
        result_derived = self._call_set({'test_derived_value': 1.0})
        assert result_derived.success is False, \
            'derived 行への /params/set が受理された（PT-2 違反）'

    def test_set_survives_malformed_requests_and_stays_alive(self):
        """D1-b 回帰試験。修正前は `_cb_set` 内の例外（例: registry に無い名前を
        `run_assertions()` の内部ヘルパーが直接引いた KeyError）が rclpy の executor
        まで伝播し、`params_audit` プロセスごと死んでいた（実際に
        `obstacle_floor_distance_m` で発生。この試験専用の最小 registry.yaml には
        その名前が無いため、素朴な /params/set 呼び出しだけで再現する）。

        不正な要求を何を送っても success=False が返り、**そのあとに続けて別の
        サービス呼び出しが成功する**（＝ノードが生きている）ことを確認する。"""
        self._publish_mode('IDLE')

        # 1) json 自体が壊れている
        assert self.set_client.wait_for_service(timeout_sec=5.0), \
            '/params/set に接続できない'
        req_bad_json = SetParams.Request()
        req_bad_json.json = '{this is not valid json'
        future = self.set_client.call_async(req_bad_json)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        assert future.done(), '壊れた json の要求後に応答が返らない（ノードが死んだ可能性）'
        assert future.result().success is False

        # 2) set_by を欠く（PT-4 違反）
        req_no_set_by = SetParams.Request()
        req_no_set_by.json = json.dumps(
            {'values': {'test_given_speed': 3.0}, 'reason': 'set_by 無し'})
        future2 = self.set_client.call_async(req_no_set_by)
        rclpy.spin_until_future_complete(self.node, future2, timeout_sec=5.0)
        assert future2.done(), 'set_by 欠落の要求後に応答が返らない（ノードが死んだ可能性）'
        assert future2.result().success is False

        # 3) values に registry に無い名前を入れる
        result_unknown = self._call_set({'no_such_param_xyz': 1.0})
        assert result_unknown.success is False

        # 4) 上の3件のあとでもノードが生きていることを、別のサービス
        #    （/params/get）の呼び出しが成功することで確認する。
        assert self.get_client.wait_for_service(timeout_sec=5.0), (
            '/params/get に接続できない（params_audit プロセスが死んでいる可能性。'
            'D1-b 回帰）')
        get_req = GetParams.Request()
        get_req.names = ['test_given_speed']
        get_future = self.get_client.call_async(get_req)
        rclpy.spin_until_future_complete(self.node, get_future, timeout_sec=5.0)
        assert get_future.done(), '/params/get の応答が返らない（ノードが死んでいる）'
        payload = json.loads(get_future.result().json)
        assert 'test_given_speed' in payload

        # 5) 続けて妥当な /params/set も正常に成功する（ノードが健全な状態のまま）
        result_ok = self._call_set({'test_given_speed': 4.0})
        assert result_ok.success is True, (
            f'不正な要求のあと、正常な /params/set まで拒否された: {result_ok.message}')


if __name__ == '__main__':
    unittest.main()
