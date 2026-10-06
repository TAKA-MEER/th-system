#!/usr/bin/env python3
"""
config_manager.py — ROS2 ノード本体
====================================
WebUI からのパラメータ調整要求を仲介する。

担当:
  - /system/state 購読 → 停止中（IDLE・PREP の地図作業中）かつジョグ中でない
    ときだけ変更を受け付ける（サーバー側の安全ガード。UI 側の表示制御だけに
    頼らない。Spec.md SD-9。1b-5 で旧 /robot/mode から付け替え）
    。/system/state 未受信・古いときは拒否する（安全側）
  - /config_manager/set_tunable_params → 対象ノードの標準 set_parameters に
    透過的にフォワード（実行時反映のみ、YAML には書かない）
  - /config_manager/save_tunable_params → 対象ノードの標準 get_parameters で
    現在値を取得し、th_config_manager.yaml_writer で該当 YAML に書き戻す
    （コメント・書式を保持）

対象パラメータの一覧は th_config_manager.tunable_targets.TUNABLE_TARGETS
を参照。新しいパラメータを追加する手順は docs/architecture.md 参照。

スレッドモデル: サービスコールバック内で対象ノードの set_parameters/
get_parameters を呼び出し、その完了を待つ（自分自身のサービス呼び出しに
対して結果を返す）。SingleThreadedExecutor + async/await コールバックの
組み合わせはこの「サービスの中からサービスを呼ぶ」構図で応答が返らず
ハングする既知の問題があるため、MultiThreadedExecutor +
ReentrantCallbackGroup + 同期的な spin_until_future_complete という
より枯れたパターンを使う。rosbridge はこのノード宛の呼び出しが詰まると
同じ WebSocket 接続上の他のトピック中継まで止めてしまう
（実機/シミュレーションで観測: 設定変更後にモード切替 UI も無反応になった）。
"""

SERVICE_TIMEOUT_SEC = 2.0
import os
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters

from th_system_msgs.msg import SystemState
from th_system_msgs.srv import SaveTunableParams, SetTunableParams

from th_config_manager.stop_only_guard import STATE_STALE_SEC, stop_only_allows
from th_config_manager.tunable_targets import TUNABLE_TARGETS
from th_config_manager.yaml_writer import update_ros_params_yaml
from th_config_manager.service_call import call_and_wait


def _param_value_to_python(value):
    t = value.type
    if t == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if t == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if t == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if t == ParameterType.PARAMETER_STRING:
        return value.string_value
    if t == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(value.byte_array_value)
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(value.bool_array_value)
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(value.integer_array_value)
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(value.double_array_value)
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return list(value.string_array_value)
    return None


class ConfigManager(Node):
    def __init__(self):
        super().__init__("config_manager")
        # 1b-5: /system/state の最新像。未受信の間は拒否する（安全側）。
        self._state_mode = None
        self._state_name = ""
        self._jog_active = False
        self._state_at = None   # 最後に受けた時刻（monotonic 秒）
        # 鮮度の上限。registry の state_stale_ms=1500 と同じ既定
        # （jog_gate / obstacle_limiter / safety_monitor と同じ値）。
        self.declare_parameter("state_stale_ms", int(STATE_STALE_SEC * 1000.0))
        # main() で spin に使う MultiThreadedExecutor を明示的に受け取る。
        # rclpy.spin_until_future_complete() は executor 未指定だと内部で
        # 一時的な別 Executor を作って同じノードを二重に spin してしまい、
        # 購読コールバックの二重発火など不定な挙動を招きうるため避ける。
        self._executor = None
        # サービスの中からサービスを呼ぶため、応答を受け取るコールバックが
        # 元のリクエスト処理コールバックと並行実行できるよう同一の
        # ReentrantCallbackGroup にまとめる（MultiThreadedExecutor 前提）。
        cbg = ReentrantCallbackGroup()
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            SystemState, "/system/state", self._cb_state, state_qos,
            callback_group=cbg)

        self._get_clients = {
            name: self.create_client(GetParameters, f"/{name}/get_parameters", callback_group=cbg)
            for name in TUNABLE_TARGETS
        }
        self._set_clients = {
            name: self.create_client(SetParameters, f"/{name}/set_parameters", callback_group=cbg)
            for name in TUNABLE_TARGETS
        }

        self.create_service(
            SetTunableParams, "/config_manager/set_tunable_params", self._cb_set,
            callback_group=cbg)
        self.create_service(
            SaveTunableParams, "/config_manager/save_tunable_params", self._cb_save,
            callback_group=cbg)

        self.get_logger().info("config_manager 起動")

    def _cb_state(self, msg: SystemState):
        self._state_mode = msg.mode
        self._state_name = msg.state
        self._jog_active = bool(msg.jog_active)
        self._state_at = time.monotonic()

    def _reject_if_not_stopped(self):
        """停止中だけの条件を満たさなければ拒否理由の文言、満たせば None。"""
        received = self._state_at is not None
        age = (time.monotonic() - self._state_at) if received else None
        stale_ms = self.get_parameter("state_stale_ms").value
        allowed, reason = stop_only_allows(
            self._state_mode, self._state_name, self._jog_active,
            received, age, stale_ms / 1000.0)
        if allowed:
            return None
        detail = {
            "state_not_received": "/system/state を受信していないため",
            "state_stale": "/system/state が古いため",
            "jog_active": "ジョグ中のため",
        }.get(reason, f"いまの状態（{self._state_mode}/{self._state_name}）では")
        return f"停止中（IDLE・PREP の地図作業中）のみ操作できます（{detail}拒否）"

    def _cb_set(self, request, response):
        rejected = self._reject_if_not_stopped()
        if rejected:
            response.success = False
            response.message = rejected.replace("操作できます", "変更できます")
            return response
        if request.node_name not in TUNABLE_TARGETS:
            response.success = False
            response.message = f"未知のノード: {request.node_name}"
            return response

        client = self._set_clients[request.node_name]
        if not client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = f"{request.node_name} の set_parameters に接続できません"
            return response

        req = SetParameters.Request()
        req.parameters = request.parameters
        result, err = call_and_wait(self, client, req, SERVICE_TIMEOUT_SEC)
        if err:
            response.success = False
            response.message = f"{request.node_name} への set_parameters 呼び出し失敗: {err}"
            return response

        failed = [r.reason for r in result.results if not r.successful]
        response.success = len(failed) == 0
        response.message = "OK" if not failed else "; ".join(failed)
        return response

    def _cb_save(self, request, response):
        rejected = self._reject_if_not_stopped()
        if rejected:
            response.success = False
            response.message = rejected.replace("操作できます", "保存できます")
            return response
        target = TUNABLE_TARGETS.get(request.node_name)
        if target is None:
            response.success = False
            response.message = f"未知のノード: {request.node_name}"
            return response

        client = self._get_clients[request.node_name]
        if not client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = f"{request.node_name} の get_parameters に接続できません"
            return response

        req = GetParameters.Request()
        req.names = target["params"]
        result, err = call_and_wait(self, client, req, SERVICE_TIMEOUT_SEC)
        if err:
            response.success = False
            response.message = f"{request.node_name} への get_parameters 呼び出し失敗: {err}"
            return response

        updates = {
            name: _param_value_to_python(value)
            for name, value in zip(target["params"], result.values)
        }

        yaml_path = os.path.join(
            get_package_share_directory(target["yaml_package"]), target["yaml_relpath"])
        try:
            update_ros_params_yaml(yaml_path, target["block_key"], updates)
        except Exception as e:
            response.success = False
            response.message = f"YAML 書き込み失敗: {e}"
            return response

        response.success = True
        response.message = f"{yaml_path} に保存しました"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ConfigManager()
    # WS-9U: call_and_wait はポーリング中に worker スレッドを 1 本塞ぐ。応答を
    # 別スレッドで捌けるよう最低 2、余裕を見て 4 本確保する（実機 8 コアでは既定
    # cpu_count のままだが、コア数の少ない環境での下限保証）。
    executor = MultiThreadedExecutor(num_threads=4)
    node._executor = executor
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
