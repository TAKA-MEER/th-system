#!/usr/bin/env python3
# ============================================================
# apply_calib — wheel_base キャリブレーション値の適用ツール
#
# 使い方:
#   ros2 run th_calibration apply_calib.py --ros-args -p wheel_base:=0.392
#
# 動作:
#   1. esp32_bridge ノードの wheel_base パラメータをランタイムで更新
#      (esp32_bridge.py の add_on_set_parameters_callback により即座に反映される)
#   2. th_bringup/config/calib.yaml に保存し、次回 bringup 起動時にも反映されるようにする
#
# 備考: wheel_radius(車輪半径)は ESP32 ファームウェアのコンパイル時定数
#   (esp32/src/config.h の WHEEL_RADIUS_M)で決まるため、本ツールでは扱わない。
#   半径補正は linear_calib.py の案内に従い config.h を書き換えて再書き込みする。
# ============================================================
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory
import os
import yaml


class ApplyCalib(Node):
    def __init__(self):
        super().__init__('apply_calib')

        self.declare_parameter('wheel_base', -1.0)   # -1=変更しない

        base = self.get_parameter('wheel_base').value

        if base <= 0:
            self.get_logger().error('変更する wheel_base が指定されていません')
            return

        updates = {'wheel_base': base}
        self.get_logger().info(f'適用するパラメータ: {updates}')

        # ── esp32_bridge へのランタイム更新 ─────────────────
        cli = self.create_client(
            SetParameters, '/esp32_bridge/set_parameters')

        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                'esp32_bridge が見つかりません。起動中か確認してください。')
        else:
            params = []
            for name, value in updates.items():
                pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                    double_value=value)
                params.append(Parameter(name=name, value=pv))

            req = SetParameters.Request(parameters=params)
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.result():
                for r in future.result().results:
                    if not r.successful:
                        self.get_logger().error(f'更新失敗: {r.reason}')
                self.get_logger().info('esp32_bridge パラメータ更新完了(即時反映)')
            else:
                self.get_logger().error('サービス呼び出し失敗')

        # ── YAML に保存 ──────────────────────────────────────
        self._save_yaml(updates)

    def _save_yaml(self, updates: dict):
        # bringup.launch.py が読み込むのと同じ share ディレクトリを解決する
        # (以前は __file__ からの相対パスで install/th_bringup/config/... を
        #  指しており、share/th_bringup/config/ と一致せず保存内容が失われていた)
        calib_path = os.path.join(
            get_package_share_directory('th_bringup'), 'config', 'calib.yaml')

        existing: dict = {}
        if os.path.exists(calib_path):
            with open(calib_path) as f:
                existing = yaml.safe_load(f) or {}

        # esp32_bridge セクションを更新
        bridge = existing.setdefault('esp32_bridge', {}).setdefault(
            'ros__parameters', {})
        bridge.update(updates)

        os.makedirs(os.path.dirname(calib_path), exist_ok=True)
        with open(calib_path, 'w') as f:
            yaml.dump(existing, f, default_flow_style=False, allow_unicode=True)

        self.get_logger().info(f'YAML 保存: {calib_path}')
        print('\n--- 保存内容 ---')
        print(yaml.dump(existing, default_flow_style=False))
        print('次回起動時にも自動反映されます。')


def main(args=None):
    rclpy.init(args=args)
    node = ApplyCalib()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
