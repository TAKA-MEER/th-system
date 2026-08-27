#!/usr/bin/env python3
# ============================================================
# lidar_filter — LiDAR 死角フィルターノード
#
# 機体外形（アルミパイプ等）による死角を inf でマスクし
# /scan_filtered を発行する。
# blind_angle_ranges は registry.yaml の 1 本が出所（params_generation が
# th_ws/data/generated/lidar_filter.yaml へ書く）。既定値は空（＝マスクなし）
# にしてある。死角がある構成なら registry.yaml 側で必ず値を持たせること
# （このノード側の既定値だけを変えても params_generation の生成物が
# 上書きするため意味がない）。
# ============================================================
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import LaserScan
import math
import copy


class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')

        # ── パラメータ ──────────────────────────────────────
        # blind_angle_ranges: [[start_deg, end_deg], ...] (0=前方 右回り正)
        # 既定値は空配列（＝マスクしない）。マスクしすぎて障害物が見えなく
        # なるより、マスクせず広く見える方が安全側（安全側は「マスクしない」）。
        # **2026-08-27 訂正**: 当初このコメントは「空の DOUBLE_ARRAY は型推論に
        # 失敗するため ParameterDescriptor で明示する」としていたが誤りだった。
        # Docker 実起動で本ノードが起動失敗することを実測で確認している
        # （rclpy ではパラメータが未初期化のまま残り get_parameter().value で
        #   ParameterUninitializedException になる。rclcpp では
        #   `parameter_value_from failed ... No parameter value set` で即死する。
        #   標準ノード tf2_ros/static_transform_publisher でも同じ現象を確認した）。
        # 真因は ROS 2 Humble が params YAML の空配列 override を解決できないこと
        # であって declare_parameter の型推論ではない。対処は生成側で行った——
        # th_bringup/launch/params_generation.py が空のキーを生成 YAML から
        # 落とすようにしたので、override が空配列になることはもう無い。
        # ここの ParameterDescriptor は主たる対処ではなく多層防御として残す。
        self.declare_parameter(
            'blind_angle_ranges', [],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('input_topic',  '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        self._blind_ranges = self._build_blind_ranges(
            self.get_parameter('blind_angle_ranges').value)
        if not self._blind_ranges:
            self.get_logger().warn(
                'blind_angle_ranges が空。死角マスクなしで起動する'
                '（死角が無いことを確認済みの構成なら正常。死角があるのに'
                'registry.yaml の値が抜けている場合は要確認）')
        self.add_on_set_parameters_callback(self._on_set_params)

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(LaserScan, out_topic, 10)
        self._sub = self.create_subscription(
            LaserScan, in_topic, self._cb, 10)

        self.get_logger().info(
            f'lidar_filter 起動  {in_topic} → {out_topic}  '
            f'死角: {len(self._blind_ranges)} 範囲')

    @staticmethod
    def _build_blind_ranges(raw):
        # フラットリスト [s0,e0, s1,e1, ...] → [(s0,e0), (s1,e1), ...]
        return [
            (math.radians(raw[i]), math.radians(raw[i + 1]))
            for i in range(0, len(raw) - 1, 2)
        ]

    def _on_set_params(self, params):
        # blind_angle_ranges は起動時に一度だけ ラジアンの (start,end) タプル列
        # へ変換してキャッシュしている。WebUI 設定パネルからのライブ反映を
        # 機能させるため、変更されたら再構築する。
        for p in params:
            if p.name == 'blind_angle_ranges':
                self._blind_ranges = self._build_blind_ranges(p.value)
        return SetParametersResult(successful=True)

    def _cb(self, msg: LaserScan):
        filtered = copy.copy(msg)
        filtered.ranges = list(msg.ranges)

        for i, r in enumerate(filtered.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            # -π〜π → 0〜2π に正規化
            angle_norm = angle % (2 * math.pi)

            for (start, end) in self._blind_ranges:
                s = start % (2 * math.pi)
                e = end   % (2 * math.pi)
                if s <= e:
                    if s <= angle_norm <= e:
                        filtered.ranges[i] = float('inf')
                        break
                else:  # 0°をまたぐ範囲
                    if angle_norm >= s or angle_norm <= e:
                        filtered.ranges[i] = float('inf')
                        break

        self._pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
