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
#
# 角度→添字の変換は scan_geometry.py（th_perception 正本。
# DetailedDesign-wp2.md WP-CALIB-01 §4.1）に委譲する。obstacle_limiter
# （th_safety、C++移植版 scan_geometry.hpp）と同じ規約で解釈することが
# 目的（B-3。片方だけ更新して片方を忘れると死角の解釈がずれる）。
# ============================================================
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import LaserScan
import copy

from th_perception.scan_geometry import sector_indices


class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')

        # ── パラメータ ──────────────────────────────────────
        # blind_angle_ranges: [start_deg, end_deg, start_deg, end_deg, ...]
        # （フラットリスト。2 個ずつ組にして [a0, a1] の閉区間として扱う）。
        # 角度は laser_link 基準・反時計回り正（scan_geometry.py 規約①。
        # sensor_msgs/LaserScan の angle_min + i*angle_increment の向きに一致）。
        # 既定値は空配列（＝マスクしない）。マスクしすぎて障害物が見えなく
        # なるより、マスクせず広く見える方が安全側（安全側は「マスクしない」）。
        #
        # **2026-08-27 訂正**: 当初は「空の DOUBLE_ARRAY は型推論に失敗するため
        # ParameterDescriptor で明示する」と考えていたが、これは誤りだった。
        # Docker 実起動で本ノードが `parameter_value_from failed for parameter
        # 'blind_angle_ranges': No parameter value set` で起動失敗することを
        # 実測で確認した——ParameterDescriptor を付けていても防げていない。
        # 本当の原因は ROS2 Humble の rcl_yaml_param_parser が空配列の
        # parameter override をそもそも解決できないこと（declare_parameter の
        # 型推論の問題ではない）。本当の対処は生成側
        # （th_bringup/launch/params_generation.py の sanitize_node_params()）
        # で、空配列のキーは生成 yaml から丸ごと落とすようにした（かつて存在した
        # reshape_blind_angles() による書き戻しは撤去済み）。したがって
        # override 自体が空配列になることはもう無い。ここでの
        # ParameterDescriptor は主たる対処ではなく多層防御として残す
        # （override が空配列である限り、型を明示しても起動失敗は避けられない）。
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
        # フラットリスト [a0,a1, a2,a3, ...]（度）→ [(a0,a1), (a2,a3), ...]。
        # 度→ラジアン変換は scan_geometry.sector_indices() の中で 1 度だけ
        # 行う（scan_geometry.py 規約②。ここでは単位変換しない）。
        return [
            (raw[i], raw[i + 1])
            for i in range(0, len(raw) - 1, 2)
        ]

    def _on_set_params(self, params):
        # blind_angle_ranges は起動時に一度だけ 度の (start,end) タプル列
        # へ変換してキャッシュしている。WebUI 設定パネルからのライブ反映を
        # 機能させるため、変更されたら再構築する。
        for p in params:
            if p.name == 'blind_angle_ranges':
                self._blind_ranges = self._build_blind_ranges(p.value)
        return SetParametersResult(successful=True)

    def _cb(self, msg: LaserScan):
        filtered = copy.copy(msg)
        filtered.ranges = list(msg.ranges)
        n = len(filtered.ranges)

        for (a0_deg, a1_deg) in self._blind_ranges:
            i0, i1 = sector_indices(a0_deg, a1_deg, msg)
            if i0 is None or i1 is None:
                # 規約⑤: 範囲外は「そのセクタは存在しない」。clamp せず
                # マスクしない（安全側＝マスクしない、をここでも踏襲する）。
                self.get_logger().warn(
                    f'blind_angle_ranges の区間 ({a0_deg}, {a1_deg}) が現在の '
                    '/scan の angle_min/angle_max の範囲外。マスクしない',
                    throttle_duration_sec=5.0)
                continue
            if i0 <= i1:
                for i in range(i0, i1 + 1):
                    filtered.ranges[i] = float('inf')
            else:  # 配列境界（angle_min/angle_max）をまたぐ区間
                for i in range(i0, n):
                    filtered.ranges[i] = float('inf')
                for i in range(0, i1 + 1):
                    filtered.ranges[i] = float('inf')

        self._pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
