#!/usr/bin/env python3
# ============================================================
# lidar_filter — LiDAR 死角フィルターノード
#
# 20mm 角アルミパイプ (4 角柱) による死角を inf でマスクし
# /scan_filtered を発行する。
# 死角角度は実測後に blind_angle_ranges パラメータで更新。
# ============================================================
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
import math
import copy


class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')

        # ── パラメータ ──────────────────────────────────────
        # blind_angle_ranges: [[start_deg, end_deg], ...] (0=前方 右回り正)
        # 初期値: 4 角柱を 45°±5° ずつでマスク (実測後に更新)
        self.declare_parameter('blind_angle_ranges',
            [40.0, 50.0,   # 右前
             130.0, 140.0, # 右後
             220.0, 230.0, # 左後
             310.0, 320.0] # 左前
        )
        self.declare_parameter('input_topic',  '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        self._blind_ranges = self._build_blind_ranges(
            self.get_parameter('blind_angle_ranges').value)
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
