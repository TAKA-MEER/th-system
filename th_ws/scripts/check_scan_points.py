#!/usr/bin/env python3
"""/scan の点数を実測する（scan_expected_points の更新用）

疎通判定（DetailedDesign-state.md §12.2 行 3）は点数の**厳密一致**を要求する。
LiDAR の設定（scan_mode など）を変えたら必ずこの値を測り直すこと。
2026-08-18 の初回測定は DenseBoost で 100 スキャンすべて 1080 点だった。

使い方（コンテナ内）:
    python3 scripts/check_scan_points.py [サンプル数]

注意: /scan は sensor_data QoS（BEST_EFFORT）なので、`ros2 topic echo` の
既定（RELIABLE）では購読できない。このスクリプトは best_effort で購読する。
"""
import sys
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Counter_(Node):
    def __init__(self, want):
        super().__init__('check_scan_points')
        self.want = want
        self.counts = Counter()
        self.n = 0
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'/scan を {want} スキャン待ちます...')

    def _on_scan(self, msg):
        self.counts[len(msg.ranges)] += 1
        self.n += 1
        if self.n % 20 == 0:
            self.get_logger().info(f'{self.n}/{self.want}')


def main():
    want = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    rclpy.init()
    node = Counter_(want)
    try:
        while rclpy.ok() and node.n < want:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass

    print()
    if not node.counts:
        print('1 スキャンも受信できなかった。LiDAR が回っているか確認すること')
        rv = 1
    else:
        for k, v in sorted(node.counts.items()):
            print(f'  {k} 点 : {v} 回')
        if len(node.counts) == 1:
            n = next(iter(node.counts))
            print(f'\n★ 一定である。registry.yaml の scan_expected_points を {n} にする')
            rv = 0
        else:
            print('\n★ 点数が一定でない。厳密一致の判定は成立しない。')
            print('  DetailedDesign-state.md §12.2 行 3 の判定方法から見直しが要る')
            rv = 2
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rv)


if __name__ == '__main__':
    main()
