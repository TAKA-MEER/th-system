#!/usr/bin/env python3
# ============================================================
# map_downsampler — /map を表示用に間引いて配信する（WS-9G）
# ============================================================
# slam_toolbox の /map（resolution 0.05m/セル）は校舎 1 周級になると数十万〜百万セルと
# 巨大になり、WebUI へ送ると 2.4GHz 無線を食い潰す。表示は 1280×720 の俯瞰なので
# 0.05m/セルは不要。このノードは /map を購読し、factor×factor に畳んだ表示用コピー
# （既定 4 → 0.20m/セル）を /route/map_view として TRANSIENT_LOCAL で配信する。
# SLAM 自身の解像度（slam_params.yaml の resolution）は変えない。
#
# 純関数 downsample_occupancy（map_downsample.py）に実体を置き、ここは ROS2 の
# 購読/間引き/publish だけを担う（route_recorder / replay_runner と同じ二層構造）。
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from nav_msgs.msg import OccupancyGrid

from th_planning.map_downsample import downsample_occupancy


class MapDownsampler(Node):
    def __init__(self):
        super().__init__('map_downsampler')

        # ── パラメータ ──────────────────────────────────────
        # WAIVER(demo): W-03 — 数値は registry.yaml 経由でなくノード内リテラル既定値
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_topic', '/route/map_view')
        # WS-9G: 0.05m/セル → 0.20m/セル（factor=4）に畳む。表示は俯瞰なので十分。
        self.declare_parameter('factor', 4)
        # 受信のたびに出さず、この間隔（ms）に間引いて配信する。
        self.declare_parameter('publish_period_ms', 2000)
        self.declare_parameter('occupied_threshold', 50)

        self._map_topic = self.get_parameter('map_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._factor = int(self.get_parameter('factor').value)
        self._publish_period_s = int(self.get_parameter('publish_period_ms').value) / 1000.0
        self._occupied_threshold = int(self.get_parameter('occupied_threshold').value)

        # slam_toolbox の /map はラッチ（TRANSIENT_LOCAL）。合わせないと 1 枚も届かない。
        map_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, map_qos)
        # 配信側も TRANSIENT_LOCAL にし、WebUI が後から繋いでも最新の 1 枚が届くようにする。
        self._pub = self.create_publisher(OccupancyGrid, self._output_topic, map_qos)

        self._last_pub_s = 0.0           # 直前の publish 時刻（単調時計）
        self._last_logged_size = None    # ログは間引き後のサイズが変わったときだけ

    def _on_map(self, msg):
        now_s = time.monotonic()
        if now_s - self._last_pub_s < self._publish_period_s:
            return   # publish_period 経過前は捨てる（タイマは持たない）
        if not msg or not msg.info:
            return
        new_data, new_w, new_h = downsample_occupancy(
            msg.data, msg.info.width, msg.info.height, self._factor,
            self._occupied_threshold)

        out = OccupancyGrid()
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = msg.header.stamp
        out.info.resolution = msg.info.resolution * self._factor
        out.info.width = new_w
        out.info.height = new_h
        out.info.origin = msg.info.origin   # origin は元のまま
        out.data = new_data

        self._pub.publish(out)
        self._last_pub_s = now_s

        size = (msg.info.width, msg.info.height, msg.info.resolution)
        if size != self._last_logged_size:
            self.get_logger().info(
                f'地図を間引いて配信: {msg.info.width}x{msg.info.height} '
                f'({msg.info.resolution}m) -> {new_w}x{new_h} ({out.info.resolution}m)')
            self._last_logged_size = size


def main(args=None):
    rclpy.init(args=args)
    node = MapDownsampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
