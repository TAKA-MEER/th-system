#!/usr/bin/env python3
"""
map_freezer.py — 地図更新 OFF の間、/map を開いた地図のまま固定する（Spec-onsite §4.0.2）。

slam_toolbox の出力を /map_live へ remap し、このノードが常に /map を配信する。
購読者（Nav2 static layer・map_downsampler・pin_registrar・WebUI）は付け替えない。

  /map_live                    ← slam_toolbox（RELIABLE・TRANSIENT_LOCAL）
  /system/state                ← mode・prev_mode・map_update
  /map_session/freeze_anchor   ← slam_control（地図を開いた・保存した等。JSON）
  /map                         → 通す／写しを出し続ける（RELIABLE・TRANSIENT_LOCAL）

判定と写しを取る契機は map_freezer_core.py（ROS 非依存・ホストの pytest で縛る）。
パラメータ republish_period_s: 凍結中に写しを再配信する周期（秒。0 で再配信しない）。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from th_system_msgs.msg import SystemState

from th_config_manager.map_freezer_core import (
    FREEZE, PASS_NO_COPY, MapFreezer, parse_anchor)

_LATCH = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    history=QoSHistoryPolicy.KEEP_LAST)
# 履歴を取りこぼさない（opening → opened の順序が要る）
_ANCHOR_QOS = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)


class MapFreezerNode(Node):

    def __init__(self):
        super().__init__('map_freezer')
        self.declare_parameter('republish_period_s', 5.0)
        period = float(self.get_parameter('republish_period_s').value)

        self._core = MapFreezer()
        self._last_decision = None
        self._pub = self.create_publisher(OccupancyGrid, '/map', _LATCH)
        self.create_subscription(OccupancyGrid, '/map_live', self._cb_live, _LATCH)
        self.create_subscription(SystemState, '/system/state', self._cb_state, _LATCH)
        self.create_subscription(String, '/map_session/freeze_anchor',
                                 self._cb_anchor, _ANCHOR_QOS)
        if period > 0.0:
            self.create_timer(period, self._cb_timer)
        self.get_logger().info(
            f'map_freezer 起動: /map_live → /map（地図更新 OFF の間は写しを出し続ける。'
            f'再配信 {period} 秒）')

    def _cb_live(self, msg):
        self._core.on_live(msg)
        self._step()

    def _cb_state(self, msg):
        self._core.on_state(msg.mode, msg.prev_mode, msg.map_update)
        self._step()

    def _cb_anchor(self, msg):
        parsed = parse_anchor(msg.data)
        if parsed is None:
            self.get_logger().warn(f'freeze_anchor を読めません: {msg.data!r}')
            return
        if self._core.on_anchor(*parsed):
            self.get_logger().info(f'地図の契機: {parsed[2]}')
        self._step()

    def _cb_timer(self):
        frozen = self._core.current_frozen()
        if frozen is not None:
            self._pub.publish(frozen)

    def _step(self):
        decision = self._core.decision()
        if decision != self._last_decision:
            self._last_decision = decision
            if decision == FREEZE:
                self.get_logger().info('地図を凍結します（地図更新 OFF）')
            elif decision == PASS_NO_COPY:
                self.get_logger().warn(
                    '凍結が要る場面ですが写しがありません。生の地図を通します')
            else:
                self.get_logger().info('地図を通します')
        out = self._core.poll()
        if out is not None:
            self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapFreezerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
