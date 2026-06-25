#!/usr/bin/env python3
# ============================================================
# person_tracker_stub — テスト用試験員位置スタブ
#
# ML ベースの実装が完成するまでの代替ノード。
# 設定可能なパターンで /person/status を発行する。
#
# 本番実装 (person_tracker) が満たすべきインターフェース:
#   Publisher: /person/status (th_system_msgs/PersonStatus)
#     - header.stamp: 現在時刻
#     - position: ロボット基準 相対位置 (m) — base_link 座標系
#     - confidence: 0.0-1.0
#     - is_lost: 検出できていない場合 True
#     - lost_reason: "DETECTION_LOST" / "LOW_CONFIDENCE" / ""
# ============================================================
import rclpy
from rclpy.node import Node
from th_system_msgs.msg import PersonStatus
import math


class PersonTrackerStub(Node):
    def __init__(self):
        super().__init__('person_tracker_stub')

        # ── パラメータ ──────────────────────────────────────
        # pattern: "static" / "walk_forward" / "oscillate" / "lost"
        self.declare_parameter('pattern',         'static')
        self.declare_parameter('initial_x',        1.5)   # m (前方)
        self.declare_parameter('initial_y',        0.0)   # m
        self.declare_parameter('walk_speed',       0.3)   # m/s
        self.declare_parameter('oscillate_amp',    0.5)   # m
        self.declare_parameter('oscillate_freq',   0.2)   # Hz
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('lost_after_sec',  -1.0)   # -1=ロストしない

        self._pattern      = self.get_parameter('pattern').value
        self._x            = self.get_parameter('initial_x').value
        self._y            = self.get_parameter('initial_y').value
        self._walk_speed   = self.get_parameter('walk_speed').value
        self._osc_amp      = self.get_parameter('oscillate_amp').value
        self._osc_freq     = self.get_parameter('oscillate_freq').value
        self._lost_after   = self.get_parameter('lost_after_sec').value
        rate_hz            = self.get_parameter('publish_rate_hz').value

        self._pub = self.create_publisher(PersonStatus, '/person/status', 10)

        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._publish)
        self._start_t = None

        self.get_logger().warn(
            f'=== person_tracker_stub 起動 [pattern={self._pattern}] ===')
        self.get_logger().warn(
            '本番運用では ML ベースの person_tracker に切り替えること')

    def _publish(self):
        now = self.get_clock().now()
        t   = now.nanoseconds * 1e-9

        if self._start_t is None:
            self._start_t = t

        elapsed = t - self._start_t

        # ロスト判定
        is_lost     = False
        lost_reason = ''
        if self._lost_after > 0 and elapsed > self._lost_after:
            is_lost     = True
            lost_reason = 'DETECTION_LOST'

        # 位置計算
        x, y = self._x, self._y
        if not is_lost:
            if self._pattern == 'walk_forward':
                x = self._x + self._walk_speed * elapsed
            elif self._pattern == 'oscillate':
                x = self._x
                y = self._osc_amp * math.sin(2 * math.pi * self._osc_freq * elapsed)

        msg = PersonStatus()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.position.x      = x
        msg.position.y      = y
        msg.position.z      = 0.0
        msg.confidence      = 0.0 if is_lost else 0.9
        msg.is_lost         = is_lost
        msg.lost_reason     = lost_reason

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerStub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
