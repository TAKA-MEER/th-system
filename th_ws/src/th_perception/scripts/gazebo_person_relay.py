#!/usr/bin/env python3
"""
gazebo_person_relay.py
========================
Gazebo の Actor（模擬試験員）位置を /person/status に変換して発行する。

Gazebo Classic は /gazebo/model_states に全モデルの位置を発行するので、
そこから "inspector" Actor の位置を取得し、ロボットの base_link 基準の
相対座標に変換して PersonStatus として発行する。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from gazebo_msgs.msg import ModelStates
from th_system_msgs.msg import PersonStatus

import tf2_ros
import tf2_geometry_msgs  # noqa
from geometry_msgs.msg import PointStamped


class GazeboPersonRelay(Node):
    def __init__(self):
        super().__init__('gazebo_person_relay')

        # ── パラメータ ──────────────────────────────────────
        self.declare_parameter('actor_name',       'inspector')
        self.declare_parameter('base_frame',       'base_link')
        self.declare_parameter('world_frame',      'odom')
        self.declare_parameter('publish_rate_hz',  10.0)
        self.declare_parameter('lost_timeout_sec',  1.0)
        # 試験員と見なす最大距離（これ以上離れると is_lost=True）
        self.declare_parameter('max_detect_range',  8.0)

        self._actor_name    = self.get_parameter('actor_name').value
        self._base_frame    = self.get_parameter('base_frame').value
        self._world_frame   = self.get_parameter('world_frame').value
        self._max_range     = self.get_parameter('max_detect_range').value
        self._lost_timeout  = self.get_parameter('lost_timeout_sec').value
        rate_hz             = self.get_parameter('publish_rate_hz').value

        # ── 状態 ────────────────────────────────────────────
        self._actor_world_x: float | None = None
        self._actor_world_y: float | None = None
        self._last_model_t  = None

        # ── TF2 ──────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Publisher ───────────────────────────────────────
        self._pub = self.create_publisher(PersonStatus, '/person/status', 10)

        # ── Subscriber: Gazebo モデル状態 ────────────────────
        self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._cb_model_states, 10)

        # ── 発行タイマー ────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            f'gazebo_person_relay 起動 → actor="{self._actor_name}"')

    def _cb_model_states(self, msg: ModelStates):
        """Gazebo の全モデル位置から Actor の位置を抜き出す"""
        if self._actor_name in msg.name:
            idx = msg.name.index(self._actor_name)
            pose = msg.pose[idx]
            self._actor_world_x = pose.position.x
            self._actor_world_y = pose.position.y
            self._last_model_t  = self.get_clock().now().nanoseconds * 1e-9

    def _publish(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        msg = PersonStatus()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame

        # モデル状態を受信していない、またはタイムアウト
        is_lost = (
            self._actor_world_x is None or
            self._last_model_t is None or
            (t - self._last_model_t) > self._lost_timeout
        )

        if is_lost:
            msg.is_lost     = True
            msg.lost_reason = 'DETECTION_LOST'
            msg.confidence  = 0.0
            self._pub.publish(msg)
            return

        # Actor のワールド座標を base_link 基準に変換
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._world_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))

            # world → base_link の平行移動のみ取得（簡易変換）
            # 厳密には回転も考慮すべきだが、平面移動のみなので
            # TF の full transform を使う

            ps = PointStamped()
            ps.header.frame_id = self._world_frame
            ps.header.stamp    = self.get_clock().now().to_msg()
            ps.point.x = self._actor_world_x
            ps.point.y = self._actor_world_y
            ps.point.z = 0.0

            ps_base = self._tf_buffer.transform(
                ps, self._base_frame,
                timeout=Duration(seconds=0.1))

            rel_x = ps_base.point.x
            rel_y = ps_base.point.y
            dist  = math.hypot(rel_x, rel_y)

            if dist > self._max_range:
                msg.is_lost     = True
                msg.lost_reason = 'DETECTION_LOST'
                msg.confidence  = 0.0
            else:
                msg.position.x = rel_x
                msg.position.y = rel_y
                msg.position.z = 0.0
                msg.confidence = max(0.0, 1.0 - dist / self._max_range)
                msg.is_lost    = False
                msg.lost_reason = ''

        except Exception as e:
            self.get_logger().warn(
                f'TF 変換失敗: {e}', throttle_duration_sec=3.0)
            msg.is_lost     = True
            msg.lost_reason = 'DETECTION_LOST'
            msg.confidence  = 0.0

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPersonRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
