#!/usr/bin/env python3
"""
gazebo_person_relay.py
========================
Gazebo の Actor（模擬試験員）位置を /person/status に変換して発行する。

Actor 位置取得は以下の 3 つの方法をフォールバック付きで試行する:
  1. /gazebo/get_entity_state サービス（最も確実、Actor/Model 両対応）
  2. /gazebo/model_states トピック（no_actor ワールドの cylinder モデル用）
  3. /gazebo/link_states トピック（Actor がリンクとして見える場合）
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import ModelStates, LinkStates
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
        self._last_update_t = None
        self._source = 'none'           # どの方法で取得しているか

        # ── TF2 ──────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Publisher ───────────────────────────────────────
        self._pub = self.create_publisher(PersonStatus, '/person/status', 10)

        # ── Callback Groups ─────────────────────────────────
        self._srv_cb_group = MutuallyExclusiveCallbackGroup()
        self._sub_cb_group = ReentrantCallbackGroup()

        # ── 方法1: Service Client (get_entity_state) ────────
        self._get_state_cli = self.create_client(
            GetEntityState, '/gazebo/get_entity_state',
            callback_group=self._srv_cb_group)

        # ── 方法2: Subscriber (model_states) ────────────────
        # no_actor ワールドでは inspector が通常モデルなので有効
        self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._cb_model_states, 10,
            callback_group=self._sub_cb_group)

        # ── 方法3: Subscriber (link_states) ─────────────────
        # Actor のリンクが含まれる場合のフォールバック
        self.create_subscription(
            LinkStates, '/gazebo/link_states',
            self._cb_link_states, 10,
            callback_group=self._sub_cb_group)

        # ── ポーリングタイマー（サービス方式） ───────────────
        self._poll_timer = self.create_timer(
            0.1, self._poll_actor_state,
            callback_group=self._srv_cb_group)

        # ── 発行タイマー ────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            f'gazebo_person_relay 起動 → actor="{self._actor_name}"')

    # ─────────────────────────────────────────────────────────
    # 方法 1: get_entity_state サービス
    # ─────────────────────────────────────────────────────────
    def _poll_actor_state(self):
        """定期的に Gazebo から Actor の位置を取得する (サービス方式)"""
        if not self._get_state_cli.service_is_ready():
            self.get_logger().info(
                '/gazebo/get_entity_state 待機中…',
                throttle_duration_sec=5.0)
            return

        req = GetEntityState.Request()
        req.name = self._actor_name
        req.reference_frame = 'world'

        future = self._get_state_cli.call_async(req)
        future.add_done_callback(self._on_entity_state)

    def _on_entity_state(self, future):
        """サービス応答コールバック"""
        try:
            resp = future.result()
            if resp.success:
                self._update_position(
                    resp.state.pose.position.x,
                    resp.state.pose.position.y,
                    'service')
        except Exception as e:
            self.get_logger().debug(
                f'get_entity_state 失敗: {e}',
                throttle_duration_sec=5.0)

    # ─────────────────────────────────────────────────────────
    # 方法 2: /gazebo/model_states トピック
    # ─────────────────────────────────────────────────────────
    def _cb_model_states(self, msg: ModelStates):
        """Gazebo の全モデル位置から Actor の位置を抜き出す"""
        if self._actor_name in msg.name:
            idx = msg.name.index(self._actor_name)
            pose = msg.pose[idx]
            self._update_position(
                pose.position.x, pose.position.y, 'model_states')

    # ─────────────────────────────────────────────────────────
    # 方法 3: /gazebo/link_states トピック
    # ─────────────────────────────────────────────────────────
    def _cb_link_states(self, msg: LinkStates):
        """Actor のリンクが link_states に含まれている場合に取得"""
        # Actor は "actor_name::actor_name_pose" または
        # "actor_name::link" のような名前で表れる可能性がある
        for i, name in enumerate(msg.name):
            if name.startswith(self._actor_name + '::'):
                pose = msg.pose[i]
                self._update_position(
                    pose.position.x, pose.position.y, 'link_states')
                return

    # ─────────────────────────────────────────────────────────
    # 共通: 位置更新
    # ─────────────────────────────────────────────────────────
    def _update_position(self, x: float, y: float, source: str):
        """位置を更新する (どの方法から呼ばれても同じ処理)"""
        self._actor_world_x = x
        self._actor_world_y = y
        self._last_update_t = self.get_clock().now().nanoseconds * 1e-9

        if self._source != source:
            self._source = source
            self.get_logger().info(
                f'Actor 位置を [{source}] 経由で取得開始')

    # ─────────────────────────────────────────────────────────
    # PersonStatus 発行
    # ─────────────────────────────────────────────────────────
    def _publish(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        msg = PersonStatus()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame

        # モデル状態を受信していない、またはタイムアウト
        is_lost = (
            self._actor_world_x is None or
            self._last_update_t is None or
            (t - self._last_update_t) > self._lost_timeout
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
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
