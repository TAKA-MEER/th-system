#!/usr/bin/env python3
"""
gazebo_person_relay.py
========================
Gazebo の Actor（模擬試験員）位置を /person/status に変換して発行する。

位置取得は以下の順でフォールバック:
  1. /gazebo/get_entity_state サービス — reference_frame=robot_name で
     ロボット相対座標を直接取得（TF 不要）
  2. /gazebo/model_states トピック — actor と robot 双方の world 座標から
     相対位置を計算（no_actor ワールドの cylinder モデル用）
  3. /gazebo/link_states トピック — Actor がリンクとして見える場合のフォールバック
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import ModelStates, LinkStates
from th_system_msgs.msg import PersonStatus


class GazeboPersonRelay(Node):
    def __init__(self):
        super().__init__('gazebo_person_relay')

        # ── パラメータ ──────────────────────────────────────
        self.declare_parameter('actor_name',       'inspector')
        self.declare_parameter('robot_name',       'th_robot')
        self.declare_parameter('base_frame',       'base_link')
        self.declare_parameter('publish_rate_hz',  10.0)
        self.declare_parameter('lost_timeout_sec',  1.0)
        self.declare_parameter('max_detect_range',  8.0)

        self._actor_name    = self.get_parameter('actor_name').value
        self._robot_name    = self.get_parameter('robot_name').value
        self._base_frame    = self.get_parameter('base_frame').value
        self._max_range     = self.get_parameter('max_detect_range').value
        self._lost_timeout  = self.get_parameter('lost_timeout_sec').value
        rate_hz             = self.get_parameter('publish_rate_hz').value

        # ── 状態（ロボット基準の相対座標を格納）────────────
        self._rel_x: float | None = None
        self._rel_y: float | None = None
        self._last_update_t = None
        self._source = 'none'

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
        self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self._cb_model_states, 10,
            callback_group=self._sub_cb_group)

        # ── 方法3: Subscriber (link_states) ─────────────────
        self.create_subscription(
            LinkStates, '/gazebo/link_states',
            self._cb_link_states, 10,
            callback_group=self._sub_cb_group)

        # ── ポーリングタイマー（サービス方式） ───────────────
        # ロボットスポーン前は reference_frame が未存在のため Gazebo エラーになる。
        # spawn_delay(4.5s)+マージン で開始を遅らせてエラースパムを抑制する。
        self._spawn_grace_done = False
        self._spawn_grace_timer = self.create_timer(
            9.0, self._start_poll_timer,  # spawn_delay(4.5s) + Gazebo処理(~3s) 後にサービス開始
            callback_group=self._srv_cb_group)
        self._poll_timer = None

        # ── 発行タイマー ────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            f'gazebo_person_relay 起動 → actor="{self._actor_name}" '
            f'robot="{self._robot_name}"')

    def _start_poll_timer(self):
        """スポーン猶予後にサービスポーリングを開始する（1回限り）。"""
        self._spawn_grace_timer.cancel()
        self._poll_timer = self.create_timer(
            0.1, self._poll_actor_state,
            callback_group=self._srv_cb_group)
        self.get_logger().info('GetEntityState ポーリング開始')

    # ─────────────────────────────────────────────────────────
    # 方法 1: get_entity_state サービス
    # reference_frame = robot_name とすることで、返却値が
    # ロボット座標系（≒base_link）での相対位置になる。TF 不要。
    # ─────────────────────────────────────────────────────────
    def _poll_actor_state(self):
        if not self._get_state_cli.service_is_ready():
            self.get_logger().info(
                '/gazebo/get_entity_state 待機中…',
                throttle_duration_sec=5.0)
            return

        req = GetEntityState.Request()
        req.name            = self._actor_name
        req.reference_frame = self._robot_name  # ロボットモデル基準 → 相対座標で返却

        future = self._get_state_cli.call_async(req)
        future.add_done_callback(self._on_entity_state)

    def _on_entity_state(self, future):
        try:
            resp = future.result()
            if resp.success:
                # reference_frame=robot_name なので、そのまま相対座標として使用
                self._update_rel_position(
                    resp.state.pose.position.x,
                    resp.state.pose.position.y,
                    'service')
        except Exception as e:
            self.get_logger().debug(
                f'get_entity_state 失敗: {e}',
                throttle_duration_sec=5.0)

    # ─────────────────────────────────────────────────────────
    # 方法 2: /gazebo/model_states トピック
    # actor と robot の両 world 座標を取得し、差分を base_link 座標へ変換
    # ─────────────────────────────────────────────────────────
    def _cb_model_states(self, msg: ModelStates):
        if (self._actor_name not in msg.name or
                self._robot_name not in msg.name):
            return

        ai = msg.name.index(self._actor_name)
        ri = msg.name.index(self._robot_name)

        ax = msg.pose[ai].position.x
        ay = msg.pose[ai].position.y
        rx = msg.pose[ri].position.x
        ry = msg.pose[ri].position.y
        rq = msg.pose[ri].orientation
        ryaw = 2.0 * math.atan2(rq.z, rq.w)

        # world 差分 → ロボット座標系（base_link）へ回転変換
        dx = ax - rx
        dy = ay - ry
        rel_x =  dx * math.cos(ryaw) + dy * math.sin(ryaw)
        rel_y = -dx * math.sin(ryaw) + dy * math.cos(ryaw)

        self._update_rel_position(rel_x, rel_y, 'model_states')

    # ─────────────────────────────────────────────────────────
    # 方法 3: /gazebo/link_states トピック
    # ─────────────────────────────────────────────────────────
    def _cb_link_states(self, msg: LinkStates):
        robot_base_prefix = self._robot_name + '::base_link'
        actor_prefix      = self._actor_name + '::'

        robot_pos  = None
        robot_quat = None
        actor_pos  = None

        for i, name in enumerate(msg.name):
            if name == robot_base_prefix or name.startswith(robot_base_prefix):
                robot_pos  = (msg.pose[i].position.x, msg.pose[i].position.y)
                robot_quat = msg.pose[i].orientation
            if name.startswith(actor_prefix) and actor_pos is None:
                actor_pos = (msg.pose[i].position.x, msg.pose[i].position.y)

        if robot_pos is None or actor_pos is None or robot_quat is None:
            return

        ryaw = 2.0 * math.atan2(robot_quat.z, robot_quat.w)
        dx = actor_pos[0] - robot_pos[0]
        dy = actor_pos[1] - robot_pos[1]
        rel_x =  dx * math.cos(ryaw) + dy * math.sin(ryaw)
        rel_y = -dx * math.sin(ryaw) + dy * math.cos(ryaw)

        self._update_rel_position(rel_x, rel_y, 'link_states')

    # ─────────────────────────────────────────────────────────
    # 共通: 相対位置更新
    # ─────────────────────────────────────────────────────────
    def _update_rel_position(self, x: float, y: float, source: str):
        self._rel_x = x
        self._rel_y = y
        self._last_update_t = self.get_clock().now().nanoseconds * 1e-9

        if self._source != source:
            self._source = source
            # model_states と service が両方成功する場合は交互に切り替わる。
            # throttle で初回と5秒ごとのみログ出力する。
            self.get_logger().info(
                f'Actor 位置を [{source}] 経由で取得',
                throttle_duration_sec=5.0)

    # ─────────────────────────────────────────────────────────
    # PersonStatus 発行
    # ─────────────────────────────────────────────────────────
    def _publish(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        msg = PersonStatus()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame

        is_lost = (
            self._rel_x is None or
            self._last_update_t is None or
            (t - self._last_update_t) > self._lost_timeout
        )

        if is_lost:
            msg.is_lost     = True
            msg.lost_reason = 'DETECTION_LOST'
            msg.confidence  = 0.0
            self._pub.publish(msg)
            return

        rel_x = self._rel_x
        rel_y = self._rel_y
        dist  = math.hypot(rel_x, rel_y)

        if dist > self._max_range:
            msg.is_lost     = True
            msg.lost_reason = 'DETECTION_LOST'
            msg.confidence  = 0.0
        else:
            msg.position.x  = rel_x
            msg.position.y  = rel_y
            msg.position.z  = 0.0
            msg.confidence  = max(0.0, 1.0 - dist / self._max_range)
            msg.is_lost     = False
            msg.lost_reason = ''

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
