#!/usr/bin/env python3
# ============================================================
# home_declarer — 待機場所宣言・LiDAR照合 (Spec-onsite §4.0, WP-ONSITE)
# ============================================================
# 当日、機体を待機場所に置いて宣言する。SLAM 自己位置(map->base_link TF)と
# 待機場所ピン(kind=HOME)のずれを offset_m/offset_deg で計算し、許容内か
# force なら success を返す。/onsite/home_declared(std_msgs/Bool, latched)を
# true にして WebUI に見せる。HOME ピンが更新されたら false に戻す。
#
# FSM には配線しない (スタンドアロンのサービス)。
import math
import signal

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)

import tf2_ros

from std_msgs.msg import Bool
from th_system_msgs.msg import PinList
from th_system_msgs.srv import DeclareHome

from th_onsite.home_declare_core import (
    HomeDeclareParams, pose_offset, within_tolerance,
)


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class HomeDeclarer(Node):
    def __init__(self):
        super().__init__('home_declarer')

        # ── パラメータ ──────────────────────────────────────
        # WAIVER(demo): W-13 — 数値はノード内リテラル既定値
        self.declare_parameter('home_declare_tolerance_m', 0.30)
        self.declare_parameter('home_declare_tolerance_deg', 15.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self._params = HomeDeclareParams(
            tolerance_m=float(self.get_parameter('home_declare_tolerance_m').value),
            tolerance_deg=float(self.get_parameter('home_declare_tolerance_deg').value),
        )
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # ── 状態 ────────────────────────────────────────────
        self._home_pose = None      # HOME ピンの (x, y, yaw) map 座標。無ければ None
        self._declared = False      # /onsite/home_declared の値

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ─────────────────────────────────────────────
        pins_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              history=QoSHistoryPolicy.KEEP_LAST)
        declared_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                  durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                  history=QoSHistoryPolicy.KEEP_LAST)

        sub_cbg = ReentrantCallbackGroup()

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(PinList, '/onsite/pins', self._on_pins,
                                 pins_qos, callback_group=sub_cbg)

        # ── Publishers ──────────────────────────────────────
        self._pub_declared = self.create_publisher(Bool, '/onsite/home_declared',
                                                   declared_qos)

        # ── Services ────────────────────────────────────────
        self.create_service(DeclareHome, '/onsite/declare_home', self._on_declare_home,
                            callback_group=sub_cbg)

        # 起動時は False を latched publish
        self._publish_declared()
        self.get_logger().info(
            f'home_declarer 起動 (tol={self._params.tolerance_m}m / '
            f'{self._params.tolerance_deg}°)')

    # ── /onsite/home_declared publish ──────────────────────
    def _publish_declared(self):
        msg = Bool()
        msg.data = self._declared
        self._pub_declared.publish(msg)

    # ── /onsite/pins 受信 ──────────────────────────────────
    def _on_pins(self, msg: PinList):
        home = None
        for p in msg.pins:
            if p.kind == 'HOME':
                pose = p.pose.position
                home = (float(pose.x), float(pose.y), _yaw_from_quat(p.pose.orientation))
                break
        if home != self._home_pose:
            self._home_pose = home
            # HOME ピンが更新されたら宣言をやり直す (false に戻す)
            if self._declared:
                self._declared = False
                self._publish_declared()
                self.get_logger().info('HOME ピン更新: 宣言を False に戻しました')

    # ── 座標変換 ──────────────────────────────────────────
    def _map_pose(self):
        """map→base_link TF から (x, y, yaw) を返す。取れなければ None。"""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y, _yaw_from_quat(tf.transform.rotation))

    # ── /onsite/declare_home サービス ──────────────────────
    def _on_declare_home(self, request, response):
        if self._home_pose is None:
            response.success = False
            response.offset_m = 0.0
            response.offset_deg = 0.0
            response.message = '待機場所ピンが未登録です'
            return response

        robot = self._map_pose()
        if robot is None:
            response.success = False
            response.offset_m = 0.0
            response.offset_deg = 0.0
            response.message = '自己位置が取得できません（SLAM 未起動？）'
            return response

        offset_m, offset_deg = pose_offset(robot, self._home_pose)
        response.offset_m = float(offset_m)
        response.offset_deg = float(offset_deg)

        if within_tolerance(offset_m, offset_deg, self._params) or request.force:
            self._declared = True
            self._publish_declared()
            response.success = True
            response.message = (
                f'宣言しました（ずれ: {offset_m:.2f} m / {offset_deg:.1f}°）')
        else:
            response.success = False
            response.message = (
                f'待機場所とのずれが大きすぎます（{offset_m:.2f} m / {offset_deg:.1f}°）。'
                f'置き直すか、force で宣言してください')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HomeDeclarer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # SIGTERM 二重 shutdown ガード (ONSITE-A と同じ)
    def _sigterm_handler(_signum, _frame):
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
