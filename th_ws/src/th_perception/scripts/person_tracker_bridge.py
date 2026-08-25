#!/usr/bin/env python3
# ============================================================
# person_tracker_bridge — human_kenchi (LiDAR脚検出) ブリッジ
#
# human_kenchi (multiple_sensor_person_tracking::PersonTracker, leg モード) が
# 発行する following_position + stop_following を th_system_msgs/PersonStatus に
# 変換して /person/status に発行する。
#
# following_position.status:
#   0 = NO_EXISTS  (脚未検出)   → is_lost=True,  lost_reason="DETECTION_LOST"
#   1 = EXISTS_LEG (脚追跡中)   → is_lost=False, lost_reason=""
# stop_following (Bool): detection_lost or target_changed_latched_ (対象切替の疑い)。
#   status=EXISTS_LEG のまま stop_following=True の場合は対象切替とみなし
#   is_lost=True, lost_reason="TARGET_SWITCHED" にして追従側を停止させる
#   (切替そのものは PersonTracker が既に内部検知しているが、以前はこの信号が
#   どこにも購読されておらず追従ロジックに一切伝わっていなかった)。
#
# following_position と stop_following はそれぞれ別トピックで publish されるため、
# 2トピック間の到着順序に依存しないよう、どちらのコールバックが来ても直近の
# 両方の値から再計算・再 publish する(古くとも最大1トラッカー周期分のずれ)。
#
# target_frame launch 引数を "base_link" にして起動する前提のため、
# following_position.pose.position はそのまま base_link 基準相対座標として使える。
# ============================================================
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))  # person_tracker_bridge_core.py と同じディレクトリ

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from th_system_msgs.msg import PersonStatus
from multiple_sensor_person_tracking.msg import FollowingPosition

from person_tracker_bridge_core import classify, STATUS_EXISTS_LEG


class PersonTrackerBridge(Node):
    def __init__(self):
        super().__init__('person_tracker_bridge')

        self._pub = self.create_publisher(PersonStatus, '/person/status', 10)
        self._last_status = STATUS_EXISTS_LEG  # 最初のメッセージが来るまでの楽観的初期値
        self._last_position = (0.0, 0.0)
        self._last_stamp = None
        self._stop_following = False

        self.create_subscription(
            FollowingPosition,
            'sobits_follower/multiple_sensor_person_tracking/following_position',
            self._following_cb,
            10,
        )
        self.create_subscription(
            Bool,
            'sobits_follower/multiple_sensor_person_tracking/stop_following',
            self._stop_cb,
            10,
        )

        self.get_logger().info(
            'person_tracker_bridge 起動 '
            '(human_kenchi following_position + stop_following → /person/status)')

    def _following_cb(self, msg: FollowingPosition):
        self._last_status = msg.status
        self._last_position = (msg.pose.position.x, msg.pose.position.y)
        self._last_stamp = msg.header.stamp
        self._publish()

    def _stop_cb(self, msg: Bool):
        self._stop_following = msg.data
        self._publish()

    def _publish(self):
        decision = classify(self._last_status, self._stop_following)

        out = PersonStatus()
        out.header.stamp = self._last_stamp if self._last_stamp is not None else self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.position.x = self._last_position[0]
        out.position.y = self._last_position[1]
        out.position.z = 0.0
        out.confidence = decision.confidence
        out.is_lost = decision.is_lost
        out.lost_reason = decision.lost_reason

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
