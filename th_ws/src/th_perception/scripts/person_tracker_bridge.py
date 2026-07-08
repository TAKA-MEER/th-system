#!/usr/bin/env python3
# ============================================================
# person_tracker_bridge — human_kenchi (LiDAR脚検出) ブリッジ
#
# human_kenchi (multiple_sensor_person_tracking::PersonTracker, leg モード) が
# 発行する following_position を th_system_msgs/PersonStatus に変換して
# /person/status に発行する。
#
# following_position.status:
#   0 = NO_EXISTS  (脚未検出)   → is_lost=True,  lost_reason="DETECTION_LOST"
#   1 = EXISTS_LEG (脚追跡中)   → is_lost=False, lost_reason=""
#
# target_frame launch 引数を "base_link" にして起動する前提のため、
# following_position.pose.position はそのまま base_link 基準相対座標として使える。
# ============================================================
import rclpy
from rclpy.node import Node
from th_system_msgs.msg import PersonStatus
from multiple_sensor_person_tracking.msg import FollowingPosition

STATUS_EXISTS_LEG = 1
TRACKED_CONFIDENCE = 0.9  # dr_spaam_param.yaml の conf_thresh を通過した検出のみ EXISTS_LEG になるため固定値


class PersonTrackerBridge(Node):
    def __init__(self):
        super().__init__('person_tracker_bridge')

        self._pub = self.create_publisher(PersonStatus, '/person/status', 10)
        self.create_subscription(
            FollowingPosition,
            'sobits_follower/multiple_sensor_person_tracking/following_position',
            self._cb,
            10,
        )

        self.get_logger().info(
            'person_tracker_bridge 起動 '
            '(human_kenchi following_position → /person/status)')

    def _cb(self, msg: FollowingPosition):
        is_lost = msg.status != STATUS_EXISTS_LEG

        out = PersonStatus()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = 'base_link'
        out.position.x      = msg.pose.position.x
        out.position.y      = msg.pose.position.y
        out.position.z      = 0.0
        out.confidence      = 0.0 if is_lost else TRACKED_CONFIDENCE
        out.is_lost         = is_lost
        out.lost_reason     = 'DETECTION_LOST' if is_lost else ''

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
