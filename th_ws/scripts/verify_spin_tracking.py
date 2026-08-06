#!/usr/bin/env python3
# ============================================================
# verify_spin_tracking.py — verify_spin_tracking.sh から呼ばれる本体
#
# odom 上で静止した試験員を置き、ロボットだけを OMEGA [rad/s] で
# 超信地旋回させる。自機回転補償 (VISION.md §4) が効いていれば、見かけの
# 移動が打ち消されて EXISTS_LEG(status=1) を維持できる。
#
# 判定力について: 補償が無い場合の1フレームあたりの見かけ移動量は
# OMEGA * PERSON_DIST / DET_HZ で、これが leg_tracking_range を超えるように
# 距離を選んである。超えていないと「補償が無くても通ってしまう」テストになり
# 意味を持たないため、実際の値を実行時に表示して確認できるようにしている。
# ============================================================
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from multiple_sensor_person_tracking.msg import FollowingPosition
from multiple_sensor_person_tracking.srv import SelectTarget

OMEGA = 0.8          # rad/s 超信地旋回 (Nav2 の旧 rotate_to_heading_angular_vel)
PERSON_DIST = 3.0    # m odom 上で静止している試験員までの距離 (VISION の d_prepare)
DET_HZ = 2.0         # DR-SPAAM の CPU 推論レート実測値
LEG_GATE = 1.10      # m leg_tracker_param.yaml の leg_tracking_range
FRAMES = 20


class SpinTest(Node):
    def __init__(self):
        super().__init__('spin_test')
        self._tfb = TransformBroadcaster(self)
        # StaticTransformBroadcaster はローカル変数にすると GC され、latch した
        # 静的 TF ごと消えるので必ず保持する
        self._static_tfb = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'base_link'
        st.child_frame_id = 'laser_link'
        st.transform.translation.z = 0.12
        st.transform.rotation.w = 1.0
        self._static_tfb.sendTransform(st)

        self._pub_scan = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self._pub_legs = self.create_publisher(
            PoseArray, '/dr_spaam/dr_spaam_detections', 10)
        self.create_subscription(
            FollowingPosition,
            'sobits_follower/multiple_sensor_person_tracking/following_position',
            self._cb_following, 10)

        self._t0 = time.time()
        self.samples = []
        self.create_timer(0.02, self._tick_tf)     # 50Hz (実機の EKF と同等)
        self.create_timer(0.05, self._tick_scan)   # 20Hz

    def _yaw(self):
        return OMEGA * (time.time() - self._t0)

    def _tick_tf(self):
        y = self._yaw()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.rotation.z = math.sin(y / 2)
        t.transform.rotation.w = math.cos(y / 2)
        self._tfb.sendTransform(t)

    def _tick_scan(self):
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser_link'
        m.angle_min = -math.pi
        m.angle_max = math.pi
        m.angle_increment = math.pi / 180
        m.range_min = 0.1
        m.range_max = 12.0
        m.ranges = [3.0] * 360
        self._pub_scan.publish(m)

    def expected(self):
        """静止した試験員の、現在の base_link から見た座標。"""
        y = self._yaw()
        c, s = math.cos(-y), math.sin(-y)
        return (c * PERSON_DIST, s * PERSON_DIST)

    def publish_detection(self):
        ex, ey = self.expected()
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'laser_link'
        p = Pose()
        p.position.x, p.position.y = ex, ey
        p.orientation.w = 1.0
        pa.poses = [p]
        self._pub_legs.publish(pa)

    def _cb_following(self, msg):
        ex, ey = self.expected()
        self.samples.append({
            'status': msg.status,
            'x': msg.pose.position.x, 'y': msg.pose.position.y,
            'ex': ex, 'ey': ey,
            'velocity': msg.velocity,
        })


def spin_for(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def main():
    rclpy.init()
    node = SpinTest()

    apparent = OMEGA * PERSON_DIST / DET_HZ
    print(f'旋回 {OMEGA} rad/s / 試験員まで {PERSON_DIST} m / 検出 {DET_HZ} Hz')
    print(f'補償が無い場合の見かけ移動量: {apparent:.2f} m/フレーム '
          f'(ゲート {LEG_GATE} m)')
    if apparent <= LEG_GATE:
        print('  警告: ゲートを超えていないため、補償が無くても通ってしまいます。'
              ' PERSON_DIST か OMEGA を上げてください')
    print()

    spin_for(node, 1.5)   # TF と scan を行き渡らせる

    cli = node.create_client(SelectTarget, '/person_tracker/select_target')
    if not cli.wait_for_service(timeout_sec=10.0):
        print('エラー: /person_tracker/select_target が見つかりません。'
              ' PersonTracker が activate されていない可能性があります',
              file=sys.stderr)
        return 1

    ex, ey = node.expected()
    req = SelectTarget.Request()
    req.candidate_index = -1
    req.x, req.y = ex, ey
    fut = cli.call_async(req)
    deadline = time.time() + 10.0
    while not fut.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not fut.done():
        print('エラー: select_target が応答しません', file=sys.stderr)
        return 1
    print('select_target:', fut.result().message)

    node.samples.clear()
    for _ in range(FRAMES):
        node.publish_detection()
        spin_for(node, 1.0 / DET_HZ)

    s = node.samples
    if not s:
        print('エラー: following_position を1件も受信しませんでした',
              file=sys.stderr)
        return 1

    ok = sum(1 for x in s if x['status'] == 1)
    print(f'\n=== 結果 ({FRAMES} フレーム) ===')
    print(f'EXISTS_LEG を維持: {ok}/{len(s)} サンプル')
    tracked = [x for x in s if x['status'] == 1]
    if tracked:
        errs = [math.hypot(x['x'] - x['ex'], x['y'] - x['ey']) for x in tracked]
        vels = [x['velocity'] for x in tracked]
        print(f'推定位置の誤差: 平均 {sum(errs)/len(errs):.3f} m  最大 {max(errs):.3f} m')
        # 試験員は静止しているので、KF の dt 修正が効いていれば速度は 0 付近。
        # 修正前は状態遷移行列が dt=0.033 固定で速度が約15倍に膨れていた
        print(f'推定速度 (静止中なので 0 付近が正): '
              f'平均 {sum(vels)/len(vels):.3f} m/s  最大 {max(vels):.3f} m/s')
    for x in s[:3] + s[-3:]:
        print(f"  status={x['status']} 推定=({x['x']:6.2f},{x['y']:6.2f}) "
              f"真値=({x['ex']:6.2f},{x['ey']:6.2f})")

    node.destroy_node()
    rclpy.shutdown()
    # 全サンプルで捕捉を維持できていれば成功
    return 0 if ok == len(s) else 2


if __name__ == '__main__':
    sys.exit(main())
