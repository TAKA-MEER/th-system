#!/usr/bin/env python3
# ============================================================
# verify_spin_tracking.py — verify_spin_tracking.sh から呼ばれる本体
#
# odom 上で静止した試験員を置き、ロボットだけを OMEGA [rad/s] で
# 超信地旋回させる。自機回転補償が効いていれば、見かけの移動が
# 打ち消されて EXISTS_LEG(status=1) を維持できる。
# 補償が無い場合は 1 フレームあたり OMEGA*d*(1/DET_HZ) だけ見かけ位置が
# 飛び、leg_tracking_range(1.10m) を超えて NO_EXISTS(0) に落ちる。
# ============================================================
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from multiple_sensor_person_tracking.msg import FollowingPosition
from multiple_sensor_person_tracking.srv import SelectTarget

OMEGA = 0.8        # rad/s 超信地旋回 (Nav2 の旧 rotate_to_heading_angular_vel)
PX, PY = 2.0, 0.0  # odom 上で静止している試験員
DET_HZ = 2.0       # DR-SPAAM の実測レート

class T(Node):
    def __init__(self):
        super().__init__("spin_test")
        self.tfb = TransformBroadcaster(self)
        st = StaticTransformBroadcaster(self)
        s = TransformStamped(); s.header.frame_id="base_link"; s.child_frame_id="laser_link"
        s.header.stamp = self.get_clock().now().to_msg(); s.transform.translation.z=0.12
        s.transform.rotation.w=1.0; st.sendTransform(s)
        self.scan = self.create_publisher(LaserScan, "/scan_filtered", 10)
        self.legs = self.create_publisher(PoseArray, "/dr_spaam/dr_spaam_detections", 10)
        self.create_subscription(FollowingPosition,
            "sobits_follower/multiple_sensor_person_tracking/following_position",
            self.cb, 10)
        self.t0 = time.time(); self.samples=[]
        self.create_timer(0.02, self.tf_tick)
        self.create_timer(0.05, self.scan_tick)
    def yaw(self): return OMEGA*(time.time()-self.t0)
    def tf_tick(self):
        y=self.yaw(); t=TransformStamped()
        t.header.stamp=self.get_clock().now().to_msg()
        t.header.frame_id="odom"; t.child_frame_id="base_link"
        t.transform.rotation.z=math.sin(y/2); t.transform.rotation.w=math.cos(y/2)
        self.tfb.sendTransform(t)
    def scan_tick(self):
        m=LaserScan(); m.header.stamp=self.get_clock().now().to_msg()
        m.header.frame_id="laser_link"; m.angle_min=-math.pi; m.angle_max=math.pi
        m.angle_increment=math.pi/180; m.range_min=0.1; m.range_max=12.0
        m.ranges=[3.0]*360; self.scan.publish(m)
    def expected(self):
        y=self.yaw(); c,s=math.cos(-y),math.sin(-y)
        return (c*PX - s*PY, s*PX + c*PY)
    def detect(self):
        ex,ey=self.expected()
        pa=PoseArray(); pa.header.stamp=self.get_clock().now().to_msg()
        pa.header.frame_id="laser_link"
        p=Pose(); p.position.x=ex; p.position.y=ey; p.orientation.w=1.0
        pa.poses=[p]; self.legs.publish(pa)
    def cb(self,msg):
        ex,ey=self.expected()
        self.samples.append((msg.status, msg.pose.position.x, msg.pose.position.y, ex, ey))

rclpy.init(); n=T()
for _ in range(60): rclpy.spin_once(n, timeout_sec=0.02)
cli=n.create_client(SelectTarget, "/person_tracker/select_target")
cli.wait_for_service(timeout_sec=5.0)
ex,ey=n.expected()
req=SelectTarget.Request(); req.candidate_index=-1; req.x=ex; req.y=ey
fut=cli.call_async(req)
while not fut.done(): rclpy.spin_once(n, timeout_sec=0.05)
print("select_target:", fut.result().message)
n.samples.clear()
N=20
for i in range(N):
    n.detect()
    end=time.time()+1.0/DET_HZ
    while time.time()<end: rclpy.spin_once(n, timeout_sec=0.01)
ok=sum(1 for s in n.samples if s[0]==1)
print(f"\n=== 旋回 {OMEGA} rad/s / 検出 {DET_HZ} Hz / {N} フレーム ===")
print(f"EXISTS_LEG を維持: {ok}/{len(n.samples)} サンプル")
err=[math.hypot(s[1]-s[3], s[2]-s[4]) for s in n.samples if s[0]==1]
if err: print(f"推定位置の誤差: 平均 {sum(err)/len(err):.3f} m  最大 {max(err):.3f} m")
for s in n.samples[:3]+n.samples[-3:]:
    print(f"  status={s[0]} 推定=({s[1]:6.2f},{s[2]:6.2f}) 真値=({s[3]:6.2f},{s[4]:6.2f})")
n.destroy_node(); rclpy.shutdown()
