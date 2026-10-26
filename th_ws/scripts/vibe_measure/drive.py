#!/usr/bin/env python3
"""揺れ計測用の決まった走行。WebUI のジョグと同じ /cmd_vel_manual_raw（-1〜1 の比率）を出す。
jog_gate → twist_mux → obstacle_limiter を通るので、機体側の上限・非常停止はそのまま効く。

  drive.py straight RATIO DIST_M [--ramp S] [--stop hard|soft] [--stop-ramp S]
  drive.py spin RATIO ANGLE_DEG [--ramp S] [--stop hard|soft] [--stop-ramp S]

距離・角度は /odom で測る。次のどれかで即 0 を出して終わる:
  /system/state が MANUAL・RUN でない／非常停止 / /odom が 0.5 秒来ない / 上限（2.0 m・200°・20 秒）を超えた
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from th_system_msgs.msg import SystemState

MAX_DIST = 2.0
MAX_ANGLE = 200.0
MAX_TIME = 20.0
RATE = 10.0

ap = argparse.ArgumentParser()
ap.add_argument('kind', choices=['straight', 'spin'])
ap.add_argument('ratio', type=float)
ap.add_argument('amount', type=float, help='straight は m、spin は deg')
ap.add_argument('--ramp', type=float, default=1.0, help='加速にかける秒')
ap.add_argument('--stop', choices=['hard', 'soft'], default='hard')
ap.add_argument('--stop-ramp', type=float, default=1.5, help='soft のとき減速にかける秒')
ap.add_argument('--shape', choices=['linear', 'scurve'], default='linear',
                help='加速・減速の形。scurve は加速度も滑らかに上げ下げする（余弦の S 字）')
ap.add_argument('--check', action='store_true', help='指令を出さず、状態と /odom が読めるかだけ見る')
a = ap.parse_args()
if not 0 < abs(a.ratio) <= 1.0:
    sys.exit('ratio は 0〜1')
if a.kind == 'straight' and not 0 < a.amount <= MAX_DIST:
    sys.exit(f'距離は {MAX_DIST} m まで')
if a.kind == 'spin' and not 0 < abs(a.amount) <= MAX_ANGLE:
    sys.exit(f'角度は {MAX_ANGLE}° まで')

rclpy.init()
node = rclpy.create_node('vibe_drive')
pub = node.create_publisher(Twist, '/cmd_vel_manual_raw', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
# WebUI と同じく、指令を出している間はジョグのリースも送る（MANUAL の PAUSE → RUN もこれで入る）
lease_pub = node.create_publisher(String, '/ui/jog_lease', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
_lease_t = [0.0]
st = {'odom': None, 'odom_t': 0.0, 'state': None}


def on_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    st['odom'] = (p.x, p.y, yaw, m.twist.twist.linear.x, m.twist.twist.angular.z)
    st['odom_t'] = time.monotonic()


node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)
node.create_subscription(SystemState, '/system/state', lambda m: st.__setitem__('state', m),
                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL))


def send(r):
    if time.monotonic() - _lease_t[0] >= 0.2:   # WebUI と同じ 5 Hz
        lease_pub.publish(String(data='vibe_drive'))
        _lease_t[0] = time.monotonic()
    t = Twist()
    if a.kind == 'straight':
        t.linear.x = r
    else:
        t.angular.z = r
    pub.publish(t)


def ok():
    s = st['state']
    if s is None or s.mode != 'MANUAL' or s.state != 'RUN' or s.estop_ui or s.estop_hw:
        return f'状態が {getattr(s, "mode", "?")}/{getattr(s, "state", "?")}（非常停止含む）'
    if s.speed_limit == 'stop':
        return f'速度の上限が stop（zone={s.zone}。WebUI の画面が開かれていないと 0 に絞られる）'
    if time.monotonic() - st['odom_t'] > 0.5:
        return '/odom が途絶'
    return None


def stop_and_exit(msg, code):
    # 0 を 3 秒送り続ける。S 字の通常減速（a=0.4、J=0.5）で最高速
    # 0.55 m/s から止まる約2.2秒に余裕を持たせ、途絶による段差を避ける。
    for _ in range(30):
        send(0.0)
        rclpy.spin_once(node, timeout_sec=0.1)
    print(msg, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


# 状態と /odom が揃うまで待つ（最大 5 秒）
t0 = time.monotonic()
while time.monotonic() - t0 < 5.0 and (ok() is not None):
    rclpy.spin_once(node, timeout_sec=0.1)
if a.check:
    o = st['odom']
    print(f"確認: 状態 {getattr(st['state'], 'mode', None)}/{getattr(st['state'], 'state', None)}・判定 {ok() or '走れる'}・"
          f"odom {'なし' if o is None else f'x={o[0]:.2f} y={o[1]:.2f} yaw={math.degrees(o[2]):.0f}°'}")
    sys.exit(0)
# MANUAL の PAUSE なら、WebUI でスティックに触れたときと同じくリースと 0 を送って RUN に入れる（最大 3 秒）
s0 = st['state']
if s0 is not None and s0.mode == 'MANUAL' and s0.state == 'PAUSE' and not (s0.estop_ui or s0.estop_hw):
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0 and ok() is not None:
        send(0.0)
        rclpy.spin_once(node, timeout_sec=0.1)
if ok() is not None:
    stop_and_exit(f'走らない: {ok()}', 2)

x0, y0, yaw0 = st['odom'][:3]
target = a.amount if a.kind == 'straight' else math.radians(abs(a.amount))
sign = 1.0 if a.ratio > 0 else -1.0
if a.kind == 'spin' and a.amount < 0:
    sign = -sign
peak = abs(a.ratio)
yaw_acc, yaw_prev = 0.0, yaw0
print(f'開始: {a.kind} 比率 {a.ratio} 目標 {a.amount} 停止 {a.stop}', flush=True)

start = time.monotonic()
phase, stop_from, stop_t0 = 'go', None, None
next_t = start
while True:
    rclpy.spin_once(node, timeout_sec=0.0)
    now = time.monotonic()
    if now < next_t:
        time.sleep(min(0.01, next_t - now))
        continue
    next_t += 1.0 / RATE
    bad = ok()
    if bad:
        stop_and_exit(f'中断: {bad}', 1)
    x, y, yaw = st['odom'][:3]
    d = yaw - yaw_prev
    yaw_acc += math.atan2(math.sin(d), math.cos(d))
    yaw_prev = yaw
    done = math.hypot(x - x0, y - y0) if a.kind == 'straight' else abs(yaw_acc)
    if done > (MAX_DIST if a.kind == 'straight' else math.radians(MAX_ANGLE)) or now - start > MAX_TIME:
        stop_and_exit(f'中断: 上限を超えた（{done:.2f}）', 1)

    el = now - start
    if phase == 'go':
        x = min(1.0, el / a.ramp) if a.ramp > 0 else 1.0
        if a.shape == 'scurve':
            x = (1 - math.cos(math.pi * x)) / 2   # 加速度が 0 から上がって 0 へ戻る
        r = peak * x
        # soft は減速分の距離を見込んで早めに減速へ入る（平均速度は半分）。
        # 指令から実速度が落ち始めるまでの遅れ（実測 0.3〜0.4 秒）も足す
        v_now = abs(st['odom'][3] if a.kind == 'straight' else st['odom'][4])
        lead = v_now * (a.stop_ramp / 2 + 0.4) if a.stop == 'soft' else 0.0
        if done >= target - lead:
            if a.stop == 'hard':
                stop_and_exit(f'完了: {done:.2f}（急停止）', 0)
            phase, stop_from, stop_t0 = 'stop', r, now
        else:
            send(sign * r)
            continue
    x = max(0.0, 1.0 - (now - stop_t0) / a.stop_ramp)
    if a.shape == 'scurve':
        x = (1 - math.cos(math.pi * x)) / 2
    r = stop_from * x
    if r <= 0.0:
        stop_and_exit(f'完了: {done:.2f}（ゆっくり停止）', 0)
    send(sign * r)
