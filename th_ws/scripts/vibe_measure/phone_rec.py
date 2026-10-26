#!/usr/bin/env python3
"""スマホ(Conduit)の Imu を CSV に書く。zenoh の ros2 bag record は Ctrl-C で閉じきらず db が壊れるため、
1 行ずつ書いて flush するだけにしている。止め方に関係なく書けた分は残る。

  phone_rec.py OUT_DIR   … sensor_msgs/Imu 型のトピックを見つけ次第すべて記録（トピックごとに 1 ファイル）
  phone_rec.py --probe N … 記録せず、N 秒受けて周期・最大間隔・加速度の大きさだけ出す
"""
import os
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

PROBE = float(sys.argv[2]) if sys.argv[1] == '--probe' else None
out = '/tmp/vibe_probe' if PROBE else sys.argv[1]
os.makedirs(out, exist_ok=True)
rclpy.init()
node = rclpy.create_node('vibe_phone_recorder')
files, subs, counts = {}, {}, {}


def cb_for(topic):
    fp = open(os.path.join(out, topic.strip('/').replace('/', '__') + '.csv'), 'w', buffering=1)
    fp.write(f'# topic {topic}\nrecv_ns,stamp_ns,frame_id,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n')
    files[topic] = fp
    counts[topic] = 0

    def cb(m):
        a, g, q = m.linear_acceleration, m.angular_velocity, m.orientation
        fp.write(f'{time.time_ns()},{m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec},{m.header.frame_id},'
                 f'{a.x!r},{a.y!r},{a.z!r},{g.x!r},{g.y!r},{g.z!r},{q.x!r},{q.y!r},{q.z!r},{q.w!r}\n')
        counts[topic] += 1
    return cb


def scan():
    for name, types in node.get_topic_names_and_types():
        if 'sensor_msgs/msg/Imu' in types and name not in subs:
            subs[name] = node.create_subscription(Imu, name, cb_for(name), qos_profile_sensor_data)
            print(f"{'受信開始' if PROBE else '記録開始'}: {name}", flush=True)


def report():
    print('  ' + '  '.join(f'{k}: {v} 件' for k, v in counts.items()) if counts else '  スマホの Imu はまだ見えない', flush=True)


def probe_report():
    import numpy as np
    if not files:
        print('  スマホの Imu が見えない（アプリの接続先・Domain ID・ディストリビューション=Humble を確認）', flush=True)
    for topic, fp in files.items():
        fp.flush()
        d = np.genfromtxt(fp.name, delimiter=',', skip_header=2, usecols=(0, 1, 3, 4, 5, 6, 7, 8))
        if d.ndim < 2 or len(d) < 3:
            print(f'  {topic}: {0 if d.ndim < 2 else len(d)} 件しか届かない', flush=True)
            continue
        d = d[-int(PROBE * 150):]
        r = np.diff(d[:, 0]) / 1e9
        h = np.diff(d[:, 1]) / 1e9
        a = np.linalg.norm(d[:, 2:5], axis=1)
        print(f'  {topic}: 受信 {1 / np.median(r):.0f} Hz・受信の最大間隔 {r.max() * 1000:.0f} ms'
              f'・スマホ時刻 {"あり" if d[0, 1] > 0 else "なし(0)"}'
              + (f'・スマホ時刻の最大間隔 {h.max() * 1000:.0f} ms' if d[0, 1] > 0 else '')
              + f'・|加速度| {a.mean():.2f}（重力込みなら 9.8 か 1.0）・角速度 {np.abs(d[:, 5:8]).max():.3f} rad/s', flush=True)


node.create_timer(1.0, scan)
if PROBE:
    node.create_timer(PROBE, probe_report)
else:
    node.create_timer(5.0, report)
try:
    rclpy.spin(node)
except BaseException:
    pass
finally:
    for fp in files.values():
        fp.flush()
        fp.close()
    os._exit(0)   # rmw_zenoh の後始末で止まることがあるので待たない
