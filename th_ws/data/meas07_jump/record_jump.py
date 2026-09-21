#!/usr/bin/env python3
"""B′（jump）の閾値を実走のデータで決めるための受動的な記録。

localization_health と同じ計算（0.5 秒ごとに map→odom を引き、前回との
並進・回転の差を出す）を**そのまま**行い、値を CSV に残す。
購読と TF の読み取りだけで、何も publish しない（速度指令は出さない）。
"""
import csv
import json
import math
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
OUTDIR = sys.argv[2] if len(sys.argv) > 2 else '/root/th_data/jumpcheck/out'
PERIOD_S = 0.5            # localization_health の jump_window_ms と同じ
CUR_TRANS = 0.11          # 現行の閾値（超過回数の集計用）
CUR_ROT = 0.0175


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pct(vs, q):
    if not vs:
        return None
    vs = sorted(vs)
    return vs[min(len(vs) - 1, max(0, int(math.ceil(q * len(vs))) - 1))]


class Rec(Node):
    def __init__(self):
        super().__init__('hwcheck_jump')
        self.t0 = time.monotonic()
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.cmd = (0.0, 0.0)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos_profile_sensor_data)
        self.prev = None
        self.rows = []
        import os
        os.makedirs(OUTDIR, exist_ok=True)
        self.f = open(f'{OUTDIR}/jump.csv', 'w', newline='', encoding='utf-8')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_s', 'x', 'y', 'yaw', 'dtrans_m', 'drot_rad', 'cmd_lin', 'cmd_ang',
                         'stamp_age_s', 'moving'])
        self.create_timer(PERIOD_S, self.tick)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def tick(self):
        t = time.monotonic() - self.t0
        try:
            tf = self.buf.lookup_transform('map', 'odom', rclpy.time.Time())
        except Exception:
            self.w.writerow([round(t, 2)] + [''] * 5 + [self.cmd[0], self.cmd[1], '', ''])
            self.f.flush()
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_of(tf.transform.rotation)
        st = tf.header.stamp
        age = self.get_clock().now().nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9)
        dtr = drot = None
        if self.prev is not None:
            dtr = math.hypot(x - self.prev[0], y - self.prev[1])
            drot = abs(wrap(yaw - self.prev[2]))
        self.prev = (x, y, yaw)
        moving = abs(self.cmd[0]) > 0.01 or abs(self.cmd[1]) > 0.01
        if dtr is not None:
            self.rows.append((t, dtr, drot, moving, self.cmd[0], self.cmd[1]))
        self.w.writerow([round(t, 2), round(x, 4), round(y, 4), round(yaw, 4),
                         '' if dtr is None else round(dtr, 4),
                         '' if drot is None else round(drot, 4),
                         round(self.cmd[0], 3), round(self.cmd[1], 3), round(age, 3), int(moving)])
        self.f.flush()

    def summary(self):
        def block(rs):
            tr = [r[1] for r in rs]
            ro = [r[2] for r in rs]
            return {
                'n': len(rs),
                'dtrans_m': {'p50': pct(tr, .5), 'p95': pct(tr, .95), 'p99': pct(tr, .99),
                             'max': max(tr) if tr else None},
                'drot_rad': {'p50': pct(ro, .5), 'p95': pct(ro, .95), 'p99': pct(ro, .99),
                             'max': max(ro) if ro else None},
                'over_current_trans': sum(1 for v in tr if v > CUR_TRANS),
                'over_current_rot': sum(1 for v in ro if v > CUR_ROT),
                'over_current_either': sum(1 for a, b in zip(tr, ro) if a > CUR_TRANS or b > CUR_ROT),
            }
        turning = [r for r in self.rows if r[3] and abs(r[5]) > 0.05]
        straight = [r for r in self.rows if r[3] and abs(r[5]) <= 0.05]
        return {
            'elapsed_s': round(time.monotonic() - self.t0, 1),
            'all': block(self.rows),
            'stationary': block([r for r in self.rows if not r[3]]),
            'moving': block([r for r in self.rows if r[3]]),
            'moving_turning(|w|>0.05)': block(turning),
            'moving_straight': block(straight),
            'top10_drot_rad': sorted([round(r[2], 4) for r in self.rows], reverse=True)[:10],
            'top10_dtrans_m': sorted([round(r[1], 4) for r in self.rows], reverse=True)[:10],
        }


def main():
    rclpy.init()
    n = Rec()
    try:
        last = time.monotonic()
        while time.monotonic() - n.t0 < DURATION_S:
            rclpy.spin_once(n, timeout_sec=0.25)
            if time.monotonic() - last > 30:
                with open(f'{OUTDIR}/summary.json', 'w', encoding='utf-8') as f:
                    json.dump(n.summary(), f, ensure_ascii=False, indent=2)
                last = time.monotonic()
    except KeyboardInterrupt:
        pass
    with open(f'{OUTDIR}/summary.json', 'w', encoding='utf-8') as f:
        json.dump(n.summary(), f, ensure_ascii=False, indent=2)
    n.f.close()
    open(f'{OUTDIR}/done', 'w').write('done\n')


if __name__ == '__main__':
    main()
