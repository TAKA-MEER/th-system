#!/usr/bin/env python3
"""DRIVE_RUNAWAY の実走記録（WP-MEAS-08 / 台帳 W-06 ③④⑤⑥ の材料）

走行中の「指令」と「実速度」を、safety_monitor の DRIVE_RUNAWAY と**同じ式・同じ入力・
同じ周期（100 ms）**で評価し直して CSV に残す。購読と標準入力の読み取りだけで、
何も publish しない（速度指令は出さない。safety_monitor も変えない）。

再現している式（safety_monitor.cpp 359〜385 行・safety_monitor_core.cpp）:
  cmd_abs      = |/cmd_vel.linear.x|                      （角速度は見ない）
  feedback_abs = |(left_speed + right_speed) / 2|         （/esp32/wheel_feedback）
  condition    = cmd_abs <= zero_thr ? feedback_abs > zero_thr
                                     : feedback_abs < cmd_abs/ratio or > cmd_abs*ratio
  fresh        = 実測が届いてから stale 以内（届いていなければ False）
  held         = fresh のときだけ condition で進め、False で 0、古いあいだは凍結
  would_fire   = held >= hold     ← 今の設定なら DRIVE_RUNAWAY が出ていたか

使い方（コンテナ内。実機の bringup と同じ ROS_DOMAIN_ID で）:
  python3 /root/th_ws/scripts/meas08_runaway_recorder.py [秒数(既定1800)] [出力ディレクトリ]
  出力: <dir>/runaway_<日時>.csv（1 行ごとに書き出す。途中で止めても残る）
  記録中に「行を入力して Enter」すると、その時刻に label 欄へ印が付く
  （例: 直進0.3 / 停止 / 左旋回 / 巻き尺始点 / 巻き尺終点）。解析は meas08_analyze.py。

CLAUDE.md「実機の計測は購読だけのスクリプトで行い、計測中に別の docker exec で ROS
コマンドを叩かない」に従う。ここから先は標準入力の印以外に何も操作しないこと。
"""
import math
import os
import select
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from th_system_msgs.msg import WheelFeedback

DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 1800.0
OUTDIR = sys.argv[2] if len(sys.argv) > 2 else '/root/th_data/runaway'

# 現行の設定（registry.yaml。記録時点の値を CSV の先頭コメントにも残す）
RATIO = 1.5           # runaway_ratio
ZERO_THR = 0.02       # runaway_zero_threshold [m/s]
HOLD_S = 0.5          # runaway_hold_ms
STALE_S = 0.25        # runaway_feedback_stale_ms
PERIOD_S = 0.1        # safety_monitor の check_period_ms


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Rec(Node):
    def __init__(self):
        super().__init__('meas08_runaway_recorder')
        self.t0 = time.monotonic()
        self.cmd = (0.0, 0.0)
        self.cmd_t = None
        self.fb = (0.0, 0.0)
        self.fb_t = None
        self.fb_arrivals = []         # 直近の周期に届いた到着時刻（ギャップ計算用）
        self.last_fb_arrival = None
        self.odom = (None, None, None)
        self.held = 0.0
        self.label = ''
        # 送り側の QoS が RELIABLE でも BEST_EFFORT の購読は繋がる（受信側を軽く保つ）
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos_profile_sensor_data)
        self.create_subscription(WheelFeedback, '/esp32/wheel_feedback', self.on_fb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        os.makedirs(OUTDIR, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.path = f'{OUTDIR}/runaway_{stamp}.csv'
        self.f = open(self.path, 'w', buffering=1, encoding='utf-8')
        self.f.write(f'# meas08 ratio={RATIO} zero_thr={ZERO_THR} hold_s={HOLD_S} '
                     f'stale_s={STALE_S} period_s={PERIOD_S}\n')
        self.f.write('t_s,wall_ns,cmd_lin,cmd_ang,cmd_age_s,fb_left,fb_right,fb_mean,fb_age_s,'
                     'fb_gap_max_s,fresh,condition,held_s,would_fire,odom_x,odom_y,odom_yaw,label\n')
        self.create_timer(PERIOD_S, self.tick)
        self.get_logger().info(f'記録開始: {self.path}（{DURATION_S:.0f} 秒。印は標準入力へ）')

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.cmd_t = time.monotonic()

    def on_fb(self, m):
        now = time.monotonic()
        self.fb = (m.left_speed, m.right_speed)
        self.fb_t = now
        if self.last_fb_arrival is not None:
            self.fb_arrivals.append(now - self.last_fb_arrival)
        self.last_fb_arrival = now

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _read_label(self):
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                # 環境によっては stdin が UTF-8 以外（C ロケール等）で decode され、
                # 日本語などのマルチバイト文字がそのまま復元できない代替サロゲート
                # （lone surrogate）になることがある（2026-09-22 実機で発生。
                # `後退0.30` の直後、印の1文字が化けて UnicodeEncodeError で記録が
                # 全体停止した）。元のバイト列へ一旦戻し、UTF-8 として読み直す。
                # それでも直せない場合は安全な文字に置き換える（記録を止めない）。
                try:
                    line = line.encode('utf-8', 'surrogateescape').decode('utf-8')
                except UnicodeError:
                    line = line.encode('utf-8', 'surrogateescape').decode('utf-8', 'replace')
                line = line.strip()
                if line:
                    self.label = line
        except Exception:
            pass

    def tick(self):
        now = time.monotonic()
        self._read_label()
        fb_mean = (self.fb[0] + self.fb[1]) / 2.0
        fb_abs = abs(fb_mean)
        cmd_abs = abs(self.cmd[0])
        if cmd_abs <= ZERO_THR:
            cond = fb_abs > ZERO_THR
        else:
            cond = fb_abs < cmd_abs / RATIO or fb_abs > cmd_abs * RATIO
        fb_age = (now - self.fb_t) if self.fb_t is not None else float('nan')
        fresh = self.fb_t is not None and fb_age <= STALE_S
        if fresh:
            self.held = self.held + PERIOD_S if cond else 0.0
        # 古いあいだは凍結（Spec-safety.md §3.5.3）: held を動かさない
        would_fire = self.held >= HOLD_S
        gap_max = max(self.fb_arrivals) if self.fb_arrivals else float('nan')
        self.fb_arrivals = []
        cmd_age = (now - self.cmd_t) if self.cmd_t is not None else float('nan')
        ox, oy, oyaw = self.odom
        fmt = lambda v, n=4: '' if v is None or (isinstance(v, float) and math.isnan(v)) else round(v, n)
        row = ','.join(str(x) for x in [
            round(now - self.t0, 2), time.time_ns(),
            round(self.cmd[0], 4), round(self.cmd[1], 4), fmt(cmd_age, 3),
            round(self.fb[0], 4), round(self.fb[1], 4), round(fb_mean, 4), fmt(fb_age, 3),
            fmt(gap_max, 3), int(fresh), int(cond), round(self.held, 2), int(would_fire),
            fmt(ox), fmt(oy), fmt(oyaw), self.label.replace(',', ' ')]) + '\n'
        try:
            self.f.write(row)
        except UnicodeEncodeError:
            # 最後の砦: 印にどうしても書けない文字が残っていても、計測の行自体は
            # 失わない（実機の計測は録り直しがきかないため。記録を止めるより
            # 印を欠けさせるほうを選ぶ）。
            self.f.write(row.encode('utf-8', 'replace').decode('utf-8'))
        self.label = ''


def main():
    rclpy.init()
    n = Rec()
    try:
        while time.monotonic() - n.t0 < DURATION_S:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    n.f.close()
    print(f'記録終了: {n.path}')


if __name__ == '__main__':
    main()
