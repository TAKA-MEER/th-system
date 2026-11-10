#!/usr/bin/env python3
"""モーターの不感帯の測定（Spec-params.md「モーターの不感帯」・Spec-checks.md 校正 2 旋回）— **通電作業**

その場旋回の角速度を小さい方から段階的に上げ、各段で車輪が指令どおりに回るかを記録する。
ファームは PID（積分あり）なので、遅い指令でも時間をかければ回り出すことがある。
「保持時間のうちに指令の 8 割に届いたか」と「回り出すまでの秒数」の両方を見る。

/cmd_vel_manual（twist_mux priority 30）へ 20Hz で publish する（meas07_drive.py と同じ経路）。
**/cmd_vel へ直接 publish してはいけない**（CLAUDE.md の不変ルール）。

前提:
  - bringup が動いていて、WebUI の画面をどこかで開いている（開いていないと速度上限が stop）
  - 画面は場外の手動走行（S-11）。スティックに一度触れて RUN にしておく
  - 周囲 0.6 m に人・物を置かない（その場で回る）。物理非常停止に手を添える
  - --i-am-supervising を付けないと走らない
使い方（コンテナ内）:
  python3 /root/th_ws/scripts/meas09_deadband.py --i-am-supervising            # 左右両回り
  python3 /root/th_ws/scripts/meas09_deadband.py --i-am-supervising --dir +1   # 左回りだけ
結果: /root/th_data/deadband_<時刻>.csv（生データ）と、標準出力の表
"""
import argparse
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from th_system_msgs.msg import WheelFeedback

RATE_HZ = 20.0
W_CAP = 0.80            # これを超える角速度は受け付けない（校正の旋回 0.77 rad/s まで）
MOVE_MPS = 0.005        # 車輪がこれを超えたら「回り出した」とみなす
REACH_RATIO = 0.8       # 保持の後半の平均が指令の 8 割以上なら「指令どおり」


class Meas(Node):
    def __init__(self, out_path, wheel_base):
        super().__init__('meas09_deadband')
        self.pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.create_subscription(WheelFeedback, '/esp32/wheel_feedback', self._on_fb, 10)
        # obstacle_limiter が実際に出した指令（削られていないかの確認用）
        self.create_subscription(Twist, '/cmd_vel', self._on_out, 10)
        self.out = []         # (t, angular.z)
        self.wheel_base = wheel_base
        self.fb = []          # (t, left, right)
        self.stop = False
        self.f = open(out_path, 'w', encoding='utf-8')
        self.f.write('t,kind,cmd_w,left,right\n')
        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

    def _sig(self, *_):
        self.stop = True

    def _on_fb(self, m):
        t = time.time()
        self.fb.append((t, m.left_speed, m.right_speed))
        self.f.write(f'{t:.3f},fb,,{m.left_speed:.4f},{m.right_speed:.4f}\n')

    def _on_out(self, m):
        t = time.time()
        self.out.append((t, m.angular.z))
        self.f.write(f'{t:.3f},out,{m.angular.z:.4f},,\n')

    def send(self, w):
        tw = Twist()
        tw.angular.z = float(w)
        self.pub.publish(tw)
        self.f.write(f'{time.time():.3f},cmd,{w:.4f},,\n')

    def _run(self, w, seconds, stoppable):
        # 時刻で測る（spin_once はメッセージが届くと待たずに戻るので、回数で数えると短くなる）
        period = 1.0 / RATE_HZ
        t_end = time.time() + seconds
        next_send = time.time()
        while time.time() < t_end:
            if stoppable and self.stop:
                return False
            if time.time() >= next_send:
                self.send(w)
                next_send += period
            rclpy.spin_once(self, timeout_sec=max(0.0, min(period, next_send - time.time())))
        return True

    def hold(self, w, seconds):
        return self._run(w, seconds, True)

    def zero(self, seconds=0.5):
        self._run(0.0, seconds, False)


def analyse(fb, t0, t1, w, wheel_base):
    """段 [t0, t1) の車輪の記録から、回り出しの秒数と後半の平均を出す。"""
    v_target = abs(w) * wheel_base / 2.0          # 片輪の目標速さ
    seg = [(t, l, r) for t, l, r in fb if t0 <= t < t1]
    if not seg:
        return None
    start = None
    move = min(MOVE_MPS, 0.5 * v_target)   # 遅い段では目標の半分で「回り出した」とみなす
    for t, l, r in seg:
        if abs(l) > move and abs(r) > move:
            start = t - t0
            break
    half = [(l, r) for t, l, r in seg if t >= (t0 + t1) / 2.0]
    mean_l = sum(abs(l) for l, _ in half) / len(half) if half else 0.0
    mean_r = sum(abs(r) for _, r in half) / len(half) if half else 0.0
    # 向きの確認（左回り w>0 なら 左 < 0 < 右）
    sgn_ok = all((r - l) * w >= 0 for l, r in half if abs(r - l) > 2 * MOVE_MPS) if half else True
    return dict(v_target=v_target, start=start, mean_l=mean_l, mean_r=mean_r,
                ratio_l=mean_l / v_target if v_target else 0.0,
                ratio_r=mean_r / v_target if v_target else 0.0, sgn_ok=sgn_ok)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--i-am-supervising', action='store_true')
    ap.add_argument('--dir', type=int, choices=[1, -1, 0], default=0,
                    help='+1=左回り, -1=右回り, 0=両方（既定）')
    ap.add_argument('--steps', default='0.02,0.04,0.06,0.08,0.10,0.12,0.15,0.20,0.25,0.30',
                    help='角速度の段 [rad/s]（カンマ区切り）')
    ap.add_argument('--hold', type=float, default=4.0, help='各段の保持秒数')
    ap.add_argument('--rest', type=float, default=1.5, help='段の間の停止秒数')
    ap.add_argument('--wheel-base', type=float, default=0.39)
    a = ap.parse_args()
    if not a.i_am_supervising:
        sys.exit('--i-am-supervising を付けて、物理非常停止に手を添えてから実行すること')
    steps = [float(x) for x in a.steps.split(',')]
    if any(s <= 0 or s > W_CAP for s in steps):
        sys.exit(f'段は 0 より大きく {W_CAP} rad/s 以下にすること')
    dirs = [1, -1] if a.dir == 0 else [a.dir]

    rclpy.init()
    out = f"/root/th_data/deadband_{time.strftime('%H%M%S')}.csv"
    n = Meas(out, a.wheel_base)
    rows = []
    try:
        n.zero(1.0)
        if not n.fb:
            print('警告: /esp32/wheel_feedback がまだ届いていない', file=sys.stderr)
        for d in dirs:
            for s in steps:
                w = d * s
                print(f'w={w:+.2f} rad/s を {a.hold:.0f} 秒', flush=True)
                t0 = time.time()
                if not n.hold(w, a.hold):
                    raise KeyboardInterrupt
                t1 = time.time()
                n.zero(a.rest)
                r = analyse(n.fb, t0, t1, w, a.wheel_base)
                outs = [z for t, z in n.out if (t0 + t1) / 2.0 <= t < t1]
                if r is not None:
                    r['out_w'] = sum(outs) / len(outs) if outs else float('nan')
                rows.append((w, r))
    except KeyboardInterrupt:
        print('中断した', file=sys.stderr)
    finally:
        n.zero(1.0)
        n.f.close()

    print(f'\n記録: {out}')
    print(' 指令w  /cmd_vel  片輪目標  回り出し秒  左の平均  右の平均  左/目標 右/目標  判定')
    for w, r in rows:
        if r is None:
            print(f'{w:+.2f}  （車輪の記録なし）')
            continue
        ok = r['ratio_l'] >= REACH_RATIO and r['ratio_r'] >= REACH_RATIO and r['sgn_ok']
        st = f"{r['start']:.2f}" if r['start'] is not None else '回らず'
        print(f"{w:+.2f}  {r.get('out_w', float('nan')):+.3f}   {r['v_target']:.4f}   {st:>8}   {r['mean_l']:.4f}   {r['mean_r']:.4f}"
              f"   {r['ratio_l']:.2f}   {r['ratio_r']:.2f}   {'OK' if ok else '不足'}"
              f"{'' if r['sgn_ok'] else '（向きが逆）'}")
    print('\n不感帯の目安: 「OK」になった最小の |w|（左右それぞれ）。回り出しが遅い段は PID の積分で動いている')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
