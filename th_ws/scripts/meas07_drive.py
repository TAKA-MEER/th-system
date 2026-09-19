#!/usr/bin/env python3
"""制動測定のための直進走行（WP-MEAS-01 / N-12）— **通電作業**

/cmd_vel_manual (twist_mux priority 30) へ 20Hz で publish する。
**/cmd_vel へ直接 publish してはいけない**（CLAUDE.md の不変ルール）。

停止の 2 経路（どちらを測るかは --mode で選ぶ）:

  ramp  : 保持のあと 0.0 を publish し続ける。ESP32 が 1.5 m/s^2 でランプ減速する。
          これが「意図的に作った制動距離」で、brake_accel_mps2 の測定対象。
  coast : 保持に入ったら**人が物理非常停止ボタンを押す**。駆動電源が切れて空走する。
          ソフトからは空走を起こせない（/safety/estop も twist_mux のタイムアウトも
          /cmd_vel=0 を出すだけで、ESP32 はランプする）。N-12 の測定対象。

安全:
  - 周囲 3m に人と物を置かない。人が張り付いた状態で行う（wp0 §6.2）
  - --i-am-supervising を付けないと走らない
  - どの終わり方でもゼロを送ってから終了する（例外・Ctrl-C・タイムアウト）
  - coast で押し忘れても --hold 経過で必ずゼロへ落とす
"""
import argparse
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

RATE_HZ = 20.0
SPEED_CAP = 1.2      # v_max = 1.12 m/s。これを超える指示は受け付けない
HOLD_CAP = 15.0      # 保持の上限（秒）
ACCEL = 0.4          # 立ち上げの傾き (m/s^2)。ESP32 側のランプより緩くする


class Driver(Node):
    def __init__(self, speed, hold, mode):
        super().__init__('meas07_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.speed, self.hold, self.mode = speed, hold, mode
        self._stop = False
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, *_):
        self._stop = True

    def _send(self, v):
        m = Twist()
        m.linear.x = float(v)
        self.pub.publish(m)

    def _hold_for(self, seconds, v, label):
        """v を出し続ける。中断されたら False を返す。"""
        n = int(seconds * RATE_HZ)
        for i in range(n):
            if self._stop:
                return False
            self._send(v)
            if i % int(RATE_HZ) == 0:
                self.get_logger().info(f'{label} {i / RATE_HZ:.0f}/{seconds:.0f}s  v={v:.2f}')
            rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)
        return True

    def run(self):
        for t in (3, 2, 1):
            self.get_logger().info(f'{t} 秒後に発進します。周囲を確認してください')
            time.sleep(1.0)
            if self._stop:
                return

        # 立ち上げ（ESP32 のランプより緩く上げ、定常に入ってから測る）
        steps = int(self.speed / ACCEL * RATE_HZ)
        for i in range(max(steps, 1)):
            if self._stop:
                return
            self._send(self.speed * (i + 1) / max(steps, 1))
            rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)

        if self.mode == 'coast':
            self.get_logger().warn('=== いま物理非常停止ボタンを押してください ===')
            self.get_logger().warn(f'押さなくても {self.hold:.0f} 秒で自動的にゼロへ落とします')
        if not self._hold_for(self.hold, self.speed, '定常'):
            return

        if self.mode == 'ramp':
            self.get_logger().info('=== ここで指令をゼロにします（ランプ停止）===')
        self._hold_for(4.0, 0.0, '停止後')

    def stop_hard(self):
        """終了時。ゼロを短時間送って確実に止める。"""
        for _ in range(int(RATE_HZ)):
            self._send(0.0)
            time.sleep(1.0 / RATE_HZ)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.3, help='定常速度 m/s')
    ap.add_argument('--hold', type=float, default=4.0, help='定常を保つ秒数')
    ap.add_argument('--mode', choices=['ramp', 'coast'], required=True)
    ap.add_argument('--i-am-supervising', action='store_true',
                    help='人が張り付いており、周囲 3m に人と物が無いことの確認')
    a = ap.parse_args()

    if not a.i_am_supervising:
        sys.exit('中止: --i-am-supervising が要ります（通電走行なので無人で回さないこと）')
    if not 0.0 < a.speed <= SPEED_CAP:
        sys.exit(f'中止: --speed は 0 より大きく {SPEED_CAP} 以下にしてください')
    if not 0.0 < a.hold <= HOLD_CAP:
        sys.exit(f'中止: --hold は 0 より大きく {HOLD_CAP} 以下にしてください')

    rclpy.init()
    node = Driver(a.speed, a.hold, a.mode)
    try:
        node.run()
    finally:
        node.stop_hard()
        node.destroy_node()
        rclpy.shutdown()
    print('終了。ゼロ指令を送出済み')


if __name__ == '__main__':
    main()
