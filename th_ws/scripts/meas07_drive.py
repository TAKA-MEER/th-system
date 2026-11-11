#!/usr/bin/env python3
"""制動測定のための直進走行（WP-MEAS-01 / N-12）— **通電作業**

/cmd_vel_manual (twist_mux priority 30) へ 20Hz で publish する。
**/cmd_vel へ直接 publish してはいけない**（CLAUDE.md の不変ルール）。

停止の 3 経路（どれを測るかは --mode で選ぶ）:

  soft-estop : 保持のあと **UI 非常停止の要求（/safety/estop_ui）を出す**。safety_monitor が
          /safety/estop を立て、obstacle_limiter は S 字ランプを通らず即 0 を出す
          （obstacle_limiter_core.cpp tier 1）。esp32_bridge も即ゼロを送り、ESP32 が
          ランプで減速する。**これが brake_accel_mps2（障害物・非常停止の経路の減速度）の
          測定対象**。要求を出した時刻は recorder が /safety/estop_ui の 0→1 として残す。
          押した状態を保つため 5Hz で True を送り続け（途切れると UI_DISCONNECTED）、
          停止後に解除（False）を送る。解除したあと画面が「再開／メニューへ」を聞くので、
          次の回の前に操作して MANUAL に入り直すこと（機体は ESTOP モードになるため）。
  ramp  : 保持のあと 0.0 を publish し続ける。**通常の停止**で、obstacle_limiter の
          S 字ランプ（normal_accel_mps2 0.4・normal_jerk_mps3 0.5）を通る。
          brake_accel_mps2 の測定にはならない（障害物・非常停止の停止は通常の加減速の
          対象外。Spec-safety §3）。通常停止の距離を見たいときだけ使う。
  coast : 保持に入ったら**人が物理非常停止ボタンを押す**。駆動電源が切れて空走する。
          ソフトからは空走を起こせない（ソフトの非常停止は /cmd_vel=0 を出すだけで、
          ESP32 はランプする）。N-12 の測定対象。

安全:
  - 周囲 3m に人と物を置かない。人が張り付いた状態で行う（wp0 §6.2）
  - --i-am-supervising を付けないと走らない
  - どの終わり方でもゼロを送ってから終了する（例外・Ctrl-C・タイムアウト）
  - coast で押し忘れても --hold 経過で必ずゼロへ落とす
  - soft-estop で出した UI 非常停止は、どの終わり方でも（例外・Ctrl-C を含む）ゼロを
    送ったあとに解除する。強制終了（SIGKILL）で解除できなかったときは、機体は
    非常停止のまま止まっている（安全側）。解除は画面か
    `ros2 service call /safety/clear_estop_ui std_srvs/srv/Trigger`
"""
import argparse
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

RATE_HZ = 20.0
SPEED_CAP = 1.2      # v_max = 1.12 m/s。これを超える指示は受け付けない
HOLD_CAP = 15.0      # 保持の上限（秒）
ESTOP_REPEAT_EVERY = 4   # UI 非常停止の要求は 20Hz の 4 周期に 1 回＝5Hz（registry の estop_ui_repeat_hz）
ACCEL = 0.4          # 立ち上げの傾き (m/s^2)。ESP32 側のランプより緩くする


class Driver(Node):
    def __init__(self, speed, hold, mode):
        super().__init__('meas07_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        # 押下の要求。safety_monitor・state_manager とも RELIABLE depth 10 で購読する
        self.pub_estop_ui = self.create_publisher(Bool, '/safety/estop_ui', 10)
        self.speed, self.hold, self.mode = speed, hold, mode
        self._estop_latched = False   # このスクリプトが UI 非常停止を出していて、まだ解除していない
        self._stop = False
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, *_):
        self._stop = True

    def _send(self, v):
        m = Twist()
        m.linear.x = float(v)
        self.pub.publish(m)

    def _send_estop_ui(self, on):
        m = Bool()
        m.data = bool(on)
        self.pub_estop_ui.publish(m)

    def _hold_for(self, seconds, v, label, estop_ui=False):
        """v を出し続ける。estop_ui なら UI 非常停止の要求も 5Hz で出し続ける。
        中断されたら False を返す。"""
        n = int(seconds * RATE_HZ)
        for i in range(n):
            if self._stop:
                return False
            if estop_ui and i % ESTOP_REPEAT_EVERY == 0:
                self._send_estop_ui(True)
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
            self.get_logger().info('=== ここで指令をゼロにします（通常停止・S 字ランプ）===')
        if self.mode == 'soft-estop':
            self.get_logger().warn('=== UI 非常停止の要求を出します（ソフト非常停止）===')
            self._estop_latched = True      # True を出す前に立てる（途中で例外でも解除する）
            self._hold_for(4.0, 0.0, '停止後（非常停止中）', estop_ui=True)
            return
        self._hold_for(4.0, 0.0, '停止後')

    def stop_hard(self):
        """終了時。ゼロを短時間送って確実に止める。"""
        for _ in range(int(RATE_HZ)):
            self._send(0.0)
            time.sleep(1.0 / RATE_HZ)
        if self._estop_latched:
            # ゼロを送ってから解除する。解除は押下の反対（False）。数回送って取りこぼさない
            for _ in range(3):
                self._send(0.0)
                self._send_estop_ui(False)
                time.sleep(0.2)
            self._estop_latched = False
            self.get_logger().warn(
                'UI 非常停止を解除しました。機体は ESTOP モードから戻る途中です。'
                '画面の指示（再開／メニューへ）に従い、次の回の前に MANUAL へ入り直してください。'
                '解除されていなければ: ros2 service call /safety/clear_estop_ui std_srvs/srv/Trigger')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.3, help='定常速度 m/s')
    ap.add_argument('--hold', type=float, default=4.0, help='定常を保つ秒数')
    ap.add_argument('--mode', choices=['soft-estop', 'ramp', 'coast'], required=True)
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
