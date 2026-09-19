#!/usr/bin/env python3
"""
crawler_teleop.py — クローラー専用キーボードテレオペ
======================================================
差動駆動クローラー (CuGo v3i) 向け。超信地旋回・緩旋回に最適化。
10 Hz 固定送信で twist_mux タイムアウト (0.5 s) を防ぐ。

キー配列:
  q   w   e    q/e : 超信地旋回 左/右  (v=0, w=±spin_speed)
  a   s   d    a/d : 緩旋回前進 左/右  (v=linear_speed, w=±arc_speed)
      x         x / Space : 停止
  z       c    z/c : 緩旋回後退 左/右  (v=-linear_speed, w=±arc_speed)

速度変更 (押すたびに変化):
  +/- : 並進速度 ±0.05 m/s
  [/] : 旋回速度 ±0.1 rad/s  (超信地旋回と緩旋回の両方)
"""

import os
import sys
import tty
import termios
import select
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

_MSG = """\
============================================================
  クローラー テレオペ (CuGo v3i)          Ctrl+C で終了
------------------------------------------------------------
 操作キー  動作
   w      直進前進
   s      直進後退
   q / e  超信地旋回 左 / 右  (v=0, その場旋回)
   a / d  緩旋回前進 左 / 右  (v>0, 円弧走行)
   z / c  緩旋回後退 左 / 右  (v<0, 円弧走行)
   x / Space  停止

 速度変更
   + / -  並進速度を ±0.05 m/s
   [ / ]  旋回速度を ±0.1 rad/s
------------------------------------------------------------
 キーを押すと速度が保持されます。'x' または Space で停止。
============================================================
"""

# (linear_scale, angular_scale)
# 実際の速度 = (scale * self._lin, scale * speed) で計算
#
# 緩旋回の angular_scale を 0.5 に設定することで
#   旋回速度 = spin_speed × 0.5 → 超信地旋回の半分のレートで弧を描く
#   旋回半径 = linear_speed / arc_angular_speed (= spin_speed × 0.5)
_KEY_BINDINGS: dict[str, tuple[float, float]] = {
    'w': ( 1.0,  0.0),   # 直進前進
    's': (-1.0,  0.0),   # 直進後退
    'q': ( 0.0,  1.0),   # 超信地旋回 左 (spin-in-place)
    'e': ( 0.0, -1.0),   # 超信地旋回 右
    'a': ( 1.0,  0.5),   # 緩旋回前進 左  (arc forward-left)
    'd': ( 1.0, -0.5),   # 緩旋回前進 右
    'z': (-1.0,  0.5),   # 緩旋回後退 左  (arc backward-left)
    'c': (-1.0, -0.5),   # 緩旋回後退 右
    'x': ( 0.0,  0.0),   # 停止
    ' ': ( 0.0,  0.0),   # 停止
}


class CrawlerTeleop(Node):
    def __init__(self):
        super().__init__('crawler_teleop')

        self.declare_parameter('linear_speed',    0.20)   # m/s
        self.declare_parameter('spin_speed',      0.80)   # rad/s (超信地旋回)
        self.declare_parameter('publish_topic',   '/cmd_vel_nav')
        self.declare_parameter('publish_rate_hz', 10.0)
        # キー入力の急変が瞬時にそのまま速度指令へ反映されると機体が激しく
        # 揺れるため、目標値へレート制限をかけながら追従させる。停止側は
        # 応答優先で別枠の上限を使う (mapless_follow_core と同じ考え方)。
        self.declare_parameter('linear_accel_mps2',  0.6)
        self.declare_parameter('linear_decel_mps2',  1.2)
        self.declare_parameter('angular_accel_rad_s2', 3.0)

        self._lin  = self.get_parameter('linear_speed').value
        self._spin = self.get_parameter('spin_speed').value
        topic      = self.get_parameter('publish_topic').value
        rate_hz    = self.get_parameter('publish_rate_hz').value
        self._lin_accel   = self.get_parameter('linear_accel_mps2').value
        self._lin_decel   = self.get_parameter('linear_decel_mps2').value
        self._ang_accel   = self.get_parameter('angular_accel_rad_s2').value
        self._rate_hz     = rate_hz

        self._pub  = self.create_publisher(Twist, topic, 10)
        self._vx   = 0.0   # 目標速度 (キー入力で即座に変わる)
        self._wz   = 0.0
        self._cur_vx = 0.0  # 実際に publish する速度 (レート制限で滑らかに追従)
        self._cur_wz = 0.0
        self._lock = threading.Lock()

        self.create_timer(1.0 / rate_hz, self._cb_publish)
        self.get_logger().info(
            f'crawler_teleop 起動 topic={topic} '
            f'linear={self._lin:.2f} spin={self._spin:.2f}')

    # ── 速度設定 ──────────────────────────────────────────────
    def set_vel(self, lin_scale: float, ang_scale: float):
        with self._lock:
            self._vx = lin_scale * self._lin
            self._wz = ang_scale * self._spin

    def bump_linear(self, delta: float):
        self._lin = round(max(0.05, min(1.0, self._lin + delta)), 2)

    def bump_spin(self, delta: float):
        self._spin = round(max(0.1, min(3.0, self._spin + delta)), 1)

    def status_line(self) -> str:
        with self._lock:
            vx, wz = self._vx, self._wz
        arc = round(self._spin * 0.5, 1)
        return (f'並進 {self._lin:.2f} m/s  超信地旋回 {self._spin:.1f} rad/s  '
                f'緩旋回 {arc:.1f} rad/s  |  vx={vx:+.2f}  wz={wz:+.2f}    ')

    # ── タイマーコールバック ───────────────────────────────────
    def _cb_publish(self):
        dt = 1.0 / self._rate_hz
        with self._lock:
            target_vx, target_wz = self._vx, self._wz
        self._cur_vx = _ramp(self._cur_vx, target_vx,
                              self._lin_accel * dt, self._lin_decel * dt)
        self._cur_wz = _ramp(self._cur_wz, target_wz, self._ang_accel * dt)

        msg = Twist()
        msg.linear.x  = self._cur_vx
        msg.angular.z = self._cur_wz
        self._pub.publish(msg)


def _ramp(current: float, target: float, max_up: float, max_down: float = None) -> float:
    """current を target へ 1 周期分だけ近づける (急加減速防止)。
    max_down を指定すると減少側に別の上限を使える(停止応答を優先するため)。"""
    if max_down is None:
        max_down = max_up
    if target > current + max_up:
        return current + max_up
    if target < current - max_down:
        return current - max_down
    return target


def _getkey(fd: int, timeout: float = 0.05) -> str:
    rlist, _, _ = select.select([fd], [], [], timeout)
    if rlist:
        return os.read(fd, 1).decode('utf-8', errors='ignore')
    return ''


def main(args=None):
    rclpy.init(args=args)
    node = CrawlerTeleop()
    print(_MSG)

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        while rclpy.ok():
            key = _getkey(fd)

            if not key:
                print(f'\r{node.status_line()}', end='', flush=True)
                continue

            if key == '\x03':       # Ctrl+C
                break
            elif key in _KEY_BINDINGS:
                ls, as_ = _KEY_BINDINGS[key]
                node.set_vel(ls, as_)
            elif key == '+':
                node.bump_linear(+0.05)
            elif key == '-':
                node.bump_linear(-0.05)
            elif key == ']':
                node.bump_spin(+0.1)
            elif key == '[':
                node.bump_spin(-0.1)

            print(f'\r{node.status_line()}', end='', flush=True)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.set_vel(0.0, 0.0)
        time.sleep(0.15)   # 停止コマンドが届くまで待機
        rclpy.shutdown()
        print('\nテレオペ終了')


if __name__ == '__main__':
    main()
