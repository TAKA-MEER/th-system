#!/usr/bin/env python3
# ============================================================
# linear_calib — 直進キャリブレーション
#
# 使い方:
#   ros2 run th_calibration linear_calib.py --ros-args \
#     -p distance:=2.0 -p speed:=0.15
#
# 手順:
#   1. ロボットを平坦な床に置き、前方に 2m のマーキングをする
#   2. 本ツールを実行する
#   3. 停止後、実際に進んだ距離を計測する
#   4. プロンプトに入力 → 補正係数を出力
#   5. apply_calib で反映
# ============================================================
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys


class LinearCalib(Node):
    def __init__(self):
        super().__init__('linear_calib')

        self.declare_parameter('distance', 2.0)    # m
        self.declare_parameter('speed',    0.15)   # m/s (低速で正確に)

        self._target_dist = self.get_parameter('distance').value
        self._speed       = self.get_parameter('speed').value

        self._odom_dist   = 0.0
        self._start_x     = None
        self._start_y     = None
        self._running     = True
        self._done        = False

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._sub = self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)

        self._timer = self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'直進キャリブレーション開始: 目標 {self._target_dist:.1f} m')

    def _cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._start_x is None:
            self._start_x = x
            self._start_y = y
        self._odom_dist = math.hypot(x - self._start_x, y - self._start_y)

    def _loop(self):
        if self._done:
            return

        if self._start_x is None:
            return  # odom 待ち

        if self._odom_dist < self._target_dist:
            cmd = Twist()
            cmd.linear.x = self._speed
            self._pub.publish(cmd)
        else:
            # 停止
            self._pub.publish(Twist())
            self._done = True
            self._timer.cancel()
            self.get_logger().info(
                f'移動完了: オドメトリ計測距離 = {self._odom_dist:.4f} m')
            self._prompt_user()

    def _prompt_user(self):
        print('\n==============================')
        print(f'  オドメトリ計測距離 : {self._odom_dist:.4f} m')
        print(f'  目標距離           : {self._target_dist:.4f} m')
        print('==============================')
        print('実際に移動した距離を巻き尺で計測してください。')
        try:
            actual = float(input('実測距離 (m) を入力 > '))
        except (ValueError, EOFError):
            print('入力エラー')
            return

        current_radius = self.get_parameter_or(
            '/esp32_bridge/wheel_radius', rclpy.parameter.Parameter(
                'wheel_radius', value=0.0391)).value

        # 補正係数 = 実測 / オドメトリ
        factor = actual / self._odom_dist
        corrected = current_radius * factor

        print('\n--- 結果 ---')
        print(f'実測距離     : {actual:.4f} m')
        print(f'オドメトリ   : {self._odom_dist:.4f} m')
        print(f'補正係数     : {factor:.6f}')
        print(f'現在 wheel_radius: {current_radius:.6f} m')
        print(f'補正後 wheel_radius: {corrected:.6f} m')
        print(f'\n適用コマンド:')
        print(f'  ros2 run th_calibration apply_calib.py --ros-args -p wheel_radius:={corrected:.6f}')


def main(args=None):
    rclpy.init(args=args)
    node = LinearCalib()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
