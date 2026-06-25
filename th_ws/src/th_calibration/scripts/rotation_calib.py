#!/usr/bin/env python3
# ============================================================
# rotation_calib — 旋回キャリブレーション
#
# 使い方:
#   ros2 run th_calibration rotation_calib.py --ros-args \
#     -p turns:=1 -p speed:=0.3
#
# 手順:
#   1. ロボットの向きに床でマーキング（開始角度を記録）
#   2. 本ツールを実行（360° × turns 回転）
#   3. 停止後、実際の回転角を分度器や壁基準で計測
#   4. プロンプトに入力 → wheel_base 補正値を出力
#   5. apply_calib で反映
# ============================================================
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class RotationCalib(Node):
    def __init__(self):
        super().__init__('rotation_calib')

        self.declare_parameter('turns',        1)      # 回転数
        self.declare_parameter('speed',        0.3)   # rad/s (低速推奨)
        self.declare_parameter('direction',    1)      # 1=反時計, -1=時計

        self._turns     = self.get_parameter('turns').value
        self._speed     = self.get_parameter('speed').value
        self._direction = self.get_parameter('direction').value

        self._target_rad = 2.0 * math.pi * self._turns
        self._odom_yaw   = 0.0
        self._accumulated = 0.0
        self._prev_yaw    = None
        self._done        = False

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._sub = self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)
        self._timer = self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'旋回キャリブレーション: {self._turns} 回転 '
            f'方向={"CCW" if self._direction > 0 else "CW"}')

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # クォータニオン → yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw  = math.atan2(siny, cosy)

        if self._prev_yaw is None:
            self._prev_yaw = yaw
            return

        # 角度差（折り返し対応）
        delta = yaw - self._prev_yaw
        if delta >  math.pi: delta -= 2 * math.pi
        if delta < -math.pi: delta += 2 * math.pi

        self._accumulated += abs(delta)
        self._prev_yaw = yaw

    def _loop(self):
        if self._done:
            return
        if self._prev_yaw is None:
            return  # odom 待ち

        if self._accumulated < self._target_rad:
            cmd = Twist()
            cmd.angular.z = self._direction * self._speed
            self._pub.publish(cmd)
        else:
            self._pub.publish(Twist())
            self._done = True
            self._timer.cancel()
            odom_deg = math.degrees(self._accumulated)
            self.get_logger().info(
                f'旋回完了: オドメトリ累積 = {odom_deg:.2f}°')
            self._prompt_user(odom_deg)

    def _prompt_user(self, odom_deg: float):
        target_deg = math.degrees(self._target_rad)
        print('\n==============================')
        print(f'  オドメトリ累積角度 : {odom_deg:.2f}°')
        print(f'  目標角度           : {target_deg:.2f}°')
        print('==============================')
        print('実際の回転角を分度器または壁の目印で計測してください。')
        try:
            actual_deg = float(input('実測角度 (°) を入力 > '))
        except (ValueError, EOFError):
            print('入力エラー')
            return

        current_base = 0.39  # デフォルト値 (apply_calib で更新済みの場合は手動で変更)
        # wheel_base 補正:
        #   旋回中の実際の走行距離 = wheel_base_real × 実測角度
        #   オドメトリの走行距離   = wheel_base_param × odom_角度
        #   精度が出るには: wheel_base_real / wheel_base_param = 実測 / オドメトリ
        factor       = math.radians(actual_deg) / math.radians(odom_deg)
        corrected    = current_base * factor

        print('\n--- 結果 ---')
        print(f'実測角度     : {actual_deg:.2f}°')
        print(f'オドメトリ   : {odom_deg:.2f}°')
        print(f'補正係数     : {factor:.6f}')
        print(f'現在 wheel_base: {current_base:.6f} m')
        print(f'補正後 wheel_base: {corrected:.6f} m')
        print(f'\n適用コマンド:')
        print(f'  ros2 run th_calibration apply_calib.py --ros-args -p wheel_base:={corrected:.6f}')


def main(args=None):
    rclpy.init(args=args)
    node = RotationCalib()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
