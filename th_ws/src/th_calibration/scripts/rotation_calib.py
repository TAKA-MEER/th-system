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
#   2. mode_manager を IDLE または MANUAL にしておく(他モードでは安全のため中断する)
#   3. 本ツールを実行（360° × turns 回転、/cmd_vel_manual → twist_mux 経由で駆動）
#   4. 停止後、実際の回転角を分度器や壁基準で計測
#   5. プロンプトに入力 → wheel_base 補正値を出力
#   6. apply_calib で反映（再起動不要で即座に反映される）
# ============================================================
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from th_system_msgs.msg import RobotMode
import math


class RotationCalib(Node):
    def __init__(self):
        super().__init__('rotation_calib')

        self.declare_parameter('turns', 1)          # 回転数
        self.declare_parameter('speed', 0.3)        # rad/s (低速推奨)
        self.declare_parameter('direction', 1)      # 1=反時計, -1=時計
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_manual')

        self._turns = self.get_parameter('turns').value
        self._speed = self.get_parameter('speed').value
        self._direction = self.get_parameter('direction').value

        self._target_rad = 2.0 * math.pi * self._turns
        self._odom_yaw = 0.0
        self._accumulated = 0.0
        self._prev_yaw = None
        self._running = False
        self._done = False
        self._mode = None
        self._mode_wait_start = self.get_clock().now()
        self._odom_wait_start = None

        self._current_base = self._fetch_current_wheel_base()

        topic = self.get_parameter('cmd_vel_topic').value
        self._pub = self.create_publisher(Twist, topic, 10)
        self._sub_odom = self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)
        self._sub_mode = self.create_subscription(
            RobotMode, '/robot/mode', self._cb_mode, 10)
        self._timer = self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'旋回キャリブレーション: {self._turns} 回転 '
            f'方向={"CCW" if self._direction > 0 else "CW"} '
            f'(publish先: {topic}, 現在の wheel_base={self._current_base:.6f} m)')

    def _fetch_current_wheel_base(self) -> float:
        cli = self.create_client(GetParameters, '/esp32_bridge/get_parameters')
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                'esp32_bridge が見つかりません。既定値 0.39 m を使用します。')
            return 0.39

        req = GetParameters.Request(names=['wheel_base'])
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        result = future.result()
        if result is None or not result.values:
            self.get_logger().warn(
                'wheel_base の取得に失敗しました。既定値 0.39 m を使用します。')
            return 0.39

        return result.values[0].double_value

    def _cb_mode(self, msg: RobotMode):
        self._mode = msg.mode

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # クォータニオン → yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        if self._prev_yaw is None:
            self._prev_yaw = yaw
            return

        # 角度差（折り返し対応）
        delta = yaw - self._prev_yaw
        if delta > math.pi: delta -= 2 * math.pi
        if delta < -math.pi: delta += 2 * math.pi

        self._accumulated += abs(delta)
        self._prev_yaw = yaw

    def _loop(self):
        if self._done:
            return

        # ── モードガード: IDLE/MANUAL 以外では走行させない ──────
        if not self._running:
            if self._mode is None:
                elapsed = (self.get_clock().now() - self._mode_wait_start).nanoseconds / 1e9
                if elapsed > 5.0:
                    self.get_logger().error(
                        'mode_manager から /robot/mode を受信できません。'
                        ' mode_manager が起動しているか確認してください。中断します。')
                    self._done = True
                    self._timer.cancel()
                return  # /robot/mode 待ち
            if self._mode not in (RobotMode.IDLE, RobotMode.MANUAL):
                self.get_logger().error(
                    f'現在のモードが IDLE/MANUAL ではありません (mode={self._mode})。'
                    ' /mode_manager/set_mode で IDLE または MANUAL に切り替えてから'
                    ' 再実行してください。中断します。')
                self._done = True
                self._timer.cancel()
                return
            self._running = True

        if self._prev_yaw is None:
            if self._odom_wait_start is None:
                self._odom_wait_start = self.get_clock().now()
            elapsed = (self.get_clock().now() - self._odom_wait_start).nanoseconds / 1e9
            if elapsed > 5.0:
                self.get_logger().error(
                    '/odom を受信できません。esp32_bridge が起動し ESP32 と接続しているか'
                    ' 確認してください。中断します。')
                self._done = True
                self._timer.cancel()
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

        # wheel_base 補正:
        #   旋回中の実際の走行距離 = wheel_base_real × 実測角度
        #   オドメトリの走行距離   = wheel_base_param × odom_角度
        #   精度が出るには: wheel_base_real / wheel_base_param = 実測 / オドメトリ
        factor = math.radians(actual_deg) / math.radians(odom_deg)
        corrected = self._current_base * factor

        print('\n--- 結果 ---')
        print(f'実測角度     : {actual_deg:.2f}°')
        print(f'オドメトリ   : {odom_deg:.2f}°')
        print(f'補正係数     : {factor:.6f}')
        print(f'現在 wheel_base: {self._current_base:.6f} m')
        print(f'補正後 wheel_base: {corrected:.6f} m')
        print('\n適用コマンド:')
        print(f'  ros2 run th_calibration apply_calib.py --ros-args -p wheel_base:={corrected:.6f}')


def main(args=None):
    rclpy.init(args=args)
    node = RotationCalib()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
