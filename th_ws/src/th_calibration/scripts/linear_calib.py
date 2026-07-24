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
#   2. mode_manager を IDLE または MANUAL にしておく(他モードでは安全のため中断する)
#   3. 本ツールを実行する(/cmd_vel_manual → twist_mux 経由で駆動、estop/fault_lock 対象)
#   4. 停止後、実際に進んだ距離を計測する
#   5. プロンプトに入力 → 補正後の WHEEL_RADIUS_M を算出
#   6. esp32/src/config.h の WHEEL_RADIUS_M を書き換えて esptool で再書き込みする
#      (車輪半径は ESP32 ファームウェアのコンパイル時定数であり、ROS 側のパラメータでは
#       ないため、ここでは反映方法の案内のみ行う)
# ============================================================
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from th_system_msgs.msg import RobotMode
import math


class LinearCalib(Node):
    def __init__(self):
        super().__init__('linear_calib')

        self.declare_parameter('distance', 2.0)              # m
        self.declare_parameter('speed', 0.15)                # m/s (低速で正確に)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_manual')
        self.declare_parameter('current_wheel_radius', 0.0391)  # config.h の現行値

        self._target_dist = self.get_parameter('distance').value
        self._speed = self.get_parameter('speed').value
        self._current_radius = self.get_parameter('current_wheel_radius').value

        self._odom_dist = 0.0
        self._start_x = None
        self._start_y = None
        self._running = False
        self._done = False
        self._mode = None
        self._mode_wait_start = self.get_clock().now()
        self._odom_wait_start = None

        topic = self.get_parameter('cmd_vel_topic').value
        self._pub = self.create_publisher(Twist, topic, 10)
        self._sub_odom = self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)
        self._sub_mode = self.create_subscription(
            RobotMode, '/robot/mode', self._cb_mode, 10)

        self._timer = self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'直進キャリブレーション開始: 目標 {self._target_dist:.1f} m '
            f'(publish先: {topic})')

    def _cb_mode(self, msg: RobotMode):
        self._mode = msg.mode

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

        if self._start_x is None:
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

        # 補正係数 = 実測 / オドメトリ
        factor = actual / self._odom_dist
        corrected = self._current_radius * factor

        print('\n--- 結果 ---')
        print(f'実測距離     : {actual:.4f} m')
        print(f'オドメトリ   : {self._odom_dist:.4f} m')
        print(f'補正係数     : {factor:.6f}')
        print(f'現在の WHEEL_RADIUS_M(config.h想定): {self._current_radius:.6f} m')
        print(f'補正後 WHEEL_RADIUS_M           : {corrected:.6f} m')
        print('\n次の手順:')
        print(' 1. th_ws/esp32/src/config.h の')
        print(f'    #define WHEEL_RADIUS_M を {corrected:.6f}f に書き換え')
        print(' 2. esptool で再書き込み (手順は setup.md / 実機メモ参照)')
        print(' 3. 再起動後に再計測して検算')


def main(args=None):
    rclpy.init(args=args)
    node = LinearCalib()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
