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
import os
import re


def read_wheel_radius_from_config_h():
    """esp32/src/config.h の WHEEL_RADIUS_M を読む。

    車輪半径は ESP32 ファームウェアのコンパイル時定数で ROS パラメータではないため、
    ここで値を二重に持つと必ず食い違う。実際、既定値が 0.0391(URDF 由来の旧値)のまま
    config.h だけ 0.019412 に更新されており、そのまま実行すると補正値が約2倍ずれる
    状態だった(2026-08-07 修正)。唯一の正である config.h を直接読む。

    見つからない場合は None を返し、呼び出し側で current_wheel_radius パラメータへ
    フォールバックする。
    """
    here = os.path.dirname(os.path.realpath(__file__))
    path = os.path.normpath(
        os.path.join(here, '..', '..', '..', 'esp32', 'src', 'config.h'))
    try:
        with open(path, encoding='utf-8') as f:
            m = re.search(
                r'^\s*#define\s+WHEEL_RADIUS_M\s+([0-9.eE+-]+)f?',
                f.read(), re.MULTILINE)
    except OSError:
        return None, path
    if not m:
        return None, path
    try:
        return float(m.group(1)), path
    except ValueError:
        return None, path


class LinearCalib(Node):
    def __init__(self):
        super().__init__('linear_calib')

        self.declare_parameter('distance', 2.0)              # m
        self.declare_parameter('speed', 0.15)                # m/s (低速で正確に)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_manual')
        # 既定 -1 = config.h から自動で読む。明示指定した場合はそちらを優先する。
        self.declare_parameter('current_wheel_radius', -1.0)

        self._target_dist = self.get_parameter('distance').value
        self._speed = self.get_parameter('speed').value

        param_radius = self.get_parameter('current_wheel_radius').value
        if param_radius > 0.0:
            self._current_radius = param_radius
            self._radius_source = 'パラメータ指定'
        else:
            radius, path = read_wheel_radius_from_config_h()
            if radius is None:
                self.get_logger().error(
                    f'{path} から WHEEL_RADIUS_M を読めませんでした。'
                    ' -p current_wheel_radius:=<現在値> で明示指定して再実行してください。'
                    ' 誤った現行値を使うと補正後の値がそのまま倍率ぶんずれます。中断します。')
                raise SystemExit(1)
            self._current_radius = radius
            self._radius_source = path

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
            f'(publish先: {topic}, '
            f'現行 WHEEL_RADIUS_M={self._current_radius:.6f} m '
            f'← {self._radius_source})')

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
        print(f'現在の WHEEL_RADIUS_M : {self._current_radius:.6f} m  ({self._radius_source})')
        print(f'補正後 WHEEL_RADIUS_M : {corrected:.6f} m')
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
