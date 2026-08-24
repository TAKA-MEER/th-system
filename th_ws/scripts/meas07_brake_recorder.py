#!/usr/bin/env python3
"""制動・空走の生データ記録（WP-MEAS-01 / N-12）

停止の 2 経路を、同じ記録で見分けられる形に残す。

  ランプ停止 : /cmd_vel が 0 になる → ESP32 が TARGET_RAMP_ACCEL_MPS2 (1.5) で減速
  空走       : 物理非常停止で駆動電源が切れる → Motor::stopAll() = PWM 0 = 空走
               （main.cpp:156-164 / motor.cpp:40-43。DIR+PWM 駆動なので能動制動は無い）

記録するのは「生の時刻と速度」だけ。判定は meas07_analyze.py が行う。
時刻は tcpdump や他の記録と突き合わせられるよう time.time_ns()（CLOCK_REALTIME）。
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from th_system_msgs.msg import WheelFeedback


class BrakeRecorder(Node):
    def __init__(self, out_path):
        super().__init__('meas07_brake_recorder')
        # 行バッファにする。SIGTERM で落とされても直前まで残す
        self._f = open(out_path, 'w', buffering=1)
        self._f.write('topic,recv_ns,a,b,c\n')

        self._v_cmd = 0.0
        self._estop_hw = 0

        self.create_subscription(
            WheelFeedback, '/esp32/wheel_feedback', self._on_wheel, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Bool, '/safety/estop_hw', self._on_estop, 10)
        # EKF が動いていれば使う。動いていなくても wheel_feedback だけで成立する
        self.create_subscription(
            Odometry, '/odometry/filtered', self._on_odom, qos_profile_sensor_data)

        self.get_logger().info(f'記録先: {out_path}')

    def _w(self, topic, a, b=0.0, c=0.0):
        self._f.write(f'{topic},{time.time_ns()},{a},{b},{c}\n')

    def _on_wheel(self, msg):
        # 左右の実速度。WheelFeedback.msg には dt が無いので、速度の積分は
        # 到着時刻の差で行う（WiFi 遅延がそのまま距離誤差になる）。
        # このため距離の正は巻尺であり、この積分は補助である。
        self._w('wheel', msg.left_speed, msg.right_speed,
                (msg.left_speed + msg.right_speed) / 2.0)

    def _on_cmd(self, msg):
        self._v_cmd = msg.linear.x
        self._w('cmd', msg.linear.x, msg.angular.z)

    def _on_estop(self, msg):
        v = 1 if msg.data else 0
        if v != self._estop_hw:
            self._estop_hw = v
            self._w('estop_hw', v)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._w('odom', p.x, p.y, msg.twist.twist.linear.x)

    def destroy_node(self):
        try:
            self._f.flush()
            self._f.close()
        finally:
            super().destroy_node()


def main():
    if len(sys.argv) < 2:
        sys.exit('使い方: meas07_brake_recorder.py <出力CSV>')
    rclpy.init()
    node = BrakeRecorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
