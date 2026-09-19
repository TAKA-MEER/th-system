#!/usr/bin/env python3
# ============================================================
# imu_calib_check — DSR1603(BNO055) キャリブレーション状態モニタ
#
# 使い方:
#   ros2 run th_calibration imu_calib_check.py
#
# 手順:
#   1. imu_enabled:=true で bringup.launch.py を起動しておく
#   2. 本ツールを実行し、案内に従ってロボットを上下左右に
#      8の字を描くように2回転させる(DSR1603マニュアル記載の手順)
#   3. sys/gyro/accel/mag が全て 3 (Fully calibrated) になったら完了
#
# 自律走行は行わない(手動操作前提の状態監視のみ)。
# ============================================================
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


LEVEL_NAMES = ('未較正(0)', '粗い(1)', '中程度(2)', '完全(3)')


class ImuCalibCheck(Node):
    def __init__(self):
        super().__init__('imu_calib_check')
        self._last_status = None
        self._reported_done = False
        self._sub = self.create_subscription(
            UInt8, '/esp32/imu_calib_status', self._cb_status, 10)
        self._wait_start = self.get_clock().now()
        self._timer = self.create_timer(1.0, self._cb_timeout_check)

        print('==============================')
        print(' DSR1603 (BNO055) キャリブレーション')
        print('==============================')
        print('ロボットを持ち上げるか手押しで、上下左右に')
        print('8の字を描くように2回転させてください。')
        print('sys/gyro/accel/mag が全て 3 になるまで継続してください。\n')

    def _cb_timeout_check(self):
        if self._last_status is None:
            elapsed = (self.get_clock().now() - self._wait_start).nanoseconds / 1e9
            if elapsed > 5.0:
                self.get_logger().error(
                    '/esp32/imu_calib_status を受信できません。'
                    ' imu_enabled:=true で bringup.launch.py を起動し、'
                    ' DSR1603 がESP32に接続されているか確認してください。')
                self._wait_start = self.get_clock().now()  # 再警告までの間隔を空ける

    def _cb_status(self, msg: UInt8):
        status = msg.data
        if status == self._last_status:
            return
        self._last_status = status

        sys_ = (status >> 6) & 0x03
        gyro = (status >> 4) & 0x03
        accel = (status >> 2) & 0x03
        mag = status & 0x03

        print(f'sys={LEVEL_NAMES[sys_]}  gyro={LEVEL_NAMES[gyro]}  '
              f'accel={LEVEL_NAMES[accel]}  mag={LEVEL_NAMES[mag]}')

        if sys_ == 3 and gyro == 3 and accel == 3 and mag == 3 and not self._reported_done:
            self._reported_done = True
            print('\n✅ キャリブレーション完了 (全項目 Fully calibrated)')
            print('このまま運用を継続できます。')


def main(args=None):
    rclpy.init(args=args)
    node = ImuCalibCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
