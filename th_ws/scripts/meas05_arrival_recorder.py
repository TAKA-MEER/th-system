#!/usr/bin/env python3
"""meas05 の ROS 側到着時刻レコーダ（`meas05_esp32_triage.sh` が起動する）。

`meas05_analyze.py` が読む `arrivals.csv` を書く。フォーマットは 1 行 1 到着の
`topic,recv_ns`（ヘッダ 1 行）。`topic` は解析器が期待する 2 つのキーだけ:

  esp32 : /esp32/wheel_feedback の到着
  lidar : /scan の到着

**時計は `CLOCK_REALTIME`。**`tcpdump` の `-tt` が同じ壁時計を打つため、
電線側と ROS 側を同じ物差しで突き合わせられる（`monotonic` にすると
突き合わせが成立しない。`meas05_analyze.py::read_arrivals` の docstring 参照）。

【このファイルが一度失われた経緯】`meas05_esp32_triage.sh` はこのスクリプトを
起動するが、**どのブランチにもコミットされていなかった**（2026-08-20 の
`*_arrivals.csv` は残っているので当時は存在した）。2026-08-31 の実機で
`can't open file ... meas05_arrival_recorder.py` で解析が空振りして発覚し、
書き直した。`meas05_analyze.py` 自体も `recovered/feat/meas-latency-triage`
にしか無く、同時に本線へ取り込んだ。

**`/scan` を購読することには計測上の意味がある。**ラズパイ AP 構成では LiDAR も
ESP32 も同じ無線区間を通るため、`/scan` を受け続けること自体が電波の占有を
再現する。購読を外すと「軽い負荷での計測」になり、実運用の跳ねを取り逃す。
"""
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from th_system_msgs.msg import WheelFeedback


def main() -> int:
    if len(sys.argv) < 2:
        print('usage: meas05_arrival_recorder.py <arrivals.csv>', file=sys.stderr)
        return 2
    out_path = sys.argv[1]

    rclpy.init()
    node = Node('meas05_arrival_recorder')
    fh = open(out_path, 'w', buffering=1, encoding='utf-8')
    fh.write('topic,recv_ns\n')

    def record(topic: str):
        def cb(_msg):
            fh.write(f'{topic},{time.clock_gettime_ns(time.CLOCK_REALTIME)}\n')
        return cb

    sensor_qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(
        WheelFeedback, '/esp32/wheel_feedback', record('esp32'), sensor_qos)
    node.create_subscription(
        LaserScan, '/scan', record('lidar'), sensor_qos)

    stop = {'now': False}

    def on_signal(_sig, _frm):
        stop['now'] = True

    # 親スクリプトは SIGTERM で止める。握って CSV を閉じてから抜ける
    # （握らないと最後の数行が失われる）。
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    node.get_logger().info(f'記録開始 → {out_path}（/esp32/wheel_feedback と /scan）')
    while rclpy.ok() and not stop['now']:
        rclpy.spin_once(node, timeout_sec=0.1)

    fh.close()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
