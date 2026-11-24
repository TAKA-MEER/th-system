#!/usr/bin/env python3
"""measure_sim_fake.py — Gazebo 上で自律測定（AUTO_MEASURE）を駆動する測定機能の代役（8-g P6）。

fk-system が `gazebo.launch.py sim:=true gui:=false` に繋いで自律測定を回すときの
手順を、そのまま再現できるようにした代役。開発モードは使わない。

  1. `/measure/heartbeat` を 5 Hz で送り続ける（`interface_version` "1"）。
  2. `/system/state` が IDLE になるのを待つ（dev_mode 無しで link_ok が出ることの確認）。
  3. `/system/trigger` へ `ui.goto`（`{"kind":"AUTO_MEASURE"}`）を送り、
     AUTO_MEASURE/WAIT になるのを待つ。その間 `/ui/active_screen` を
     `screen_id` S-60・`client_id` fk-measure で 2 Hz 送り続ける
     （測定機能の画面の在席。無いと `zone: NA`・`speed_limit: stop` で
     `/cmd_vel` が 0 のままになる）。
  4. `/measure/status` の自己位置（map フレーム）から前方 1.5 m の点を
     `meas.goal`（`/measure/command`）として送る。
  5. `/measure/result` にその `command_id` の `ARRIVED` が 1 回返るのを待つ。
     途中の `/cmd_vel` の最大値を記録して出す。

使い方（Docker コンテナ内。gazebo.launch.py を起動した別のシェルから。
`th_ws/scripts/` はコンテナにマウントされている）:

    python3 /root/th_ws/scripts/measure_sim_fake.py --timeout 180

終了コード: ARRIVED が 1 回返れば 0、タイムアウト・拒否なら 1。
ルドダウンは stdout の JSON（`result` キー）で出す。`--goal-dist` で
目的地の前方距離を変えられる（既定 1.5 m。panel_room の机上確認用）。
"""
import argparse
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from geometry_msgs.msg import Twist
from th_system_msgs.msg import (ActiveScreen, MeasureHeartbeat, MeasureResult,
                                MeasureStatus, SystemState)
from th_system_msgs.srv import MeasureCommand, UiTrigger


def _latched():
    return QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


class FakeMeasureFunction(Node):
    """心拍の送信と、/measure/status・/system/state・/measure/result の記録だけを持つ。"""

    def __init__(self):
        super().__init__('measure_sim_fake')
        self._lock = threading.Lock()
        self._status = None
        self._state = None
        self._results = []
        self._cmd_vel_max = 0.0
        self._hb_seq = 0

        self.create_subscription(MeasureStatus, '/measure/status', self._on_status, _latched())
        self.create_subscription(SystemState, '/system/state', self._on_state, _latched())
        self.create_subscription(MeasureResult, '/measure/result', self._on_result, 20)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self._pub_hb = self.create_publisher(MeasureHeartbeat, '/measure/heartbeat', 10)
        self._hb_timer = self.create_timer(0.2, self._publish_hb)  # 5 Hz
        self._pub_screen = self.create_publisher(ActiveScreen, '/ui/active_screen', 10)
        self._screen_timer = self.create_timer(0.5, self._publish_screen)  # 2 Hz

        self.cli_trigger = self.create_client(UiTrigger, '/system/trigger')
        self.cli_measure = self.create_client(MeasureCommand, '/measure/command')

    def _on_status(self, msg):
        with self._lock:
            self._status = msg

    def _on_state(self, msg):
        with self._lock:
            self._state = msg

    def _on_result(self, msg):
        with self._lock:
            self._results.append(msg)

    def _on_cmd_vel(self, msg):
        v = abs(float(msg.linear.x))
        with self._lock:
            if v > self._cmd_vel_max:
                self._cmd_vel_max = v

    def _publish_hb(self):
        self._hb_seq += 1
        msg = MeasureHeartbeat()
        msg.interface_version = "1"
        msg.sender = "fk-system"
        msg.seq = self._hb_seq
        self._pub_hb.publish(msg)

    def _publish_screen(self):
        msg = ActiveScreen()
        msg.screen_id = 'S-60'
        msg.client_id = 'fk-measure'
        msg.interacting = True
        msg.last_input = self.get_clock().now().to_msg()
        self._pub_screen.publish(msg)

    # ── 照会（呼び出し側は executor を裏で回している） ──────────────────────
    def latest_status(self):
        with self._lock:
            return self._status

    def latest_state(self):
        with self._lock:
            return self._state

    def results_for(self, cid):
        with self._lock:
            return [r for r in self._results if r.command_id == cid]

    def cmd_vel_max(self):
        with self._lock:
            return self._cmd_vel_max

    def wait_ready(self, timeout):
        return (self.cli_trigger.wait_for_service(timeout_sec=timeout)
                and self.cli_measure.wait_for_service(timeout_sec=timeout))

    def call_trigger(self, trigger, arg_json, timeout=10.0):
        req = UiTrigger.Request()
        req.trigger = trigger
        req.arg_json = arg_json
        req.requester = 'measure_sim_fake'
        return self._call(self.cli_trigger, req, timeout)

    def call_measure(self, req, timeout=10.0):
        return self._call(self.cli_measure, req, timeout)

    @staticmethod
    def _call(client, req, timeout):
        future = client.call_async(req)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None


def main(args=None):
    ap = argparse.ArgumentParser(description='測定機能の代役（Gazebo 自律測定の確認）')
    ap.add_argument('--timeout', type=float, default=180.0,
                    help='全体の制限時間 [s]（既定 180）')
    ap.add_argument('--goal-dist', type=float, default=1.5,
                    help='目的地の前方距離 [m]（既定 1.5）')
    ap.add_argument('--goal-x', type=float, default=None,
                    help='目的地の x [m]（map フレーム。指定すると前方距離より優先）')
    ap.add_argument('--goal-y', type=float, default=None,
                    help='目的地の y [m]（map フレーム。指定すると前方距離より優先）')
    ap.add_argument('--goal-yaw', type=float, default=None,
                    help='目的地の向き [rad]（未指定は現在向きのまま）')
    ap.add_argument('--cid', default='sim-goal-1')
    ns, _ = ap.parse_known_args(args)

    rclpy.init()
    node = FakeMeasureFunction()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    out = {'ok': False, 'cmd_vel_max_mps': 0.0}
    deadline = time.monotonic() + ns.timeout
    try:
        if not node.wait_ready(timeout=30.0):
            out['error'] = 'service_not_ready'
            print(json.dumps(out))
            return 1

        # 1. IDLE を待つ（dev_mode 無し。link_ok が出て初めて入れる）。
        while time.monotonic() < deadline:
            st = node.latest_state()
            if st is not None and st.mode == 'IDLE':
                break
            time.sleep(0.1)
        st = node.latest_state()
        if st is None or st.mode != 'IDLE':
            out['error'] = 'not_idle'
            out['state'] = None if st is None else (st.mode, st.state)
            print(json.dumps(out))
            return 1
        out['idle_reached'] = True

        # 2. ui.goto AUTO_MEASURE。
        resp = node.call_trigger('ui.goto', '{"kind": "AUTO_MEASURE"}')
        out['goto_accepted'] = bool(resp and resp.accepted)
        if resp is not None:
            out['goto_reject_reason'] = resp.reject_reason_key
        while time.monotonic() < deadline:
            st = node.latest_state()
            if st is not None and st.mode == 'AUTO_MEASURE' and st.state == 'WAIT':
                break
            time.sleep(0.1)
        st = node.latest_state()
        if st is None or (st.mode, st.state) != ('AUTO_MEASURE', 'WAIT'):
            out['error'] = 'not_auto_measure_wait'
            out['state'] = None if st is None else (st.mode, st.state)
            print(json.dumps(out))
            return 1
        out['auto_measure_wait'] = True

        # 3. 自己位置の前方を meas.goal にする。
        status = node.latest_status()
        if status is None or not status.pose_valid:
            out['error'] = 'pose_not_valid'
            print(json.dumps(out))
            return 1
        gx = status.x + math.cos(status.yaw) * ns.goal_dist
        gy = status.y + math.sin(status.yaw) * ns.goal_dist
        gyaw = status.yaw
        if ns.goal_x is not None and ns.goal_y is not None:
            gx, gy = ns.goal_x, ns.goal_y
        if ns.goal_yaw is not None:
            gyaw = ns.goal_yaw
        req = MeasureCommand.Request()
        req.command_id = ns.cid
        req.command = 'meas.goal'
        req.map_instance_id = status.map_instance_id
        req.x = float(gx)
        req.y = float(gy)
        req.yaw = float(gyaw)
        resp = node.call_measure(req)
        out['goal_accepted'] = bool(resp and resp.accepted)
        if resp is not None:
            out['goal_reject_reason'] = resp.reject_reason
            out['goal_mode_state'] = (resp.mode, resp.state)
        if not (resp and resp.accepted):
            out['error'] = 'goal_rejected'
            print(json.dumps(out))
            return 1
        out['goal'] = {'x': float(gx), 'y': float(gy), 'yaw': float(gyaw),
                       'map_instance_id': status.map_instance_id}

        # 4. ARRIVED を待つ。
        while time.monotonic() < deadline:
            got = [r for r in node.results_for(ns.cid) if r.result == 'ARRIVED']
            if got:
                out['arrived_count'] = len(got)
                out['ok'] = True
                out['cmd_vel_max_mps'] = node.cmd_vel_max()
                print(json.dumps(out))
                return 0
            time.sleep(0.1)
        out['error'] = 'arrived_timeout'
        st = node.latest_state()
        out['state'] = None if st is None else (st.mode, st.state)
        status = node.latest_status()
        out['pending'] = None if status is None else (
            status.pending_command_id, status.pending_command)
        out['cmd_vel_max_mps'] = node.cmd_vel_max()
        print(json.dumps(out))
        return 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
