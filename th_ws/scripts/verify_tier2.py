#!/usr/bin/env python3
"""Tier 2 の状態トピックを実機構成なしで検証する使い捨てスクリプト。

CLI (ros2 topic echo) は WSL 内 DDS の発見が不安定なので、購読と publish を
1 プロセスにまとめて確実に取る。

前提: person_predictor / follow_planner_mapless / mode_manager が起動済み。
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from th_system_msgs.msg import (
    PersonStatus, RobotMode, FollowStatus, SearchStatus, SummonStatus,
)
from th_system_msgs.srv import SetMode


class Verifier(Node):
    def __init__(self):
        super().__init__('tier2_verifier')
        self.search_log = []   # (t, phase)
        self.follow_log = []   # (t, planner, state, reason, dist)
        self.follow_raw = {}   # (planner, state, reason) -> 受信数
        self.summon_log = []

        self.create_subscription(SearchStatus, '/person/search_status',
                                 self._cb_search, 10)
        self.create_subscription(FollowStatus, '/follow/status',
                                 self._cb_follow, 10)
        self.create_subscription(SummonStatus, '/summon_navigator/status',
                                 self._cb_summon, 10)

        self.pub_person = self.create_publisher(PersonStatus, '/person/status', 10)
        # follow_planner_mapless は BEST_EFFORT で購読しているので合わせる
        self.pub_scan = self.create_publisher(
            LaserScan, '/scan_filtered',
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.set_mode_cli = self.create_client(SetMode, '/mode_manager/set_mode')
        self.t0 = time.time()

    def scan(self, front_dist=None):
        """全方位 5m のスキャンを流す。front_dist を与えると正面だけその距離にする"""
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser_link'
        m.angle_min = -math.pi
        m.angle_max = math.pi
        n = 360
        m.angle_increment = (m.angle_max - m.angle_min) / n
        m.range_min = 0.05
        m.range_max = 10.0
        ranges = [5.0] * n
        if front_dist is not None:
            # 正面 (angle=0 → index n/2) 付近を近距離にする
            for i in range(n // 2 - 15, n // 2 + 15):
                ranges[i] = front_dist
        m.ranges = ranges
        self.pub_scan.publish(m)

    def _cb_search(self, msg):
        entry = (round(time.time() - self.t0, 1), msg.phase)
        if not self.search_log or self.search_log[-1][1] != msg.phase:
            self.search_log.append(entry)

    def _cb_follow(self, msg):
        key = (msg.planner, msg.state, msg.reason)
        # 生の受信数も数える。INACTIVE が「モード離脱時に1回だけ」になっている
        # ことの確認は、遷移ログ (重複除去済み) では見えないため
        self.follow_raw[key] = self.follow_raw.get(key, 0) + 1
        if not self.follow_log or self.follow_log[-1][1:4] != key:
            self.follow_log.append(
                (round(time.time() - self.t0, 1), *key, round(msg.person_distance, 2)))

    def _cb_summon(self, msg):
        self.summon_log.append((round(time.time() - self.t0, 1), msg.event))

    def person(self, x, y, lost, reason=''):
        m = PersonStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.position.x = float(x)
        m.position.y = float(y)
        m.confidence = 0.0 if lost else 0.9
        m.is_lost = lost
        m.lost_reason = reason
        self.pub_person.publish(m)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.requested_mode = mode
        req.requester = 'tier2_verifier'
        self.set_mode_cli.call_async(req)

    def spin_for(self, sec, person=None, scan='clear'):
        """sec 秒スピンする。person=(x,y,lost,reason) なら 10Hz で流し続ける。
        scan: 'clear'(障害物なし) / float(正面の距離) / None(スキャンを流さない)"""
        end = time.time() + sec
        while time.time() < end:
            if person is not None:
                self.person(*person)
            if scan == 'clear':
                self.scan()
            elif isinstance(scan, (int, float)):
                self.scan(float(scan))
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    v = Verifier()
    print('サービス待機...')
    v.set_mode_cli.wait_for_service(timeout_sec=10.0)
    v.spin_for(2.0)

    # ── 1. 捜索段階の遷移 ─────────────────────────────────
    print('[1] 試験員を捕捉 → ロスト → 20秒観測')
    v.spin_for(1.5, person=(2.0, 0.0, False))
    v.spin_for(20.0, person=(2.0, 0.0, True, 'DETECTION_LOST'))
    print('    再捕捉')
    v.spin_for(3.0, person=(2.0, 0.0, False))

    # ── 2. FOLLOWING_MAPLESS での follow/status ───────────
    print('[2] FOLLOWING_MAPLESS へ遷移 (2.0m・障害物なし → tracking のはず)')
    v.set_mode(RobotMode.FOLLOWING_MAPLESS)
    v.spin_for(4.0, person=(2.0, 0.0, False))

    print('    正面 0.5m に障害物 → obstacle_ahead (C7)')
    v.spin_for(4.0, person=(2.0, 0.0, False), scan=0.5)

    print('    障害物を除去 → tracking へ復帰 (N12)')
    v.spin_for(4.0, person=(2.0, 0.0, False))

    print('    試験員が 0.5m まで接近 → person_close (N9)')
    v.spin_for(4.0, person=(0.5, 0.0, False))

    print('    離れる 1.5m → tracking へ復帰 (N10)')
    v.spin_for(4.0, person=(1.5, 0.0, False))

    print('    スキャンを止める → no_scan (C8)')
    v.spin_for(4.0, person=(1.5, 0.0, False), scan=None)

    print('[3] IDLE へ戻す (INACTIVE が1回だけ出るはず)')
    v.set_mode(RobotMode.IDLE)
    v.spin_for(6.0, scan=None)

    # ── 結果 ──────────────────────────────────────────────
    print('\n=== /person/search_status の phase 遷移 ===')
    for t, phase in v.search_log:
        print(f'  {t:6.1f}s  {phase}')

    print('\n=== /follow/status の遷移 ===')
    for row in v.follow_log:
        t, planner, state, reason, dist = row
        print(f'  {t:6.1f}s  planner={planner:8s} state={state:9s} '
              f'reason={reason:16s} dist={dist}')

    print('\n=== /follow/status の生の受信数 ===')
    print('  (INACTIVE はモード離脱時の1回だけ、が期待値)')
    for key, n in sorted(v.follow_raw.items()):
        planner, state, reason = key
        print(f'  {n:4d} 回  planner={planner:8s} state={state:9s} reason={reason}')

    if v.summon_log:
        print('\n=== /summon_navigator/status ===')
        for t, ev in v.summon_log:
            print(f'  {t:6.1f}s  {ev}')

    v.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
