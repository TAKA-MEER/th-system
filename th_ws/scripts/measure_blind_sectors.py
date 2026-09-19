#!/usr/bin/env python3
"""LiDAR 死角セクタの実測（WP-CALIB-01 の最小手動版）

周囲 500mm 以内の実体を除去した状態で /scan を集め、機体構造で
恒常的に塞がれている方位を検出する。出力は registry.yaml の
blind_angle_ranges に入れる平坦配列（度・laser_link 基準・反時計回り正）。

使い方（th_robot コンテナ内）:
    python3 scripts/measure_blind_sectors.py [サンプル数] [近傍しきい値m]
"""
import sys
import math
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

NEAR_M = float(sys.argv[2]) if len(sys.argv) > 2 else 0.6
WANT = int(sys.argv[1]) if len(sys.argv) > 1 else 100
PERSIST = 0.90  # この割合以上のスキャンで塞がれていたら「死角」


class Collector(Node):
    def __init__(self):
        super().__init__('measure_blind_sectors')
        self.scans = []
        self.meta = None
        self.create_subscription(LaserScan, '/scan', self._cb, qos_profile_sensor_data)
        self.get_logger().info(f'/scan を {WANT} スキャン集めます (near<{NEAR_M}m)...')

    def _cb(self, msg: LaserScan):
        if self.meta is None:
            self.meta = (msg.angle_min, msg.angle_increment, len(msg.ranges),
                         msg.range_min, msg.range_max)
        self.scans.append(list(msg.ranges))
        if len(self.scans) % 20 == 0:
            self.get_logger().info(f'{len(self.scans)}/{WANT}')


def main():
    rclpy.init()
    node = Collector()
    while rclpy.ok() and len(node.scans) < WANT:
        rclpy.spin_once(node, timeout_sec=1.0)
    if not node.scans:
        print('1 スキャンも受信できず。LiDAR が回っているか確認')
        sys.exit(1)

    amin, ainc, n, rmin, rmax = node.meta
    print(f'\nangle_min={math.degrees(amin):.2f}deg  '
          f'angle_increment={math.degrees(ainc):.4f}deg  points={n}  '
          f'range_min={rmin}  range_max={rmax}  scans={len(node.scans)}')

    # 各ビームの統計
    blocked = [0] * n      # 塞がれ (near かつ有効 or 0/inf/nan)
    near_cnt = [0] * n
    invalid_cnt = [0] * n
    med = [float('nan')] * n
    per_beam_vals = [[] for _ in range(n)]
    for ranges in node.scans:
        for i in range(min(n, len(ranges))):
            r = ranges[i]
            if r is None or math.isnan(r) or math.isinf(r) or r <= 0.0 or r < rmin:
                invalid_cnt[i] += 1
            elif r < NEAR_M:
                near_cnt[i] += 1
                per_beam_vals[i].append(r)
            else:
                per_beam_vals[i].append(r)
    for i in range(n):
        if per_beam_vals[i]:
            med[i] = median(per_beam_vals[i])

    ns = len(node.scans)
    # 死角＝周囲を除去したのに近距離の安定した返りが出続けるビーム（＝機体構造）
    is_blind = [near_cnt[i] >= PERSIST * ns for i in range(n)]
    # 参考: 遠方で無効が続くビーム（吸収体/ガラス/隙間の可能性。マスクしない）
    is_invalid = [invalid_cnt[i] >= PERSIST * ns and near_cnt[i] < PERSIST * ns
                  for i in range(n)]

    def beam_deg(i):
        # scan_geometry 規約: angle = angle_min + i*inc, [-180,180) へ正規化
        a = amin + i * ainc
        a = (a + math.pi) % (2 * math.pi) - math.pi
        return math.degrees(a)

    # 連続する死角ビームをセクタへ（index 昇順。ラップは後で結合）
    GAP = 2  # この本数以下の非死角ビームは同一構造とみなして繋ぐ
    sectors = []
    i = 0
    while i < n:
        if is_blind[i]:
            j = i
            while True:
                k = j + 1
                while k < n and k - j - 1 < GAP and not is_blind[k]:
                    k += 1
                if k < n and is_blind[k]:
                    j = k
                else:
                    break
            sectors.append((i, j))
            i = j + 1
        else:
            i += 1
    # index 0 と n-1 が両方死角ならラップ結合
    if len(sectors) >= 2 and sectors[0][0] == 0 and sectors[-1][1] == n - 1:
        first = sectors.pop(0)
        last = sectors.pop(-1)
        sectors.append((last[0], first[1] + n))  # j に n を足してラップ表現

    print(f'\n死角ビーム総数: {sum(is_blind)} / {n}  '
          f'({100*sum(is_blind)/n:.1f}%)')
    print('\n検出セクタ（laser_link 基準・度・反時計回り正）:')
    if not sectors:
        print('  なし（構造による恒常的な死角は検出されなかった）')
    flat = []
    # 2026-09-09 実測後の追認（実機フィードバック）: 支柱のグレージング角
    # (=セクタ端で斜めに柱をかすめるビーム)は、90%persist判定と
    # NEAR_M しきい値の境界で本来の柱の幅より数度(4〜7度)過小に切り詰め
    # られ、地図に柱の点群が漏れて写る不具合が実機で確認された。
    # 1.0deg では吸収しきれないため 5.0deg に拡大。
    MARGIN = 5.0  # deg 余裕
    for (i0, j) in sectors:
        i1 = j % n
        d0 = beam_deg(i0)
        d1 = beam_deg(i1)
        # ビーム中心 -> セクタ端は半ビーム外側 + マージン
        half = math.degrees(ainc) / 2
        a0 = d0 - half - MARGIN
        a1 = d1 + half + MARGIN
        span = (j - i0 + 1) * math.degrees(ainc)
        # このセクタ内の代表距離
        contam = sum(1 for k in range(i0, j + 1)
                     if not math.isnan(med[k % n]) and med[k % n] > 0.6)
        mvals = [med[k % n] for k in range(i0, j + 1) if not math.isnan(med[k % n])]
        rep = f'{min(mvals):.3f}..{max(mvals):.3f}m' if mvals else 'inf/invalid'
        print(f'  beam[{i0}..{i1}]  中心 {d0:+.1f}..{d1:+.1f}deg  '
              f'幅~{span:.1f}deg  構造距離 {rep}  室内が見えるビーム {contam}本')
        flat += [round(a0, 1), round(a1, 1)]

    inv_beams=[i for i in range(n) if is_invalid[i]]
    if inv_beams:
        print(f"\n[参考] 遠方で無効が続くビーム {len(inv_beams)} 本（マスク対象外・要目視）:")
        runs=[]; i=0
        while i<len(inv_beams):
            j=i
            while j+1<len(inv_beams) and inv_beams[j+1]==inv_beams[j]+1: j+=1
            runs.append((inv_beams[i],inv_beams[j])); i=j+1
        for a,b in runs:
            print(f"  beam[{a}..{b}]  {beam_deg(a):+.1f}..{beam_deg(b):+.1f}deg")
    print(f'\nregistry.yaml blind_angle_ranges (平坦配列):\n  {flat}')
    print(f'\nライブ確認用:\n  ros2 param set /lidar_filter blind_angle_ranges "{flat}"')

    # 前方/後方コーンとの重なり（L5: obstacle_limiter が v_reverse=0.25 に丸める）
    FWD = 28.6  # obstacle_cone_half_width_rad 0.5rad
    REV = 34.4  # 0.6rad
    def overlaps(lo, hi, c, hw):
        # 円環上の重なり判定（度）
        def norm(x): return (x + 180) % 360 - 180
        for k in range(0, len(flat), 2):
            a0, a1 = flat[k], flat[k + 1]
            # ざっくり: セクタ内の任意点が [c-hw, c+hw] に入るか
            pts = [a0, a1, (a0 + a1) / 2]
            for p in pts:
                if abs(norm(p - c)) <= hw:
                    return True
        return False
    fwd_hit = overlaps(0, 0, 0.0, FWD)
    rev_hit = overlaps(0, 0, 180.0, REV)
    print(f'\nL5 影響（obstacle_limiter）:')
    print(f'  前方コーン ±{FWD}deg と重なる: {fwd_hit}  '
          f'→ True なら前進が v_reverse(demo 0.25m/s) に制限される')
    print(f'  後方コーン ±{REV}deg と重なる: {rev_hit}  '
          f'→ True なら後退が v_reverse に制限される')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
