#!/usr/bin/env python3
"""ESP32 の受信ギャップの 3 択切り分け（手順書 §4.3 の 2）

同じ時計で採った 2 つの記録を突き合わせ、跳ねの原因を 3 つに切り分ける。

  A. 電線上もギャップ ＋ TCP 再送あり  → 伝送方式（TCP）の問題。AP では直らない
  B. 電線上もギャップ ＋ 再送なし      → ESP32 が送っていない。ファーム側の問題
  C. 電線上はギャップ無し（ROS だけ）  → esp32_bridge（Python）の問題。ソフトで直る

usage: meas05_analyze.py <arrivals.csv> <tcpdump.txt> [閾値ms]
"""
import re
import sys
from collections import defaultdict

# ESP32 は PC の 8766 に**接続しにいく側**なので、下り（ESP32→PC）は dst port 8766。
WS_PORT = 8766
LINE_RE = re.compile(
    r'^(?P<t>\d+\.\d+) IP '
    r'(?P<src>[\d.]+)\.(?P<sport>\d+) > (?P<dst>[\d.]+)\.(?P<dport>\d+): '
    r'Flags \[(?P<flags>[^\]]*)\].*\blength (?P<len>\d+)$'
)
# seq は LINE_RE に含めない。任意グループにすると遅延量指定子に食われて
# 常にスキップされ、再送が 1 件も検出されなくなる（合成データで確認済み）。
SEQ_RE = re.compile(r'\bseq (\d+)')


def read_arrivals(path):
    """topic -> [t_sec]。recv_ns は CLOCK_REALTIME。"""
    out = defaultdict(list)
    with open(path) as fh:
        next(fh, None)
        for line in fh:
            parts = line.strip().split(',')
            if len(parts) != 2:
                continue
            out[parts[0]].append(int(parts[1]) / 1e9)
    return out


def read_wire(path):
    """下り（ESP32→PC）のデータ segment: [(t, seq_start, length, is_retx)]"""
    segs = []
    # seq は**コネクションごと**に管理する。tcpdump の相対 seq は接続のたびに
    # 1 に戻るため、全体で 1 つの max_seq を持つと再接続後の全 segment が
    # 再送と誤判定される（実 tcpdump 出力で 5 件中 4 件を誤判定して発覚）。
    # ESP32 は 5 分ごとに強制再接続する（ws_link.cpp の FORCE_RECONNECT_MS）。
    max_seq = defaultdict(lambda: -1)
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            m = LINE_RE.match(line)
            if not m or int(m.group('dport')) != WS_PORT:
                continue
            conn = (m.group('src'), m.group('sport'))
            if 'S' in m.group('flags'):   # SYN = 新しいコネクション
                max_seq[conn] = -1
            length = int(m.group('len'))
            if length == 0:            # ACK のみ。データではない
                continue
            ms = SEQ_RE.search(line)
            seq0 = int(ms.group(1)) if ms else -1
            # 再送 = 同一コネクションで既に見た seq 以下を再び送ってきたデータ
            is_retx = 0 <= seq0 <= max_seq[conn]
            if seq0 > max_seq[conn]:
                max_seq[conn] = seq0
            segs.append((float(m.group('t')), seq0, length, is_retx))
    return segs


def gaps(times, thresh_s):
    return [(times[i - 1], times[i], times[i] - times[i - 1])
            for i in range(1, len(times)) if times[i] - times[i - 1] > thresh_s]


def pct(values, q):
    if not values:
        return float('nan')
    s = sorted(values)
    return s[min(len(s) - 1, int(q * len(s)))]


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    thresh_s = (float(sys.argv[3]) if len(sys.argv) > 3 else 250.0) / 1000.0

    arr = read_arrivals(sys.argv[1])
    segs = read_wire(sys.argv[2])
    ros_t = arr.get('esp32', [])
    wire_t = [s[0] for s in segs]

    if len(ros_t) < 10:
        print(f'ERROR: ESP32 の ROS 受信が {len(ros_t)} 件しかない。'
              f'ブリッジが動いていないか、ESP32 が繋がっていない', file=sys.stderr)
        return 1

    print('=' * 64)
    print('■ 到着間隔のまとめ（生の受信時刻ベース。窓の分位点ではない）')
    print('=' * 64)
    for name, times in (('ESP32 (ROS)', ros_t),
                        ('ESP32 (電線上)', wire_t),
                        ('LiDAR (ROS)', arr.get('lidar', []))):
        if len(times) < 2:
            print(f'{name:<16} 記録なし')
            continue
        d = [(times[i] - times[i - 1]) * 1000 for i in range(1, len(times))]
        print(f'{name:<16} n={len(times):<6} p50={pct(d, .50):7.1f}ms  '
              f'p99={pct(d, .99):7.1f}ms  max={max(d):7.1f}ms  '
              f'>{thresh_s*1000:.0f}ms={sum(1 for x in d if x > thresh_s*1000)}回')

    ros_gaps = gaps(ros_t, thresh_s)
    n_retx = sum(1 for s in segs if s[3])
    print()
    print(f'TCP 再送（下り・データ segment）: {n_retx} / {len(segs)} 件')

    if not ros_gaps:
        print(f'\n跳ね（>{thresh_s*1000:.0f}ms）は 1 回も出なかった。'
              '記録時間を延ばすか、跳ねが出た条件を再現すること。')
        return 0

    print()
    print('=' * 64)
    print(f'■ 跳ね {len(ros_gaps)} 回の切り分け')
    print('=' * 64)
    t0 = ros_t[0]
    verdicts = []
    for a, b, g in ros_gaps:
        # 電線上にも同じ長さのギャップがあったかを見る（件数ではなく最大間隔）。
        pad = 0.05
        w = [t for t in wire_t if a - pad <= t <= b + pad]
        if len(w) < 2:
            wire_gap = g          # 電線上も沈黙していた
        else:
            wire_gap = max(w[i] - w[i - 1] for i in range(1, len(w)))
        n_rx = sum(1 for s in segs if a - pad <= s[0] <= b + pad and s[3])
        if wire_gap >= 0.7 * g:   # 電線上も止まっていた
            v = ('A（TCP 再送）' if n_rx > 0 else 'B（ESP32 が送っていない）')
        else:
            v = 'C（esp32_bridge の詰まり）'
        verdicts.append(v[0])
        print(f't+{a-t0:7.1f}s  ROS {g*1000:6.1f}ms  '
              f'電線上 {wire_gap*1000:6.1f}ms  再送 {n_rx:2d} 件  → {v}')

    print()
    print('■ 結論')
    for code, label, action in (
            ('A', '伝送方式（TCP）の問題',
             'AP を替えても直らない。UDP 化か、WS のフレーム分割の見直し'),
            ('B', 'ESP32 が送っていない',
             'ファーム側。IMU の I2C 読みか制御ループの詰まりを疑う'),
            ('C', 'esp32_bridge（Python）の詰まり',
             'ソフトで直せる。50Hz の _drain_rx_queue と GIL 競合を疑う')):
        n = verdicts.count(code)
        if n:
            print(f'  {code}: {n}/{len(verdicts)} 回 — {label}\n     → {action}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
