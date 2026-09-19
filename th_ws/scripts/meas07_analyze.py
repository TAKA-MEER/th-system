#!/usr/bin/env python3
"""制動・空走の解析（WP-MEAS-01 / N-12）

使い方:
    python3 meas07_analyze.py <csv> [巻尺の実測 m]

停止事象を自動で見つけ、初速・停止までの時間・実効減速度・積分距離を出す。

  ランプ停止: cmd が正 → 0 になった時刻を起点
  空走      : estop_hw が 0 → 1 になった時刻を起点

**採るのは最も悪い値**（wp0 §6.3 FMEA①: 平均を採ると制動距離を短く見積もり、
以降の全距離が危険側にずれる）。

距離の正は巻尺である。WheelFeedback に dt が無く、積分は到着時刻の差に頼るため
WiFi 遅延がそのまま誤差になる（recorder のコメント参照）。
"""
import sys
from collections import defaultdict

STOPPED_MPS = 0.02      # これ以下を停止とみなす
STOPPED_HOLD_S = 0.30   # 停止が続いたら確定（ノイズで途中 0 になるのを弾く）
MIN_V0 = 0.10           # これ未満の初速の事象は無視する
MIN_SAMPLES = 5         # 減速中の車輪標本がこれ未満なら、値は量子化の産物とみなす
                        # （/esp32/wheel_feedback は 10Hz。0.3m/s からの停止は
                        #   0.2〜0.4 秒＝2〜4 点しか無く、減速度を分解できない）


def read(path):
    rows = defaultdict(list)
    with open(path, encoding='utf-8') as f:
        header = f.readline()
        if not header.startswith('topic,'):
            sys.exit(f'CSV の書式が違う: {path}')
        for line in f:
            p = line.rstrip('\n').split(',')
            if len(p) < 5:
                continue
            try:
                rows[p[0]].append((int(p[1]) / 1e9, float(p[2]), float(p[3]), float(p[4])))
            except ValueError:
                continue
    return rows


def speed_at(wheel, t):
    """t 以前で最も近い車輪速度。無ければ None。"""
    best = None
    for ts, _l, _r, v in wheel:
        if ts <= t:
            best = v
        else:
            break
    return best


def find_events(rows):
    """(種別, 起点時刻) の一覧。"""
    ev = []
    prev = 0.0
    for ts, v, _a, _b in rows.get('cmd', []):
        if prev > MIN_V0 and abs(v) <= 1e-6:
            ev.append(('ランプ停止', ts))
        prev = abs(v)
    for ts, v, _a, _b in rows.get('estop_hw', []):
        if v >= 0.5:
            ev.append(('空走', ts))
    ev.sort(key=lambda e: e[1])
    return ev


def measure(wheel, t0):
    """起点 t0 からの停止時間と積分距離。"""
    v0 = speed_at(wheel, t0)
    if v0 is None or abs(v0) < MIN_V0:
        return None
    dist = 0.0
    n_samples = 0
    n_at_stop = 0
    t_prev = t0
    v_prev = abs(v0)
    stopped_since = None
    for ts, _l, _r, v in wheel:
        if ts <= t0:
            continue
        v = abs(v)
        dt = ts - t_prev
        if dt <= 0 or dt > 1.0:      # 欠測をまたいだら積分しない
            t_prev, v_prev = ts, v
            continue
        dist += (v_prev + v) / 2.0 * dt      # 台形積分
        t_prev, v_prev = ts, v
        n_samples += 1
        if v <= STOPPED_MPS:
            if stopped_since is None:
                stopped_since = ts
                n_at_stop = n_samples
            elif ts - stopped_since >= STOPPED_HOLD_S:
                t_stop = stopped_since - t0
                if t_stop <= 0:
                    return None
                return abs(v0), t_stop, dist, abs(v0) / t_stop, n_at_stop
        else:
            stopped_since = None
    return None


def main():
    if len(sys.argv) < 2:
        sys.exit('使い方: meas07_analyze.py <csv> [巻尺の実測 m]')
    rows = read(sys.argv[1])
    wheel = rows.get('wheel', [])
    if not wheel:
        sys.exit('wheel の行が無い。ESP32 が繋がっていたか確認すること')

    events = find_events(rows)
    if not events:
        sys.exit('停止事象が見つからない。cmd が 0 になった／estop_hw が立った記録が無い')

    print(f'{"種別":<10}{"初速 m/s":>10}{"停止まで s":>12}{"積分距離 m":>12}{"実効減速度 m/s2":>18}')
    print('-' * 62)
    results = []
    for kind, t0 in events:
        m = measure(wheel, t0)
        if m is None:
            continue
        v0, t_stop, dist, decel, n = m
        results.append((kind, v0, t_stop, dist, decel, n))
        warn = '  ← 標本 %d 点。粗すぎる' % n if n < MIN_SAMPLES else ''
        print(f'{kind:<10}{v0:>10.3f}{t_stop:>12.3f}{dist:>12.3f}{decel:>18.3f}{warn}')

    if not results:
        sys.exit('停止事象はあったが、初速が足りず測定にならなかった')

    print()
    for kind in ('ランプ停止', '空走'):
        sub = [r for r in results if r[0] == kind]
        if not sub:
            continue
        worst_decel = min(r[4] for r in sub)          # 最も減速しない＝最悪
        worst_dist = max(r[3] for r in sub)
        print(f'{kind}: {len(sub)} 本  '
              f'最悪の減速度 {worst_decel:.3f} m/s2  最長の距離 {worst_dist:.3f} m')

    if len(sys.argv) >= 3:
        tape = float(sys.argv[2])
        odo = max(r[3] for r in results)
        print(f'\n巻尺 {tape:.3f} m / 積分 {odo:.3f} m  '
              f'差 {tape - odo:+.3f} m（{(tape - odo) / tape * 100:+.1f} %）')
        print('★ registry へ入れるのは巻尺側。積分はスリップを拾えない')

    thin = [r for r in results if r[5] < MIN_SAMPLES]
    if thin:
        print(f'\n★ 警告: {len(thin)}/{len(results)} 本は減速中の標本が {MIN_SAMPLES} 点未満で、')
        print('  減速度を分解できていない。**この値を registry へ入れてはいけない。**')
        print('  速度を上げて測り直すこと（1.0 m/s なら停止に約 0.7 秒＝7 点）。')

    print('\n注意: brake_accel_mps2 に入れるのは「ランプ停止」の最悪値。')
    print('      「空走」は N-12（intrusion_budget_m を消費する側）で、別物。')


if __name__ == '__main__':
    main()
