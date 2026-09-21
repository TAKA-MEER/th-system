#!/usr/bin/env python3
"""meas08_runaway_recorder.py の CSV を解析する（標準ライブラリのみ・ホストで動く）

  python3 meas08_analyze.py <csv> [<csv> ...] [--tape 3.00]

- 動作の種類ごと（停止／直進／その場旋回／弧）に、DRIVE_RUNAWAY の条件がどれだけ成立したか
- 比（実速度/指令）の定常値 ＝ wheel_radius_scale を疑う材料
- その場旋回中の実速度の平均（＝ Case A の誤発火の根）
- 停止・発進の直後に条件が成立し続けた最長時間（＝ 保持時間の下限の材料）
- 実測の到着ギャップと「古い」時間の割合（凍結窓の実際）
- 印（label）ごとの集計
- 比・保持時間・除外方式を振り直したときの発火回数（設定を決めるための掃引）
- --tape: 印「巻き尺始点」「巻き尺終点」間の /odom 距離と、巻き尺の実距離との比
"""
import csv
import math
import sys

ANG_THR = 0.05      # |角速度| がこれ以下なら「回っていない」
ZERO_THR_DEFAULT = 0.02


def load(path):
    meta = {}
    rows = []
    with open(path, encoding='utf-8') as f:
        first = f.readline()
        if first.startswith('#'):
            for kv in first.lstrip('# ').split():
                if '=' in kv:
                    k, v = kv.split('=', 1)
                    try:
                        meta[k] = float(v)
                    except ValueError:
                        pass
            header = f.readline()
        else:
            header = first
        r = csv.DictReader(f, fieldnames=header.strip().split(','))
        for d in r:
            def fl(k):
                v = d.get(k, '')
                return float(v) if v not in ('', None) else None
            rows.append({
                't': fl('t_s'), 'cmd_lin': fl('cmd_lin') or 0.0, 'cmd_ang': fl('cmd_ang') or 0.0,
                'fb_l': fl('fb_left') or 0.0, 'fb_r': fl('fb_right') or 0.0,
                'fb_mean': fl('fb_mean') or 0.0, 'fb_age': fl('fb_age_s'),
                'gap': fl('fb_gap_max_s'), 'fresh': int(fl('fresh') or 0),
                'cond': int(fl('condition') or 0), 'held': fl('held_s') or 0.0,
                'fire': int(fl('would_fire') or 0),
                'ox': fl('odom_x'), 'oy': fl('odom_y'), 'label': (d.get('label') or '').strip()})
    return meta, rows


def pct(vs, q):
    if not vs:
        return None
    vs = sorted(vs)
    return vs[min(len(vs) - 1, max(0, int(math.ceil(q * len(vs))) - 1))]


def fmt(v, n=3):
    return '-' if v is None else f'{v:.{n}f}'


def kind(r, zero_thr):
    lin, ang = abs(r['cmd_lin']), abs(r['cmd_ang'])
    if lin <= zero_thr and ang <= ANG_THR:
        return 'stop'
    if lin <= zero_thr:
        return 'spin'
    return 'straight' if ang <= ANG_THR else 'arc'


def cond_of(cmd_lin, fb_mean, ratio, zero_thr):
    c, f = abs(cmd_lin), abs(fb_mean)
    if c <= zero_thr:
        return f > zero_thr
    return f < c / ratio or f > c * ratio


def simulate(rows, ratio, hold, zero_thr, period, skip_spin=False):
    """今の実装（凍結つき）と同じ規則で発火の回数（立ち上がり）を数え直す"""
    held, firing, episodes = 0.0, False, 0
    for r in rows:
        if r['fresh']:
            cond = cond_of(r['cmd_lin'], r['fb_mean'], ratio, zero_thr)
            if skip_spin and abs(r['cmd_lin']) <= zero_thr and abs(r['cmd_ang']) > ANG_THR:
                cond = False
            held = held + period if cond else 0.0
        now = held >= hold
        if now and not firing:
            episodes += 1
        firing = now
    return episodes


def longest_run(rows, pred):
    best = cur = 0
    for r in rows:
        cur = cur + 1 if pred(r) else 0
        best = max(best, cur)
    return best


def analyze(path, tape):
    meta, rows = load(path)
    if not rows:
        print(f'{path}: 行がありません')
        return
    period = meta.get('period_s', 0.1)
    zero_thr = meta.get('zero_thr', ZERO_THR_DEFAULT)
    ratio0, hold0 = meta.get('ratio', 1.5), meta.get('hold_s', 0.5)
    print(f'\n===== {path} =====')
    print(f'記録: {len(rows)} 周期 / {rows[-1]["t"] - rows[0]["t"]:.0f} 秒。設定: 比 {ratio0}・停止閾値 {zero_thr}・'
          f'保持 {hold0} s・鮮度 {meta.get("stale_s", "?")} s')
    gaps = [r['gap'] for r in rows if r['gap'] is not None]
    nonfresh = sum(1 for r in rows if not r['fresh'])
    print(f'実測の到着ギャップ（周期内最大）: p99 {fmt(pct(gaps, .99))} s / 最大 {fmt(max(gaps) if gaps else None)} s ／ '
          f'「古い」（凍結）周期: {nonfresh} ({100 * nonfresh / len(rows):.1f} %)')
    print(f'記録上の DRIVE_RUNAWAY 発火（would_fire の立ち上がり）: '
          f'{simulate(rows, ratio0, hold0, zero_thr, period)} 回')

    print('\n--- 動作の種類ごと ---')
    print('種類      周期   条件成立  最長連続  発火   実速度/指令(定常 p10/p50/p90)')
    for k in ('stop', 'straight', 'spin', 'arc'):
        rs = [r for r in rows if kind(r, zero_thr) == k]
        if not rs:
            continue
        cond_n = sum(r['cond'] for r in rs)
        run = longest_run(rows, lambda r, k=k: kind(r, zero_thr) == k and r['cond']) * period
        fires = simulate(rs, ratio0, hold0, zero_thr, period)
        ratios = ''
        if k in ('straight',):
            st = [abs(r['fb_mean']) / abs(r['cmd_lin']) for i, r in enumerate(rs)
                  if i >= 10 and abs(rs[i - 10]['cmd_lin'] - r['cmd_lin']) < 0.02 and r['fresh']]
            ratios = f'{fmt(pct(st, .1), 2)} / {fmt(pct(st, .5), 2)} / {fmt(pct(st, .9), 2)}（{len(st)} 周期）'
        print(f'{k:<9} {len(rs):>5} {cond_n:>8} {run:>7.1f}s {fires:>5}   {ratios}')

    spin = [abs(r['fb_mean']) for r in rows if kind(r, zero_thr) == 'spin' and r['fresh']]
    if spin:
        bad = sum(1 for v in spin if v > zero_thr)
        print(f'\nその場旋回中の |左右平均| : p50 {fmt(pct(spin, .5))} / p95 {fmt(pct(spin, .95))} / 最大 {fmt(max(spin))} m/s'
              f' ／ 停止閾値 {zero_thr} 超え {bad}/{len(spin)} 周期（超えるほど Case A で誤発火しうる）')

    # 停止直後の尾（指令が非ゼロ→ゼロに落ちてから、実速度が停止閾値以下になるまで）
    tails, starts = [], []
    for i in range(1, len(rows)):
        a, b = rows[i - 1], rows[i]
        if abs(a['cmd_lin']) > zero_thr and abs(b['cmd_lin']) <= zero_thr:
            n = 0
            while i + n < len(rows) and abs(rows[i + n]['fb_mean']) > zero_thr and abs(rows[i + n]['cmd_lin']) <= zero_thr:
                n += 1
            tails.append((n * period, abs(a['cmd_lin'])))
        if abs(a['cmd_lin']) <= zero_thr and abs(b['cmd_lin']) > zero_thr:
            n = 0
            while i + n < len(rows) and rows[i + n]['cond'] and abs(rows[i + n]['cmd_lin']) > zero_thr:
                n += 1
            starts.append((n * period, abs(b['cmd_lin'])))
    if tails:
        print(f'\n停止指令のあと実速度が停止閾値を超えていた時間: 事象 {len(tails)} 件・中央 {fmt(pct([t for t, _ in tails], .5), 2)} s・'
              f'最大 {fmt(max(t for t, _ in tails), 2)} s（直前の指令速度 {fmt(max(v for _, v in tails), 2)} m/s のとき）')
    if starts:
        print(f'発進のあと条件が成立し続けた時間: 事象 {len(starts)} 件・中央 {fmt(pct([t for t, _ in starts], .5), 2)} s・'
              f'最大 {fmt(max(t for t, _ in starts), 2)} s')

    print('\n--- 印（label）ごと ---')
    seg, cur = {}, ''
    for r in rows:
        if r['label']:
            cur = r['label']
        seg.setdefault(cur or '(印なし)', []).append(r)
    for lab, rs in seg.items():
        cn = sum(r['cond'] for r in rs)
        run = longest_run(rs, lambda r: r['cond']) * period
        print(f'  {lab:<20} {len(rs):>5} 周期 / 条件成立 {cn:>4} / 最長連続 {run:.1f} s / '
              f'発火 {simulate(rs, ratio0, hold0, zero_thr, period)} 回')

    print('\n--- 設定を振り直したときの発火回数（全走行・立ち上がり。少ないほど誤発火しにくい）---')
    holds = [0.5, 0.8, 1.0, 1.5, 2.0]
    print('比    除外   ' + ''.join(f'hold {h:<4}' for h in holds))
    for ratio in (1.3, 1.5, 1.7, 2.0, 3.0):
        for skip in (False, True):
            print(f'{ratio:<5} {"旋回除外" if skip else "なし    "} ' +
                  ''.join(f'{simulate(rows, ratio, h, zero_thr, period, skip):<9}' for h in holds))

    if tape:
        # 印「巻き尺始点…」と、そのあとの最初の「巻き尺終点…」を対にして、対ごとに出す
        pairs, s = [], None
        for r in rows:
            if r['ox'] is None:
                continue
            if '巻き尺始点' in r['label']:
                s = r
            elif '巻き尺終点' in r['label'] and s is not None:
                pairs.append((s, r))
                s = None
        print('\n--- 巻き尺 ---')
        if not pairs:
            print('印「巻き尺始点」「巻き尺終点」の対が見つかりません')
        for k, (s, e) in enumerate(pairs, 1):
            od = math.hypot(e['ox'] - s['ox'], e['oy'] - s['oy'])
            print(f'{k} 本目 [{s["label"]} → {e["label"]}]: /odom の距離 {od:.3f} m ／ 巻き尺 {tape:.3f} m ／ '
                  f'実距離÷odom = {tape / od:.3f}（1.0 からのずれ {100 * (tape / od - 1):+.1f} %。'
                  f'wheel_radius_scale_max_dev は ±10 %）')

def main():
    args = sys.argv[1:]
    tape = None
    if '--tape' in args:
        i = args.index('--tape')
        tape = float(args[i + 1])
        del args[i:i + 2]
    if not args:
        print(__doc__)
        sys.exit(1)
    for p in args:
        analyze(p, tape)


if __name__ == '__main__':
    main()
