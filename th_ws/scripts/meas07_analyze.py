#!/usr/bin/env python3
"""制動・空走の解析（WP-MEAS-01 / N-12）

使い方:
    python3 meas07_analyze.py <csv> [巻尺の実測 m]

停止事象を自動で見つけ、初速・停止までの時間・実効減速度・積分距離を出す。

  ソフト非常停止: estop_ui が 0 → 1 になった時刻を起点（meas07_drive --mode soft-estop）
  空走          : estop_hw が 0 → 1 になった時刻を起点（物理非常停止）
  ランプ停止    : cmd が定常から下がり始めた時刻を起点（S 字ランプで 0 へ）
  即0停止       : cmd が定常から即 0 に落ちた（記録に非常停止の要求が無いもの）
  非常停止と同じ停止は 1 回しか数えない（find_events の docstring）

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
ZERO_EPS = 1e-6        # cmd がこれ以下なら 0 とみなす
ONSET_FRAC = 0.10       # 定常からこの割合だけ下がった最初の標本を起点にする。
                        # S 字の立ち上がりは緩く、0.3% でも約 0.1 秒の差が出る。10% は
                        # 車輪の速度が目に見えて落ち始める点で、R-66（手計算）と一致する
IMMEDIATE_MAX_S = 0.5   # 起点から 0 に着くまでがこれ以内なら S 字ランプではなく即 0
PLATEAU_TOL = 0.01      # 定常とみなす幅 m/s
PLATEAU_MIN_S = 1.0     # 定常がこれ以上続いた停止だけを「ランプ停止」に数える
DEDUP_BEFORE_S = 0.3    # 非常停止の要求の前後これだけ以内に起点がある cmd の落ち方は
DEDUP_AFTER_S = 1.0     #   その非常停止の結果なので「ランプ停止」に数えない
MIN_SAMPLES = 5         # 減速中の車輪標本がこれ未満なら、値は量子化の産物とみなす
                        # （/esp32/wheel_feedback は 10Hz。0.3m/s からの停止は
                        #   0.2〜0.4 秒＝2〜4 点しか無く、減速度を分解できない）


KINDS = ('ソフト非常停止', 'ランプ停止', '即0停止', '空走')


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


def _find_cmd_stops(cmd):
    """/cmd_vel が定常から下がって 0 に着いた停止を探す。

    返すのは (起点時刻, 定常の指令値, 起点から 0 に着くまでの秒) の一覧。
    起点は「定常から ONSET_FRAC 以上下がった最初の標本」の時刻。S 字ランプでは cmd が
    0 に着く時点で車輪はほぼ止まっているので、0 に着いた時刻を起点にしてはいけない
    （旧版の誤り）。定常から 1〜2 標本で 0 へ落ちる停止（ロックによる即 0 など）も
    同じ規則で、起点は落ち始めの標本になる。

    定常（PLATEAU_TOL 以内）が PLATEAU_MIN_S 以上続いていた停止だけを数える。
    押している間だけ出すジョグの上げ下げ・停止中の揺らぎは定常ではないので除く。
    """
    out = []
    for z in range(1, len(cmd)):
        if abs(cmd[z][1]) > ZERO_EPS or abs(cmd[z - 1][1]) <= ZERO_EPS:
            continue
        # 0 に着いた点から時間をさかのぼり、減少していく区間の頭を探す
        j = z - 1
        while j > 0 and abs(cmd[j - 1][1]) >= abs(cmd[j][1]) - 1e-9:
            j -= 1
        peak = max(abs(c[1]) for c in cmd[j:z])
        if peak < MIN_V0:
            continue
        # 定常の終わり（ピーク近傍の最後の標本）と、そこから前へ続く長さ
        p = z - 1
        while p > j and abs(cmd[p][1]) < peak - PLATEAU_TOL:
            p -= 1
        m = p
        while m > 0 and abs(cmd[m - 1][1]) >= peak - PLATEAU_TOL:
            m -= 1
        if cmd[p][0] - cmd[m][0] < PLATEAU_MIN_S:
            continue
        # 起点: 定常の終わり以降で、ピークから ONSET_FRAC 以上下がった最初の標本
        k = p
        while k < z and abs(cmd[k][1]) >= peak * (1.0 - ONSET_FRAC):
            k += 1
        out.append((cmd[k][0], peak, cmd[z][0] - cmd[k][0]))
    return out


def find_events(rows):
    """(種別, 起点時刻) の一覧。

    ソフト非常停止: /safety/estop_ui が 0→1 になった時刻（meas07_drive --mode soft-estop）
    空走          : /safety/estop_hw が 0→1 になった時刻（物理非常停止）
    ランプ停止    : 指令が定常から ONSET_FRAC 以上下がった時刻。0 に着くまでが
                    IMMEDIATE_MAX_S を超える（S 字ランプを通る通常停止）
    即0停止       : 同じく定常から 0 へ IMMEDIATE_MAX_S 以内に落ちた。記録に非常停止の
                    要求が無い（ロック・PAUSE など、要因が記録されていない即 0）

    非常停止の要求の直後に cmd が即 0 になる停止は、その非常停止の事象と同じものなので
    「即0停止」に数えない（二重に数えると、起点が遅れた粗い事象が並ぶ）。
    """
    ev = []
    estop_times = []
    for ts, v, _a, _b in rows.get('estop_ui', []):
        if v >= 0.5:
            ev.append(('ソフト非常停止', ts))
            estop_times.append(ts)
    for ts, v, _a, _b in rows.get('estop_hw', []):
        if v >= 0.5:
            ev.append(('空走', ts))
            estop_times.append(ts)
    for t0, _peak, fall_s in _find_cmd_stops(rows.get('cmd', [])):
        if fall_s <= IMMEDIATE_MAX_S:
            if any(te - DEDUP_BEFORE_S <= t0 <= te + DEDUP_AFTER_S for te in estop_times):
                continue
            ev.append(('即0停止', t0))
        else:
            ev.append(('ランプ停止', t0))
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
    for kind in KINDS:
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

    print('\n注意: brake_accel_mps2（障害物・非常停止の経路の減速度）に入れるのは')
    print('      「ソフト非常停止」の最悪値（meas07_drive --mode soft-estop。停止の要求の時刻から')
    print('      数えるので、安全監視の周期ぶん（約 0.1 秒）も含む＝安全側）。')
    print('      「ランプ停止」は通常の S 字ランプ（normal_accel_mps2・normal_jerk_mps3）を通った')
    print('      停止で、brake_accel_mps2 ではない。「即0停止」は記録に要因の無い cmd の即 0')
    print('      （参考）。「空走」は N-12（intrusion_budget_m を消費する側）で別物。')


if __name__ == '__main__':
    main()
