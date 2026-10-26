#!/usr/bin/env python3
"""昇降機上部の揺れ解析。

  analyze.py --info NAME   … 記録の中身（トピック・件数・周期・加速度の大きさ）を見るだけ
  analyze.py NAME [NAME..] … 解析して /root/th_data/vibe/NAME/ に summary.md / summary.json / *.png を出す

座標の決め方（スマホの置き方に依存しないため）:
  1. 記録開始直後の静止区間の平均加速度を重力とみなし、それが +z になるよう回す
  2. 水平面内の向きは、オドメトリの前後加速度と最もよく一致する方向を「前(x)」とする
     （直進の加減速が入っていない記録では決まらないので --yaw で与える）
  台車の BNO055（/esp32/imu_data）も同じ手順で揃え、上部と下部を比べる。
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy import signal

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

ROOT = '/root/th_data/vibe'
FS = 100.0          # 解析用の等間隔サンプリング [Hz]
G = 9.80665
HP_HZ = 0.5         # 「揺れ」とみなす下限周波数。これより遅い成分は走行の加減速そのもの
LP_REF_HZ = 2.0     # 軸合わせに使う低周波成分


# ---------------------------------------------------------------- 読み込み
def read_bag(path):
    """{topic: (type, [(recv_ns, msg), ...])}"""
    if not os.path.isdir(path):
        return {}
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {k: (v, []) for k, v in types.items()}
    cls = {}
    while r.has_next():
        topic, data, t = r.read_next()
        typ = types[topic]
        if typ not in cls:
            try:
                cls[typ] = get_message(typ)
            except Exception:
                cls[typ] = None
        if cls[typ] is None:
            continue
        out[topic][1].append((t, deserialize_message(data, cls[typ])))
    return out


def stamp_ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


def imu_msgs_to_arrays(msgs):
    recv = np.array([t for t, _ in msgs], dtype=np.int64)
    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in msgs])
    gyr = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in msgs])
    hdr = np.array([stamp_ns(m) for _, m in msgs], dtype=np.int64)
    return recv, hdr, acc, gyr


def read_phone(path):
    """phone_rec.py の CSV。{topic: (recv, hdr, acc, gyr)}"""
    out = {}
    if not os.path.isdir(path):
        return out
    for fn in sorted(os.listdir(path)):
        if not fn.endswith('.csv'):
            continue
        with open(os.path.join(path, fn)) as fp:
            topic = fp.readline().split(' ', 2)[2].strip()
        d = np.genfromtxt(os.path.join(path, fn), delimiter=',', skip_header=2, usecols=range(9),
                          dtype=None, encoding=None)
        if d.size == 0:
            out[topic] = None
            continue
        d = np.atleast_1d(d)
        cols = [d[f'f{i}'] for i in range(9)]
        out[topic] = (cols[0].astype(np.int64), cols[1].astype(np.int64),
                      np.column_stack(cols[3:6]).astype(float), np.column_stack(cols[6:9]).astype(float))
    return out


def imu_series(arrays, use_header=True):
    """Imu 列を (t[s, PC受信時刻基準], acc[N,3], gyr[N,3], 時刻情報) に。

    スマホの header.stamp はスマホの時計。間隔はそちらが正確（Wi-Fi の揺らぎを含まない）
    なので、間隔は header、絶対時刻は「受信 - header」の下側 5% 点でずらして PC 時刻に載せる。
    """
    recv, hdr, acc, gyr = arrays
    info = {}
    # 受信順はヘッダ順と入れ替わることがあるので、ヘッダで並べてから重複が無いかを見る
    ok = use_header and np.all(hdr > 0) and np.all(np.diff(np.sort(hdr)) > 0)
    if ok:
        lat = (recv - hdr) / 1e9
        off = np.percentile(lat, 5)
        info['clock_offset_s'] = float(off)           # PC - スマホ（+ 最小遅延）
        info['latency_spread_s'] = float(np.percentile(lat, 95) - off)
        t = hdr / 1e9 + off
        info['timebase'] = 'header'
    else:
        t = recv / 1e9
        info['timebase'] = 'recv'
    order = np.argsort(t)
    t, acc, gyr = t[order], acc[order], gyr[order]
    dt = np.diff(t)
    info['n'] = len(t)
    info['rate_hz'] = float(1.0 / np.median(dt)) if len(dt) else 0.0
    info['max_gap_s'] = float(dt.max()) if len(dt) else 0.0
    mag = np.linalg.norm(acc, axis=1)
    info['acc_norm_median'] = float(np.median(mag))
    # 単位が g で来ていたら m/s^2 に直す
    if 0.5 < info['acc_norm_median'] < 2.0:
        acc = acc * G
        info['acc_unit'] = 'g→m/s^2 に換算'
    elif info['acc_norm_median'] < 0.5:
        info['acc_unit'] = '重力を含まない（user acceleration）'
    else:
        info['acc_unit'] = 'm/s^2'
    return t, acc, gyr, info


def resample(t_src, x_src, t):
    x_src = np.atleast_2d(x_src.T).T
    return np.column_stack([np.interp(t, t_src, x_src[:, i]) for i in range(x_src.shape[1])])


def butter(x, hz, kind):
    sos = signal.butter(2, hz, btype=kind, fs=FS, output='sos')
    return signal.sosfiltfilt(sos, x, axis=0)


# ---------------------------------------------------------------- 軸合わせ
def rot_to_z(g):
    """ベクトル g を +z に向ける回転行列。"""
    a = g / np.linalg.norm(g)
    b = np.array([0.0, 0.0, 1.0])
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def rot_z(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])  # 水平の (cos th, sin th) 方向を +x に


def align(acc, gyr, static, a_ref, yaw_deg=None, wz_ref=None):
    """重力で傾きを、前後加速度で向きを合わせる。返り値は base_link 相当（x前・y左・z上）。"""
    has_gravity = np.linalg.norm(acc[static].mean(axis=0)) > 5.0
    if has_gravity:
        R1 = rot_to_z(acc[static].mean(axis=0))
    else:
        R1 = np.eye(3)  # 重力が抜かれた加速度。傾きは決められないので画面が上向き前提
    a1, g1 = acc @ R1.T, gyr @ R1.T
    note = {}
    if yaw_deg is not None:
        th = np.radians(yaw_deg)
        note['yaw_source'] = '指定'
    else:
        h = butter(a1[:, :2] - a1[static, :2].mean(axis=0), LP_REF_HZ, 'low')
        m = np.abs(a_ref) > 0.03
        if m.sum() < FS * 1.0:
            raise SystemExit('前後の加減速が少なく向きを決められない。--yaw で与えるか、直進の発進・停止を含めて記録する')
        # 回帰係数ではなく共分散の向きを使う（係数だとノイズだけの軸に大きな値が乗る）
        cov = h[m].T @ a_ref[m]
        th = np.arctan2(cov[1], cov[0])
        pred = h[m] @ np.array([np.cos(th), np.sin(th)])
        note['yaw_source'] = '前後加速度から推定'
        note['yaw_fit_corr'] = float(np.corrcoef(pred, a_ref[m])[0, 1])
        # 前後加速度の大きさがオドメトリと合っているか。1 から大きく外れたら単位を疑う
        note['gain_vs_odom'] = float(pred @ a_ref[m] / (a_ref[m] @ a_ref[m]))
    R = rot_z(th) @ R1
    a, g = acc @ R.T, gyr @ R.T
    # 上下を取り違えていないか（z 軸まわりの角速度がオドメトリと同符号か）
    if wz_ref is not None and np.std(wz_ref) > 0.05:
        note['wz_corr'] = float(np.corrcoef(butter(g[:, 2], LP_REF_HZ, 'low'), wz_ref)[0, 1])
    note['yaw_deg'] = float(np.degrees(th))
    note['gravity_in_accel'] = bool(has_gravity)
    if has_gravity:
        a[:, 2] -= np.linalg.norm(acc[static].mean(axis=0))
    return a, g, note


# ---------------------------------------------------------------- 指標
def stats(x):
    return {'rms': float(np.sqrt(np.mean(x ** 2))), 'peak': float(np.max(np.abs(x))),
            'p2p': float(np.ptp(x))}


def tilt_deg(gyr_axis):
    """角速度を積分した傾き。ドリフトを消すため前後で高域通過する。"""
    w = butter(gyr_axis, HP_HZ / 2, 'high')
    return np.degrees(butter(np.cumsum(w) / FS, HP_HZ / 2, 'high'))


def psd_peaks(f, p, n=3, fmin=0.5, fmax=20.0):
    m = (f >= fmin) & (f <= fmax)
    idx, _ = signal.find_peaks(p[m])
    idx = idx[np.argsort(p[m][idx])[::-1][:n]]          # 強い順
    idx = [i for i in idx if p[m][i] >= 0.1 * p[m][idx[0]]]   # 最大の 1 割未満は出さない
    return [{'hz': float(f[m][i]), 'psd': float(p[m][i])} for i in idx]


def classify(v, w, dv):
    """各時刻を 停止 / 加減速 / 旋回 / 定速 に分ける（優先順はこの逆: 加減速 > 旋回 > 定速）。"""
    cls = np.full(len(v), 'cruise', dtype=object)
    stop = (np.abs(v) < 0.02) & (np.abs(w) < 0.05)
    cls[np.abs(w) > 0.15] = 'turn'
    cls[np.abs(dv) > 0.05] = 'accel'
    cls[stop] = 'stop'
    return cls


LABEL = {'stop': '停止', 'accel': '加速・減速', 'turn': '旋回', 'cruise': '定速直進'}


def settle_events(t, v, ahp, stopmask):
    """走行から停止に移った直後の揺れ残り（振幅と収まるまでの時間）。"""
    ev = []
    edges = np.where(np.diff(stopmask.astype(int)) == 1)[0]
    for i in edges:
        if i < FS * 1.0 or not np.any(np.abs(v[int(i - FS):i]) > 0.05):
            continue
        j = min(len(t), i + int(FS * 5))
        mag = np.linalg.norm(ahp[i:j, :2], axis=1)
        env = butter(np.abs(mag), 2.0, 'low') if len(mag) > 20 else mag
        peak = float(mag.max())
        below = np.where(env < max(0.1 * peak, 0.05))[0]
        ev.append({'t': float(t[i] - t[0]), 'peak_mps2': peak,
                   'settle_s': float(below[0] / FS) if len(below) else None})
    return ev


# ---------------------------------------------------------------- 本体
def imu_line(inf):
    line = (f"  {inf['rate_hz']:.1f} Hz  最大間隔 {inf['max_gap_s']*1000:.0f} ms"
            f"  |a| 中央値 {inf['acc_norm_median']:.2f} ({inf['acc_unit']})  時刻 {inf['timebase']}")
    if 'clock_offset_s' in inf:
        line += f"  時計差 {inf['clock_offset_s']:+.3f}s 遅延幅 {inf['latency_spread_s']*1000:.0f}ms"
    return line


def info(name):
    print('== phone')
    ph = read_phone(os.path.join(ROOT, name, 'phone'))
    if not ph:
        print('  (記録なし)')
    for k, arr in ph.items():
        n = 0 if arr is None else len(arr[0])
        print(f'  {k:32s} {n:6d} 件' + (imu_line(imu_series(arr)[3]) if n > 1 else ''))
    for side in ('robot',):
        bag = read_bag(os.path.join(ROOT, name, side))
        print(f'== {side}')
        if not bag:
            print('  (記録なし)')
        for k, (typ, msgs) in sorted(bag.items()):
            line = f'  {k:32s} {typ:36s} {len(msgs):6d} 件'
            if typ == 'sensor_msgs/msg/Imu' and len(msgs) > 1:
                _, _, _, inf = imu_series(imu_msgs_to_arrays(msgs), use_header=False)
                line += imu_line(inf)
            print(line)


def analyze(name, yaw_phone, yaw_base, static_s):
    d = os.path.join(ROOT, name)
    phone, robot = read_phone(os.path.join(d, 'phone')), read_bag(os.path.join(d, 'robot'))
    cands = [(len(v[0]), k) for k, v in phone.items() if v is not None and len(v[0]) > 100]
    if not cands:
        raise SystemExit(f'{name}: スマホの Imu が記録されていない')
    tp = max(cands)[1]
    t_p, acc_p, gyr_p, inf_p = imu_series(phone[tp], use_header=True)

    odom = robot.get('/odom', (None, []))[1]
    if not odom:
        raise SystemExit(f'{name}: /odom が記録されていない（機体側の記録が空）')
    t_o = np.array([t for t, _ in odom]) / 1e9
    vo = np.array([[m.twist.twist.linear.x, m.twist.twist.angular.z] for _, m in odom])

    t0, t1 = max(t_p[0], t_o[0]), min(t_p[-1], t_o[-1])
    if t1 - t0 < 5:
        raise SystemExit(f'{name}: スマホと機体の記録が重なる時間が 5 秒未満')
    t = np.arange(t0, t1, 1 / FS)
    v, w = resample(t_o, vo, t).T
    v_lp = butter(v, LP_REF_HZ, 'low')
    a_ref = np.gradient(v_lp, 1 / FS)
    w_lp = butter(w, LP_REF_HZ, 'low')

    cmd = robot.get('/cmd_vel', (None, []))[1]
    if cmd:
        tc = np.array([x for x, _ in cmd]) / 1e9
        vc = resample(tc, np.array([[m.linear.x, m.angular.z] for _, m in cmd]), t)
    else:
        vc = None

    static = np.arange(len(t)) < int(FS * static_s)
    static &= (np.abs(v) < 0.01) & (np.abs(w) < 0.02)
    if static.sum() < FS * 0.5:
        raise SystemExit(f'{name}: 記録開始 {static_s} 秒に静止区間が無い。走り出す前に数秒置いてから記録する')

    A_p, W_p, note_p = align(resample(t_p, acc_p, t), resample(t_p, gyr_p, t), static, a_ref, yaw_phone, w_lp)

    sides = {'top': (A_p, W_p, note_p, inf_p)}
    if robot.get('/esp32/imu_data', (None, []))[1]:
        t_b, acc_b, gyr_b, inf_b = imu_series(imu_msgs_to_arrays(robot['/esp32/imu_data'][1]), use_header=False)
        try:
            A_b, W_b, note_b = align(resample(t_b, acc_b, t), resample(t_b, gyr_b, t), static, a_ref, yaw_base, w_lp)
            sides['base'] = (A_b, W_b, note_b, inf_b)
        except SystemExit as e:
            print(f'台車 IMU は使わない: {e}', file=sys.stderr)

    cls = classify(v_lp, w_lp, a_ref)
    res = {'name': name, 'phone_topic': tp, 'duration_s': float(t1 - t0), 'sides': {}, 'classes': {}}
    hp = {}
    for side, (A, W, note, inf) in sides.items():
        ahp = butter(A, HP_HZ, 'high')
        hp[side] = ahp
        roll, pitch = tilt_deg(W[:, 0]), tilt_deg(W[:, 1])
        f, Px = signal.welch(ahp[~static, 0], fs=FS, nperseg=int(FS * 8))
        _, Py = signal.welch(ahp[~static, 1], fs=FS, nperseg=int(FS * 8))
        res['sides'][side] = {'align': note, 'stream': inf,
                              'psd_peaks_x': psd_peaks(f, Px), 'psd_peaks_y': psd_peaks(f, Py),
                              'settle': settle_events(t, v_lp, ahp, cls == 'stop')}
        hp[side + '_tilt'] = (roll, pitch)
        hp[side + '_psd'] = (f, Px, Py)
        hp[side + '_raw'] = A
        for c in LABEL:
            m = cls == c
            if m.sum() < FS * 0.5:
                continue
            res['classes'].setdefault(c, {'seconds': float(m.sum() / FS)})[side] = {
                'acc_x': stats(A[m, 0]), 'acc_y': stats(A[m, 1]), 'acc_z': stats(A[m, 2]),
                'vib_x': stats(ahp[m, 0]), 'vib_y': stats(ahp[m, 1]), 'vib_z': stats(ahp[m, 2]),
                'roll_deg': stats(roll[m]), 'pitch_deg': stats(pitch[m])}

    # 指令に対する遅れ: 指令（または実速度）の加速度と上部の前後加速度の相互相関
    ref = np.gradient(butter(vc[:, 0], LP_REF_HZ, 'low'), 1 / FS) if vc is not None else a_ref
    top_lp = butter(A_p[:, 0], 5.0, 'low')
    lags = np.arange(-int(FS), int(FS) + 1)
    xc = [np.corrcoef(np.roll(ref, k)[int(FS):-int(FS)], top_lp[int(FS):-int(FS)])[0, 1] for k in lags]
    if np.all(np.isnan(xc)):   # 前後の加減速が無い記録（その場旋回など）
        res['cmd_to_top_lag_s'] = res['cmd_to_top_corr'] = None
    else:
        k = int(np.nanargmax(xc))
        res['cmd_to_top_lag_s'] = float(lags[k] / FS)
        res['cmd_to_top_corr'] = float(xc[k])
    res['cmd_source'] = '/cmd_vel' if vc is not None else '/odom（/cmd_vel 無し）'

    with open(os.path.join(d, 'summary.json'), 'w') as fp:
        json.dump(res, fp, ensure_ascii=False, indent=1)
    plot(d, t - t[0], v, w, vc, cls, sides, hp)
    write_md(d, res)
    print(open(os.path.join(d, 'summary.md')).read())
    return res


COLOR = {'stop': '#dddddd', 'accel': '#ffd7a8', 'turn': '#c9e4ff', 'cruise': '#d8f0d0'}


def shade(ax, t, cls):
    i = 0
    while i < len(cls):
        j = i
        while j < len(cls) and cls[j] == cls[i]:
            j += 1
        ax.axvspan(t[i], t[min(j, len(t) - 1)], color=COLOR[cls[i]], lw=0)
        i = j


def plot(d, t, v, w, vc, cls, sides, hp):
    plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']
    fig, ax = plt.subplots(5, 1, figsize=(14, 13), sharex=True)
    for a in ax:
        shade(a, t, cls)
    ax[0].plot(t, v, label='v 実速度 [m/s]')
    if vc is not None:
        ax[0].plot(t, vc[:, 0], '--', label='v 指令')
    ax[0].plot(t, w, label='ω 実 [rad/s]', alpha=.7)
    for i, (k, lab) in enumerate([(0, '前後'), (1, '左右'), (2, '上下')]):
        for side, sty in (('top', '-'), ('base', '-')):
            if side + '_raw' in hp:
                ax[i + 1].plot(t, hp[side + '_raw'][:, k], sty, lw=.7, alpha=.85 if side == 'top' else .6,
                               label=f'{"上部(スマホ)" if side == "top" else "台車(BNO055)"} {lab}加速度 [m/s²]')
    for side in ('top', 'base'):
        if side + '_tilt' in hp:
            r, p = hp[side + '_tilt']
            nm = '上部' if side == 'top' else '台車'
            ax[4].plot(t, p, lw=.8, label=f'{nm} ピッチ [deg]')
            ax[4].plot(t, r, lw=.8, label=f'{nm} ロール [deg]')
    for a in ax:
        a.legend(loc='upper right', fontsize=8)
        a.grid(alpha=.3)
    ax[-1].set_xlabel('時間 [s]　背景色: 灰=停止 橙=加減速 青=旋回 緑=定速')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'timeseries.png'), dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
    for side in ('top', 'base'):
        if side + '_psd' in hp:
            f, Px, Py = hp[side + '_psd']
            nm = '上部' if side == 'top' else '台車'
            ax[0].semilogy(f, Px, label=nm)
            ax[1].semilogy(f, Py, label=nm)
    for a, ttl in zip(ax, ('前後', '左右')):
        a.set_xlim(0, 20)
        a.set_title(f'{ttl}加速度のパワースペクトル（{HP_HZ} Hz 以上）')
        a.set_xlabel('周波数 [Hz]')
        a.set_ylabel('(m/s²)²/Hz')
        a.grid(alpha=.3, which='both')
        a.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'psd.png'), dpi=110)
    plt.close(fig)


def write_md(d, r):
    L = [f"# 揺れ測定 {r['name']}", '',
         f"- 解析した長さ {r['duration_s']:.1f} 秒 / スマホのトピック `{r['phone_topic']}`"]
    for side, s in r['sides'].items():
        nm = '上部(スマホ)' if side == 'top' else '台車(BNO055)'
        st, al = s['stream'], s['align']
        L.append(f"- {nm}: {st['rate_hz']:.0f} Hz・最大欠け {st['max_gap_s']*1000:.0f} ms・{st['acc_unit']}・"
                 f"向き {al['yaw_deg']:.0f}°（{al['yaw_source']}"
                 + (f"・一致度 {al['yaw_fit_corr']:.2f}" if 'yaw_fit_corr' in al else '')
                 + (f"・オドメトリ比 {al['gain_vs_odom']:.2f}" if 'gain_vs_odom' in al else '')
                 + (f"・旋回の符号一致 {al['wz_corr']:+.2f}" if 'wz_corr' in al else '') + '）')
    for side, s in r['sides'].items():
        gn = s['align'].get('gain_vs_odom')
        if gn is not None and not 0.6 < gn < 1.6:
            L.append(f"- **{'上部' if side == 'top' else '台車'}の前後加速度がオドメトリの {gn:.2f} 倍。単位か取り付けを疑う**"
                     "（重力抜きの g 単位なら約 0.1 倍に出る。スマホが滑っていても外れる）")
    if 'base' in r['sides'] and r['sides']['base']['stream']['rate_hz'] < 40:
        fb = r['sides']['base']['stream']['rate_hz']
        L.append(f"- **台車 IMU は {fb:.0f} Hz しか出ていない。台車との比較は {fb / 2:.0f} Hz 未満の成分だけ意味がある**"
                 "（それより速い揺れは台車側では見えず、補間の形が混ざる）")
    L += ['', f"揺れ = {HP_HZ} Hz 以上の成分（走行の加減速そのものを除いたもの）。単位 m/s²、傾きは deg。", '',
          '| 状態 | 秒 | 位置 | 前後 揺れRMS / 最大 | 左右 揺れRMS / 最大 | 上下 揺れRMS | ピッチ 振れ幅 | ロール 振れ幅 | 前後加速度 最大(生) |',
          '|---|---:|---|---|---|---:|---:|---:|---:|']
    for c, lab in LABEL.items():
        if c not in r['classes']:
            continue
        cc = r['classes'][c]
        for side in ('top', 'base'):
            if side not in cc:
                continue
            m = cc[side]
            L.append(f"| {lab} | {cc['seconds']:.1f} | {'上部' if side == 'top' else '台車'} | "
                     f"{m['vib_x']['rms']:.3f} / {m['vib_x']['peak']:.2f} | {m['vib_y']['rms']:.3f} / {m['vib_y']['peak']:.2f} | "
                     f"{m['vib_z']['rms']:.3f} | {m['pitch_deg']['p2p']:.2f} | {m['roll_deg']['p2p']:.2f} | {m['acc_x']['peak']:.2f} |")
    L += ['', '## 周波数（揺れのスペクトルの山。強い順）', '']
    for side, s in r['sides'].items():
        fx = ', '.join(f"{p['hz']:.2f}" for p in s['psd_peaks_x']) or '-'
        fy = ', '.join(f"{p['hz']:.2f}" for p in s['psd_peaks_y']) or '-'
        L.append(f"- {'上部' if side == 'top' else '台車'}: 前後 {fx} Hz ／ 左右 {fy} Hz")
    L += ['', '## 指令との対応', '',
          (f"- {r['cmd_source']} の加速度に対して、上部の前後加速度は {r['cmd_to_top_lag_s']*1000:+.0f} ms 遅れ"
           f"（相関 {r['cmd_to_top_corr']:.2f}）") if r['cmd_to_top_lag_s'] is not None
          else '- 前後の加減速が無いので遅れは出さない', '- 停止直後の揺れ残り（上部）:']
    for e in r['sides']['top']['settle']:
        st = f"{e['settle_s']:.2f} 秒で収まる" if e['settle_s'] is not None else '5 秒以内に収まらない'
        L.append(f"  - {e['t']:.1f} 秒: 最大 {e['peak_mps2']:.2f} m/s²、{st}")
    if not r['sides']['top']['settle']:
        L.append('  - 走行→停止の区切りが無い')
    L += ['', '![時系列](timeseries.png)', '', '![スペクトル](psd.png)']
    with open(os.path.join(d, 'summary.md'), 'w') as fp:
        fp.write('\n'.join(L) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('names', nargs='+')
    ap.add_argument('--info', action='store_true')
    ap.add_argument('--yaw', type=float, help='スマホの前方向を手で与える [deg]（自動推定が効かない記録用）')
    ap.add_argument('--yaw-base', type=float)
    ap.add_argument('--static', type=float, default=3.0, help='記録開始から何秒を静止とみなすか')
    a = ap.parse_args()
    for n in a.names:
        if a.info:
            info(n)
        else:
            analyze(n, a.yaw, a.yaw_base, a.static)


if __name__ == '__main__':
    main()
