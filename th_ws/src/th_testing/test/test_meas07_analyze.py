"""test_meas07_analyze.py — 制動測定の解析（scripts/meas07_analyze.py）の試験。

ROS2 不要・ホストの素の pytest で走る。合成した cmd／wheel／estop の行で、
停止事象の起点と種別を縛る（11-10 の記録で、S 字ランプの停止が「cmd が 0 に着いた
時刻」を起点に取り違えられ、空走と二重に数えられていた）。

縛るもの:
  1. S 字ランプの停止は、cmd が定常から下がり始めた時刻が起点（0 に着いた時刻ではない）
  2. 非常停止の要求（estop_ui）の直後に cmd が即 0 になる停止は、ソフト非常停止の 1 件だけ
  3. 物理非常停止（estop_hw）も同じ。「空走」の 1 件だけで、ランプ停止／即0停止が重ならない
  4. 非常停止の記録が無い即 0 は「即0停止」で、ランプ停止とは別
  5. 定常が短いジョグは数えない
  6. 11-10 の実記録（brake_070617.csv）から期待どおりの本数・近い値が出る
"""
import os
import sys
import importlib.util

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_TH_WS = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
_SCRIPT = os.path.join(_TH_WS, 'scripts', 'meas07_analyze.py')
_DATA = os.path.join(_TH_WS, 'data')

_spec = importlib.util.spec_from_file_location('meas07_analyze_under_test', _SCRIPT)
ana = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ana)

DT = 0.05   # cmd は 20Hz
T_BASE = 1000.0


def _cmd_run(t_start, plateau_v, plateau_s, fall_s, rows):
    """定常を plateau_s 秒保ち、fall_s 秒かけて 0 へ落とす。fall_s=0 は定常から即 0。
    戻り値は (定常の終わりの時刻, 0 に着いた時刻)。"""
    t = t_start
    n = int(plateau_s / DT)
    for _ in range(n):
        rows.append((t, plateau_v, 0.0, 0.0))
        t += DT
    t_plateau_end = t - DT
    if fall_s > 0:
        m = int(fall_s / DT)
        for i in range(1, m):
            rows.append((t, plateau_v * (1.0 - i / m), 0.0, 0.0))
            t += DT
    rows.append((t, 0.0, 0.0, 0.0))
    return t_plateau_end, t


def _wheel_from_cmd(cmd, lag_s=0.0, tail_s=1.5, hz=10.0, decay_s=0.0):
    """cmd を lag_s 遅らせて車輪速度にする。cmd が 0 に着いたあとも decay_s の間は
    直前の速度から線形に 0 へ落とす（即 0 の停止で車輪が止まるまでの時間を模す）。"""
    out = []
    t_end = cmd[-1][0] + tail_s
    t = cmd[0][0]
    while t <= t_end:
        v = 0.0
        for ts, c, _a, _b in cmd:
            if ts <= t - lag_s:
                v = c
            else:
                break
        out.append((t, v, v, v))
        t += 1.0 / hz
    return out


def _rows(cmd, wheel, estop_hw=(), estop_ui=()):
    return {'cmd': cmd, 'wheel': wheel,
            'estop_hw': [(t, v, 0.0, 0.0) for t, v in estop_hw],
            'estop_ui': [(t, v, 0.0, 0.0) for t, v in estop_ui]}


def _kinds(events):
    return [k for k, _t in events]


def test_ramp_stop_origin_is_where_cmd_starts_falling_not_where_it_reaches_zero():
    cmd = []
    t_plateau_end, t_zero = _cmd_run(T_BASE, 0.3, 3.0, 1.4, cmd)
    ev = ana.find_events(_rows(cmd, _wheel_from_cmd(cmd)))
    assert _kinds(ev) == ['ランプ停止']
    t0 = ev[0][1]
    # 起点は定常の終わりの直後（10% 下がった標本）。0 に着いた時刻より 1 秒以上前
    assert t_plateau_end < t0 <= t_plateau_end + 0.3
    assert t_zero - t0 > 1.0


def test_ramp_stop_measure_uses_that_origin():
    cmd = []
    _cmd_run(T_BASE, 0.3, 3.0, 1.4, cmd)
    wheel = _wheel_from_cmd(cmd)
    t0 = ana.find_events(_rows(cmd, wheel))[0][1]
    v0, t_stop, dist, _decel, _n = ana.measure(wheel, t0)
    assert v0 == pytest.approx(0.3, abs=0.03)
    # 起点から 0 に着くまで約 1.3 秒。0 に着いた時刻を起点にしていたら 0.3 秒未満になる
    assert 1.0 < t_stop < 1.5
    assert dist > 0.12


def test_soft_estop_with_immediate_zero_is_one_event():
    cmd = []
    t_plateau_end, _ = _cmd_run(T_BASE, 0.3, 3.0, 0.0, cmd)
    t_req = t_plateau_end + 0.02
    rows = _rows(cmd, _wheel_from_cmd(cmd), estop_ui=[(t_req, 1), (t_req + 4.0, 0)])
    ev = ana.find_events(rows)
    assert _kinds(ev) == ['ソフト非常停止']
    assert ev[0][1] == pytest.approx(t_req)


def test_hw_estop_with_immediate_zero_is_one_coast_event():
    cmd = []
    t_plateau_end, _ = _cmd_run(T_BASE, 0.3, 3.0, 0.0, cmd)
    rows = _rows(cmd, _wheel_from_cmd(cmd), estop_hw=[(t_plateau_end + 0.05, 1)])
    assert _kinds(ana.find_events(rows)) == ['空走']


def test_estop_release_edge_is_not_an_event():
    rows = _rows([], [], estop_hw=[(T_BASE, 0)], estop_ui=[(T_BASE, 0)])
    assert ana.find_events(rows) == []


def test_ramp_then_coast_in_one_recording_are_counted_once_each():
    cmd = []
    _cmd_run(T_BASE, 0.3, 3.0, 1.4, cmd)
    # 20 秒あとに空走（物理非常停止）。cmd の後続は 0 のまま挟む
    t2 = T_BASE + 20.0
    t_plateau_end2, _ = _cmd_run(t2, 0.3, 3.0, 0.0, cmd)
    rows = _rows(cmd, _wheel_from_cmd(cmd), estop_hw=[(t_plateau_end2 + 0.05, 1)])
    ev = ana.find_events(rows)
    assert _kinds(ev) == ['ランプ停止', '空走']


def test_soft_estop_and_ramp_and_coast_three_kinds_no_double_count():
    cmd = []
    _cmd_run(T_BASE, 0.3, 3.0, 1.4, cmd)
    pe_soft, _ = _cmd_run(T_BASE + 20.0, 0.3, 3.0, 0.0, cmd)
    pe_hw, _ = _cmd_run(T_BASE + 40.0, 0.3, 3.0, 0.0, cmd)
    rows = _rows(cmd, _wheel_from_cmd(cmd),
                 estop_hw=[(pe_hw + 0.05, 1)],
                 estop_ui=[(pe_soft + 0.02, 1), (pe_soft + 4.0, 0)])
    assert _kinds(ana.find_events(rows)) == ['ランプ停止', 'ソフト非常停止', '空走']


def test_immediate_zero_without_estop_record_is_its_own_kind():
    cmd = []
    _cmd_run(T_BASE, 0.55, 3.0, 0.0, cmd)
    assert _kinds(ana.find_events(_rows(cmd, _wheel_from_cmd(cmd)))) == ['即0停止']


def test_short_jog_is_not_a_stop_event():
    cmd = []
    _cmd_run(T_BASE, 0.25, 0.4, 1.0, cmd)
    assert ana.find_events(_rows(cmd, _wheel_from_cmd(cmd))) == []


def test_slow_cmd_never_reaching_event_threshold_is_ignored():
    cmd = []
    _cmd_run(T_BASE, 0.05, 3.0, 0.5, cmd)   # MIN_V0 未満
    assert ana.find_events(_rows(cmd, _wheel_from_cmd(cmd))) == []


# ── 11-10 の実記録 ──────────────────────────────────────────

def _real(name):
    path = os.path.join(_DATA, name)
    assert os.path.exists(path), f'実記録が無い: {path}'
    return ana.read(path)


def _measured(rows, kind):
    out = []
    for k, t0 in ana.find_events(rows):
        if k != kind:
            continue
        m = ana.measure(rows['wheel'], t0)
        if m:
            out.append(m)
    return out


def test_real_1110_ramp_stops_match_r66_and_coast_not_doubled():
    rows = _real('brake_070617.csv')
    ev = ana.find_events(rows)
    coast = [t for k, t in ev if k == '空走']
    assert len(coast) == 7          # estop_hw の 0→1 は 7 回（うち走行中に押したのは 4 回）
    assert len(_measured(rows, '空走')) == 4
    # 空走と同じ停止が「ランプ停止」「即0停止」として重ならない
    for k, t in ev:
        if k != '空走':
            assert all(abs(t - tc) > 1.0 for tc in coast), (k, t)

    ramps = _measured(rows, 'ランプ停止')
    slow = [m for m in ramps if 0.28 <= m[0] <= 0.32]   # 0.3 m/s
    fast = [m for m in ramps if 0.52 <= m[0] <= 0.58]   # 0.55 m/s
    assert len(slow) >= 3 and len(fast) >= 3
    # R-66: 0.3 → 0.82〜0.92 秒・0.14〜0.16 m ／ 0.55 → 1.40〜1.42 秒・0.43〜0.45 m
    # 起点の取り方で ±0.1 秒ほどずれるので幅を持たせる
    for m in slow[:3]:
        assert 0.8 <= m[1] <= 1.05
        assert 0.13 <= m[2] <= 0.18
    for m in fast[:3]:
        assert 1.3 <= m[1] <= 1.55
        assert 0.35 <= m[2] <= 0.46


def test_real_1110_coast_is_short():
    rows = _real('brake_070617.csv')
    coast = _measured(rows, '空走')[:3]
    assert len(coast) == 3
    for m in coast:
        assert m[1] < 0.3          # 押してから車輪の停止まで約 0.2 秒


@pytest.mark.parametrize('name', ['brake_032431.csv', 'brake_032737.csv'])
def test_old_august_records_still_analyzable(name):
    rows = _real(name)
    assert ana.find_events(rows)       # 壊れず、事象が 1 件以上出る
