"""
test_route_replay_core.py
==========================
教示経路再生コア (route_replay_core) の単体テスト。

ROS2 なし・純粋 Python で実行可能。
pytest で実行: python3 -m pytest test/test_route_replay_core.py -v
"""

import math
import sys
import os
import pytest

# ── パスを通す（colcon build 前にも直接 pytest できるように）
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'th_planning'))
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'th_planning', 'th_planning'))

import ast

from route_replay_core import (
    ReplayParams, ReplayCommand,
    reverse_points, rotate_toward, pure_pursuit, advance_index, normalize_angle,
    align_path_to_current, ramp_toward, scale_replay_params,
)

REPLAY_RUNNER = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_planning', 'scripts',
    'replay_runner.py'))


def test_ramp_toward_clamps_step_and_reaches_target():
    assert ramp_toward(0.0, 1.0, 0.2) == pytest.approx(0.2)      # 上げ: max_step まで
    assert ramp_toward(1.0, 0.0, 0.3) == pytest.approx(0.7)      # 下げ: max_step まで
    assert ramp_toward(0.05, 0.0, 0.2) == pytest.approx(0.0)     # 近ければ target ちょうど
    assert ramp_toward(-0.5, 0.5, 100) == pytest.approx(0.5)     # step 十分なら一発
    assert ramp_toward(0.3, 0.9, 0.0) == pytest.approx(0.9)      # max_step<=0 は即 target


def test_reverse_points_reverses_and_rotates_yaw():
    pts = [(0.0, 0.0, 0.0), (1.0, 0.0, math.pi / 2), (2.0, 1.0, math.pi)]
    rev = reverse_points(pts)
    assert len(rev) == 3
    assert rev[0][0] == pytest.approx(2.0) and rev[0][1] == pytest.approx(1.0)
    assert rev[0][2] == pytest.approx(0.0)
    assert rev[1][0] == pytest.approx(1.0) and rev[1][1] == pytest.approx(0.0)
    assert rev[1][2] == pytest.approx(-math.pi / 2)  # yaw pi/2 -> 3pi/2 -> -pi/2
    assert rev[2] == (0.0, 0.0, math.pi)             # yaw 0 -> pi


def test_rotate_toward_sign_and_done():
    params = ReplayParams(max_yaw_rate_rps=0.8, yaw_tol_rad=0.10)
    # 現在 yaw 0 → 目標 pi/2 → 正(+)方向に最大旋回
    w, done = rotate_toward(0.0, math.pi / 2, params)
    assert not done
    assert w == pytest.approx(0.8)
    # 誤差 tol 以内 → done=True, w=0
    w2, done2 = rotate_toward(0.0, 0.05, params)
    assert done2
    assert w2 == pytest.approx(0.0)


def test_pure_pursuit_turns_back_toward_path():
    params = ReplayParams(lookahead_m=0.5, cruise_speed_mps=0.25,
                          max_yaw_rate_rps=0.8)
    # 直線経路 x 軸 (0,0)->(10,0)。ロボットは y=+1 (左にずれ) yaw=0 (経路に平行)
    robot = (0.0, 1.0, 0.0)
    points = [(float(i), 0.0, 0.0) for i in range(0, 11)]
    cmd = pure_pursuit(robot, points, params)
    # 左にずれてる→右(負 w)へ戻る向き
    assert cmd.w < 0.0
    assert not cmd.arrived


def test_pure_pursuit_straight_w_zero():
    params = ReplayParams(lookahead_m=0.5, cruise_speed_mps=0.25,
                          max_yaw_rate_rps=0.8)
    # 経路正上、真直ぐ向いている
    robot = (0.0, 0.0, 0.0)
    points = [(float(i), 0.0, 0.0) for i in range(0, 11)]
    cmd = pure_pursuit(robot, points, params)
    assert cmd.w == pytest.approx(0.0, abs=1e-6)
    assert cmd.v == pytest.approx(params.cruise_speed_mps)


def test_pure_pursuit_arrives_near_end():
    params = ReplayParams(lookahead_m=0.4, arrive_dist_m=0.2)
    # 最終点 (1,0)。ロボットが (0.9, 0) で最終点まで 0.1 <= 0.2 → 到着
    robot = (0.9, 0.0, 0.0)
    points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    cmd = pure_pursuit(robot, points, params)
    assert cmd.arrived
    assert cmd.v == pytest.approx(0.0)
    assert cmd.w == pytest.approx(0.0)
    assert cmd.target_index == 1


def test_pure_pursuit_not_arrived_mid_path():
    params = ReplayParams(lookahead_m=0.4, arrive_dist_m=0.2)
    robot = (0.0, 0.0, 0.0)
    points = [(float(i), 0.0, 0.0) for i in range(0, 11)]
    cmd = pure_pursuit(robot, points, params)
    assert not cmd.arrived


def test_from_index_monotonic_increases_target():
    params = ReplayParams(lookahead_m=0.4)
    points = [(float(i), 0.0, 0.0) for i in range(0, 11)]
    robot = (0.0, 0.0, 0.0)
    c1 = pure_pursuit(robot, points, params, from_index=0)
    c2 = pure_pursuit(robot, points, params, from_index=4)
    c3 = pure_pursuit(robot, points, params, from_index=8)
    assert c1.target_index <= c2.target_index <= c3.target_index


def test_advance_index_moves_past_passed_points():
    params = ReplayParams(arrive_dist_m=0.2)
    points = [(float(i), 0.0, 0.0) for i in range(0, 11)]  # 1m 間隔
    # ロボットが x=4.5 → セグメント 0-1..3-4 の足を越えている → index 4
    assert advance_index((4.5, 0.0, 0.0), points, 0, params) == 4
    # 進んだ index からさらに前進
    assert advance_index((7.2, 0.0, 0.0), points, 4, params) == 7
    # まだ最初のセグメント上 → 進まない
    assert advance_index((0.1, 0.0, 0.0), points, 0, params) == 0
    # index は後退しない
    assert advance_index((1.0, 0.0, 0.0), points, 5, params) == 5
    # 空・単一点で落ちない
    assert advance_index((0.0, 0.0, 0.0), [], 0, params) == 0
    assert advance_index((0.0, 0.0, 0.0), [(0.0, 0.0, 0.0)], 0, params) == 0


def test_advance_index_captures_near_next_point():
    # 密な点列（~0.1m 間隔）。arrive_dist_m 0.2 以内の次点は通過扱いで飛ばす
    params = ReplayParams(arrive_dist_m=0.2)
    points = [(0.1 * i, 0.0, 0.0) for i in range(0, 30)]
    # ロボットが x=1.0 付近 → 次点(index+1)が 0.2 以内の間ずっと進む
    idx = advance_index((1.0, 0.0, 0.0), points, 0, params)
    assert 8 <= idx <= 12   # 1.0m 近傍の点まで進む


def test_pure_pursuit_skips_points_behind_robot():
    # ロボットが経路の途中(x=5)で +x を向いている。from_index が古く 0 のままでも
    # 背後(x<5)の点は目標に選ばず、前方の点を選ぶ（U ターン固着の防止）。
    params = ReplayParams(lookahead_m=0.4, arrive_dist_m=0.2)
    points = [(float(i), 0.0, 0.0) for i in range(0, 11)]
    cmd = pure_pursuit((5.0, 0.0, 0.0), points, params, from_index=0)
    assert cmd.target_index >= 5
    assert not cmd.arrived
    assert cmd.v > 0.0
    assert abs(cmd.w) < 1e-6           # まっすぐ前方 → 旋回なし


def test_pure_pursuit_arrived_when_from_index_at_last():
    params = ReplayParams(lookahead_m=0.4, arrive_dist_m=0.2)
    points = [(float(i), 0.0, 0.0) for i in range(0, 11)]
    # 最終点まで距離があっても from_index が末尾なら到着扱い（過走の吸収）
    cmd = pure_pursuit((9.5, 0.0, 0.0), points, params, from_index=10)
    assert cmd.arrived


def _simulate_replay(points, start_pose, params, *, dt=0.05, max_ticks=4000):
    """pure_pursuit + advance_index を単純なユニサイクルモデルで多ティック回す。
    返り値: (arrived, final_from_index, min_v, total_abs_yaw_change)"""
    x, y, yaw = start_pose
    from_index = 0
    min_v = float('inf')
    total_dyaw = 0.0
    prev_yaw = yaw
    for _ in range(max_ticks):
        from_index = advance_index((x, y, yaw), points, from_index, params)
        cmd = pure_pursuit((x, y, yaw), points, params, from_index)
        if cmd.arrived:
            return True, from_index, min_v, total_dyaw
        min_v = min(min_v, cmd.v)
        x += cmd.v * math.cos(yaw) * dt
        y += cmd.v * math.sin(yaw) * dt
        yaw = normalize_angle(yaw + cmd.w * dt)
        total_dyaw += abs(normalize_angle(yaw - prev_yaw))
        prev_yaw = yaw
    return False, from_index, min_v, total_dyaw


def test_replay_simulation_completes_curved_route_without_uturn():
    # ~0.11m 間隔で緩く左に曲がる経路（実記録 route_1788241427965.json に近い密度）
    params = ReplayParams()
    points = []
    x, y, th = 0.0, 0.0, 0.0
    for _ in range(40):
        points.append((x, y, th))
        x += 0.11 * math.cos(th)
        y += 0.11 * math.sin(th)
        th += 0.05
    arrived, final_idx, min_v, total_dyaw = _simulate_replay(
        points, (0.0, 0.0, 0.0), params)
    assert arrived, f'完走しなかった (from_index={final_idx}/{len(points) - 1})'
    assert final_idx >= len(points) - 2
    assert min_v >= -1e-6, '後退指令が出た（U ターン痕跡）'
    # 経路全体の向き変化は ~2.0rad。U ターンが混ざると数倍に膨らむ
    assert total_dyaw < 4.0, f'旋回量が過大 (U ターンの疑い): {total_dyaw:.2f}'


# ── align_path_to_current（WAIVER W-01 の実害対策）─────────────────
def test_align_identity_when_current_equals_recorded_start():
    pts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, math.pi / 2)]
    out = align_path_to_current(pts, recorded_start=(0.0, 0.0, 0.0),
                                current=(0.0, 0.0, 0.0))
    for a, b in zip(out, pts):
        assert a[0] == pytest.approx(b[0])
        assert a[1] == pytest.approx(b[1])
        assert a[2] == pytest.approx(b[2])


def test_align_pure_translation():
    pts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    out = align_path_to_current(pts, recorded_start=(0.0, 0.0, 0.0),
                                current=(10.0, 5.0, 0.0))
    assert out[0] == pytest.approx((10.0, 5.0, 0.0))
    assert out[2][0] == pytest.approx(12.0)
    assert out[2][1] == pytest.approx(5.0)


def test_align_rotation_90deg():
    # 記録は +x 方向へ 2m 直進。現在向きは +90°（+y 方向）。
    pts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    out = align_path_to_current(pts, recorded_start=(0.0, 0.0, 0.0),
                                current=(0.0, 0.0, math.pi / 2))
    # 変換後は +y 方向へ 2m 進む形になる。
    assert out[0] == pytest.approx((0.0, 0.0, math.pi / 2))
    assert out[2][0] == pytest.approx(0.0, abs=1e-9)
    assert out[2][1] == pytest.approx(2.0)
    assert out[2][2] == pytest.approx(math.pi / 2)


def test_align_first_point_always_lands_on_current():
    pts = [(3.0, -1.0, 1.2), (4.0, -1.0, 1.0), (5.0, 0.0, 0.5)]
    cur = (7.0, 8.0, -2.0)
    out = align_path_to_current(pts, recorded_start=(3.0, -1.0, 1.2), current=cur)
    assert out[0][0] == pytest.approx(cur[0])
    assert out[0][1] == pytest.approx(cur[1])
    assert out[0][2] == pytest.approx(cur[2])
    # 形（隣接点間の距離）は保存される。
    def seglen(p):
        return [math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1])
                for i in range(len(p) - 1)]
    assert seglen(out) == pytest.approx(seglen(pts))


# ── scale_replay_params（WS-9T: 再生速度を比率で可変にする）────────────
_BASE = ReplayParams(lookahead_m=0.4, cruise_speed_mps=0.45, max_yaw_rate_rps=0.9,
                     arrive_dist_m=0.2, yaw_tol_rad=0.1)


def test_scale_replay_params_ratio_1_is_base_max_end():
    p = scale_replay_params(_BASE, 1.0, cruise_min=0.12, yaw_min=0.4)
    assert p.cruise_speed_mps == pytest.approx(0.45)
    assert p.max_yaw_rate_rps == pytest.approx(0.9)


def test_scale_replay_params_ratio_0_is_min_end():
    p = scale_replay_params(_BASE, 0.0, cruise_min=0.12, yaw_min=0.4)
    assert p.cruise_speed_mps == pytest.approx(0.12)
    assert p.max_yaw_rate_rps == pytest.approx(0.4)


def test_scale_replay_params_midpoint_is_linear():
    p = scale_replay_params(_BASE, 0.5, cruise_min=0.12, yaw_min=0.4)
    assert p.cruise_speed_mps == pytest.approx(0.12 + (0.45 - 0.12) * 0.5)
    assert p.max_yaw_rate_rps == pytest.approx(0.4 + (0.9 - 0.4) * 0.5)


def test_scale_replay_params_clamps_out_of_range_ratio():
    lo = scale_replay_params(_BASE, -3.0, cruise_min=0.12, yaw_min=0.4)
    hi = scale_replay_params(_BASE, 9.0, cruise_min=0.12, yaw_min=0.4)
    assert lo.cruise_speed_mps == pytest.approx(0.12)
    assert hi.cruise_speed_mps == pytest.approx(0.45)


def test_scale_replay_params_leaves_other_fields_untouched():
    p = scale_replay_params(_BASE, 0.3, cruise_min=0.12, yaw_min=0.4)
    assert p.lookahead_m == _BASE.lookahead_m
    assert p.arrive_dist_m == _BASE.arrive_dist_m
    assert p.yaw_tol_rad == _BASE.yaw_tol_rad
    # 元のインスタンスは変更しない（複製を返す）。
    assert _BASE.cruise_speed_mps == 0.45


# ── replay_runner.py の配線（ast 静的検査。ROS 不要）──────────────────
def _runner_tree():
    with open(REPLAY_RUNNER, encoding='utf-8') as f:
        return ast.parse(f.read(), filename=REPLAY_RUNNER)


def test_replay_runner_subscribes_speed_scale_and_uses_scale_helper():
    """WS-9T: /replay/speed_scale を購読し、受信ハンドラで scale_replay_params を呼ぶ。"""
    src = open(REPLAY_RUNNER, encoding='utf-8').read()
    assert "'/replay/speed_scale'" in src, '/replay/speed_scale の購読が無い'
    tree = _runner_tree()
    handler = next((n for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef) and n.name == '_on_speed_scale'), None)
    assert handler is not None, '_on_speed_scale ハンドラが無い'
    calls = [n for n in ast.walk(handler)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
             and n.func.id == 'scale_replay_params']
    assert calls, '_on_speed_scale が scale_replay_params を呼んでいない'


def test_replay_runner_speed_defaults_raised():
    """WS-9T: cruise/yaw の最大端（既定）を 0.45 / 0.9 に引き上げてあること。"""
    tree = _runner_tree()
    decls = {}
    for n in ast.walk(tree):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter' and len(n.args) >= 2
                and isinstance(n.args[0], ast.Constant)
                and isinstance(n.args[1], ast.Constant)):
            decls[n.args[0].value] = n.args[1].value
    assert decls.get('cruise_speed_mps') == 0.45, decls.get('cruise_speed_mps')
    assert decls.get('max_yaw_rate_rps') == 0.9, decls.get('max_yaw_rate_rps')
    assert 'replay_cruise_min_mps' in decls
    assert 'replay_speed_default_ratio' in decls
