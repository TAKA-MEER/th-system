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

from route_replay_core import (
    ReplayParams, ReplayCommand,
    reverse_points, rotate_toward, pure_pursuit, advance_index, normalize_angle,
)


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


def test_advance_index_exists_and_moves_forward():
    params = ReplayParams(arrive_dist_m=0.2)
    points = [(float(i), 0.0, 0.0) for i in range(0, 11)]
    # ロボットが 4.5 付近 → index 前進
    idx = advance_index((4.5, 0.0, 0.0), points, 0, params)
    assert idx >= 0
