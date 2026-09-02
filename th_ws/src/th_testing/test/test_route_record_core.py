"""
test_route_record_core.py
==========================
教示経路記録コア (route_record_core) の単体テスト。

ROS2 なし・純粋 Python で実行可能。
pytest で実行: python3 -m pytest test/test_route_record_core.py -v
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

from route_record_core import (
    RouteRecordParams, RouteData, RouteRecorderCore,
    polyline_length, route_to_dict, route_from_dict, normalize_angle,
    decimate_polyline,
)


def test_start_sets_initial_point_and_start_yaw():
    rec = RouteRecorderCore()
    rec.start(1.0, 2.0, 0.7)
    assert rec.point_count == 1
    assert rec.start_yaw == pytest.approx(0.7)
    assert rec.recorded_m == pytest.approx(0.0)


def test_points_property_returns_a_copy_of_the_recorded_list():
    rec = RouteRecorderCore(RouteRecordParams(sample_min_dist_m=0.1, sample_min_yaw_rad=0.2))
    rec.start(0.0, 0.0, 0.0)
    rec.add_pose(0.2, 0.0, 0.0)
    pts = rec.points
    assert pts == [(0.0, 0.0, 0.0), (0.2, 0.0, 0.0)]
    pts.append((9.0, 9.0, 0.0))          # 返り値をいじっても内部は変わらない
    assert rec.point_count == 2


def test_small_movement_is_thinned_out():
    params = RouteRecordParams(sample_min_dist_m=0.10, sample_min_yaw_rad=0.20)
    rec = RouteRecorderCore(params)
    rec.start(0.0, 0.0, 0.0)
    # 距離 0.05 < 0.10、かつ向きも変わらない → 点は増えない
    assert rec.add_pose(0.05, 0.0, 0.0) is False
    assert rec.add_pose(0.09, 0.01, 0.01) is False
    assert rec.point_count == 1


def test_movement_beyond_min_dist_adds_point():
    params = RouteRecordParams(sample_min_dist_m=0.10, sample_min_yaw_rad=0.20)
    rec = RouteRecorderCore(params)
    rec.start(0.0, 0.0, 0.0)
    assert rec.add_pose(0.12, 0.0, 0.0) is True
    assert rec.point_count == 2


def test_yaw_change_adds_point_even_without_distance():
    params = RouteRecordParams(sample_min_dist_m=0.10, sample_min_yaw_rad=0.20)
    rec = RouteRecorderCore(params)
    rec.start(0.0, 0.0, 0.0)
    # 距離 0.05 < 0.10 だが yaw 差 0.5 >= 0.20 → 追加
    assert rec.add_pose(0.05, 0.0, 0.5) is True
    assert rec.point_count == 2


def test_recorded_m_known_path():
    rec = RouteRecorderCore()
    rec.start(0.0, 0.0, 0.0)
    rec.add_pose(1.0, 0.0, 0.0)
    rec.add_pose(1.0, 1.0, 0.0)
    # (0,0)->(1,0)->(1,1) = 1.0 + 1.0 = 2.0
    assert rec.recorded_m == pytest.approx(2.0)
    assert polyline_length(rec.finalize("r", "n", 0).points) == pytest.approx(2.0)


def test_finalize_route_to_dict_keys_and_length():
    params = RouteRecordParams(sample_min_dist_m=0.001)
    rec = RouteRecorderCore(params)
    rec.start(0.0, 0.0, 0.5)
    rec.add_pose(1.0, 0.0, 0.5)
    data = rec.finalize("route-1", "demo", 1234, frame_id="odom", generation=2)
    d = route_to_dict(data)
    assert set(d.keys()) == {
        'id', 'name', 'generation', 'length_m', 'point_count',
        'start_yaw', 'recorded_at_ms', 'frame_id', 'points',
    }
    assert d['id'] == "route-1"
    assert d['length_m'] == pytest.approx(rec.recorded_m)
    assert d['point_count'] == 2
    assert d['points'] == [[0.0, 0.0, 0.5], [1.0, 0.0, 0.5]]


def test_route_dict_round_trip():
    rec = RouteRecorderCore(RouteRecordParams(sample_min_dist_m=0.001))
    rec.start(0.0, 0.0, 0.1)
    rec.add_pose(0.5, 0.3, 0.2)
    rec.add_pose(1.0, 1.0, 0.5)
    data = rec.finalize("route-2", "rt", 999, frame_id="map", generation=3)
    d = route_to_dict(data)
    back = route_from_dict(d)
    assert back.points == data.points
    assert back.points[0] == (0.0, 0.0, 0.1)
    assert back.id == data.id and back.generation == data.generation
    assert back.frame_id == "map"


def test_normalize_angle_range():
    assert normalize_angle(math.pi) == pytest.approx(math.pi)
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(1.5 * math.pi) == pytest.approx(-math.pi / 2)
    assert normalize_angle(2.0 * math.pi) == pytest.approx(0.0)


# ── WS-9F: decimate_polyline（/route/preview の帯域対策・純関数） ──────────

def _pts(n):
    return [(i, i * 0.5, 0.0) for i in range(n)]


def test_decimate_keeps_content_unchanged_when_within_budget():
    pts = _pts(77)
    assert decimate_polyline(pts, 400) == pts   # 「そのまま（中身を変えずに）返す」


def test_decimate_shrinks_and_keeps_first_and_last():
    pts = _pts(1200)
    out = decimate_polyline(pts, 400)
    assert len(out) <= 400
    assert out[0] == pts[0]
    assert out[-1] == pts[-1]


def test_decimate_preserves_order_and_is_a_subsequence_without_duplicates():
    pts = _pts(1200)
    out = decimate_polyline(pts, 400)
    assert len(out) <= 400
    # 元の部分列になっている（並べ替えていない）
    it = iter(pts)
    assert all(any(p == q for q in it) for p in out)
    # 重複しない
    assert len(set(out)) == len(out)


def test_decimate_max_points_zero_or_negative_returns_unchanged():
    pts = _pts(50)
    assert decimate_polyline(pts, 0) == pts
    assert decimate_polyline(pts, -3) == pts


def test_decimate_max_points_one_keeps_only_first():
    pts = _pts(50)
    assert decimate_polyline(pts, 1) == [pts[0]]


def test_decimate_empty_list():
    assert decimate_polyline([], 5) == []


def test_decimate_1200_to_400_school_loop_keeps_endpoints():
    # 校舎 1 周（100m 級・1000 点前後）の実寸に相当
    pts = _pts(1200)
    out = decimate_polyline(pts, 400)
    assert len(out) <= 400
    assert out[0] == (0, 0.0, 0.0)
    assert out[-1] == (1199, 1199 * 0.5, 0.0)


def test_decimate_does_not_mutate_input():
    pts = _pts(1200)
    snapshot = list(pts)
    decimate_polyline(pts, 400)
    assert pts == snapshot

