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
)


def test_start_sets_initial_point_and_start_yaw():
    rec = RouteRecorderCore()
    rec.start(1.0, 2.0, 0.7)
    assert rec.point_count == 1
    assert rec.start_yaw == pytest.approx(0.7)
    assert rec.recorded_m == pytest.approx(0.0)


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
