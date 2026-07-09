"""
test_mapless_follow_logic.py
==============================
MAP不要(Nav2/SLAM占有格子非依存)の純粋軌跡追従モード
コアロジック単体テスト

ROS2 なし・純粋 Python で実行可能。
pytest で実行: pytest test/test_mapless_follow_logic.py -v
"""

import math
import sys
import os
import pytest

# ── パスを通す(colcon build 前にも直接 pytest できるように)
# th_planning をパッケージとしてインポートできるようにする
# (mapless_follow_core.py 内の `from .follow_planner_core import ...`
#  という相対インポートを解決するため、bare モジュールではなく
#  パッケージ経由でインポートする必要がある)
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_planning'))

from th_planning.mapless_follow_core import (
    MaplessState, MaplessFollowParams, MaplessFollowCore,
    next_mapless_state, is_path_blocked,
)


DEFAULT_PARAMS = MaplessFollowParams(
    lookback_distance             = 1.0,
    trail_sample_interval_m       = 0.0,   # テストでは常に軌跡点を追加する
    trail_max_points              = 2000,
    stop_distance                 = 1.0,
    resume_distance                = 1.3,
    obstacle_check_distance_m      = 1.0,
    obstacle_check_half_width_deg  = 20.0,
    v_max                         = 0.3,
    k_ang                         = 2.0,
    stop_radius_m                  = 0.2,
    update_rate_hz                  = 10.0,
)

ORIGIN_POSE = (0.0, 0.0, 0.0)

N_SAMPLES = 72
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / N_SAMPLES


def clear_scan():
    """全方向 inf(障害物なし)のスキャン"""
    return [float('inf')] * N_SAMPLES


def scan_with_obstacle_at(angle_rad, distance):
    """指定角度のサンプルだけ distance、他は inf にしたスキャン"""
    ranges = [float('inf')] * N_SAMPLES
    idx = round((angle_rad - ANGLE_MIN) / ANGLE_INC) % N_SAMPLES
    ranges[idx] = distance
    return ranges


# ════════════════════════════════════════════════════════════
# 1. 距離帯による状態遷移(ヒステリシス)
# ════════════════════════════════════════════════════════════
class TestNextMaplessState:
    STOP, RESUME = 1.0, 1.3

    def _next(self, state, dist):
        return next_mapless_state(state, dist, self.STOP, self.RESUME)

    def test_tracking_stays_above_stop_distance(self):
        assert self._next(MaplessState.TRACKING, 1.5) == MaplessState.TRACKING

    def test_tracking_to_stopped_below_stop_distance(self):
        assert self._next(MaplessState.TRACKING, 0.99) == MaplessState.STOPPED

    def test_stopped_stays_below_resume(self):
        assert self._next(MaplessState.STOPPED, 1.2) == MaplessState.STOPPED

    def test_stopped_to_tracking_at_resume(self):
        assert self._next(MaplessState.STOPPED, 1.3) == MaplessState.TRACKING

    def test_no_chattering_at_boundary(self):
        state = MaplessState.TRACKING
        seq = [0.9, 1.1, 1.2, 1.1, 1.3]
        results = []
        for d in seq:
            state = self._next(state, d)
            results.append(state)
        assert results == [
            MaplessState.STOPPED, MaplessState.STOPPED, MaplessState.STOPPED,
            MaplessState.STOPPED, MaplessState.TRACKING,
        ]


# ════════════════════════════════════════════════════════════
# 2. 進路上障害物チェック(生 LiDAR スキャン)
# ════════════════════════════════════════════════════════════
class TestIsPathBlocked:
    def test_clear_path_not_blocked(self):
        assert not is_path_blocked(
            clear_scan(), ANGLE_MIN, ANGLE_INC,
            direction_rad=0.0, half_width_rad=math.radians(20), max_check_dist=1.0)

    def test_obstacle_ahead_blocks(self):
        ranges = scan_with_obstacle_at(0.0, 0.5)
        assert is_path_blocked(
            ranges, ANGLE_MIN, ANGLE_INC,
            direction_rad=0.0, half_width_rad=math.radians(20), max_check_dist=1.0)

    def test_obstacle_outside_cone_ignored(self):
        ranges = scan_with_obstacle_at(math.pi / 2, 0.5)
        assert not is_path_blocked(
            ranges, ANGLE_MIN, ANGLE_INC,
            direction_rad=0.0, half_width_rad=math.radians(20), max_check_dist=1.0)

    def test_obstacle_too_far_ignored(self):
        ranges = scan_with_obstacle_at(0.0, 5.0)
        assert not is_path_blocked(
            ranges, ANGLE_MIN, ANGLE_INC,
            direction_rad=0.0, half_width_rad=math.radians(20), max_check_dist=1.0)

    def test_nan_ignored(self):
        ranges = clear_scan()
        idx = round((0.0 - ANGLE_MIN) / ANGLE_INC) % N_SAMPLES
        ranges[idx] = float('nan')
        assert not is_path_blocked(
            ranges, ANGLE_MIN, ANGLE_INC,
            direction_rad=0.0, half_width_rad=math.radians(20), max_check_dist=1.0)

    def test_direction_offset(self):
        ranges = scan_with_obstacle_at(math.pi / 2, 0.5)
        assert is_path_blocked(
            ranges, ANGLE_MIN, ANGLE_INC,
            direction_rad=math.pi / 2, half_width_rad=math.radians(20), max_check_dist=1.0)


# ════════════════════════════════════════════════════════════
# 3. MaplessFollowCore 統合テスト
# ════════════════════════════════════════════════════════════
class TestMaplessFollowCore:
    def _core(self, **overrides):
        params = MaplessFollowParams(**{**DEFAULT_PARAMS.__dict__, **overrides})
        return MaplessFollowCore(params)

    def test_tracking_drives_via_pure_pursuit(self):
        core = self._core()
        core.update(person_x=5.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        out = core.update(person_x=6.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.TRACKING
        assert out.reason == "tracking"
        assert out.linear_x == pytest.approx(0.3)
        assert out.angular_z == pytest.approx(0.0)

    def test_person_close_stops_without_evade(self):
        core = self._core()
        out = core.update(person_x=0.5, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.STOPPED
        assert out.reason == "person_close"
        assert out.linear_x == 0.0
        assert out.angular_z == 0.0

    def test_resumes_after_person_moves_away(self):
        core = self._core()
        core.update(person_x=0.5, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.STOPPED
        out = core.update(person_x=1.3, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.TRACKING
        assert out.reason == "tracking"

    def test_obstacle_ahead_stops_regardless_of_person_distance(self):
        core = self._core()
        core.update(person_x=5.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        blocked_scan = scan_with_obstacle_at(0.0, 0.5)
        out = core.update(person_x=6.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=blocked_scan, scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.TRACKING
        assert out.reason == "obstacle_ahead"
        assert out.linear_x == 0.0
        assert out.angular_z == 0.0

    def test_no_pose_stops_and_skips_trail_update(self):
        core = self._core()
        out = core.update(person_x=5.0, person_y=0.0, robot_pose_odom=None,
                           scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert out.reason == "no_pose"
        assert out.linear_x == 0.0
        assert len(core.trail) == 0

    def test_no_scan_stops_but_trail_still_updates(self):
        core = self._core()
        out = core.update(person_x=5.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=None, scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert out.reason == "no_scan"
        assert out.linear_x == 0.0
        assert len(core.trail) == 1

    def test_trail_continues_updating_while_stopped(self):
        core = self._core()
        core.update(person_x=0.5, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        core.update(person_x=0.6, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.STOPPED
        assert len(core.trail) == 2

    def test_reset_returns_to_tracking_and_clears_state(self):
        core = self._core()
        core.update(person_x=0.5, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.STOPPED
        core.reset()
        assert core.state == MaplessState.TRACKING
        assert len(core.trail) == 0

    def test_clear_trail(self):
        core = self._core()
        core.update(person_x=5.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert len(core.trail) == 1
        core.clear_trail()
        assert len(core.trail) == 0
