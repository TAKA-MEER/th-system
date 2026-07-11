"""
test_follow_planner_logic.py
==============================
followLogic.md v2 — 距離帯ベース追従ロジック コアロジック単体テスト

ROS2 なし・純粋 Python で実行可能。
pytest で実行: pytest test/test_follow_planner_logic.py -v
"""

import math
import sys
import os
import pytest
from collections import deque

# ── パスを通す（colcon build 前にも直接 pytest できるように）
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'th_planning'))
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'th_planning', 'th_planning'))

from follow_planner_core import (
    FollowParams, FollowPlannerCore, FollowState,
    update_trail, get_trail_goal,
    next_follow_state,
    find_nearest_open_direction, compute_evade_goal,
    orient_to_evade_route, pure_pursuit_control,
    to_absolute, to_relative,
)


# ────────────────────────────────────────────────────────────
# ヘルパー
# ────────────────────────────────────────────────────────────
def clearance_full(dx, dy):
    """全方向に常に 2m の自由空間があるスタブ"""
    return 2.0


def clearance_zero(dx, dy):
    """全方向に自由空間がないスタブ（壁際）"""
    return 0.0


def clearance_forward_only(dx, dy):
    """+x 方向(dx≈1, dy≈0)のみ自由空間があるスタブ"""
    return 2.0 if dx > 0.9 else 0.0


DEFAULT_PARAMS = FollowParams(
    lookback_distance          = 1.0,
    trail_sample_interval_m    = 0.0,   # テストでは常に軌跡点を追加する
    trail_max_points           = 2000,
    d_prepare                  = 3.0,
    d_evade                    = 2.0,
    distance_hysteresis_m      = 0.2,
    evade_scan_directions      = 8,
    evade_scan_max_dist        = 3.0,
    evade_route_length_m       = 2.0,
    retreat_check_clearance    = 0.5,
    retreat_speed              = 0.15,
    evade_k_ang                = 2.0,
    evade_stop_radius_m        = 0.2,
    evade_orient_tolerance_rad = 0.05,
    goal_deadzone_m            = 0.0,   # テストではデッドゾーンを無効化
    update_rate_hz              = 10.0,
)

ORIGIN_POSE = (0.0, 0.0, 0.0)


# ════════════════════════════════════════════════════════════
# 1. 座標変換
# ════════════════════════════════════════════════════════════
class TestFrameConversion:
    def test_round_trip(self):
        pose = (1.0, 2.0, math.pi / 2)
        rel = (1.0, 0.5)
        abs_pt = to_absolute(pose, rel)
        back = to_relative(pose, abs_pt)
        assert back[0] == pytest.approx(rel[0])
        assert back[1] == pytest.approx(rel[1])

    def test_identity_pose_is_noop(self):
        rel = (3.0, -2.0)
        assert to_absolute(ORIGIN_POSE, rel) == pytest.approx(rel)
        assert to_relative(ORIGIN_POSE, rel) == pytest.approx(rel)

    def test_rotation_90deg(self):
        pose = (0.0, 0.0, math.pi / 2)
        abs_pt = to_absolute(pose, (1.0, 0.0))
        assert abs_pt[0] == pytest.approx(0.0, abs=1e-9)
        assert abs_pt[1] == pytest.approx(1.0)


# ════════════════════════════════════════════════════════════
# 2. 軌跡追従（状態A: TRACKING） §3
# ════════════════════════════════════════════════════════════
class TestTrail:
    def test_update_trail_appends_first_point(self):
        trail = deque(maxlen=10)
        update_trail(trail, (1.0, 2.0))
        assert list(trail) == [(1.0, 2.0)]

    def test_update_trail_filters_small_movement(self):
        trail = deque(maxlen=10)
        update_trail(trail, (0.0, 0.0), min_delta=0.05)
        update_trail(trail, (0.01, 0.0), min_delta=0.05)
        assert len(trail) == 1

    def test_update_trail_appends_beyond_threshold(self):
        trail = deque(maxlen=10)
        update_trail(trail, (0.0, 0.0), min_delta=0.05)
        update_trail(trail, (0.2, 0.0), min_delta=0.05)
        assert len(trail) == 2

    def test_get_trail_goal_empty_returns_robot_pos(self):
        trail = deque(maxlen=10)
        goal = get_trail_goal(trail, lookback_distance=1.0, robot_pos=(5.0, 5.0))
        assert goal == (5.0, 5.0)

    def test_get_trail_goal_single_point(self):
        trail = deque(maxlen=10)
        trail.append((3.0, 4.0))
        goal = get_trail_goal(trail, lookback_distance=1.0)
        assert goal == (3.0, 4.0)

    def test_get_trail_goal_lookback_exact(self):
        trail = deque(maxlen=10)
        trail.append((0.0, 0.0))
        trail.append((1.0, 0.0))
        goal = get_trail_goal(trail, lookback_distance=1.0)
        assert goal == pytest.approx((0.0, 0.0))

    def test_get_trail_goal_lookback_exceeds_trail_returns_oldest(self):
        trail = deque(maxlen=10)
        trail.append((0.0, 0.0))
        trail.append((0.5, 0.0))
        goal = get_trail_goal(trail, lookback_distance=10.0)
        assert goal == pytest.approx((0.0, 0.0))


# ════════════════════════════════════════════════════════════
# 3. 距離帯による状態遷移 §2
# ════════════════════════════════════════════════════════════
class TestNextFollowState:
    D_PREPARE, D_EVADE, HYST = 3.0, 2.0, 0.2

    def _next(self, state, dist):
        return next_follow_state(state, dist, self.D_PREPARE, self.D_EVADE, self.HYST)

    def test_tracking_stays_above_d_prepare(self):
        assert self._next(FollowState.TRACKING, 3.5) == FollowState.TRACKING

    def test_tracking_to_prepare_below_d_prepare(self):
        assert self._next(FollowState.TRACKING, 2.99) == FollowState.PREPARE

    def test_prepare_stays_between_bounds(self):
        assert self._next(FollowState.PREPARE, 2.5) == FollowState.PREPARE

    def test_prepare_to_evading_below_d_evade(self):
        assert self._next(FollowState.PREPARE, 1.99) == FollowState.EVADING

    def test_prepare_no_return_without_hysteresis_margin(self):
        # d_prepare ちょうどでは復帰しない（d_prepare + hysteresis 必要）
        assert self._next(FollowState.PREPARE, 3.0) == FollowState.PREPARE
        assert self._next(FollowState.PREPARE, 3.19) == FollowState.PREPARE

    def test_prepare_to_tracking_requires_hysteresis(self):
        assert self._next(FollowState.PREPARE, 3.2) == FollowState.TRACKING

    def test_evading_stays_below_release(self):
        assert self._next(FollowState.EVADING, 2.1) == FollowState.EVADING

    def test_evading_to_prepare_at_release(self):
        assert self._next(FollowState.EVADING, 2.2) == FollowState.PREPARE

    def test_no_chattering_at_boundary(self):
        state = FollowState.TRACKING
        seq = [2.9, 3.1, 2.9, 3.1, 3.3]
        results = []
        for d in seq:
            state = self._next(state, d)
            results.append(state)
        assert results == [
            FollowState.PREPARE, FollowState.PREPARE, FollowState.PREPARE,
            FollowState.PREPARE, FollowState.TRACKING,
        ]


# ════════════════════════════════════════════════════════════
# 4. 退避方向探索（状態B: PREPARE） §4.1
# ════════════════════════════════════════════════════════════
class TestFindNearestOpenDirection:
    def test_all_directions_free_picks_first_max(self):
        angle, clear = find_nearest_open_direction(
            clearance_full, num_directions=8, max_check_dist=3.0, min_clearance=0.5)
        assert clear == pytest.approx(2.0)
        assert angle == pytest.approx(0.0)

    def test_only_one_direction_clear(self):
        angle, clear = find_nearest_open_direction(
            clearance_forward_only, num_directions=8, max_check_dist=3.0, min_clearance=0.5)
        assert angle == pytest.approx(0.0)
        assert clear == pytest.approx(2.0)

    def test_no_direction_meets_min_clearance(self):
        angle, clear = find_nearest_open_direction(
            clearance_zero, num_directions=8, max_check_dist=3.0, min_clearance=0.5)
        assert clear < 0.0

    def test_clamped_to_max_check_dist(self):
        angle, clear = find_nearest_open_direction(
            lambda dx, dy: 10.0, num_directions=4, max_check_dist=1.5, min_clearance=0.5)
        assert clear == pytest.approx(1.5)


class TestComputeEvadeGoal:
    def test_along_x_axis(self):
        goal = compute_evade_goal((1.0, 2.0), 0.0, 2.0)
        assert goal == pytest.approx((3.0, 2.0))

    def test_along_y_axis(self):
        goal = compute_evade_goal((1.0, 2.0), math.pi / 2, 2.0)
        assert goal[0] == pytest.approx(1.0, abs=1e-9)
        assert goal[1] == pytest.approx(4.0)


# ════════════════════════════════════════════════════════════
# 5. 向き合わせ制御 §4.3
# ════════════════════════════════════════════════════════════
class TestOrientToEvadeRoute:
    def test_within_tolerance_stops(self):
        v, w = orient_to_evade_route(ORIGIN_POSE, escape_angle=0.01, angle_tolerance=0.05)
        assert (v, w) == (0.0, 0.0)

    def test_positive_error_turns_positive(self):
        v, w = orient_to_evade_route(ORIGIN_POSE, escape_angle=math.radians(30), k_ang=2.0)
        assert v == 0.0
        assert w > 0.0

    def test_negative_error_turns_negative(self):
        v, w = orient_to_evade_route(ORIGIN_POSE, escape_angle=math.radians(-30), k_ang=2.0)
        assert v == 0.0
        assert w < 0.0

    def test_linear_always_zero(self):
        v, _ = orient_to_evade_route(ORIGIN_POSE, escape_angle=math.pi)
        assert v == 0.0


# ════════════════════════════════════════════════════════════
# 6. Pure Pursuit §3 (EVADING 走行に使用)
# ════════════════════════════════════════════════════════════
class TestPurePursuitControl:
    def test_within_stop_radius_returns_zero(self):
        v, w = pure_pursuit_control(ORIGIN_POSE, (0.1, 0.0), stop_radius=0.3)
        assert (v, w) == (0.0, 0.0)

    def test_velocity_scales_with_distance(self):
        v, w = pure_pursuit_control(ORIGIN_POSE, (0.75, 0.0), v_max=0.6, stop_radius=0.2)
        assert v == pytest.approx(0.6 * 0.5)
        assert w == pytest.approx(0.0)

    def test_velocity_clamped_at_v_max(self):
        v, _ = pure_pursuit_control(ORIGIN_POSE, (5.0, 0.0), v_max=0.6, stop_radius=0.2)
        assert v == pytest.approx(0.6)

    def test_angle_error_drives_omega(self):
        # 小さい角度誤差では w = k_ang * 誤差 がそのまま出る
        v, w = pure_pursuit_control(ORIGIN_POSE, (1.0, 0.2), v_max=0.6, k_ang=2.0,
                                    stop_radius=0.2, w_max=1.0)
        assert w == pytest.approx(2.0 * math.atan2(0.2, 1.0))

    def test_omega_clamped_at_w_max(self):
        # 真横のゴール (角度誤差 π/2): k_ang*誤差 ≈ 3.14 だが w_max=1.0 にクランプ
        # (実機で w=5.1 rad/s の非現実的な指令が出た問題の回帰テスト)
        v, w = pure_pursuit_control(ORIGIN_POSE, (0.0, 1.0), v_max=0.6, k_ang=2.0,
                                    stop_radius=0.2, w_max=1.0)
        assert w == pytest.approx(1.0)
        v, w = pure_pursuit_control(ORIGIN_POSE, (0.0, -1.0), v_max=0.6, k_ang=2.0,
                                    stop_radius=0.2, w_max=1.0)
        assert w == pytest.approx(-1.0)

    def test_linear_reduced_when_goal_behind(self):
        # 真後ろのゴール: cos(π)=-1 → 前進成分ゼロ (その場旋回で向き直る)
        v, w = pure_pursuit_control(ORIGIN_POSE, (-2.0, 0.0), v_max=0.6, k_ang=2.0,
                                    stop_radius=0.2, w_max=1.0)
        assert v == pytest.approx(0.0)
        assert abs(w) == pytest.approx(1.0)
        # 真横のゴール: cos(π/2)=0 → 前進ゼロ
        v, _ = pure_pursuit_control(ORIGIN_POSE, (0.0, 1.0), v_max=0.6, stop_radius=0.2)
        assert v == pytest.approx(0.0)


# ════════════════════════════════════════════════════════════
# 7. FollowPlannerCore 統合テスト
# ════════════════════════════════════════════════════════════
class TestFollowPlannerCore:
    def _core(self, **overrides):
        params = FollowParams(**{**DEFAULT_PARAMS.__dict__, **overrides})
        return FollowPlannerCore(params)

    def test_tracking_produces_nav_goal_from_trail(self):
        core = self._core()
        core.update(person_x=5.0, person_y=0.0, t=0.0,
                     clearance_fn=clearance_full, robot_pose_odom=ORIGIN_POSE)
        out = core.update(person_x=6.0, person_y=0.0, t=0.1,
                           clearance_fn=clearance_full, robot_pose_odom=ORIGIN_POSE)
        assert core.state == FollowState.TRACKING
        assert out.kind == "nav_goal"
        assert out.goal_x == pytest.approx(5.0)
        assert out.goal_y == pytest.approx(0.0)

    def test_robot_pose_odom_none_during_tracking_returns_no_update(self):
        core = self._core()
        out = core.update(person_x=5.0, person_y=0.0, t=0.0,
                           clearance_fn=clearance_full, robot_pose_odom=None)
        assert out.kind == "no_update"
        assert core.state == FollowState.TRACKING
        assert len(core.trail) == 0

    def test_prepare_state_orients_no_forward_motion(self):
        core = self._core()
        out = core.update(person_x=0.0, person_y=2.5, t=0.0,
                           clearance_fn=clearance_forward_only, robot_pose_odom=ORIGIN_POSE)
        assert core.state == FollowState.PREPARE
        assert out.kind == "retreat"
        assert out.linear_x == 0.0

    def test_robot_pose_odom_none_during_prepare_returns_stop(self):
        core = self._core()
        out = core.update(person_x=0.0, person_y=2.5, t=0.0,
                           clearance_fn=clearance_full, robot_pose_odom=None)
        assert core.state == FollowState.PREPARE
        assert out.kind == "stop"
        assert out.retreat_blocked is True

    def test_prepare_scan_failure_returns_stop(self):
        core = self._core()
        out = core.update(person_x=0.0, person_y=2.5, t=0.0,
                           clearance_fn=clearance_zero, robot_pose_odom=ORIGIN_POSE)
        assert core.state == FollowState.PREPARE
        assert out.kind == "stop"
        assert out.retreat_blocked is True

    def test_evading_state_drives_via_pure_pursuit(self):
        core = self._core()
        out1 = core.update(person_x=0.0, person_y=2.5, t=0.0,
                            clearance_fn=clearance_forward_only, robot_pose_odom=ORIGIN_POSE)
        assert core.state == FollowState.PREPARE
        out2 = core.update(person_x=0.0, person_y=1.0, t=0.1,
                            clearance_fn=clearance_forward_only, robot_pose_odom=ORIGIN_POSE)
        assert core.state == FollowState.EVADING
        assert out2.kind == "retreat"
        assert out2.linear_x == pytest.approx(0.15)
        assert out2.angular_z == pytest.approx(0.0)

    def test_evading_release_returns_to_prepare(self):
        core = self._core()
        core.update(person_x=0.0, person_y=2.5, t=0.0,
                     clearance_fn=clearance_forward_only, robot_pose_odom=ORIGIN_POSE)
        core.update(person_x=0.0, person_y=1.0, t=0.1,
                     clearance_fn=clearance_forward_only, robot_pose_odom=ORIGIN_POSE)
        assert core.state == FollowState.EVADING
        core.update(person_x=0.0, person_y=2.2, t=0.2,
                     clearance_fn=clearance_forward_only, robot_pose_odom=ORIGIN_POSE)
        assert core.state == FollowState.PREPARE

    def test_reset_returns_to_tracking_and_clears_state(self):
        core = self._core()
        core.update(person_x=0.0, person_y=1.0, t=0.0,
                     clearance_fn=clearance_full, robot_pose_odom=ORIGIN_POSE)
        assert core.state == FollowState.PREPARE
        core.reset()
        assert core.state == FollowState.TRACKING
        assert len(core.trail) == 0

    def test_clear_trail(self):
        core = self._core()
        core.update(person_x=5.0, person_y=0.0, t=0.0,
                     clearance_fn=clearance_full, robot_pose_odom=ORIGIN_POSE)
        assert len(core.trail) == 1
        core.clear_trail()
        assert len(core.trail) == 0

    def test_goal_deadzone_suppresses_repeat_goal(self):
        core = self._core(goal_deadzone_m=0.5)
        out1 = core.update(person_x=5.0, person_y=0.0, t=0.0,
                            clearance_fn=clearance_full, robot_pose_odom=ORIGIN_POSE)
        out2 = core.update(person_x=5.05, person_y=0.0, t=0.1,
                            clearance_fn=clearance_full, robot_pose_odom=ORIGIN_POSE)
        assert out1.kind == "nav_goal"
        assert out2.kind == "no_update"
