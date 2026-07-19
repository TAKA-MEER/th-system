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
    mapless_target_speed, rate_limit,
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

    def test_person_at_check_distance_not_blocked(self):
        """追従対象自身の反射(person_angle/person_range と一致)は障害物とみなさない"""
        ranges = scan_with_obstacle_at(0.0, 0.4)
        assert not is_path_blocked(
            ranges, ANGLE_MIN, ANGLE_INC,
            direction_rad=0.0, half_width_rad=math.radians(20), max_check_dist=1.0,
            person_angle_rad=0.0, person_range_m=0.4)

    def test_real_obstacle_nearer_than_person_still_blocks(self):
        """追従対象より手前の障害物は距離が異なるため通常どおり検知される"""
        ranges = scan_with_obstacle_at(0.0, 0.4)
        assert is_path_blocked(
            ranges, ANGLE_MIN, ANGLE_INC,
            direction_rad=0.0, half_width_rad=math.radians(20), max_check_dist=1.0,
            person_angle_rad=0.0, person_range_m=0.9)

    def test_obstacle_outside_person_tolerance_still_blocks(self):
        """追従対象の位置から exclude_tolerance を超えて離れた反射は通常どおり検知される"""
        ranges = scan_with_obstacle_at(0.0, 0.4)
        assert is_path_blocked(
            ranges, ANGLE_MIN, ANGLE_INC,
            direction_rad=0.0, half_width_rad=math.radians(20), max_check_dist=1.0,
            person_angle_rad=0.0, person_range_m=0.9, person_exclude_tolerance_m=0.1)


# ════════════════════════════════════════════════════════════
# 3. 速度プロファイル(制動距離ベースの目標速度・加減速レート制限)
# ════════════════════════════════════════════════════════════
class TestMaplessTargetSpeed:
    def test_zero_at_stop_distance(self):
        assert mapless_target_speed(1.0, stop_distance=1.0,
                                     max_decel_mps2=1.0, v_max=0.3) == pytest.approx(0.0)

    def test_saturates_at_v_max_when_margin_is_large(self):
        assert mapless_target_speed(1.0 + 0.5, stop_distance=1.0,
                                     max_decel_mps2=1.0, v_max=0.3) == pytest.approx(0.3)

    def test_braking_distance_profile_midway(self):
        # margin=0.02m, v = sqrt(2 * 1.0 * 0.02) = 0.2
        assert mapless_target_speed(1.02, stop_distance=1.0,
                                     max_decel_mps2=1.0, v_max=0.3) == pytest.approx(0.2)

    def test_never_negative_when_closer_than_stop_distance(self):
        assert mapless_target_speed(0.2, stop_distance=1.0,
                                     max_decel_mps2=1.0, v_max=0.3) == pytest.approx(0.0)

    def test_higher_v_max_still_bounded_by_braking_distance(self):
        """v_max を上げても、近距離では制動距離の制約で低く抑えられる
        (追従対象への追突防止)"""
        near = mapless_target_speed(1.05, stop_distance=1.0, max_decel_mps2=1.0, v_max=1.0)
        assert near == pytest.approx(math.sqrt(2 * 1.0 * 0.05))
        assert near < 1.0


class TestRateLimit:
    def test_caps_acceleration(self):
        assert rate_limit(target=1.0, prev=0.0, max_delta=0.1) == pytest.approx(0.1)

    def test_caps_deceleration(self):
        assert rate_limit(target=0.0, prev=0.3, max_delta=0.1) == pytest.approx(0.2)

    def test_passes_through_within_limit(self):
        assert rate_limit(target=0.25, prev=0.2, max_delta=0.1) == pytest.approx(0.25)

    def test_asymmetric_decel_faster_than_accel(self):
        """減速側だけ別の(大きい)上限を使える — 停止応答を加速立ち上がりより速くする"""
        assert rate_limit(target=0.0, prev=0.5, max_delta=0.1,
                           max_delta_down=0.3) == pytest.approx(0.2)
        assert rate_limit(target=1.0, prev=0.0, max_delta=0.1,
                           max_delta_down=0.3) == pytest.approx(0.1)


# ════════════════════════════════════════════════════════════
# 4. MaplessFollowCore 統合テスト
# ════════════════════════════════════════════════════════════
class TestMaplessFollowCore:
    def _core(self, **overrides):
        params = MaplessFollowParams(**{**DEFAULT_PARAMS.__dict__, **overrides})
        return MaplessFollowCore(params)

    def test_tracking_drives_via_pure_pursuit(self):
        """十分な周期を経て速度が定常状態(v_max)に収束することを確認する"""
        core = self._core()
        core.update(person_x=5.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        out = None
        for _ in range(10):
            out = core.update(person_x=6.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                               scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.TRACKING
        assert out.reason == "tracking"
        assert out.linear_x == pytest.approx(0.3)
        assert out.angular_z == pytest.approx(0.0)

    def test_speed_ramps_up_gradually_not_instantly(self):
        """急発進防止: 静止状態から数周期(max_linear_accel_mps2 で決まる)かけて
        v_max まで立ち上がり、1周期でいきなり目標速度には飛ばない"""
        core = self._core(max_linear_accel_mps2=1.0, update_rate_hz=10.0)
        out1 = core.update(person_x=5.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                            scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        out2 = core.update(person_x=6.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                            scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert out1.linear_x == pytest.approx(0.1)
        assert out2.linear_x == pytest.approx(0.2)
        assert out2.linear_x < 0.3   # 目標(v_max, 距離が遠いので飽和)へ一気に到達していない

    def test_speed_resumes_from_zero_after_stop_not_from_stale_value(self):
        """STOPPED から復帰した直後は前回速度を引き継がず 0 から加速する"""
        core = self._core(max_linear_accel_mps2=1.0, update_rate_hz=10.0)
        # 遠くで巡航速度に到達させる
        core.update(person_x=5.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        for _ in range(10):
            core.update(person_x=6.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                         scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.TRACKING
        # 急接近 → STOPPED
        core.update(person_x=0.5, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert core.state == MaplessState.STOPPED
        # 離れて復帰した直後の一周期目は 0 から加速し始める(いきなり巡航速度に戻らない)
        out = core.update(person_x=6.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert 0.0 < out.linear_x <= 0.1 + 1e-9

    def test_large_steering_error_reduces_forward_speed(self):
        """大きく曲がる必要がある時は並進を絞る(曲がりきれず壁に衝突するのを防ぐ)"""
        core = self._core()
        out = core.update(person_x=0.0, person_y=2.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert out.reason == "tracking"
        assert out.linear_x == pytest.approx(0.0, abs=1e-9)
        assert out.angular_z != 0.0

    def test_w_max_rad_s_clamps_turn_rate(self):
        core = self._core(w_max_rad_s=0.5)
        out = core.update(person_x=0.0, person_y=2.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert out.angular_z == pytest.approx(0.5)

    def test_decel_uses_decel_limit_not_accel_limit(self):
        """接近時の減速は max_linear_decel_mps2 (加速側より大) でレート制限される"""
        core = self._core(v_max=1.0, max_linear_accel_mps2=1.0,
                           max_linear_decel_mps2=3.0, update_rate_hz=10.0)
        core.update(person_x=5.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        for _ in range(15):
            out = core.update(person_x=6.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                               scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert out.linear_x == pytest.approx(1.0)
        # 突然 stop_distance ぎりぎりまで接近: 1周期の減速幅は decel/hz = 0.3
        out = core.update(person_x=1.05, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert out.linear_x == pytest.approx(0.7)

    def test_notify_lost_resets_prev_speed_so_recovery_ramps_from_zero(self):
        """ロスト中に notify_lost() を呼んでおけば再検出後は 0 から加速し直す
        (ロスト直前の速度のまま進み続けるのを防ぐ。ROS2ノード側は notify_lost()
        と併せて毎周期停止指令を publish し続ける想定)"""
        core = self._core(max_linear_accel_mps2=1.0, update_rate_hz=10.0)
        core.update(person_x=5.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        for _ in range(10):
            core.update(person_x=6.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                         scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        core.notify_lost()
        out = core.update(person_x=6.0, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert 0.0 < out.linear_x <= 0.1 + 1e-9

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

    def test_person_own_leg_reflection_does_not_false_trigger_obstacle_stop(self):
        """
        追従対象の脚表面までの生LiDAR距離は、脚トラッカーが報告する重心距離
        (person_x/y から算出) と数十cm程度ずれうる。stop_distance(0.5m) と
        obstacle_check_distance_m(0.45m) が近接した設定では、このズレだけで
        追従対象自身を障害物として誤検知し得ることの回帰テスト。
        """
        core = self._core(stop_distance=0.5, resume_distance=0.7,
                           obstacle_check_distance_m=0.45, lookback_distance=1.0)
        core.update(person_x=0.9, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        # 追従対象の重心距離は 0.75m だが、脚表面での反射は 0.5m とする
        blocked_scan = scan_with_obstacle_at(0.0, 0.5)
        out = core.update(person_x=0.75, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=blocked_scan, scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert out.reason == "tracking"

    def test_real_obstacle_still_stops_despite_person_exclusion(self):
        """追従対象の重心距離から大きく離れた反射は通常どおり障害物として検知する"""
        core = self._core(stop_distance=0.5, resume_distance=0.7,
                           obstacle_check_distance_m=0.45, lookback_distance=1.0)
        core.update(person_x=0.9, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                     scan_ranges=clear_scan(), scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        blocked_scan = scan_with_obstacle_at(0.0, 0.2)
        out = core.update(person_x=0.75, person_y=0.0, robot_pose_odom=ORIGIN_POSE,
                           scan_ranges=blocked_scan, scan_angle_min=ANGLE_MIN, scan_angle_increment=ANGLE_INC)
        assert out.reason == "obstacle_ahead"

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
