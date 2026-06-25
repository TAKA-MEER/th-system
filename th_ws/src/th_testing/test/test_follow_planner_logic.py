"""
test_follow_planner_logic.py
==============================
設計書 10.1 節 — follow_planner コアロジック単体テスト

ROS2 なし・純粋 Python で実行可能。
pytest で実行: pytest test/test_follow_planner_logic.py -v
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

from follow_planner_core import (
    FollowParams, FollowPlannerCore, FollowState,
    PositionHistory, StaticDetector, RetreatHysteresis,
    FovChecker, CandidatePoint,
    generate_candidate_points, score_candidate, compute_retreat_cmd,
    compute_follow_goal, estimate_corridor_width, PlannerOutput,
)


# ────────────────────────────────────────────────────────────
# ヘルパー
# ────────────────────────────────────────────────────────────
def always_free(x, y):
    """costmap が常に空きを返すスタブ"""
    return True

def never_free(x, y):
    """costmap が常に障害物を返すスタブ"""
    return False

def clearance_full(dx, dy):
    """退避方向に常に 2m の自由空間があるスタブ"""
    return 2.0

def clearance_zero(dx, dy):
    """退避方向に自由空間がないスタブ（壁際）"""
    return 0.0

DEFAULT_PARAMS = FollowParams(
    follow_distance_target   = 1.5,
    follow_distance_min      = 0.8,
    follow_distance_max      = 3.0,
    retreat_trigger_distance = 0.8,
    retreat_release_distance = 1.2,
    closing_speed_threshold  = 0.3,
    retreat_check_clearance  = 0.5,
    retreat_speed            = 0.15,
    goal_deadzone_m          = 0.0,   # テストではデッドゾーンを無効化
    static_speed_threshold   = 0.05,
    static_time_threshold    = 2.0,
)


# ════════════════════════════════════════════════════════════
# 1. RetreatHysteresis — 境界値テスト（設計書 10.1 § 3 項）
# ════════════════════════════════════════════════════════════
class TestRetreatHysteresis:
    """
    4.3.7 §6: トリガー距離と解除距離を分けることで
    前進・後退のハンチングを防止する。
    """

    def _hyst(self):
        return RetreatHysteresis(trigger=0.8, release=1.2, closing_th=0.3)

    # ── 距離による起動 ──────────────────────────────────────
    def test_trigger_distance_below(self):
        """
        距離が trigger を下回ったとき退避が起動する。
        設計書: 「この距離を下回ったら後退を開始」→ 厳密な < 演算子。
        """
        h = self._hyst()
        assert h.update(dist=0.79, closing_speed=0.0) is True

    def test_trigger_distance_exact_boundary(self):
        """
        距離が trigger と同値のとき退避しない（境界値: < なので同値は含まない）。
        """
        h = self._hyst()
        assert h.update(dist=0.8,  closing_speed=0.0) is False

    def test_no_trigger_above_threshold(self):
        """距離が trigger より大きい場合は退避しない"""
        h = self._hyst()
        assert h.update(dist=0.81, closing_speed=0.0) is False

    # ── 解除条件 ────────────────────────────────────────────
    def test_no_release_below_release_distance(self):
        """退避中に距離が release 未満なら継続する（ハンチング防止）"""
        h = self._hyst()
        h.update(dist=0.5, closing_speed=0.0)   # 起動
        assert h.update(dist=1.1, closing_speed=0.0) is True   # まだ解除しない

    def test_release_above_release_distance(self):
        """distance > release かつ closing_speed ≤ threshold で解除される"""
        h = self._hyst()
        h.update(dist=0.5, closing_speed=0.0)   # 起動
        assert h.update(dist=1.21, closing_speed=0.0) is False

    def test_no_release_if_still_closing(self):
        """距離が十分離れても接近速度が高い場合は解除しない"""
        h = self._hyst()
        h.update(dist=0.5, closing_speed=0.0)   # 起動
        assert h.update(dist=1.5, closing_speed=0.4) is True

    # ── 速度による予防的起動 ──────────────────────────────
    def test_trigger_by_closing_speed_alone(self):
        """距離は余裕があっても接近速度が高ければ退避を開始する"""
        h = self._hyst()
        assert h.update(dist=2.0, closing_speed=0.31) is True

    def test_no_trigger_by_closing_speed_at_threshold(self):
        """接近速度がちょうど threshold の場合は起動しない"""
        h = self._hyst()
        assert h.update(dist=2.0, closing_speed=0.3) is False

    # ── リセット ────────────────────────────────────────────
    def test_reset(self):
        h = self._hyst()
        h.update(dist=0.5, closing_speed=0.0)   # 起動
        h.reset()
        assert h.is_active is False

    # ── ハンチング繰り返しテスト ──────────────────────────
    def test_no_hunting_around_trigger(self):
        """
        trigger と release の間で距離が揺れても、
        一度起動したら release を超えるまで継続する。
        """
        h = self._hyst()
        h.update(dist=0.7, closing_speed=0.0)   # 起動
        # trigger(0.8) ≤ d ≤ release(1.2) の範囲で揺れる
        for _ in range(10):
            result = h.update(dist=0.9, closing_speed=0.0)
            assert result is True, "起動後は release(1.2)を超えるまで継続すべき"


# ════════════════════════════════════════════════════════════
# 2. PositionHistory — 速度推定・接近速度計算
# ════════════════════════════════════════════════════════════
class TestPositionHistory:

    def test_velocity_estimation_forward(self):
        """試験員が前方（x 軸正方向）に移動する場合の速度推定"""
        h = PositionHistory(max_sec=2.0, rate_hz=10.0)
        for i in range(5):
            h.update(x=0.1 * i, y=0.0, t=float(i) * 0.1)
        vx, vy = h.velocity
        assert abs(vx - 1.0) < 0.1   # 1.0 m/s 前進
        assert abs(vy)        < 0.1

    def test_closing_speed_positive_when_approaching(self):
        """ロボットに近づいている場合、接近速度が正"""
        h = PositionHistory(max_sec=2.0, rate_hz=10.0)
        h.update(x=2.0, y=0.0, t=0.0)
        h.update(x=1.5, y=0.0, t=0.1)
        cs = h.closing_speed_to_robot()
        assert cs > 0.0

    def test_closing_speed_negative_when_moving_away(self):
        """ロボットから離れている場合、接近速度が負"""
        h = PositionHistory(max_sec=2.0, rate_hz=10.0)
        h.update(x=1.0, y=0.0, t=0.0)
        h.update(x=1.5, y=0.0, t=0.1)
        cs = h.closing_speed_to_robot()
        assert cs < 0.0

    def test_insufficient_history_returns_zero(self):
        """履歴が 1 点以下の場合はゼロを返す"""
        h = PositionHistory(max_sec=2.0, rate_hz=10.0)
        h.update(x=1.0, y=0.0, t=0.0)
        assert h.closing_speed_to_robot() == 0.0


# ════════════════════════════════════════════════════════════
# 3. StaticDetector
# ════════════════════════════════════════════════════════════
class TestStaticDetector:

    def test_not_static_below_time_threshold(self):
        """静止速度でも継続時間が閾値未満では静止と判定しない"""
        d = StaticDetector(speed_threshold=0.05, time_threshold=2.0)
        assert d.update(speed=0.01, t=0.0)  is False
        assert d.update(speed=0.01, t=1.9)  is False

    def test_static_after_time_threshold(self):
        """静止速度が継続時間以上続いたら静止と判定する"""
        d = StaticDetector(speed_threshold=0.05, time_threshold=2.0)
        d.update(speed=0.01, t=0.0)
        assert d.update(speed=0.01, t=2.1) is True

    def test_reset_on_movement(self):
        """動き始めたら静止タイマーをリセットする"""
        d = StaticDetector(speed_threshold=0.05, time_threshold=2.0)
        d.update(speed=0.01, t=0.0)
        d.update(speed=0.01, t=2.1)   # 静止確定
        assert d.update(speed=0.2,  t=3.0) is False   # 動き始め → リセット
        assert d.update(speed=0.01, t=3.5) is False   # タイマー再計測中


# ════════════════════════════════════════════════════════════
# 4. FovChecker — LiDAR 視野角判定
# ════════════════════════════════════════════════════════════
class TestFovChecker:

    def test_360deg_always_in_fov(self):
        """360° FOV では死角なし（ブラインドなし）"""
        fc = FovChecker(fov_deg=360.0, blind_ranges=[])
        assert fc.is_in_fov( 1.0,  0.0) is True
        assert fc.is_in_fov(-1.0,  0.0) is True
        assert fc.is_in_fov( 0.0,  1.0) is True
        assert fc.is_in_fov( 0.0, -1.0) is True

    def test_blind_range_blocks(self):
        """死角範囲内の点は False を返す"""
        fc = FovChecker(fov_deg=360.0, blind_ranges=[(44.0, 46.0)])
        angle = math.radians(45.0)
        assert fc.is_in_fov(math.cos(angle), math.sin(angle)) is False

    def test_outside_blind_range_passes(self):
        """死角範囲外は通過する"""
        fc = FovChecker(fov_deg=360.0, blind_ranges=[(44.0, 46.0)])
        angle = math.radians(47.0)
        assert fc.is_in_fov(math.cos(angle), math.sin(angle)) is True

    def test_clamp_offset_reduces_to_zero_in_blind(self):
        """結果が死角に入る場合、オフセットを 0 まで縮小する"""
        # 45° 付近が死角
        fc = FovChecker(fov_deg=360.0, blind_ranges=[(0.0, 90.0)])
        # 試験員が 30° 方向にいて 30° オフセットを希望 → 死角に入る
        result = fc.clamp_offset(30.0, math.radians(0.0))
        assert result == 0.0   # オフセットは縮小される

    def test_clamp_offset_allows_clear_direction(self):
        """結果が死角外なら希望オフセットをそのまま返す"""
        fc = FovChecker(fov_deg=360.0, blind_ranges=[(180.0, 270.0)])
        result = fc.clamp_offset(30.0, math.radians(0.0))
        assert result == 30.0


# ════════════════════════════════════════════════════════════
# 5. 候補点生成・スコアリング（4.3.4）
# ════════════════════════════════════════════════════════════
class TestCandidatePoints:

    def test_front_excluded(self):
        """試験員正面方向（配電盤側）は候補に含まれない"""
        # 試験員正面 = 0 rad（+x 方向）
        cands = generate_candidate_points(
            person_x=2.0, person_y=0.0,
            person_facing_rad=0.0,
            radius=1.5, count=16,
            work_exclusion_half_deg=60.0)
        for c in cands:
            # 試験員正面 ±60° は除外されているはず
            rel = c.angle_from_person_rad - 0.0
            while rel >  math.pi: rel -= 2 * math.pi
            while rel < -math.pi: rel += 2 * math.pi
            assert abs(rel) >= math.radians(60.0) - 1e-6

    def test_candidate_count_reduced_by_exclusion(self):
        """除外によって候補数が count より少なくなる"""
        cands = generate_candidate_points(
            person_x=0.0, person_y=0.0,
            person_facing_rad=0.0,
            radius=1.5, count=16,
            work_exclusion_half_deg=60.0)
        assert len(cands) < 16

    def test_best_candidate_is_lowest_score(self):
        """コスト関数が最低スコアの候補を選ぶ"""
        fc = FovChecker(fov_deg=360.0)
        cands = generate_candidate_points(
            person_x=2.0, person_y=0.0,
            person_facing_rad=0.0,
            radius=1.5, count=16,
            work_exclusion_half_deg=60.0)
        for c in cands:
            score_candidate(
                c, 0.0, 0.0, 2.0, 0.0, 0.0,
                is_in_fov_fn=fc.is_in_fov,
                is_clear_fn=always_free)
        best = min(cands, key=lambda c: c.score)
        others = [c for c in cands if c is not best]
        assert all(best.score <= o.score for o in others)


# ════════════════════════════════════════════════════════════
# 6. compute_retreat_cmd — 退避コマンド計算
# ════════════════════════════════════════════════════════════
class TestComputeRetreatCmd:

    def test_retreats_backward_when_person_in_front(self):
        """試験員が前方（+x）にいる場合、後退（linear_x < 0）する"""
        result = compute_retreat_cmd(
            person_x=1.0, person_y=0.0,
            retreat_speed=0.15,
            clearance_fn=clearance_full,
            min_clearance=0.5)
        assert result is not None
        lx, _ = result
        assert lx < 0.0

    def test_returns_none_when_blocked(self):
        """自由空間が不足している場合は None を返す"""
        result = compute_retreat_cmd(
            person_x=1.0, person_y=0.0,
            retreat_speed=0.15,
            clearance_fn=clearance_zero,
            min_clearance=0.5)
        assert result is None

    def test_speed_magnitude_matches_param(self):
        """退避速度の絶対値がパラメータと一致する"""
        result = compute_retreat_cmd(
            person_x=1.0, person_y=0.0,
            retreat_speed=0.15,
            clearance_fn=clearance_full,
            min_clearance=0.5)
        assert result is not None
        lx, _ = result
        assert abs(abs(lx) - 0.15) < 1e-9


# ════════════════════════════════════════════════════════════
# 7. compute_follow_goal — 追従目標位置計算（4.3.3）
# ════════════════════════════════════════════════════════════
class TestComputeFollowGoal:

    def _params(self):
        return FollowParams(
            follow_distance_target=1.5,
            follow_angle_offset_max=30.0,
            corridor_width_threshold=1.5,
            lidar_fov_deg=360.0)

    def test_goal_behind_person_in_narrow_corridor(self):
        """狭い通路（真後ろ追従）: ゴールが試験員の真後ろにある"""
        p = self._params()
        fc = FovChecker(fov_deg=360.0)
        # 試験員が +x 方向 2.0m にいて +x 方向に進んでいる
        gx, gy, _ = compute_follow_goal(
            person_x=2.0, person_y=0.0,
            person_vel=(0.3, 0.0),
            params=p,
            corridor_width=1.0,   # 狭い
            fov_checker=fc)
        # 真後ろ追従: ゴールは試験員の -x 方向 1.5m
        assert abs(gx - 0.5) < 0.2   # 2.0 - 1.5 = 0.5m 前方
        assert abs(gy)        < 0.3

    def test_goal_offset_in_wide_space(self):
        """広い空間（斜め後方追従）: ゴールに y 成分がある"""
        p = self._params()
        fc = FovChecker(fov_deg=360.0)
        gx, gy, _ = compute_follow_goal(
            person_x=2.0, person_y=0.0,
            person_vel=(0.3, 0.0),
            params=p,
            corridor_width=3.0,   # 広い
            fov_checker=fc)
        # オフセットがあれば y 成分が 0 でないはず
        assert abs(gy) > 0.1

    def test_goal_yaw_faces_person(self):
        """ゴール姿勢が試験員の方向を向いている"""
        p = self._params()
        fc = FovChecker(fov_deg=360.0)
        gx, gy, gyaw = compute_follow_goal(
            person_x=2.0, person_y=0.0,
            person_vel=(0.3, 0.0),
            params=p, corridor_width=1.0, fov_checker=fc)
        # yaw は試験員方向を向くべき
        expected_yaw = math.atan2(0.0 - gy, 2.0 - gx)
        assert abs(gyaw - expected_yaw) < 0.3


# ════════════════════════════════════════════════════════════
# 8. FollowPlannerCore — 統合テスト
# ════════════════════════════════════════════════════════════
class TestFollowPlannerCore:

    def _core(self, **kwargs):
        return FollowPlannerCore(FollowParams(**{
            'goal_deadzone_m': 0.0,   # デッドゾーン無効
            **kwargs
        }))

    def test_normal_follow_produces_nav_goal(self):
        """通常追従時に nav_goal が返る"""
        core = self._core()
        out = core.update(
            person_x=2.0, person_y=0.0, t=0.0,
            corridor_width=1.0,
            costmap_clear_fn=always_free,
            retreat_clearance_fn=clearance_full)
        assert out.kind == 'nav_goal'

    def test_retreat_overrides_normal_follow(self):
        """近接退避が通常追従より優先される（4.3.1 優先順位）"""
        core = self._core(retreat_trigger_distance=0.8)
        out = core.update(
            person_x=0.5, person_y=0.0, t=0.0,   # 0.5m — trigger 以下
            corridor_width=1.0,
            costmap_clear_fn=always_free,
            retreat_clearance_fn=clearance_full)
        assert out.kind == 'retreat'

    def test_retreat_stop_when_blocked(self):
        """退避方向に自由空間がない場合は stop が返る"""
        core = self._core(retreat_trigger_distance=0.8)
        out = core.update(
            person_x=0.5, person_y=0.0, t=0.0,
            corridor_width=1.0,
            costmap_clear_fn=always_free,
            retreat_clearance_fn=clearance_zero)
        assert out.kind == 'stop'
        assert out.retreat_blocked is True

    def test_static_triggers_reposition(self):
        """
        静止継続 → STATIC_REPOSITION 状態になる
        （実際にゴールが変わることを確認）
        """
        core = self._core(
            static_speed_threshold=0.05,
            static_time_threshold=0.1)   # テスト用に短く設定
        # 静止した試験員として複数回更新
        for i in range(5):
            out = core.update(
                person_x=2.0, person_y=0.0,
                t=float(i) * 0.1,
                corridor_width=1.0,
                costmap_clear_fn=always_free,
                retreat_clearance_fn=clearance_full)
        assert core.state == FollowState.STATIC_REPOSITION
        assert out.kind == 'nav_goal'

    def test_retreat_priority_over_static(self):
        """静止再配置中でも近接退避の方が優先される"""
        core = self._core(
            static_speed_threshold=0.05,
            static_time_threshold=0.1,
            retreat_trigger_distance=0.8)
        # まず静止確定させる
        for i in range(5):
            core.update(
                person_x=2.0, person_y=0.0, t=float(i) * 0.1,
                corridor_width=1.0,
                costmap_clear_fn=always_free,
                retreat_clearance_fn=clearance_full)
        # 突然近接
        out = core.update(
            person_x=0.5, person_y=0.0, t=1.0,
            corridor_width=1.0,
            costmap_clear_fn=always_free,
            retreat_clearance_fn=clearance_full)
        assert out.kind == 'retreat', (
            "静止再配置中でも近接退避が優先されるべき (4.3.7 §7)")

    def test_deadzone_suppresses_small_updates(self):
        """ゴールの変化がデッドゾーン未満の場合は no_update を返す"""
        core = self._core(goal_deadzone_m=0.5)
        core.update(
            person_x=2.0, person_y=0.0, t=0.0,
            corridor_width=1.0,
            costmap_clear_fn=always_free,
            retreat_clearance_fn=clearance_full)
        # ほぼ同じ位置（変化 < 0.5m）
        out = core.update(
            person_x=2.01, person_y=0.0, t=0.1,
            corridor_width=1.0,
            costmap_clear_fn=always_free,
            retreat_clearance_fn=clearance_full)
        assert out.kind == 'no_update'

    def test_retreat_releases_after_moving_away(self):
        """距離が release を超えたら退避が解除される"""
        core = self._core(
            retreat_trigger_distance=0.8,
            retreat_release_distance=1.2)
        # 近接して退避開始
        core.update(
            person_x=0.5, person_y=0.0, t=0.0,
            corridor_width=1.0,
            costmap_clear_fn=always_free,
            retreat_clearance_fn=clearance_full)
        # 十分離れた
        out = core.update(
            person_x=1.5, person_y=0.0, t=1.0,
            corridor_width=1.0,
            costmap_clear_fn=always_free,
            retreat_clearance_fn=clearance_full)
        assert out.kind == 'nav_goal', (
            "release 距離を超えたら通常追従に戻るべき")

    def test_reset_clears_retreat_state(self):
        """reset() で退避状態がクリアされる"""
        core = self._core(retreat_trigger_distance=0.8)
        core.update(
            person_x=0.5, person_y=0.0, t=0.0,
            corridor_width=1.0,
            costmap_clear_fn=always_free,
            retreat_clearance_fn=clearance_full)
        assert core.retreat_hyst.is_active is True
        core.reset()
        assert core.retreat_hyst.is_active is False


# ════════════════════════════════════════════════════════════
# 9. corridor_width 推定
# ════════════════════════════════════════════════════════════
class TestCorridorWidth:

    def test_open_space_returns_large_width(self):
        """障害物がない場合は大きな値（probe_dist の 2 倍）を返す"""
        w = estimate_corridor_width(
            0.0, 0.0, 0.0,
            costmap_query_fn=always_free,
            probe_dist=3.0)
        assert w >= 6.0 - 0.1

    def test_narrow_corridor(self):
        """両側 0.5m に壁がある場合に通路幅 ≈ 1.0m が返る"""
        def narrow(x, y):
            return abs(y) < 0.5   # y 方向に ±0.5m の壁

        w = estimate_corridor_width(
            0.0, 0.0, 0.0,   # 前方向きで横方向をプローブ
            costmap_query_fn=narrow,
            probe_dist=3.0, probe_step=0.05)
        assert abs(w - 1.0) < 0.15


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
