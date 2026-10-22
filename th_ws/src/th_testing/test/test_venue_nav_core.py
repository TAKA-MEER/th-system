"""venue_nav_core の純ロジックテスト (WP-ONSITE-02。ROS2 不要)"""
import math
import os
import sys

# ── パスを通す（colcon build 前にも直接 pytest できるように）──
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_onsite'))

from th_onsite.venue_nav_core import (  # noqa: E402
    GOTO_BLOCKED_MODES, VenueNavParams, align_cmd_wz, arrived, find_home_goal,
    select_pin_allowed, select_pin_reject_message,
    should_unblock_for_arrival, yaw_error)


def approx(a, b, eps=1e-6):
    assert abs(a - b) < eps, f'{a} != {b}'


class TestYawError:
    def test_basic_positive(self):
        approx(yaw_error(0, math.pi / 2), math.pi / 2)

    def test_basic_negative(self):
        approx(yaw_error(math.pi / 2, 0), -math.pi / 2)

    def test_wrap_plus_pi(self):
        # (3, -3): 差 = -6。(-6+π) % 2π - π ≈ +0.283
        approx(yaw_error(3, -3), 0.283185307179586, 1e-3)

    def test_wrap_minus_pi(self):
        # (-3, 3): 差 = 6。(6+π) % 2π - π ≈ -0.283
        approx(yaw_error(-3, 3), -0.283185307179586, 1e-3)


class TestAlignCmdWz:
    def test_small_error_done(self):
        # 誤差 0.05 < tol 0.09 → (0.0, True)
        wz, done = align_cmd_wz(0, 0.05, VenueNavParams(align_tolerance_rad=0.09))
        assert wz == 0.0
        assert done is True

    def test_large_error_clamp(self):
        # 誤差 1.0 → clamp(1.2*1.0, ±0.6)=0.6, False
        wz, done = align_cmd_wz(0, 1.0, VenueNavParams())
        approx(wz, 0.6)
        assert done is False

    def test_negative_error_clamp(self):
        # 誤差 -1.0 → -0.6, False
        wz, done = align_cmd_wz(0, -1.0, VenueNavParams())
        approx(wz, -0.6)
        assert done is False


class TestArrived:
    def test_arrived_true(self):
        # (0,0)-(0.2,0) tol 0.3 → True
        assert arrived(0, 0, 0.2, 0, VenueNavParams(arrival_xy_tol_m=0.30)) is True

    def test_arrived_false(self):
        # (0,0)-(0.5,0) → False
        assert arrived(0, 0, 0.5, 0, VenueNavParams(arrival_xy_tol_m=0.30)) is False


class TestShouldUnblockForArrival:
    P = VenueNavParams(arrival_xy_tol_m=0.30)

    def test_within_tol_true(self):
        # 実機で膠着した 0.259m は tol 0.30 内 → True（BLOCKED から NAV に戻す）
        assert should_unblock_for_arrival(
            (3.656, -3.684), {'x': 3.509, 'y': -3.897}, self.P) is True

    def test_outside_tol_false(self):
        assert should_unblock_for_arrival(
            (0.0, 0.0), {'x': 1.0, 'y': 0.0}, self.P) is False

    def test_none_robot_false(self):
        assert should_unblock_for_arrival(None, {'x': 0.0, 'y': 0.0}, self.P) is False

    def test_none_goal_false(self):
        assert should_unblock_for_arrival((0.0, 0.0), None, self.P) is False


class TestFindHomeGoal:
    def test_home_found(self):
        # HOME ピン (1,2,yaw=0) を返す（PANEL があっても HOME を優先）
        pins = [
            {'kind': 'PANEL', 'x': 9, 'y': 9, 'yaw': 0.5},
            {'kind': 'HOME', 'x': 1, 'y': 2, 'yaw': 0},
        ]
        assert find_home_goal(pins) == {'x': 1, 'y': 2, 'yaw': 0}

    def test_no_home_returns_none(self):
        # PANEL のみ → None
        pins = [{'kind': 'PANEL', 'x': 9, 'y': 9, 'yaw': 0.5}]
        assert find_home_goal(pins) is None

    def test_empty_returns_none(self):
        assert find_home_goal([]) is None

    def test_none_returns_none(self):
        assert find_home_goal(None) is None


# ── Nav2 の成功の誤報（2026-10-09 実機）──
from th_onsite.venue_nav_core import success_is_plausible, SPURIOUS_SUCCESS_DIST_M  # noqa: E402


def test_success_plausible_near_goal():
    goal = {'x': 3.0, 'y': 0.0}
    assert success_is_plausible((3.0, 0.0), goal)
    assert success_is_plausible((3.0 - SPURIOUS_SUCCESS_DIST_M, 0.0), goal)   # ちょうど境界は信じる


def test_success_implausible_when_far():
    goal = {'x': 0.0, 'y': 0.0}
    assert not success_is_plausible((0.857, -1.52), goal)   # 実機: 1.7 m 手前
    assert not success_is_plausible((SPURIOUS_SUCCESS_DIST_M + 0.01, 0.0), goal)


def test_success_unknown_pose_is_trusted():
    assert success_is_plausible(None, {'x': 0.0, 'y': 0.0})
    assert success_is_plausible((0.0, 0.0), None)


# ── 塞がれた経路の先の確認（Spec-onsite §6.3）────────────────────────────
from th_onsite.venue_nav_core import (  # noqa: E402
    COSTMAP_BLOCKED_MIN, PATH_AHEAD_CHECK_M, path_ahead_clear,
)


class _Grid:
    def __init__(self, w=200, h=200, res=0.05, ox=-5.0, oy=-5.0):
        self.width, self.height, self.resolution = w, h, res
        self.origin_x, self.origin_y = ox, oy
        self.data = [0] * (w * h)

    def paint(self, x0, x1, val, y0=-0.5, y1=0.5):
        for iy in range(self.height):
            y = self.origin_y + (iy + 0.5) * self.resolution
            for ix in range(self.width):
                x = self.origin_x + (ix + 0.5) * self.resolution
                if x0 <= x < x1 and y0 <= y <= y1:
                    self.data[iy * self.width + ix] = val


_PATH = [(0.0, 0.0), (0.75, 0.0), (1.5, 0.0), (2.25, 0.0), (3.0, 0.0)]


class TestPathAheadClear:
    def test_free(self):
        assert path_ahead_clear(_PATH, (0.0, 0.0), _Grid())

    def test_lethal_within_range_blocks(self):
        g = _Grid()
        g.paint(1.0, 1.1, 100)
        assert not path_ahead_clear(_PATH, (0.0, 0.0), g)

    def test_inscribed_blocks(self):
        g = _Grid()
        g.paint(1.0, 1.1, 99)
        assert not path_ahead_clear(_PATH, (0.0, 0.0), g)

    def test_inflation_passes(self):
        g = _Grid()
        g.paint(1.0, 1.1, COSTMAP_BLOCKED_MIN - 1)
        assert path_ahead_clear(_PATH, (0.0, 0.0), g)

    def test_unknown_is_not_an_obstacle(self):
        g = _Grid()
        g.paint(1.0, 1.1, -1)
        assert path_ahead_clear(_PATH, (0.0, 0.0), g)

    def test_obstacle_between_poses_is_seen(self):
        # 経路の点(0.75, 1.5)の間にだけある薄い障害物（点の判定だけだとすり抜ける）
        g = _Grid()
        g.paint(1.1, 1.15, 100)
        assert not path_ahead_clear(_PATH, (0.0, 0.0), g)

    def test_beyond_range_is_ignored(self):
        g = _Grid()
        g.paint(PATH_AHEAD_CHECK_M + 0.3, PATH_AHEAD_CHECK_M + 0.5, 100)
        assert path_ahead_clear(_PATH, (0.0, 0.0), g)

    def test_range_is_measured_from_nearest_point(self):
        g = _Grid()
        g.paint(2.0, 2.1, 100)
        assert path_ahead_clear(_PATH, (0.0, 0.0), g)
        assert not path_ahead_clear(_PATH, (1.5, 0.0), g)

    def test_behind_robot_is_ignored(self):
        g = _Grid()
        g.paint(0.5, 0.6, 100)
        assert path_ahead_clear(_PATH, (1.5, 0.0), g)

    def test_out_of_grid_is_not_clear(self):
        g = _Grid(w=40, h=40, ox=-1.0, oy=-1.0)  # x は -1..1 まで
        assert not path_ahead_clear(_PATH, (0.0, 0.0), g)

    def test_unknowns_are_not_clear(self):
        assert not path_ahead_clear(_PATH, (0.0, 0.0), None)
        assert not path_ahead_clear(_PATH, None, _Grid())
        assert not path_ahead_clear([], (0.0, 0.0), _Grid())


class TestSelectPinAllowed:
    """SM-3.1.2-118: 移動中・一時停止中は行き先（保留ゴール）を選べない。"""

    def test_blocked_modes_match_transition_table(self):
        assert GOTO_BLOCKED_MODES == {'PANEL_NAV', 'SUMMON', 'HOME_NAV'}
        # 機体側の二重の守り（state_manager の遷移表）と同じ集合であること。
        import yaml
        path = os.path.join(os.path.dirname(__file__), '..', '..', 'th_state',
                            'config', 'transitions.yaml')
        with open(path, encoding='utf-8') as f:
            rows = yaml.safe_load(f)
        rows = rows['transitions'] if isinstance(rows, dict) else rows
        row = next(r for r in rows if r['id'] == 'T-NAV-goto-reject')
        assert set(row['mode']) == set(GOTO_BLOCKED_MODES)
        assert row['state'] == '*' and row['event'] == 'ui.goto' and row['reject']

    def test_moving_modes_rejected(self):
        for m in ('PANEL_NAV', 'SUMMON', 'HOME_NAV'):
            assert not select_pin_allowed(m)

    def test_choosable_modes_accepted(self):
        for m in ('IDLE', 'AT_HOME', 'AT_PANEL', 'PREP', ''):
            assert select_pin_allowed(m), m

    def test_message_distinguishes_pause(self):
        assert '一時停止' in select_pin_reject_message('PANEL_NAV', 'PAUSE')
        assert '移動中' in select_pin_reject_message('PANEL_NAV', 'NAV')
