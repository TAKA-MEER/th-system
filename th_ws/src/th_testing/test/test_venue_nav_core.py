"""venue_nav_core の純ロジックテスト (WP-ONSITE-02。ROS2 不要)"""
import math
import os
import sys

# ── パスを通す（colcon build 前にも直接 pytest できるように）──
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_onsite'))

from th_onsite.venue_nav_core import (  # noqa: E402
    GOTO_BLOCKED_MODES, VenueNavParams, align_cmd_wz, arrived, find_home_goal,
    VENUE_NOT_LOADED_MESSAGE, select_pin_allowed, select_pin_reject_message,
    venue_blocks_autonomy,
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


# ── 引いた経路を機体の形で検査する（Spec-onsite §6.1.1。2026-10-22）──────────
import time  # noqa: E402

from th_onsite.venue_nav_core import (  # noqa: E402
    FOOTPRINT_BLOCKED_MIN, FOOTPRINT_HALF_M, footprint_collision_at,
    path_footprint_collision,
)


class _FGrid(_Grid):
    """述語で区画を塗れる障害物マップ（既定は 0.05 m・200x200・原点 (-5, -5)）。"""

    def fill(self, pred, val):
        for iy in range(self.height):
            y = self.origin_y + (iy + 0.5) * self.resolution
            for ix in range(self.width):
                x = self.origin_x + (ix + 0.5) * self.resolution
                if pred(x, y):
                    self.data[iy * self.width + ix] = val


def _corridor(gap, val=100, x0=1.0, x1=2.0):
    """x0..x1 の間だけ、y=0 を中心に壁面の間隔が gap の通路。"""
    g = _FGrid()
    half = gap / 2.0
    g.fill(lambda x, y: x0 <= x < x1 and abs(y) > half, val)
    return g


def _straight(x0=-2.0, x1=4.0, step=0.05):
    n = int(round((x1 - x0) / step))
    return [(x0 + k * step, 0.0) for k in range(n + 1)]


class TestPathFootprintCollision:
    def test_wide_corridor_passes(self):
        assert path_footprint_collision(_straight(), _corridor(1.3)) is None

    def test_one_metre_corridor_passes(self):
        # 実機で自動走行に成功した幅（Spec-onsite §6.1.1）
        assert path_footprint_collision(_straight(), _corridor(1.0)) is None

    def test_exact_fit_passes(self):
        # 壁面の間隔 0.60 m（セル境界）は機体 0.55 m が入る
        assert path_footprint_collision(_straight(), _corridor(0.60)) is None

    def test_narrow_gap_rejected(self):
        # 壁面の間隔 0.50 m は機体が入らない
        hit = path_footprint_collision(_straight(), _corridor(0.50))
        assert hit is not None
        assert 0.7 <= hit['x'] <= 2.3
        assert hit['cell'][2] == 100

    def test_cell_extent_is_counted(self):
        # 壁のセル中心の間隔は 0.55 m（機体幅と同じ）だが、壁面の間隔は 0.50 m。
        # セル中心だけで見ると通してしまう。セルの正方形で見ること。
        g = _FGrid()
        g.fill(lambda x, y: 1.0 <= x < 2.0 and abs(y) > 0.24, 100)
        assert path_footprint_collision(_straight(), g) is not None

    def test_inscribed_value_is_not_a_hit(self):
        # 内接(99)は壁そのものではない。致死(100)だけを当たりにする
        assert FOOTPRINT_BLOCKED_MIN == 100
        assert path_footprint_collision(_straight(), _corridor(0.50, val=99)) is None

    def test_unknown_is_not_a_hit(self):
        assert path_footprint_collision(_straight(), _corridor(0.50, val=-1)) is None

    def test_free_map_passes(self):
        assert path_footprint_collision(_straight(), _FGrid()) is None

    def test_diagonal_corridor(self):
        path = [(-2.0 + 0.05 * k, -2.0 + 0.05 * k) for k in range(81)]

        def corridor(width):
            g = _FGrid()
            g.fill(lambda x, y: abs(y - x) / math.sqrt(2.0) > width / 2.0
                   and -1.0 < x < 1.0, 100)
            return g

        assert path_footprint_collision(path, corridor(1.3)) is None
        assert path_footprint_collision(path, corridor(0.5)) is not None

    def test_zigzag_path_heading_is_smoothed(self):
        # 8 近傍の階段状の経路（0.05 m 刻み）でも、向きが揺れて広い通路を拒否しない
        path = []
        x = y = -2.0
        for k in range(80):
            x += 0.05
            path.append((x, y))
            y += 0.05
            path.append((x, y))
        g = _FGrid()
        g.fill(lambda px, py: abs(py - px) / math.sqrt(2.0) > 0.9, 100)
        assert path_footprint_collision(path, g) is None

    def test_goal_yaw_hits_wall(self):
        # 終点(2,0)。進行方向(0)では壁面 x=2.35 に届かないが、45 度の向きは角が当たる
        g = _FGrid()
        g.fill(lambda x, y: x >= 2.35 and abs(y) < 1.0, 100)
        path = _straight(-1.0, 2.0)
        assert path_footprint_collision(path, g, goal_yaw=0.0) is None
        hit = path_footprint_collision(path, g, goal_yaw=math.pi / 4)
        assert hit is not None
        assert abs(hit['x'] - 2.0) < 1e-6
        approx(hit['yaw'], math.pi / 4)

    def test_start_may_spin_to_a_clear_heading(self):
        # 出発点は右の壁(x=0.35)に 135 度の向きで角が当たるが、その場で回れば当たらない。
        # 進行方向(左上)へ動けば壁から離れる。
        g = _FGrid()
        g.fill(lambda x, y: x >= 0.35 and abs(y) < 1.0, 100)
        path = [(-0.05 * k, 0.05 * k) for k in range(41)]
        assert footprint_collision_at(g, 0.0, 0.0, 3 * math.pi / 4) is not None
        assert path_footprint_collision(path, g) is None

    def test_start_boxed_in_is_rejected(self):
        # どの向きでも当たる袋小路の出発点
        g = _FGrid()
        g.fill(lambda x, y: abs(x) > 0.3 or abs(y) > 0.3, 100)
        path = [(0.05 * k, 0.0) for k in range(21)]
        assert path_footprint_collision(path, g) is not None

    def test_path_off_map_is_rejected(self):
        hit = path_footprint_collision([(0.0, 0.0), (9.0, 0.0)], _FGrid())
        assert hit is not None and hit['cell'][2] == -1

    def test_none_grid_or_empty_path_is_none(self):
        assert path_footprint_collision(_straight(), None) is None
        assert path_footprint_collision([], _FGrid()) is None

    def test_single_point_path(self):
        g = _FGrid()
        assert path_footprint_collision([(0.0, 0.0)], g, goal_yaw=0.3) is None
        g.fill(lambda x, y: abs(x) < 0.1 and abs(y) < 0.1, 100)
        assert path_footprint_collision([(0.0, 0.0)], g, goal_yaw=0.3) is not None

    def test_collision_at_reports_the_cell(self):
        g = _FGrid()
        g.fill(lambda x, y: abs(x - 0.2) < 0.03 and abs(y) < 0.03, 100)
        cx, cy, v = footprint_collision_at(g, 0.0, 0.0, 0.0)
        assert v == 100 and abs(cx - 0.2) < 0.06
        assert footprint_collision_at(g, 0.0, 0.0, 0.0, half=0.15) is None

    def test_footprint_half_matches_nav2_params(self):
        # nav2_params.yaml の global_costmap footprint（±0.275）と同じこと
        here = os.path.dirname(__file__)
        path = os.path.join(here, '..', '..', 'th_bringup', 'config',
                            'nav2_params.yaml')
        text = open(path, encoding='utf-8').read()
        assert '[[0.275, 0.275], [-0.275, 0.275]' in text
        assert FOOTPRINT_HALF_M == 0.275

    def test_long_path_is_fast(self):
        g = _corridor(1.3, x0=-4.0, x1=4.0)
        path = _straight(-4.5, 4.5)
        t0 = time.perf_counter()
        for _ in range(5):
            assert path_footprint_collision(path, g) is None
        per_call = (time.perf_counter() - t0) / 5
        assert per_call < 0.25, f'{per_call * 1000:.0f} ms'


class TestVenueBlocksAutonomy:
    """SM-3.1.2-119。UNLOADED と届いているときだけ塞ぐ。"""

    def test_only_unloaded_blocks(self):
        assert venue_blocks_autonomy('UNLOADED') is True

    def test_loaded_mapping_and_unreceived_do_not_block(self):
        # LOCALIZING＝会場地図あり／MAPPING＝前日 PREP の地図作成中／''＝未受信（起動直後）
        for mode in ('LOCALIZING', 'MAPPING', '', 'whatever'):
            assert venue_blocks_autonomy(mode) is False, mode

    def test_message_tells_what_to_do(self):
        assert '会場地図' in VENUE_NOT_LOADED_MESSAGE
        assert '開き直' in VENUE_NOT_LOADED_MESSAGE
