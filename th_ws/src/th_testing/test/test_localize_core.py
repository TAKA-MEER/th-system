"""
test_localize_core.py
======================
localize_core（W-01 P1: 確度スコア s・m と粗→細探索）の単体テスト。

numpy / scipy が要るため、ホストの素の python3 では走らない（Docker の colcon test で回す。
CLAUDE.md「ホストで走らない試験」の一覧に載せてある）。
**pytest.importorskip で黙ってスキップさせない**: ホストで緑に見えて何も検証していない状態を避ける。
numpy が無ければ import で素直に落ちる。

合成地図（すべて 0.05 m/セル）:
  - L 字の廊下（腕の長さが違い凹みがある）: 角では 2 番目が劣り m が大きい。直線部は m が小さい
  - 正方形の部屋: 90° 回転で自分に重なる → 別の場所に同点候補があり m が小さい
  - 長い直線廊下: スキャンの測距距離を短く切ると進行方向に区別がつかない → m が小さい
スキャンは地図と同じ世界から作る（模擬スキャン。実スキャンより楽観的。実機は P6）。
"""
import math

import numpy as np

from th_planning import localize_core as lc

RES = 0.05
N_BEAMS = 360
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / N_BEAMS
# 試験用の死角（laser_link 基準・度・[start, end] の平坦配列）。前方右側の 30° 幅
BLIND_DEG = [-60.0, -30.0]


# ------------------------------------------------------------
# 合成地図
# ------------------------------------------------------------
def _grid(w_m, h_m):
    return np.zeros((int(round(h_m / RES)), int(round(w_m / RES))), dtype=bool)


def _box(occ, x0, y0, x1, y1):
    """矩形の外周 1 セル厚の壁を引く（メートル）。"""
    a, b = int(round(x0 / RES)), int(round(x1 / RES))
    c, d = int(round(y0 / RES)), int(round(y1 / RES))
    occ[c, a:b + 1] = True
    occ[d, a:b + 1] = True
    occ[c:d + 1, a] = True
    occ[c:d + 1, b] = True


def _make(occ):
    free = _free_inside(occ)
    return lc.build_likelihood_field(occ, RES, 0.0, 0.0, free=free)


def _free_inside(occ):
    """壁の内側（外周の外を除く）を free にする。外から flood fill して外側を除く。"""
    H, W = occ.shape
    outside = np.zeros_like(occ)
    stack = [(0, 0)]
    while stack:
        y, x = stack.pop()
        if y < 0 or y >= H or x < 0 or x >= W or outside[y, x] or occ[y, x]:
            continue
        outside[y, x] = True
        stack.extend([(y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)])
    return ~occ & ~outside


def make_l_corridor():
    """L 字廊下（幅 2 m）。横腕は x 1..13・縦腕は y 1..17 と長さが違い、横腕の南壁に凹みがある。

    腕の長さを変え凹みを入れるのは、L 字の対角線に対する鏡像対称（廊下は左右対称なので
    鏡像のスキャンは回転で重なる）を崩して、全域探索で一意に決まる地図にするため。
    """
    occ = _grid(14.0, 18.0)
    pts = [(1, 1), (13, 1), (13, 17), (11, 17), (11, 3), (1, 3)]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        a, b = sorted((int(round(x0 / RES)), int(round(x1 / RES))))
        c, d = sorted((int(round(y0 / RES)), int(round(y1 / RES))))
        occ[c:d + 1, a:b + 1] = True
    # 横腕の北壁（y=3）に深さ 1.5 m・幅 1.5 m の凹み（x 4..5.5）
    a, b = int(round(4.0 / RES)), int(round(5.5 / RES))
    c, d = int(round(3.0 / RES)), int(round(4.5 / RES))
    occ[c:d + 1, a:b + 1] = True
    occ[c + 1:d, a + 1:b] = False
    occ[c, a + 1:b] = False
    return occ


def make_symmetric_room():
    """12 m 四方の空の正方形（4 回回転対称）。壁 1..11。"""
    occ = _grid(12.0, 12.0)
    _box(occ, 1.0, 1.0, 11.0, 11.0)
    return occ


def make_long_corridor():
    """幅 2 m・長さ 40 m の直線廊下。"""
    occ = _grid(42.0, 4.0)
    _box(occ, 1.0, 1.0, 41.0, 3.0)
    return occ


# ------------------------------------------------------------
# 模擬スキャン
# ------------------------------------------------------------
def raycast(occ, x, y, yaw, max_range, seed=None, noise=0.0):
    """laser_link (x, y, yaw) から N_BEAMS 本。当たらなければ inf。"""
    H, W = occ.shape
    ang = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INC
    dx, dy = np.cos(ang + yaw), np.sin(ang + yaw)
    rng = np.full(N_BEAMS, np.inf)
    alive = np.ones(N_BEAMS, dtype=bool)
    d = 0.0
    step = RES * 0.5
    while alive.any() and d < max_range:
        d += step
        gx = np.floor((x + dx * d) / RES).astype(int)
        gy = np.floor((y + dy * d) / RES).astype(int)
        oob = (gx < 0) | (gx >= W) | (gy < 0) | (gy >= H)
        hit = np.zeros(N_BEAMS, dtype=bool)
        inb = alive & ~oob
        hit[inb] = occ[gy[inb], gx[inb]]
        rng[hit] = d
        alive &= ~hit & ~oob
    if noise and seed is not None:
        rng = rng + np.random.default_rng(seed).normal(0.0, noise, N_BEAMS)
    return rng


def _search(fld, ranges, **kw):
    kw.setdefault("max_range", 40.0)
    return lc.search(fld, ranges, ANGLE_MIN, ANGLE_INC, **kw)


def _pos_err(c, x, y):
    return math.hypot(c.x - x, c.y - y)


def _yaw_err_deg(c, yaw):
    return abs(math.degrees((c.yaw - yaw + math.pi) % (2 * math.pi) - math.pi))


# 全試験で使う L 字地図と、廊下内の真値姿勢（laser_link）
L_TRUTHS = [(3.0, 2.0, 0.3), (8.0, 2.0, -2.0), (12.0, 6.0, 1.7), (12.0, 14.0, -1.2)]


def _l():
    occ = make_l_corridor()
    return occ, _make(occ)


# ------------------------------------------------------------
# 尤度場・s
# ------------------------------------------------------------
def test_likelihood_field_distance_is_metres():
    occ = np.zeros((20, 20), dtype=bool)
    occ[10, 10] = True
    fld = lc.build_likelihood_field(occ, 0.1, 0.0, 0.0)
    assert fld.dt[10, 10] == 0.0
    assert abs(fld.dt[10, 15] - 0.5) < 1e-5


def test_likelihood_field_rejects_empty_map():
    try:
        lc.build_likelihood_field(np.zeros((5, 5), dtype=bool), 0.1, 0.0, 0.0)
    except ValueError:
        return
    raise AssertionError("占有セルの無い地図を受け付けてしまった")


def test_s_is_high_at_truth_and_drops_when_shifted():
    occ, fld = _l()
    x, y, yaw = 8.0, 2.0, -2.0
    r = raycast(occ, x, y, yaw, 40.0)
    s_true = lc.score_pose(fld, r, ANGLE_MIN, ANGLE_INC, (x, y, yaw))
    assert s_true >= 0.95
    # 位置を廊下の幅方向へ 0.7 m ずらす・向きを 25° ずらす、のどちらでも大きく下がる
    # （廊下の軸方向は並進不変なので、軸方向のずらしは s が下がらない）
    s_pos = lc.score_pose(fld, r, ANGLE_MIN, ANGLE_INC, (x, y + 0.7, yaw))
    s_yaw = lc.score_pose(fld, r, ANGLE_MIN, ANGLE_INC, (x, y, yaw + math.radians(25)))
    assert s_pos < s_true - 0.3
    assert s_yaw < s_true - 0.3


def test_s_is_in_unit_range_and_zero_for_no_valid_beams():
    occ, fld = _l()
    r = np.full(N_BEAMS, np.inf)
    s = lc.score_pose(fld, r, ANGLE_MIN, ANGLE_INC, (3.0, 2.0, 0.0))
    assert s == 0.0
    r = raycast(occ, 3.0, 2.0, 0.0, 40.0)
    assert 0.0 <= lc.score_pose(fld, r, ANGLE_MIN, ANGLE_INC, (3.0, 2.0, 0.5)) <= 1.0


# ------------------------------------------------------------
# 死角
# ------------------------------------------------------------
def test_valid_beam_mask_excludes_blind_sector():
    r = np.full(N_BEAMS, 3.0)
    ang = lc.beam_angles(N_BEAMS, ANGLE_MIN, ANGLE_INC)
    deg = np.degrees(ang)
    # 境界のビームは浮動小数の丸めで内外が揺れるので、0.5° の余裕を持って内側・外側を見る
    inside = (deg > BLIND_DEG[0] + 0.5) & (deg < BLIND_DEG[1] - 0.5)
    outside = (deg < BLIND_DEG[0] - 0.5) | (deg > BLIND_DEG[1] + 0.5)
    assert inside.sum() > 10
    mask = lc.valid_beam_mask(r, ANGLE_MIN, ANGLE_INC, BLIND_DEG)
    assert not mask[inside].any()
    assert mask[outside].all()
    # 死角なしなら全部使える
    assert lc.valid_beam_mask(r, ANGLE_MIN, ANGLE_INC).all()


def test_blind_mask_wraps_around_180():
    ang = lc.beam_angles(N_BEAMS, ANGLE_MIN, ANGLE_INC)
    deg = (np.degrees(ang) + 180.0) % 360.0 - 180.0
    mask = lc.blind_beam_mask(ang, [170.0, -170.0])
    assert mask[(deg >= 175.0) | (deg <= -175.0)].all()
    assert not mask[np.abs(deg) < 100.0].any()


def test_blind_beams_do_not_affect_score():
    """死角の中のビームが壁や物でふさがれていても（ゴミ値でも）s が変わらない。"""
    occ, fld = _l()
    x, y, yaw = 8.0, 2.0, -2.0
    clean = raycast(occ, x, y, yaw, 40.0)
    ang = lc.beam_angles(N_BEAMS, ANGLE_MIN, ANGLE_INC)
    deg = np.degrees(ang)
    blind_idx = (deg >= BLIND_DEG[0]) & (deg <= BLIND_DEG[1])
    garbage = clean.copy()
    garbage[blind_idx] = 0.4  # 機体の一部が写った近距離値
    s_clean = lc.score_pose(fld, clean, ANGLE_MIN, ANGLE_INC, (x, y, yaw),
                            blind_ranges_deg=BLIND_DEG)
    s_garbage = lc.score_pose(fld, garbage, ANGLE_MIN, ANGLE_INC, (x, y, yaw),
                              blind_ranges_deg=BLIND_DEG)
    assert s_garbage == s_clean
    # 死角を知らせなければゴミが効いて下がる（試験自体が死角の影響を見られている確認）
    s_unaware = lc.score_pose(fld, garbage, ANGLE_MIN, ANGLE_INC, (x, y, yaw))
    assert s_unaware < s_clean - 0.05


def test_search_ignores_blind_garbage():
    occ, fld = _l()
    x, y, yaw = 3.0, 2.0, 0.3
    r = raycast(occ, x, y, yaw, 40.0)
    ang = lc.beam_angles(N_BEAMS, ANGLE_MIN, ANGLE_INC)
    deg = np.degrees(ang)
    blind_idx = (deg >= BLIND_DEG[0]) & (deg <= BLIND_DEG[1])
    r[blind_idx] = 0.4
    res = _search(fld, r, blind_ranges_deg=BLIND_DEG)
    assert res.best is not None
    assert _pos_err(res.best, x, y) <= 0.5 and _yaw_err_deg(res.best, yaw) <= 5.0
    assert res.s >= 0.95


# ------------------------------------------------------------
# 全域探索
# ------------------------------------------------------------
def test_global_search_finds_truth_in_l_corridor():
    occ, fld = _l()
    for k, (x, y, yaw) in enumerate(L_TRUTHS):
        r = raycast(occ, x, y, yaw, 40.0, seed=k, noise=0.02)
        res = _search(fld, r)
        assert not res.timed_out
        assert res.best is not None
        assert _pos_err(res.best, x, y) <= 0.5, (x, y, yaw, res.best)
        assert _yaw_err_deg(res.best, yaw) <= 5.0, (x, y, yaw, res.best)
        assert res.s >= 0.9
        assert res.n_coarse > 0
        assert res.candidates[0] is res.best
        assert all(a.s >= b.s for a, b in zip(res.candidates, res.candidates[1:]))


def test_l_corridor_has_large_margin():
    """角では 2 番目が劣る（m が大きい）。直線部では壁沿いに並進しても s がほぼ同じで m は
    小さい（P0 の実測と同じ性質。だから m が要る）ので、特徴のある角で見る。"""
    occ, fld = _l()
    x, y, yaw = 12.0, 2.0, 1.7
    res = _search(fld, raycast(occ, x, y, yaw, 40.0))
    assert res.second is not None
    assert res.m >= 0.15, res.m
    # 2 番目は「別の場所」であって、最良の近傍ではない
    assert _pos_err(res.second, res.best.x, res.best.y) >= 1.0


def test_symmetric_room_has_small_margin():
    """4 回回転対称の部屋では別の場所に同点候補があり、m がほぼ 0（取り違えが検知できる）。"""
    occ = make_symmetric_room()
    fld = _make(occ)
    x, y, yaw = 3.0, 3.0, 0.4
    res = _search(fld, raycast(occ, x, y, yaw, 40.0))
    assert res.best is not None and res.second is not None
    assert res.s >= 0.9
    assert res.m <= 0.1, res.m
    # 2 番目は最良から m_sep_m 以上離れた別の場所を指している
    assert _pos_err(res.second, res.best.x, res.best.y) >= 1.0
    # 対称でない L 字より明確に m が小さい
    occ_l, fld_l = _l()
    m_l = _search(fld_l, raycast(occ_l, 12.0, 2.0, 1.7, 40.0)).m
    assert res.m + 0.15 < m_l


def test_long_corridor_is_ambiguous_when_end_walls_out_of_range():
    """直線廊下の中ほどで、端の壁が測距範囲外なら進行方向に区別がつかず m が小さい。"""
    occ = make_long_corridor()
    fld = _make(occ)
    x, y, yaw = 20.0, 2.0, 0.0
    short = raycast(occ, x, y, yaw, 6.0)  # 6 m 先までしか見えない（端の壁は 20 m 先）
    res = _search(fld, short, max_range=6.0)
    assert res.s >= 0.9
    assert res.m <= 0.1, res.m


# ------------------------------------------------------------
# 範囲指定（widen_search）
# ------------------------------------------------------------
def test_region_search_finds_truth_when_inside_region():
    occ, fld = _l()
    x, y, yaw = 8.0, 2.0, -2.0
    r = raycast(occ, x, y, yaw, 40.0)
    res = _search(fld, r, center=(9.0, 2.0), radius=3.0)
    assert res.best is not None
    assert _pos_err(res.best, x, y) <= 0.5 and _yaw_err_deg(res.best, yaw) <= 5.0


def test_region_search_never_returns_outside_region():
    """真値が範囲外のとき、範囲内の別の姿勢を返す。範囲の外は 1 件も返さない。"""
    occ, fld = _l()
    x, y, yaw = 3.0, 2.0, 0.3           # 真値は横腕の左端
    r = raycast(occ, x, y, yaw, 40.0)
    center, radius = (12.0, 9.0), 2.0   # 縦腕の上のほう（真値から遠い）
    res = _search(fld, r, center=center, radius=radius)
    assert res.best is not None
    for c in res.candidates + ([res.second] if res.second else []):
        assert math.hypot(c.x - center[0], c.y - center[1]) <= radius + 1e-6, c
    assert _pos_err(res.best, x, y) > 3.0  # 範囲外の真値を返していない


def test_region_args_must_be_paired():
    occ, fld = _l()
    r = raycast(occ, 3.0, 2.0, 0.3, 40.0)
    for kw in ({"center": (3.0, 2.0)}, {"radius": 2.0}):
        try:
            _search(fld, r, **kw)
        except ValueError:
            continue
        raise AssertionError("center/radius の片方だけを受け付けた: %r" % (kw,))


def test_small_region_has_no_second_candidate():
    """範囲が m_sep_m より狭いと別の場所を取れない（second=None、m=s）。"""
    occ, fld = _l()
    r = raycast(occ, 8.0, 2.0, -2.0, 40.0)
    res = _search(fld, r, center=(8.0, 2.0), radius=1.0)
    assert res.second is None
    assert res.m == res.s


# ------------------------------------------------------------
# 打ち切り
# ------------------------------------------------------------
class _TickClock:
    """呼ばれるたびに step 秒進む時計（実時間に依存しない打ち切り試験用）。"""

    def __init__(self, step):
        self.t = 0.0
        self.step = step

    def __call__(self):
        self.t += self.step
        return self.t


def test_timeout_during_coarse_returns_nothing():
    occ, fld = _l()
    r = raycast(occ, 8.0, 2.0, -2.0, 40.0)
    res = _search(fld, r, timeout_s=0.0)
    assert res.timed_out
    assert res.best is None and res.s == 0.0 and res.m == 0.0


def test_timeout_during_refinement_keeps_partial_best_but_zero_margin():
    occ, fld = _l()
    r = raycast(occ, 8.0, 2.0, -2.0, 40.0)
    full = _search(fld, r)
    assert not full.timed_out and full.m > 0.0
    # 打ち切りの時刻を、粗探索は済み・精密化の途中にあたる所へ寄せる。時計が呼ばれる回数を
    # 数え、粗探索のチャンクを抜けた直後から期限切れにする（回数は粗探索のチャンク数 + 2）。
    n_chunks = math.ceil(full.n_coarse / lc.LOCALIZE_DEFAULTS["chunk"])

    class _Clock:
        def __init__(self):
            self.n = 0

        def __call__(self):
            self.n += 1
            # 1 回目は開始時刻。粗探索の n_chunks 回の確認 + 精密化 3 件ぶんまでは未到達
            return 0.0 if self.n <= 1 + n_chunks + 3 else 100.0

    res = _search(fld, r, timeout_s=10.0, clock=_Clock())
    assert res.timed_out
    assert res.best is not None and len(res.candidates) == 3
    assert res.m == 0.0  # 途中打ち切りでは確度を主張しない


def test_no_timeout_by_default_and_elapsed_is_reported():
    occ, fld = _l()
    r = raycast(occ, 3.0, 2.0, 0.3, 40.0)
    res = _search(fld, r, clock=_TickClock(0.001))
    assert not res.timed_out
    assert res.elapsed_s > 0.0


def test_params_override_and_unknown_param_rejected():
    occ, fld = _l()
    r = raycast(occ, 3.0, 2.0, 0.3, 40.0)
    res = _search(fld, r, params={"n_top": 3})
    assert len(res.candidates) <= 3
    try:
        _search(fld, r, params={"no_such": 1})
    except ValueError:
        return
    raise AssertionError("未知のパラメータを受け付けた")
