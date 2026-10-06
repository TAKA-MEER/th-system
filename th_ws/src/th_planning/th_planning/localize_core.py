#!/usr/bin/env python3
# ============================================================
# localize_core.py — 全域ローカライズの確度スコアと粗→細探索（W-01 P1）
# ============================================================
# 設計: docs/plan/detailed/DetailedDesign-transit-localize.md
#       docs/plan/detailed/DetailedDesign-transit-localize-options.md §1・§2
# 下敷き: docs/plan/detailed/tools/w01_p0_measure.py（P0 の実測で刻み・m の数え方を確認済み）
#
# ROS2 を import しない純粋関数。numpy と scipy（distance_transform_edt）が要る。
# ホストの python3 には numpy が無いので、この試験は Docker（colcon test）で回す。
# route_replay_core.py に入れなかったのは、そちらの試験がホストで走っており、
# numpy を要求するとホストで全滅するため。
#
# 座標・姿勢の約束:
#   - 姿勢 (x, y, yaw) は「laser_link の map フレームでの姿勢」。base_link ではない。
#     呼び出し側（replay_runner 等）が TF で変換して渡す。
#   - 地図は占有格子 occ[gy, gx]（True=占有）。セル (gx, gy) の左下隅が
#     (origin_x + gx*res, origin_y + gy*res)。pgm は先頭行が最大 y なので、
#     読み込み側で上下反転して渡すこと（P0 の load_map と同じ）。
#   - スキャンは ranges と angle_min / angle_increment で受ける（laser_link 基準・反時計回り正）。
#
# 確度:
#   s = 有効ビームの端点のうち、尤度場（占有セルまでの距離）が許容 tol 以下の割合（0〜1）。
#   m = 最良候補の s と、「最良から LOCALIZE_DEFAULTS['m_sep_m'] 以上離れた別の場所」の
#       最良候補の s との差。最良の近傍（同じ場所の別刻み）は 2 番目にしない
#       （近傍を 2 番目にすると m が常に小さくなる。options §2）。
# ============================================================
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

# ------------------------------------------------------------
# 既定値（1 か所に集約。名前は P5 で registry に載せる。値の根拠は P0 の測定記録）
# ------------------------------------------------------------
LOCALIZE_DEFAULTS = {
    # 粗探索の刻み。1 m では真値の谷が埋もれた（P0）ので 0.5 m / 10° ＋ビーム間引き
    "coarse_xy_m": 0.5,
    "coarse_yaw_deg": 10.0,
    "beam_stride": 4,            # 粗探索だけ n 本に 1 本を使う（細探索は全ビーム）
    "tol_coarse_m": 0.30,        # 一致とみなす許容（段階ごと。粗いほど広く）
    "tol_mid_m": 0.15,
    "tol_fine_m": 0.10,
    "nms_sep_m": 1.5,            # 粗探索の上位候補どうしの最小距離
    "n_top": 60,                 # 精密化する粗候補の数
    "m_sep_m": 2.0,              # 「別の場所」とみなす距離
    # 精密化 B: ±1 m / 0.25 m、±15° / 5°。精密化 C: ±0.3 m / 0.1 m、±5° / 1°
    "refine_b": (1.0, 0.25, 15.0, 5.0),
    "refine_c": (0.3, 0.1, 5.0, 1.0),
    "min_range_m": 0.1,
    "max_range_m": 40.0,
    "chunk": 20000,              # 候補を何件ずつ採点するか（メモリ上限と打ち切り確認の粒度）
    # ── 確度の判定境界（W-01 P2。値の根拠は P0 の測定記録） ──
    # s 単独では成功と失敗が重なる（成功の s 最小 0.75〜0.97・失敗の s 最大
    # 0.87〜0.95）。occluded 条件の成功中央値 0.86〜0.87 を下回らない出発点として
    # 0.8 に置く。実スキャン（P0 は同一地図の模擬で楽観的）での詰めは P6。
    "localize_match_low": 0.8,   # s がこれ未満 → widen の引き金（evt.localize_low）
    # 失敗 17 件の m はすべて 0.20 以下。m < 0.2 の警告で失敗は全部拾えるが
    # 成功にも誤警告が出る（成功の m 最小は 0.00〜0.05）。警告≠不成立の二段構え。
    "localize_margin_low": 0.2,   # m がこれ未満 → low_margin（似た場所あり。READY へは進む）
    # m < 0.05 で失敗の大半は止まるが、m = 0.16〜0.20 の廊下沿いズレはすり抜ける
    # ため READY 目視と併用する（P0 §4）。不成立単独に頼らない。
    "localize_margin_min": 0.05,  # m がこれ未満 → 不成立（LOCALIZE に留まる）
    # ── 探索窓（W-01 P2） ──
    # load_route 直後の探索は経路始点の周辺だけ見る。m_sep_m（2.0）より狭いと
    # 「別の場所」の候補が取れず m が効かない（P1→P2 への申送り）ため、それより
    # 広く取る。widen は最良候補の周辺のさらに広い窓。global は全域（窓なし）。
    "search_radius_m": 5.0,      # 初期探索の窓半径（経路始点中心）
    "widen_radius_m": 10.0,      # widen 再探索の窓半径（最良候補中心）
    # ── 途中復帰（W-01 P4。出発点。実スキャンでの詰めは P6） ──
    # 確定姿勢からこの距離以内の前向き点が無ければ途中復帰しない
    # （LOCALIZE に留まる。index 0 から走り出すと離れた始点へ向かうため）。
    "resume_max_dist_m": 2.0,
}


# ------------------------------------------------------------
# 型
# ------------------------------------------------------------
@dataclass
class LikelihoodField:
    """占有格子から作った尤度場。dt[gy, gx] は最寄りの占有セルまでの距離（m）。"""
    dt: np.ndarray
    free: np.ndarray
    res: float
    ox: float
    oy: float

    @property
    def height(self) -> int:
        return int(self.dt.shape[0])

    @property
    def width(self) -> int:
        return int(self.dt.shape[1])


@dataclass
class Candidate:
    x: float
    y: float
    yaw: float
    s: float


@dataclass
class SearchResult:
    best: Optional[Candidate]            # 最良候補。打ち切りで何も得られなければ None
    s: float                             # 最良候補の一致率（best が None なら 0）
    m: float                             # マージン。best が None・打ち切り時は 0（未確定）
    second: Optional[Candidate]          # m の相手（別の場所の最良）。無ければ None
    candidates: List[Candidate] = field(default_factory=list)  # 精密化した候補（s の降順）
    timed_out: bool = False
    elapsed_s: float = 0.0
    n_coarse: int = 0                    # 粗探索で採点した候補数


# ------------------------------------------------------------
# 尤度場
# ------------------------------------------------------------
def build_likelihood_field(occ, resolution: float, origin_x: float, origin_y: float,
                           free=None) -> LikelihoodField:
    """占有格子（bool、True=占有）から尤度場を作る。

    free: 粗探索の候補位置にしてよいセル（bool）。省略時は「占有でないセル全部」。
    未知セルは通過可として扱う（楽観側。P0 と同じ）ので、実用では free を
    pgm の 254 だけに絞って渡す。
    """
    occ = np.asarray(occ, dtype=bool)
    if occ.ndim != 2:
        raise ValueError("occ は 2 次元")
    if not occ.any():
        raise ValueError("占有セルが 1 つも無い地図では尤度場が作れない")
    free = (~occ) if free is None else np.asarray(free, dtype=bool)
    if free.shape != occ.shape:
        raise ValueError("free と occ の形が違う")
    dt = (distance_transform_edt(~occ) * float(resolution)).astype(np.float32)
    return LikelihoodField(dt=dt, free=free, res=float(resolution),
                           ox=float(origin_x), oy=float(origin_y))


# ------------------------------------------------------------
# 有効ビーム
# ------------------------------------------------------------
def beam_angles(n: int, angle_min: float, angle_increment: float) -> np.ndarray:
    return angle_min + np.arange(n) * angle_increment


def blind_beam_mask(angles: np.ndarray, blind_ranges_deg: Sequence[float]) -> np.ndarray:
    """死角（laser_link 基準・度・[start, end] の平坦配列）に入るビームを True にする。

    registry.yaml の blind_angle_ranges と同じ形。start > end は ±180° をまたぐ帯。
    """
    mask = np.zeros(len(angles), dtype=bool)
    if blind_ranges_deg is None or len(blind_ranges_deg) == 0:
        return mask
    if len(blind_ranges_deg) % 2 != 0:
        raise ValueError("blind_ranges_deg は [start, end] の対の平坦配列")
    deg = (np.degrees(angles) + 180.0) % 360.0 - 180.0
    for a0, a1 in zip(blind_ranges_deg[0::2], blind_ranges_deg[1::2]):
        if a0 <= a1:
            mask |= (deg >= a0) & (deg <= a1)
        else:
            mask |= (deg >= a0) | (deg <= a1)
    return mask


def valid_beam_mask(ranges, angle_min: float, angle_increment: float,
                    blind_ranges_deg: Sequence[float] = (),
                    min_range: float = LOCALIZE_DEFAULTS["min_range_m"],
                    max_range: float = LOCALIZE_DEFAULTS["max_range_m"]) -> np.ndarray:
    """採点に使ってよいビーム（有限・測距範囲内・死角の外）。"""
    r = np.asarray(ranges, dtype=np.float64)
    ang = beam_angles(len(r), angle_min, angle_increment)
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(r) & (r > min_range) & (r < max_range - 1e-6)
    return ok & ~blind_beam_mask(ang, blind_ranges_deg)


# ------------------------------------------------------------
# 採点
# ------------------------------------------------------------
def score_poses(fld: LikelihoodField, ranges, angle_min: float, angle_increment: float,
                poses, tol: float, valid: np.ndarray) -> np.ndarray:
    """poses (C,3) = x, y, yaw のそれぞれの一致率 s (C,)。valid は使うビーム（bool, len(ranges)）。

    端点が地図外のビームは不一致として数える。使えるビームが 0 本なら全候補 0。
    """
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    r = np.asarray(ranges, dtype=np.float64)
    ang = beam_angles(len(r), angle_min, angle_increment)
    rv = r[valid]
    bv = ang[valid]
    if rv.size == 0 or len(poses) == 0:
        return np.zeros(len(poses))
    yaw = poses[:, 2:3]
    ex = poses[:, 0:1] + rv[None, :] * np.cos(yaw + bv[None, :])
    ey = poses[:, 1:2] + rv[None, :] * np.sin(yaw + bv[None, :])
    gx = np.floor((ex - fld.ox) / fld.res).astype(np.int64)
    gy = np.floor((ey - fld.oy) / fld.res).astype(np.int64)
    oob = (gx < 0) | (gx >= fld.width) | (gy < 0) | (gy >= fld.height)
    look = np.full(gx.shape, np.inf, dtype=np.float32)
    ok = ~oob
    look[ok] = fld.dt[gy[ok], gx[ok]]
    return (look <= tol).mean(axis=1)


def score_pose(fld: LikelihoodField, ranges, angle_min: float, angle_increment: float,
               pose: Tuple[float, float, float],
               tol: float = LOCALIZE_DEFAULTS["tol_fine_m"],
               blind_ranges_deg: Sequence[float] = (),
               min_range: float = LOCALIZE_DEFAULTS["min_range_m"],
               max_range: float = LOCALIZE_DEFAULTS["max_range_m"]) -> float:
    """1 つの姿勢の一致率 s（0〜1）。全ビーム（間引きなし）で採る。"""
    valid = valid_beam_mask(ranges, angle_min, angle_increment, blind_ranges_deg,
                            min_range, max_range)
    return float(score_poses(fld, ranges, angle_min, angle_increment,
                             np.array([pose], dtype=np.float64), tol, valid)[0])


# ------------------------------------------------------------
# 候補の生成
# ------------------------------------------------------------
def _coarse_candidates(fld: LikelihoodField, dxy: float, dyaw_deg: float,
                       center: Optional[Tuple[float, float]],
                       radius: Optional[float]) -> np.ndarray:
    step = max(1, int(round(dxy / fld.res)))
    ys, xs = np.nonzero(fld.free[::step, ::step])
    wx = fld.ox + (xs * step + 0.5) * fld.res
    wy = fld.oy + (ys * step + 0.5) * fld.res
    if center is not None:
        keep = np.hypot(wx - center[0], wy - center[1]) <= radius
        wx, wy = wx[keep], wy[keep]
    th = np.radians(np.arange(0.0, 360.0, dyaw_deg))
    th = (th + math.pi) % (2 * math.pi) - math.pi
    n, k = len(wx), len(th)
    cands = np.empty((n * k, 3))
    cands[:, 0] = np.tile(wx, k)
    cands[:, 1] = np.tile(wy, k)
    cands[:, 2] = np.repeat(th, n)
    return cands


def _refine_grid(c, dxy: float, xy_step: float, dth_deg: float, th_step_deg: float,
                 center: Optional[Tuple[float, float]],
                 radius: Optional[float]) -> np.ndarray:
    xs = np.arange(-dxy, dxy + 1e-9, xy_step)
    ts = np.radians(np.arange(-dth_deg, dth_deg + 1e-9, th_step_deg))
    gx, gy, gt = np.meshgrid(xs, xs, ts, indexing="ij")
    g = np.stack([c[0] + gx.ravel(), c[1] + gy.ravel(), c[2] + gt.ravel()], axis=1)
    if center is not None:
        # 範囲を指定されたときは、精密化でも範囲の外へ出さない
        keep = np.hypot(g[:, 0] - center[0], g[:, 1] - center[1]) <= radius + 1e-9
        g = g[keep]
    return g


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _nms(xy: np.ndarray, s: np.ndarray, n: int, sep: float) -> List[int]:
    order = np.argsort(-s, kind="stable")
    picked: List[int] = []
    for i in order:
        if all(math.hypot(xy[i, 0] - xy[j, 0], xy[i, 1] - xy[j, 1]) >= sep for j in picked):
            picked.append(int(i))
            if len(picked) >= n:
                break
    return picked


class _Deadline:
    def __init__(self, timeout_s, clock):
        self.clock = clock
        self.t0 = clock()
        self.limit = None if timeout_s is None else self.t0 + float(timeout_s)

    def expired(self) -> bool:
        return self.limit is not None and self.clock() >= self.limit

    def elapsed(self) -> float:
        return self.clock() - self.t0


# ------------------------------------------------------------
# 粗→細の探索
# ------------------------------------------------------------
def search(fld: LikelihoodField, ranges, angle_min: float, angle_increment: float,
           blind_ranges_deg: Sequence[float] = (),
           center: Optional[Tuple[float, float]] = None,
           radius: Optional[float] = None,
           timeout_s: Optional[float] = None,
           clock=time.monotonic,
           min_range: float = LOCALIZE_DEFAULTS["min_range_m"],
           max_range: float = LOCALIZE_DEFAULTS["max_range_m"],
           params: Optional[dict] = None) -> SearchResult:
    """姿勢の粗→細探索。

    center・radius を省略すると地図全域（global_localize）。両方与えると、中心から
    radius 以内の位置だけを候補にし、精密化後の結果も範囲の外へ出さない（widen_search）。
    範囲が m_sep_m より狭いと「別の場所」の候補が取れず、m は s と同じ値になる
    （second=None。取り違えの検知は効かない。呼び出し側がそのつもりで使うこと）。

    timeout_s: 所要時間の上限（秒）。超えたら打ち切って timed_out=True で返す。
      粗探索の途中なら best=None。精密化の途中なら、それまでに精密化できた最良を
      best に入れるが、m は 0（確度を主張しない）にする。
    clock: 時計（試験で差し替える）。
    params: LOCALIZE_DEFAULTS の一部を上書きする辞書。
    """
    p = dict(LOCALIZE_DEFAULTS)
    if params:
        unknown = set(params) - set(p)
        if unknown:
            raise ValueError("未知のパラメータ: %s" % sorted(unknown))
        p.update(params)
    if (center is None) != (radius is None):
        raise ValueError("center と radius は両方指定するか、両方省略する")
    if radius is not None and radius <= 0:
        raise ValueError("radius は正")
    if center is not None:
        center = (float(center[0]), float(center[1]))

    dl = _Deadline(timeout_s, clock)
    ranges = np.asarray(ranges, dtype=np.float64)
    valid = valid_beam_mask(ranges, angle_min, angle_increment, blind_ranges_deg,
                            min_range, max_range)
    sub = np.zeros(len(ranges), dtype=bool)
    sub[::max(1, int(p["beam_stride"]))] = True
    valid_coarse = valid & sub

    def done(best=None, s=0.0, m=0.0, second=None, cands=(), timed_out=False, n=0):
        return SearchResult(best=best, s=s, m=m, second=second, candidates=list(cands),
                            timed_out=timed_out, elapsed_s=dl.elapsed(), n_coarse=n)

    # --- A: 粗探索（間引きビーム）。チャンクごとに時間を見る ---
    A = _coarse_candidates(fld, p["coarse_xy_m"], p["coarse_yaw_deg"], center, radius)
    if len(A) == 0 or not valid.any():
        return done(n=len(A))
    sA = np.empty(len(A))
    chunk = int(p["chunk"])
    for i in range(0, len(A), chunk):
        if dl.expired():
            return done(timed_out=True, n=len(A))
        sA[i:i + chunk] = score_poses(fld, ranges, angle_min, angle_increment,
                                      A[i:i + chunk], p["tol_coarse_m"], valid_coarse)
    top = _nms(A[:, :2], sA, int(p["n_top"]), p["nms_sep_m"])

    def refine(c0) -> Candidate:
        B = _refine_grid(c0, *p["refine_b"], center, radius)
        sB = score_poses(fld, ranges, angle_min, angle_increment, B, p["tol_mid_m"], valid)
        b = B[int(np.argmax(sB))]
        C = _refine_grid(b, *p["refine_c"], center, radius)
        sC = score_poses(fld, ranges, angle_min, angle_increment, C, p["tol_fine_m"], valid)
        j = int(np.argmax(sC))
        return Candidate(float(C[j, 0]), float(C[j, 1]), _wrap(float(C[j, 2])), float(sC[j]))

    # --- B+C: 上位候補を精密化 ---
    refined: List[Candidate] = []
    for i in top:
        if dl.expired():
            refined.sort(key=lambda c: -c.s)
            best = refined[0] if refined else None
            return done(best=best, s=best.s if best else 0.0, m=0.0,
                        cands=refined, timed_out=True, n=len(A))
        refined.append(refine(A[i]))
    refined.sort(key=lambda c: -c.s)
    best = refined[0]

    # --- m: 最良から m_sep_m 以上離れた粗候補のうち最良を精密化して差を取る（P0 と同じ） ---
    d = np.hypot(A[:, 0] - best.x, A[:, 1] - best.y)
    far = np.flatnonzero(d >= p["m_sep_m"])
    second = None
    if far.size:
        if dl.expired():
            return done(best=best, s=best.s, m=0.0, cands=refined, timed_out=True, n=len(A))
        second = refine(A[far[int(np.argmax(sA[far]))]])
    s2 = second.s if second is not None else 0.0
    return done(best=best, s=best.s, m=best.s - s2, second=second, cands=refined, n=len(A))


# ------------------------------------------------------------
# 確度の判定（W-01 P2）
# ------------------------------------------------------------
# RouteStatus.localize_quality に載る 4 値（''／searching／unknown は
# ノード側が lifecycle で付けるためここでは出さない）。
#   high       成立（s・m とも閾値以上。reload → evt.localize_done）
#   low_margin 成立したが m < localize_margin_low（似た場所あり。READY へは進む）
#   low        s < localize_match_low（evt.localize_low → widen_search）
#   failed     m < localize_margin_min（不成立。evt.localize_done を出さず留まる）
# 順序が意味を持つ: s 不足を先に見て widen の引き金にし、次に m の下限で
# 不成立を止め、最後に警告帯を切り分ける。
def judge_localize(s: float, m: float,
                   match_low: float = LOCALIZE_DEFAULTS["localize_match_low"],
                   margin_low: float = LOCALIZE_DEFAULTS["localize_margin_low"],
                   margin_min: float = LOCALIZE_DEFAULTS["localize_margin_min"]) -> str:
    if s < match_low:
        return "low"
    if m < margin_min:
        return "failed"
    if m < margin_low:
        return "low_margin"
    return "high"


# ------------------------------------------------------------
# 保存 pgm／yaml の読み込み（W-01 P2）
# ------------------------------------------------------------
# 教示保存時に slam_toolbox の SaveMap で併存させた <base>.pgm／.yaml を読み、
# 粗探索用の尤度場を作る。map_server への依存は持たない（replay_runner が
# deserialize の前に地図を読まないと順序が逆になる。options §1）。
# yaml は pyyaml に頼らず自前で読む（要るのは平坦な 6 キーだけ）。
def _parse_map_yaml(path: str) -> dict:
    vals: dict = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, _, raw = line.partition(":")
            vals[key.strip()] = raw.strip()
    for key in ("image", "resolution", "origin", "negate",
                "occupied_thresh", "free_thresh"):
        if key not in vals:
            raise ValueError(f"{path}: map yaml に {key} が無い")
    try:
        res = float(vals["resolution"])
        origin = [float(v) for v in vals["origin"].strip("[]").split(",")]
        negate = int(vals["negate"]) != 0
        occ_th = float(vals["occupied_thresh"])
        free_th = float(vals["free_thresh"])
    except ValueError as e:
        raise ValueError(f"{path}: map yaml の数値が読めない: {e}")
    if len(origin) != 3:
        raise ValueError(f"{path}: origin は [x, y, yaw] の 3 要素のはず: {origin}")
    if abs(_wrap(origin[2])) > 1e-6:
        raise ValueError(f"{path}: origin の yaw が 0 でない（回転した格子に"
                         f"は未対応）: {origin[2]}")
    return {"image": vals["image"], "resolution": res,
            "origin": (origin[0], origin[1]), "negate": negate,
            "occupied_thresh": occ_th, "free_thresh": free_th}


def _read_pgm(path: str) -> Tuple[np.ndarray, int, int]:
    """pgm（P5 バイナリ／P2 アスキー）を読む。(pixels[H, W] uint8, W, H)。"""
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"{path}: pgm の magic が P5/P2 でない: {magic!r}")
        tokens: List[bytes] = []
        # ヘッダ（幅・高さ・最大値）をコメント越しに集める
        raw = b""
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: pgm のヘッダが途中で終わった")
            line = line.split(b"#", 1)[0]
            raw += b" " + line
            tokens = raw.split()
        width, height, maxval = (int(tokens[0]), int(tokens[1]), int(tokens[2]))
        if width <= 0 or height <= 0 or maxval <= 0 or maxval > 65535:
            raise ValueError(f"{path}: pgm のヘッダが不正: {width}x{height} max={maxval}")
        if maxval > 255:
            raise ValueError(f"{path}: 2 バイト/画素の pgm には未対応: max={maxval}")
        if magic == b"P5":
            # ヘッダの直後（単一の空白の後）が画素列。tokens の余りは捨て、
            # 現在位置から読み直す代わりに残りをそのまま読む。
            rest = f.read()
            # tokens 消費後の区切り空白を 1 バイト戻す必要はない（read が
            # ヘッダ行末の改行の次を指している）。画素が足りなければ不正。
            if len(rest) < width * height:
                raise ValueError(f"{path}: pgm の画素が足りない "
                                 f"({len(rest)} < {width * height})")
            pixels = np.frombuffer(rest[:width * height], dtype=np.uint8)
        else:
            vals = np.fromstring(f.read().decode("ascii"), dtype=int, sep=" ")
            if vals.size < width * height:
                raise ValueError(f"{path}: pgm の画素が足りない")
            pixels = vals[:width * height].astype(np.uint8)
    return pixels.reshape(height, width), width, height


def load_pgm_map(pgm_path: str, yaml_path: str) -> LikelihoodField:
    """保存 pgm／yaml から探索用の尤度場を作る。

    pgm の先頭行が最大 y なので上下反転して渡す（P0 の load_map と同じ約束）。
    画素値→占有の換算は map_server と同じ流儀（occupied_thresh／free_thresh・
    negate）。未知セルは free 側に入れず、候補位置にもしない（build の既定
    free=~occ と違い、ここでは pgm の白だけを free にする。壁の外側の未知を
    候補にすると地図外の姿勢が最良になりやすいため）。
    """
    meta = _parse_map_yaml(yaml_path)
    pixels, _w, _h = _read_pgm(pgm_path)
    # 上下反転: pgm の 0 行目が最大 y。反転後は 0 行目が origin_y（最小 y）。
    img = pixels[::-1, :].astype(np.float64) / 255.0
    # 占有確率 o（map_server と同じ流儀。既定は黒=占有・白=空き。negate で反転）。
    occ_prob = 1.0 - img
    if meta["negate"]:
        occ_prob = 1.0 - occ_prob
    occ = occ_prob > meta["occupied_thresh"]
    free = occ_prob < meta["free_thresh"]
    return build_likelihood_field(occ, meta["resolution"],
                                  meta["origin"][0], meta["origin"][1],
                                  free=free)
