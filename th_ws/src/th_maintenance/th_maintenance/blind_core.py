"""blind_core.py — 校正 BLIND（LiDAR 死角マスク）の純粋関数（ROS2・ファイル非依存）。

`DetailedDesign-maintenance.md` §3.3 の BLIND の流れと、Spec-checks.md §3.5 の
幅の上限（2026-10-01 ユーザー決定）の写実。`scripts/calib_runner.py` が使う。

**死角マスクは「その角度の点を障害物として見ない」設定**。広げすぎると本物の障害物が
見えなくなるので、1 区間・総幅・区間数に上限を設け、超える選択は適用しない
（`obstacle_limiter` の C++ にも同じ検査がある。値は registry の `blind_max_*`）。

角度はすべて度・laser_link 基準・反時計回り正。区間 `(a0, a1)` は a0 から反時計回りに a1 まで
（`a1 < a0` なら ±180° をまたぐ。`scan_geometry.sector_indices` と同じ解釈）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from th_maintenance.calib_core import Verdict

# registry.yaml（blind_max_*）と一致させる既定値。
MAX_SECTOR_DEG = 30.0
MAX_TOTAL_DEG = 90.0
MAX_SECTORS = 8

_EPS = 1e-9
_SAMPLE_STEP_DEG = 0.1

Pair = Tuple[float, float]


@dataclass(frozen=True)
class BlindSelection:
    ok: bool
    ranges: Tuple[Pair, ...] = ()
    reason: str = ""
    total_deg: float = 0.0


def normalize_deg(a: float) -> float:
    """(-180, 180] へ。"""
    r = math.fmod(a, 360.0)
    if r > 180.0:
        r -= 360.0
    elif r <= -180.0:
        r += 360.0
    return r


def sector_width_deg(a0: float, a1: float) -> float:
    """a0 から反時計回りに a1 までの幅。a0 == a1（幅ゼロ）は 0。"""
    w = math.fmod(a1 - a0, 360.0)
    if w < 0.0:
        w += 360.0
    return w


def pairs_from_flat(flat: Sequence[float]) -> List[Pair]:
    """平坦配列 `[a0,a1,a2,a3,...]` → ペア列（奇数個の余りは捨てる）。"""
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]


def flat_from_pairs(pairs: Sequence[Pair]) -> List[float]:
    out: List[float] = []
    for a0, a1 in pairs:
        out.extend([float(a0), float(a1)])
    return out


def _merge(pairs: Sequence[Pair]) -> List[Pair]:
    """重なる・接する区間を 1 つにする（±180° をまたぐ結合も含む）。"""
    items = []  # (start in [0,360), end = start + width)
    for a0, a1 in pairs:
        w = sector_width_deg(a0, a1)
        s = math.fmod(a0, 360.0)
        if s < 0.0:
            s += 360.0
        items.append([s, s + w])
    items.sort()
    merged: List[List[float]] = []
    for s, e in items:
        if merged and s <= merged[-1][1] + _EPS:
            merged[-1][1] = max(merged[-1][1], e)
        else:
            merged.append([s, e])
    # 末尾が 360° を越えて先頭へ届く場合
    if len(merged) > 1 and merged[-1][1] - 360.0 >= merged[0][0] - _EPS:
        last = merged.pop()
        merged[0][0] = last[0] - 360.0
        merged[0][1] = max(merged[0][1], last[1] - 360.0)
    return [(round(normalize_deg(s), 2), round(normalize_deg(e), 2)) for s, e in merged]


def validate_selection(raw: Sequence[Sequence[float]], *,
                       max_sector_deg: float = MAX_SECTOR_DEG,
                       max_total_deg: float = MAX_TOTAL_DEG,
                       max_sectors: int = MAX_SECTORS) -> BlindSelection:
    """画面から届いた `[[a0,a1], ...]` を検査・正規化・結合し、上限を超えたら不合格にする。

    空（マスクをすべて外す）は有効（死角の削除）。幅ゼロ・非有限・形の崩れは不合格。
    上限は**結合後**の区間で判定する（細かく分けて上限をすり抜けられない）。
    """
    if not isinstance(raw, (list, tuple)):
        return BlindSelection(False, reason="invalid_format")
    pairs: List[Pair] = []
    for r in raw:
        if not isinstance(r, (list, tuple)) or len(r) != 2:
            return BlindSelection(False, reason="invalid_format")
        try:
            a0, a1 = float(r[0]), float(r[1])
        except (TypeError, ValueError):
            return BlindSelection(False, reason="invalid_format")
        if not (math.isfinite(a0) and math.isfinite(a1)):
            return BlindSelection(False, reason="invalid_format")
        if sector_width_deg(a0, a1) <= _EPS:
            return BlindSelection(False, reason="zero_width")
        pairs.append((a0, a1))
    merged = _merge(pairs)
    return check_limits(merged, max_sector_deg=max_sector_deg,
                        max_total_deg=max_total_deg, max_sectors=max_sectors)


def check_limits(pairs: Sequence[Pair], *, max_sector_deg: float = MAX_SECTOR_DEG,
                 max_total_deg: float = MAX_TOTAL_DEG,
                 max_sectors: int = MAX_SECTORS) -> BlindSelection:
    """正規化済みの区間が上限内か。"""
    widths = [sector_width_deg(a0, a1) for a0, a1 in pairs]
    total = sum(widths)
    if any(w <= _EPS for w in widths):
        return BlindSelection(False, tuple(pairs), "zero_width", total)
    if len(pairs) > max_sectors:
        return BlindSelection(False, tuple(pairs), f"too_many_sectors:{len(pairs)}>{max_sectors}",
                              total)
    widest = max(widths, default=0.0)
    if widest > max_sector_deg + _EPS:
        return BlindSelection(False, tuple(pairs),
                              f"sector_too_wide:{widest:.1f}>{max_sector_deg:.1f}", total)
    if total > max_total_deg + _EPS:
        return BlindSelection(False, tuple(pairs),
                              f"total_too_wide:{total:.1f}>{max_total_deg:.1f}", total)
    return BlindSelection(True, tuple(pairs), "", total)


def in_ranges(angle_deg: float, pairs: Sequence[Pair]) -> bool:
    for a0, a1 in pairs:
        off = sector_width_deg(a0, angle_deg)
        if off <= sector_width_deg(a0, a1) + _EPS:
            return True
    return False


def persistent_near_ranges(frames: Sequence[Sequence[float]],
                           near_m: float) -> List[float]:
    """複数フレームの全てで近距離（0 < r < near_m）だったビームだけ値（最大）を残し、他は inf。

    1 フレームの推定は人が横切っただけでも死角に見える。「恒常的」に写り込むものだけを
    `estimate_blind_sectors` に渡すための前処理。長さの違うフレームは捨てる。
    """
    if not frames:
        return []
    n = len(frames[0])
    use = [f for f in frames if len(f) == n]
    out: List[float] = []
    for i in range(n):
        worst = 0.0
        ok = True
        for f in use:
            r = f[i]
            if not (math.isfinite(r) and 0.0 < r < near_m):
                ok = False
                break
            worst = max(worst, r)
        out.append(worst if ok else math.inf)
    return out


def shift_bands(bands: Sequence[Pair], angle_min_deg: float) -> List[Pair]:
    """`check_core.estimate_blind_sectors` の帯（添字 × 刻み＝0..360°。`angle_min` を見ていない）を、
    LaserScan の実角度（`angle_min` 起点）へ直す。`angle_min=-180°` の LiDAR ではこれが必要。"""
    return [(s + angle_min_deg, e + angle_min_deg) for s, e in bands]


def residual_offset_deg(estimated: Sequence[Pair], selected: Sequence[Pair]) -> float:
    """推定した写り込み帯（0..360°）のうち、選んだ範囲の**外**に残る連続幅の最大（度）。

    帯が選択範囲に収まっていれば 0。帯が全く選ばれていなければ帯の幅そのもの。
    """
    worst = 0.0
    for s, e in estimated:
        if e - s <= 0.0:
            continue
        run = 0.0
        a = s
        while a <= e + _EPS:
            if in_ranges(normalize_deg(a), selected):
                run = 0.0
            else:
                run += _SAMPLE_STEP_DEG
                worst = max(worst, run)
            a += _SAMPLE_STEP_DEG
    return worst


def verify_blind(estimated: Sequence[Pair], selected: Sequence[Pair],
                 tolerance_deg: Optional[float]) -> Verdict:
    """検証（Spec-checks.md §3.5）: 選んだ範囲の外に残る写り込みのずれが許容以内か。"""
    if tolerance_deg is None or not math.isfinite(tolerance_deg) or tolerance_deg <= 0.0:
        return Verdict(False, "tolerance_undefined")
    off = residual_offset_deg(estimated, selected)
    if off > tolerance_deg + _EPS:
        return Verdict(False, f"offset_exceeds_tolerance:{off:.2f}>{tolerance_deg:.2f}")
    return Verdict(True, f"offset:{off:.2f}")


def count_masked(selected: Sequence[Pair], angle_min: float, angle_increment: float,
                 ranges: Sequence[float]) -> int:
    """このマスクで `ranges` のうち有効な点（有限・正）がいくつ消えるか（プレビュー）。

    添字は `lidar_filter` と同じ `scan_geometry.sector_indices` で引く（食い違いを作らない）。
    """
    from th_perception.scan_geometry import sector_indices

    class _S:
        pass

    s = _S()
    s.angle_min, s.angle_increment, s.ranges = angle_min, angle_increment, ranges
    n = len(ranges)
    idx = set()
    for a0, a1 in selected:
        i0, i1 = sector_indices(a0, a1, s)
        if i0 is None or i1 is None:
            continue
        if i0 <= i1:
            idx.update(range(i0, i1 + 1))
        else:
            idx.update(range(i0, n))
            idx.update(range(0, i1 + 1))
    return sum(1 for i in idx if math.isfinite(ranges[i]) and ranges[i] > 0.0)


def unmasked_inside(selected: Sequence[Pair], angle_min: float, angle_increment: float,
                    filtered: Sequence[float]) -> int:
    """`/scan_filtered` の、選んだ範囲の内側でまだ有限な点の数（0 でなければマスクが効いていない）。"""
    return count_masked(selected, angle_min, angle_increment, filtered)
