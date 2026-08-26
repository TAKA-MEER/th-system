#!/usr/bin/env python3
"""
gen_obstacle_cone_vectors.py
==============================
Python 版 `mapless_follow_core.observe_cone()`（th_planning）と C++ 版
`th_safety::observe_cone()`（`th_safety/src/obstacle_limiter_core.cpp`）の
突き合わせテスト（`test_obstacle_cone_equivalence.cpp`）のためのテスト
ベクタ生成器。

DetailedDesign-safety.md §3.4.3 の指示:
  「既存 is_path_blocked() は bool しか返さないので、最近傍距離を返す形に
   拡張して移植する。拡張後、既存 Python 実装との等価性テストを書く」

pybind11 は使わない方針（このパケットの指示）。`test_scan_geometry_equivalence`
（B-3）と同じ JSON ベクタ方式にそろえる: Python 正本を実際に呼び出して
期待値を計算し、JSON に固定化する。C++ 側はこの JSON を読み込み、同じ入力を
`th_safety::observe_cone()` に通して結果を突き合わせる。

── ビット一致ではなく「包含性」を検証する（重要・DetailedDesign-open.md N-12） ──
実装当初は「Python と C++ が bit-for-bit 一致する」ことを狙っていたが、実測で
成り立たないことが分かった。原因は集合の作り方の違い:
  - Python 版 `observe_cone()`: 各ビームの中心角度が direction±half_width の
    範囲に入っているかを個別に判定する（角度ベース）。
  - C++ 版 `observe_cone()`: コーンの両端の角度を `scan_geometry.sector_indices()`
    で添字に変換し（floor による丸め）、その閉区間 `[i0, i1]` を機械的に走査する
    （添字ベース）。floor は「その角度を含むビン」を返すため、下端側 (i0) は
    コーンの外側（direction-half_width よりさらに外）のビームを拾うことがある。
  この違いにより **C++ が選ぶビーム集合は常に Python が選ぶビーム集合の
  上位集合になる**（実測 7,500 件中、一致 6,799 件・C++ がより近い障害物を
  検出 701 件・Python の方が近い障害物を検出した違反 0 件。half_width が
  angle_increment 程度以下の狭いコーンで頻発する）。C++ が「見落とす」方向の
  食い違いが無いことが安全上重要な性質であり、この生成器・対応する gtest は
  **「C++ の最近傍距離は常に Python の最近傍距離以下」という包含性**を検証する
  （ビット一致ではない。詳細は DetailedDesign-open.md N-12）。

生成物は th_safety/test/data/obstacle_cone_vectors.json に**コミットする**
（生成のたびに値が変わると gtest が再現しないテストになるため）。

── 対象を全周スキャンに限定する理由 ──────────────────────────
半周スキャン等、コーンが scan の角度範囲を部分的にしかカバーしない構成は
対象外にする。Python 版 `observe_cone()` には「未観測(covered)」という
概念自体が無く、C++版 `ConeObservation.covered` との対応が一意に決まらない。
全周スキャン（angle_increment × num_ranges がちょうど 2π）なら
`sector_indices()` のコーン境界は常に範囲内に写像されるため C++ 側は必ず
covered=true になる。

── ベクタの構成 ──────────────────────────────────────────
1. 基本ケース: 素直な入出力(空/単一/複数障害物、nan/負値の無視等)。
2. 境界ビームの狙い撃ち: direction±half_width の境界添字ちょうど/1つ外側に
   障害物を置く(少数の代表例。人間が読める名前を付けてある)。
3. 食い違いを確実に踏む決め打ちケース(このパケットで追加。N-12 の根拠):
   - half_width_rad < angle_increment(コーン幅がビーム間隔より狭い)
   - コーン境界がグリッド線のごく近傍(±1e-6 度)
   - ±180 度(配列境界)をまたぐ方向
4. ランダム fuzz(固定シード。乱数だが再生成しても同じ値になる): 3種類の
   スキャン構成 × 方向・半幅・レンジ値をランダムに振り、包含性検証の
   統計的な広さを担保する。

実行方法:
    python3 th_ws/src/th_testing/tools/gen_obstacle_cone_vectors.py
"""
from __future__ import annotations

import json
import math
import os
import random
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Union

_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_perception_core = os.path.join(_repo_root, 'th_perception', 'th_perception')
# th_planning はパッケージとして import する(mapless_follow_core.py 内の
# `from .follow_planner_core import ...` という相対インポートを解決するため、
# bare モジュールではなくパッケージのルート(th_planning/ 直下)を sys.path に
# 足す。test_mapless_follow_logic.py の conftest と同じ考え方)。
_planning_pkg_root = os.path.join(_repo_root, 'th_planning')
for _p in (_perception_core, _planning_pkg_root):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from scan_geometry import sector_indices  # noqa: E402  (境界添字の計算専用)
from th_planning.mapless_follow_core import observe_cone  # noqa: E402  (期待値の計算元=正本)


@dataclass
class DummyScan:
    angle_min: float
    angle_increment: float
    ranges: List[float] = field(default_factory=list)

    @property
    def num_ranges(self) -> int:
        return len(self.ranges)


def _full_circle(n: int) -> DummyScan:
    """angle_min=-pi・ちょうど1周(2*pi)を n 点でカバーする全周スキャン。"""
    return DummyScan(angle_min=-math.pi, angle_increment=(2.0 * math.pi) / n,
                      ranges=[30.0] * n)  # 既定値=遠方(30m)。個々のケースで上書きする。


def _sector_bounds(scan: DummyScan, direction_deg: float, half_width_deg: float):
    """コーン境界の添字 (i0, i1) を scan_geometry の正本で求める(生成専用)。"""
    dummy = DummyScan(angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                       ranges=[0.0] * scan.num_ranges)
    return sector_indices(direction_deg - half_width_deg, direction_deg + half_width_deg, dummy)


def _enc(r: float) -> Union[float, str]:
    """JSON に書けない特殊値(inf/nan)を文字列マーカーにする(自作JSONパーサ側で復元)。"""
    if math.isnan(r):
        return 'nan'
    if math.isinf(r):
        return 'inf' if r > 0 else '-inf'
    return round(r, 6)


def _make_vector(name: str, scan: DummyScan, ranges: List[float],
                  direction_rad: float, half_width_rad: float) -> dict:
    expected = observe_cone(ranges, scan.angle_min, scan.angle_increment,
                             direction_rad, half_width_rad)
    return {
        'name': name,
        'scan': {
            'angle_min': scan.angle_min,
            'angle_increment': scan.angle_increment,
            'num_ranges': scan.num_ranges,
        },
        'ranges': [_enc(r) for r in ranges],
        'direction_rad': direction_rad,
        'half_width_rad': half_width_rad,
        # 全周スキャン限定(モジュール docstring参照)なので C++ 側は常に covered=true
        # になるはず。もし false が返ったらそれ自体が食い違い(scan の geometry を
        # 作り損ねている)なので gtest 側でも明示的に assert する。
        'expected_covered': True,
        'expected_nearest_m': None if expected is None else expected,
    }


def _make_vector_deg(name: str, scan: DummyScan, ranges: List[float],
                      direction_deg: float, half_width_deg: float) -> dict:
    return _make_vector(name, scan, ranges, math.radians(direction_deg), math.radians(half_width_deg))


def _boundary_vector(name: str, scan: DummyScan, direction_deg: float, half_width_deg: float,
                      place: str, distance: float = 0.42) -> Optional[dict]:
    """コーン境界の添字ちょうど/1つ外側のビームに障害物を置いたケースを作る。

    place:
      'at_i0'           コーン開始側の境界添字ちょうど(C++ の走査範囲に含まれる)
      'just_outside_i0' その1つ手前(C++ の走査範囲から外れる)
      'at_i1'           コーン終了側の境界添字ちょうど(C++ の走査範囲に含まれる)
      'just_outside_i1' その1つ先(C++ の走査範囲から外れる)
    """
    i0, i1 = _sector_bounds(scan, direction_deg, half_width_deg)
    n = scan.num_ranges
    ranges = [30.0] * n

    if place == 'at_i0':
        if i0 is None:
            return None
        ranges[i0] = distance
    elif place == 'just_outside_i0':
        if i0 is None:
            return None
        ranges[(i0 - 1) % n] = distance
    elif place == 'at_i1':
        if i1 is None:
            return None
        ranges[i1] = distance
    elif place == 'just_outside_i1':
        if i1 is None:
            return None
        ranges[(i1 + 1) % n] = distance
    else:
        raise ValueError(f'unknown place: {place}')

    return _make_vector_deg(name, scan, ranges, direction_deg, half_width_deg)


def _basic_cases(scan: DummyScan, tag: str) -> List[dict]:
    n = scan.num_ranges
    vectors: List[dict] = []

    vectors.append(_make_vector_deg(
        f'{tag}_all_inf_returns_none', scan, [float('inf')] * n, 0.0, 20.0))

    vectors.append(_make_vector_deg(
        f'{tag}_far_background_finite', scan, [30.0] * n, 0.0, 20.0))

    ranges = [30.0] * n
    idx0 = round((0.0 - scan.angle_min) / scan.angle_increment) % n
    ranges[idx0] = 0.5
    vectors.append(_make_vector_deg(f'{tag}_single_obstacle_at_center', scan, ranges, 0.0, 20.0))

    ranges = [30.0] * n
    idx90 = round((math.radians(90.0) - scan.angle_min) / scan.angle_increment) % n
    ranges[idx90] = 0.3
    vectors.append(_make_vector_deg(
        f'{tag}_obstacle_outside_cone_ignored', scan, ranges, 0.0, 20.0))

    ranges = [30.0] * n
    for deg, dist in [(-10.0, 1.2), (0.0, 0.6), (10.0, 2.0)]:
        idx = round((math.radians(deg) - scan.angle_min) / scan.angle_increment) % n
        ranges[idx] = dist
    vectors.append(_make_vector_deg(f'{tag}_picks_nearest_of_multiple', scan, ranges, 0.0, 20.0))

    ranges = [30.0] * n
    idx_nan = round((math.radians(5.0) - scan.angle_min) / scan.angle_increment) % n
    ranges[idx_nan] = float('nan')
    idx_neg = round((math.radians(-5.0) - scan.angle_min) / scan.angle_increment) % n
    ranges[idx_neg] = -1.0
    ranges[idx0] = 0.5
    vectors.append(_make_vector_deg(f'{tag}_nan_and_negative_ignored', scan, ranges, 0.0, 20.0))

    ranges = [30.0] * n
    ranges[idx0] = 0.7
    vectors.append(_make_vector_deg(f'{tag}_zero_half_width', scan, ranges, 0.0, 0.0))

    ranges = [30.0] * n
    idx_far = round((math.radians(179.0) - scan.angle_min) / scan.angle_increment) % n
    ranges[idx_far] = 0.9
    vectors.append(_make_vector_deg(f'{tag}_almost_full_circle_cone', scan, ranges, 0.0, 179.9))

    ranges = [30.0] * n
    idx_wrap = round((math.radians(-178.0) - scan.angle_min) / scan.angle_increment) % n
    ranges[idx_wrap] = 0.4
    vectors.append(_make_vector_deg(f'{tag}_direction_wraparound', scan, ranges, 179.0, 10.0))

    return vectors


def _boundary_matrix(scan: DummyScan, tag: str) -> List[dict]:
    """境界の狙い撃ち(代表例のみ。全数は N-12 対応のランダム fuzz が担う)。"""
    boundary_cases = [
        (0.0, 20.0),
        (137.0, 22.5),
        (-33.3, 12.7),   # 度・半幅ともグリッド非整合
    ]
    vectors: List[dict] = []
    for direction_deg, half_width_deg in boundary_cases:
        for place in ('at_i0', 'just_outside_i0', 'at_i1', 'just_outside_i1'):
            name = (f'{tag}_boundary_d{direction_deg}_hw{half_width_deg}_{place}'
                    .replace('.', 'p').replace('-', 'm'))
            vec = _boundary_vector(name, scan, direction_deg, half_width_deg, place)
            if vec is not None:
                vectors.append(vec)
    return vectors


def _narrow_cone_divergence_cases(scan: DummyScan, tag: str) -> List[dict]:
    """N-12 の根拠: 食い違いを確実に踏む決め打ちケース。

    half_width_rad < angle_increment(コーン幅がビーム間隔より狭い)・
    コーン境界がグリッド線のごく近傍・±180度をまたぐ方向、の3系統。
    ranges はビーム添字が分かるよう単調増加(1.0 + 0.01*i)にしてある
    ―― C++ が下端側 (i0) に余分な1本を含めると、そのビームは Python の
    集合中の最小添字のビームより添字が1つ小さい=距離が必ず小さいため、
    「C++ がより近い障害物を検出した(cpp_smaller)」を機械的に作れる。
    """
    n = scan.num_ranges
    inc_deg = math.degrees(scan.angle_increment)
    ramp = [1.0 + 0.01 * i for i in range(n)]
    vectors: List[dict] = []

    # 系統1: half_width < angle_increment (グリッドに整合しない方向を複数)
    for k, direction_deg in enumerate([3.7, 41.2, 88.8, 132.6, -17.9, -101.4]):
        half_width_deg = 0.3 * inc_deg
        vectors.append(_make_vector_deg(
            f'{tag}_narrow_hw_lt_increment_{k}', scan, ramp, direction_deg, half_width_deg))

    # 系統2: コーン境界がグリッド線のごく近傍(±1e-6度。floor+epsilonの丸め境界)
    for k, direction_deg in enumerate([10.0, 55.5, -70.0]):
        half_width_deg = 2.0 * inc_deg
        for eps_deg in (-1e-6, 1e-6):
            name = f'{tag}_near_grid_boundary_{k}_{"minus" if eps_deg < 0 else "plus"}'
            vectors.append(_make_vector_deg(
                name, scan, ramp, direction_deg + eps_deg, half_width_deg))

    # 系統3: ±180度(配列境界)をまたぐ方向・狭いコーン
    for k, direction_deg in enumerate([179.5, -179.5, 179.9, -179.9]):
        half_width_deg = 0.4 * inc_deg
        vectors.append(_make_vector_deg(
            f'{tag}_wraparound_narrow_{k}', scan, ramp, direction_deg, half_width_deg))

    return vectors


def _random_fuzz_cases(scan: DummyScan, tag: str, count: int, rng: random.Random) -> List[dict]:
    """統計的な広さを担保するランダム fuzz(固定シードで再現可能)。

    half_width は angle_increment の 0.02〜3 倍に振り、食い違いが出やすい
    狭いコーンを重点的に(だが排他的ではなく)サンプルする。
    """
    n = scan.num_ranges
    inc = scan.angle_increment
    vectors: List[dict] = []
    for k in range(count):
        ranges = []
        for _ in range(n):
            x = rng.random()
            if x < 0.08:
                ranges.append(float('inf'))
            elif x < 0.10:
                ranges.append(float('nan'))
            elif x < 0.11:
                ranges.append(-1.0)
            else:
                ranges.append(round(rng.uniform(0.05, 20.0), 4))
        direction_rad = rng.uniform(-math.pi, math.pi)
        half_width_rad = rng.uniform(0.02, 3.0) * inc
        vectors.append(_make_vector(f'{tag}_fuzz_{k}', scan, ranges, direction_rad, half_width_rad))
    return vectors


def build_vectors() -> List[dict]:
    small = _full_circle(72)      # 5度刻み(コンパクト。fuzz の主力)
    full = _full_circle(360)      # 1度刻み
    oddN = _full_circle(1081)     # 実機ライクな割り切れない刻み(gen_scan_geometry_vectors.py の rplidar 相当と同じ発想)

    rng = random.Random(20260826)  # 固定シード(再生成しても同じベクタになる)

    vectors: List[dict] = []

    for scan, tag in [(small, 'n72'), (full, 'n360')]:
        vectors += _basic_cases(scan, tag)
        vectors += _boundary_matrix(scan, tag)
        vectors += _narrow_cone_divergence_cases(scan, tag)

    # oddN(1081点)は代表的な決め打ちケースのみ(ファイルサイズを抑える)。
    vectors += _narrow_cone_divergence_cases(oddN, 'n1081')

    # ランダム fuzz: 主力は小さいスキャン(n72)、中規模(n360)は控えめ。
    # 大きいスキャン(n1081)はファイルサイズが1件あたり大きいため fuzz は
    # 行わず、上の決め打ちケースのみで代表させる。
    vectors += _random_fuzz_cases(small, 'n72', 150, rng)
    vectors += _random_fuzz_cases(full, 'n360', 30, rng)

    return vectors


def main() -> None:
    vectors = build_vectors()
    out_path = os.path.join(
        _repo_root, 'th_safety', 'test', 'data', 'obstacle_cone_vectors.json')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({'vectors': vectors}, f, indent=None, separators=(',', ':'),
                   ensure_ascii=False, sort_keys=True)
        f.write('\n')
    print(f'{len(vectors)} 件のベクタを書き出した: {out_path}')


if __name__ == '__main__':
    main()
