#!/usr/bin/env python3
"""
gen_scan_geometry_vectors.py
==============================
scan_geometry.py（Python 正本）と scan_geometry.hpp（C++ 移植版）の
等価性テスト（B-3。DetailedDesign-wp2.md WP-CALIB-01 §7）のためのテスト
ベクタ生成器。

pybind11 は使わない方針（このパケットの指示）なので、Python 側で正本の
angle_to_index() / sector_indices() を実際に呼び出して期待値を計算し、
JSON に固定化する。C++ 側（test_scan_geometry_equivalence.cpp）はこの
JSON を読み込み、同じ入力を scan_geometry.hpp に通して結果を突き合わせる。

生成物は th_safety/test/data/scan_geometry_vectors.json に**コミットする**
（生成のたびに値が変わると gtest が再現しないテストになるため。
再生成が必要になったらこのスクリプトを再実行してコミットし直す）。

実行方法:
    python3 th_ws/src/th_testing/tools/gen_scan_geometry_vectors.py

境界値を必ず含める:
  - angle_min ちょうど（角度③正規化の下端）
  - angle_max ちょうど（配列の最終添字）
  - 範囲外（angle_min の手前 / angle_max の先）
  - 0（配列境界 ±pi）をまたぐセクタ
  - 通常の連続セクタ
  - 幅ゼロのセクタ（a0 == a1）
  - 負の angle_increment（一部のセンサ・逆走査構成であり得る）
"""
from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass
from typing import List, Optional

# 正本 th_perception/th_perception/scan_geometry.py を import できるように
# パスを足す（conftest.py と同じ考え方。ただしこのスクリプトは
# th_testing/tools/ に置くので相対階層が conftest.py の test/ とは 1 つ違う）。
_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_perception_core = os.path.join(_repo_root, 'th_perception', 'th_perception')
if _perception_core not in sys.path:
    sys.path.insert(0, _perception_core)

from scan_geometry import angle_to_index, sector_indices  # noqa: E402


@dataclass
class DummyScan:
    angle_min: float
    angle_increment: float
    ranges: List[float]

    @property
    def num_ranges(self) -> int:
        return len(self.ranges)


def _scan_dict(scan: DummyScan) -> dict:
    return {
        'angle_min': scan.angle_min,
        'angle_increment': scan.angle_increment,
        'num_ranges': scan.num_ranges,
    }


def _full_circle(step_deg: float = 1.0) -> DummyScan:
    n = int(round(360.0 / step_deg))
    return DummyScan(angle_min=-math.pi, angle_increment=math.radians(step_deg),
                      ranges=[0.0] * n)


def _half_circle(step_deg: float = 1.0) -> DummyScan:
    n = int(round(180.0 / step_deg)) + 1
    return DummyScan(angle_min=math.radians(-90.0), angle_increment=math.radians(step_deg),
                      ranges=[0.0] * n)


def _rplidar_like() -> DummyScan:
    # 実機ライクな「きれいでない」角度刻み（0.25°付近だが端数が出る値）。
    # 1440 点で概ね 1 周をカバーする構成を模す。
    n = 1440
    increment = (2.0 * math.pi) / n
    angle_min = -math.pi
    return DummyScan(angle_min=angle_min, angle_increment=increment, ranges=[0.0] * n)


def _negative_increment_half_circle(step_deg: float = 1.0) -> DummyScan:
    # 一部のセンサ/逆走査構成では angle_increment が負になり得る
    # （添字が増えるほど角度が減る）。angle_min=+90°から-1°刻みで-90°まで。
    n = int(round(180.0 / step_deg)) + 1
    return DummyScan(angle_min=math.radians(90.0), angle_increment=math.radians(-step_deg),
                      ranges=[0.0] * n)


def _angle_to_index_case(name: str, scan: DummyScan, angle_rad: float) -> dict:
    expected: Optional[int] = angle_to_index(angle_rad, scan)
    return {
        'name': name,
        'type': 'angle_to_index',
        'scan': _scan_dict(scan),
        'angle_rad': angle_rad,
        'expected_index': expected,
    }


def _sector_indices_case(name: str, scan: DummyScan, a0_deg: float, a1_deg: float) -> dict:
    i0, i1 = sector_indices(a0_deg, a1_deg, scan)
    return {
        'name': name,
        'type': 'sector_indices',
        'scan': _scan_dict(scan),
        'a0_deg': a0_deg,
        'a1_deg': a1_deg,
        'expected_i0': i0,
        'expected_i1': i1,
    }


def build_vectors() -> List[dict]:
    full = _full_circle(1.0)
    half = _half_circle(1.0)
    rplidar = _rplidar_like()
    neg_inc = _negative_increment_half_circle(1.0)

    n_full = full.num_ranges
    angle_max_full = full.angle_min + (n_full - 1) * full.angle_increment

    n_half = half.num_ranges
    angle_max_half = half.angle_min + (n_half - 1) * half.angle_increment

    n_rp = rplidar.num_ranges
    angle_max_rp = rplidar.angle_min + (n_rp - 1) * rplidar.angle_increment

    n_neg = neg_inc.num_ranges
    angle_max_neg = neg_inc.angle_min + (n_neg - 1) * neg_inc.angle_increment

    vectors: List[dict] = [
        # ── angle_to_index: 境界値 ──────────────────────────
        _angle_to_index_case('full_angle_min_exact', full, full.angle_min),
        _angle_to_index_case('full_angle_max_exact', full, angle_max_full),
        _angle_to_index_case('full_midpoint_grid', full, full.angle_min + 90 * full.angle_increment),
        _angle_to_index_case(
            'full_between_grid_floor_low',
            full, full.angle_min + 10 * full.angle_increment + 0.4 * full.angle_increment),
        _angle_to_index_case(
            'full_equivalent_angle_ccw_wrap',
            full, full.angle_min + 3.0 * math.pi),  # 2π 分ずれた同一物理角度

        _angle_to_index_case('half_angle_min_exact', half, half.angle_min),
        _angle_to_index_case('half_angle_max_exact', half, angle_max_half),
        _angle_to_index_case(
            'half_out_of_range_beyond_max', half, angle_max_half + half.angle_increment * 5),
        _angle_to_index_case(
            'half_out_of_range_before_min', half, half.angle_min - half.angle_increment * 5),
        _angle_to_index_case('half_out_of_range_far', half, math.radians(170.0)),

        _angle_to_index_case('rplidar_angle_min_exact', rplidar, rplidar.angle_min),
        _angle_to_index_case('rplidar_angle_max_exact', rplidar, angle_max_rp),
        _angle_to_index_case(
            'rplidar_grid_1000', rplidar, rplidar.angle_min + 1000 * rplidar.angle_increment),
        _angle_to_index_case(
            'rplidar_grid_1439', rplidar, rplidar.angle_min + 1439 * rplidar.angle_increment),

        _angle_to_index_case('neg_inc_angle_min_exact', neg_inc, neg_inc.angle_min),
        _angle_to_index_case('neg_inc_angle_max_exact', neg_inc, angle_max_neg),
        _angle_to_index_case(
            'neg_inc_grid_45', neg_inc, neg_inc.angle_min + 45 * neg_inc.angle_increment),
        _angle_to_index_case(
            'neg_inc_out_of_range', neg_inc, math.radians(-170.0)),

        _angle_to_index_case('empty_ranges', DummyScan(angle_min=-math.pi, angle_increment=0.01,
                                                         ranges=[]), 0.0),

        # ── sector_indices: 0 をまたぐ／またがない／幅ゼロ／範囲外 ──
        _sector_indices_case('full_wraparound_sector', full, 170.0, -170.0),
        _sector_indices_case('full_non_wraparound_front_sector', full, 310.0, 40.0),
        _sector_indices_case('full_normal_contiguous_sector', full, 40.0, 130.0),
        _sector_indices_case('full_zero_width_sector', full, 90.0, 90.0),
        _sector_indices_case('full_full_min_to_max', full, -180.0, 179.0),

        _sector_indices_case('half_partial_out_of_range', half, 0.0, 170.0),
        _sector_indices_case('half_both_out_of_range', half, 170.0, -170.0),
        _sector_indices_case('half_within_range', half, -30.0, 30.0),

        _sector_indices_case('rplidar_wraparound_sector', rplidar, 175.0, -175.0),
        _sector_indices_case('rplidar_normal_sector', rplidar, 10.0, 50.0),

        _sector_indices_case('neg_inc_within_range', neg_inc, -30.0, 30.0),
        _sector_indices_case('neg_inc_out_of_range_pair', neg_inc, 170.0, -170.0),
    ]
    return vectors


def main() -> None:
    vectors = build_vectors()
    out_path = os.path.join(
        _repo_root, 'th_safety', 'test', 'data', 'scan_geometry_vectors.json')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({'vectors': vectors}, f, indent=2, ensure_ascii=False, sort_keys=True)
        f.write('\n')
    print(f'{len(vectors)} 件のベクタを書き出した: {out_path}')


if __name__ == '__main__':
    main()
