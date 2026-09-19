"""
test_scan_geometry.py
======================
scan_geometry.py（th_perception。角度 → LaserScan 添字の変換規約の正本）
の単体テスト。ROS2 なし・純粋 Python で実行可能。

満たす仕様: docs/plan/detailed/DetailedDesign-wp2.md WP-CALIB-01 §4.1・§7
  - test_angle_to_index          : 規約①〜④
  - test_wraparound_sector       : 0 をまたぐ区間（sector_indices が
                                    i0 > i1 を返し、呼び出し側が 2 区間に
                                    割れること）
  - test_out_of_range_not_clamped: 規約⑤（clamp せず None）

C++ 移植版 (th_safety/include/th_safety/scan_geometry.hpp) との等価性
（B-3）は test_scan_geometry_equivalence（gtest、th_safety）が別途担保する。
このファイルは Python 単体の振る舞いのみを検査する。
"""
import math
from dataclasses import dataclass, field
from typing import List

import pytest

from scan_geometry import angle_to_index, sector_indices


@dataclass
class DummyScan:
    """sensor_msgs/LaserScan の代わりに使う最小限のダミー。
    angle_to_index / sector_indices は angle_min / angle_increment /
    ranges（長さのみ使用）しか見ないため、これで足りる（ROS2 非依存を保つ）。
    """
    angle_min: float
    angle_increment: float
    ranges: List[float] = field(default_factory=list)

    @classmethod
    def full_circle_deg_step(cls, step_deg: float = 1.0):
        """angle_min=-180°, 1° 刻みで 360 点（角度計算がきれいな数になるテスト用）。"""
        n = int(round(360.0 / step_deg))
        return cls(angle_min=-math.pi, angle_increment=math.radians(step_deg),
                    ranges=[0.0] * n)

    @classmethod
    def half_circle_deg_step(cls, step_deg: float = 1.0):
        """angle_min=-90°, angle_max=+90°（180° 分。範囲外を作るためのテスト用）。"""
        n = int(round(180.0 / step_deg)) + 1
        return cls(angle_min=math.radians(-90.0), angle_increment=math.radians(step_deg),
                    ranges=[0.0] * n)


class TestAngleToIndex:
    """規約①〜④: laser_link 基準・ラジアン・CCW 正、[-pi,pi) 正規化、floor 添字。"""

    def test_angle_min_maps_to_index_zero(self):
        scan = DummyScan.full_circle_deg_step(1.0)
        assert angle_to_index(scan.angle_min, scan) == 0

    def test_kth_grid_point_maps_to_index_k(self):
        scan = DummyScan.full_circle_deg_step(1.0)
        for k in (1, 10, 90, 180, 270, 359):
            angle = scan.angle_min + k * scan.angle_increment
            assert angle_to_index(angle, scan) == k

    def test_last_valid_index(self):
        scan = DummyScan.full_circle_deg_step(1.0)
        n = len(scan.ranges)
        angle = scan.angle_min + (n - 1) * scan.angle_increment
        assert angle_to_index(angle, scan) == n - 1

    def test_floor_rounds_toward_lower_index(self):
        """格子点の直前（半分未満のずれ）は下側の添字に floor される（規約④）。"""
        scan = DummyScan.full_circle_deg_step(1.0)
        angle = scan.angle_min + 10 * scan.angle_increment + 0.4 * scan.angle_increment
        assert angle_to_index(angle, scan) == 10

    def test_ccw_positive_convention(self):
        """反時計回り正（規約①）: angle_min から increment だけ進めた角度は
        angle_min より数値として大きい（CCW）方向に対応する。"""
        scan = DummyScan.full_circle_deg_step(1.0)
        idx_at_min = angle_to_index(scan.angle_min, scan)
        idx_after_one_step = angle_to_index(scan.angle_min + scan.angle_increment, scan)
        assert idx_after_one_step == idx_at_min + 1

    def test_normalization_makes_equivalent_angles_agree(self):
        """規約③: [-pi, pi) の外側で表現された角度（例: 3*pi/2 rad = -pi/2 rad の
        別表現）も正規化後は同じ添字になる。"""
        scan = DummyScan.full_circle_deg_step(1.0)
        idx_direct = angle_to_index(-math.pi / 2.0, scan)
        idx_wrapped = angle_to_index(3.0 * math.pi / 2.0, scan)  # 同じ物理角度の別表現
        assert idx_direct is not None
        assert idx_direct == idx_wrapped


class TestWraparoundSector:
    """0 をまたぐ区間: sector_indices() は i0 > i1 を返し、
    連結（2 区間への分割）は呼び出し側の責務（§4.1）。"""

    def test_sector_crossing_array_boundary_returns_i0_greater_than_i1(self):
        # angle_min=-180°, 1°刻み。170°→-170°（=190°）は ±180°（配列境界）を
        # またぐので、i0（170°側）が末尾付近、i1（-170°側）が先頭付近になる。
        scan = DummyScan.full_circle_deg_step(1.0)
        i0, i1 = sector_indices(170.0, -170.0, scan)
        assert i0 is not None and i1 is not None
        assert i0 > i1
        # 具体値も確認（angle_min=-180 起点の 1°刻みなので厳密な整数になる）
        assert i0 == 350
        assert i1 == 10

    def test_wraparound_sector_can_be_joined_by_caller(self):
        """呼び出し側の連結ロジック（§10 の完了条件スクリプトと同じ方式）で
        期待どおりの添字集合になることを確認する。"""
        scan = DummyScan.full_circle_deg_step(1.0)
        i0, i1 = sector_indices(170.0, -170.0, scan)
        n = len(scan.ranges)
        masked = set(range(i0, i1 + 1)) if i0 <= i1 else (
            set(range(i0, n)) | set(range(0, i1 + 1)))
        # 170°(=350) から 359, 0 から 10(=-170°) までの 21 点
        assert masked == set(range(350, 360)) | set(range(0, 11))

    def test_non_wraparound_sector_returns_i0_less_equal_i1(self):
        """front（0°）をまたぐが配列境界（±180°）はまたがない区間は
        通常の連続区間として i0 <= i1 になる（310°→40° は -50°→40° と同義で
        配列境界を越えないため wraparound ではない）。"""
        scan = DummyScan.full_circle_deg_step(1.0)
        i0, i1 = sector_indices(310.0, 40.0, scan)
        assert i0 is not None and i1 is not None
        assert i0 <= i1


class TestOutOfRangeNotClamped:
    """規約⑤: 範囲外は clamp せず「そのセクタは存在しない」= None。"""

    def test_angle_beyond_angle_max_returns_none(self):
        scan = DummyScan.half_circle_deg_step(1.0)  # -90°〜+90° のみ
        assert angle_to_index(math.radians(170.0), scan) is None

    def test_angle_before_angle_min_returns_none(self):
        scan = DummyScan.half_circle_deg_step(1.0)  # -90°〜+90° のみ
        assert angle_to_index(math.radians(-170.0), scan) is None

    def test_angle_exactly_at_angle_max_is_in_range(self):
        """angle_max ちょうど（境界値）は含まれる（clamp ではなく本来の最終添字）。"""
        scan = DummyScan.half_circle_deg_step(1.0)
        n = len(scan.ranges)
        angle_max = scan.angle_min + (n - 1) * scan.angle_increment
        assert angle_to_index(angle_max, scan) == n - 1

    def test_sector_indices_partial_out_of_range_returns_none_for_that_side(self):
        scan = DummyScan.half_circle_deg_step(1.0)
        i0, i1 = sector_indices(0.0, 170.0, scan)  # 170° は範囲外
        assert i0 is not None
        assert i1 is None

    def test_empty_ranges_returns_none(self):
        scan = DummyScan(angle_min=-math.pi, angle_increment=0.01, ranges=[])
        assert angle_to_index(0.0, scan) is None
