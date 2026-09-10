"""pin_clearance_core.py — ピン登録座標の壁近接チェック（純ロジック・ROS2 非依存）

2026-09-10 実機で、盤に近すぎる位置に登録した配電盤ピンへ Nav2 が経路を引けず
`PANEL_NAV/BLOCKED` に張り付いた（内接半径 + 到着許容ぶんの余地が全部 253 帯に
入っていた）。登録時に「壁からの距離」を測って警告する。

OccupancyGrid の値は -1=未知 / 0..100=占有確率。`lethal_threshold`（既定 100）
以上を「占有」とみなす。
"""
from __future__ import annotations

import math


def nearest_lethal_distance(grid, width, height, resolution, origin_x, origin_y,
                            x, y, search_r_m, lethal_threshold=100):
    """(x, y)[m] から search_r_m[m] 以内で最も近い占有セルまでの距離[m]。

    grid: 長さ width*height の行優先リスト（0..100 / -1）。
    見つからなければ float('inf')。grid が空・範囲外なら inf。
    """
    if not grid or width <= 0 or height <= 0 or resolution <= 0:
        return float('inf')
    cx = int((x - origin_x) / resolution)
    cy = int((y - origin_y) / resolution)
    span = int(math.ceil(search_r_m / resolution))
    best_sq = None
    for j in range(cy - span, cy + span + 1):
        if j < 0 or j >= height:
            continue
        base = j * width
        for i in range(cx - span, cx + span + 1):
            if i < 0 or i >= width:
                continue
            v = grid[base + i]
            if v is None or v < lethal_threshold:
                continue
            # セル中心の world 座標
            wx = origin_x + (i + 0.5) * resolution
            wy = origin_y + (j + 0.5) * resolution
            d_sq = (wx - x) ** 2 + (wy - y) ** 2
            if best_sq is None or d_sq < best_sq:
                best_sq = d_sq
    if best_sq is None:
        return float('inf')
    return math.sqrt(best_sq)


def clearance_verdict(nearest_m, min_clearance_m):
    """'ok' か 'warn'。nearest_m >= min_clearance_m なら ok。"""
    return 'ok' if nearest_m >= min_clearance_m else 'warn'


def retreat_pose(x, y, yaw, retreat_m):
    """yaw の逆方向へ retreat_m だけ退避した (x2, y2)。

    2 点指示の①→②の yaw は「機体が向いてほしい方向」＝盤の方を向いているので、
    -yaw 方向へ動かせば盤から離れる。yaw と retreat_m はそのまま保持（呼び出し側）。
    """
    return (x - retreat_m * math.cos(yaw), y - retreat_m * math.sin(yaw))
