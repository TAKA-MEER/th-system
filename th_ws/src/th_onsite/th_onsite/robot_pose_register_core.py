"""
robot_pose_register_core.py
============================
機体姿勢（ROBOT_POSE）でのピン登録の純ロジック（ROS2 非依存）

pin_registrar.py の `_place_pin_effect` が持っていた pin dict 構築ロジック
（HOME は既存置換・PANEL は連番）を純関数に切り出したもの。
ROS2 ノードはこのモジュールを import して使用する。
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional


def _now_unix() -> int:
    return int(time.time())


def build_pin_from_robot_pose(kind: str, x: float, y: float,
                              yaw: float, existing_pins: List[Dict]
                              ) -> Dict:
    """機体姿勢の (x, y, yaw) から登録する pin dict を 1 件組み立てる。

    現行 `_place_pin_effect` のロジックをそのまま純関数化したもの:
    - HOME は常に 1 個のみ（既存 HOME を置換して id='home'）。
    - PANEL は既存 PANEL 数 + 1 の連番 id（panel_1, panel_2, ...）。
    - registered_at は現在 Unix 時刻。
    - 座標は 3 桁に丸める（yaw も同様）。
    """
    if kind == 'HOME':
        pid = 'home'
    else:
        existing = [p for p in existing_pins if p.get('kind') == 'PANEL']
        pid = f'panel_{len(existing) + 1}'
    return {
        'id': pid,
        'name': '',
        'kind': kind,
        'pose': {'x': round(float(x), 3), 'y': round(float(y), 3),
                 'yaw': round(float(yaw), 3)},
        'registered_at': _now_unix(),
    }
