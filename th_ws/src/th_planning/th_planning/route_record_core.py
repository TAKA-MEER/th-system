"""
route_record_core.py
====================
教示経路の記録コアロジック（ROS2 非依存）

手動運転中のロボット姿勢列を、距離・角度のしきい値で間引いて経路
(RouteData) に落とす。ROS2 インポートを一切含まない純粋 Python モジュール。
ROS2 ノード側（後続パケット）がこのモジュールを import して使用する。

設計方針:
  - sample_min_dist_m: 前回サンプルからこの距離以上動いたら点を追加
  - sample_min_yaw_rad: 距離が足りなくてもこの角度以上向きが変わったら追加
  - 向き差は [-pi, pi] に正規化して比較する
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

Pose2D = Tuple[float, float, float]   # (x, y, theta[rad])


# ──────────────────────────────────────────────────────────────────
# パラメータ群（外部から差し替え可能）
# ──────────────────────────────────────────────────────────────────
@dataclass
class RouteRecordParams:
    sample_min_dist_m:  float = 0.10   # m 前回サンプルからこの距離以上動いたら点を追加
    sample_min_yaw_rad: float = 0.20   # rad 距離が足りなくてもこの角度以上向きが変わったら追加


# ──────────────────────────────────────────────────────────────────
# 記録データ
# ──────────────────────────────────────────────────────────────────
@dataclass
class RouteData:
    id: str
    name: str
    generation: int
    start_yaw: float
    recorded_at_ms: int
    frame_id: str
    points: List[Pose2D] = field(default_factory=list)


def normalize_angle(a: float) -> float:
    """角度を [-pi, pi] に正規化する"""
    return math.atan2(math.sin(a), math.cos(a))


def polyline_length(points: Sequence[Pose2D]) -> float:
    """点列の折れ線長 [m] を返す（0/1 点なら 0.0）"""
    total = 0.0
    for i in range(1, len(points)):
        total += math.hypot(points[i][0] - points[i - 1][0],
                            points[i][1] - points[i - 1][1])
    return total


def route_to_dict(route: RouteData) -> dict:
    """RouteData を dict へ変換する。

    キーは RouteInfo.msg のフィールド並び (id/name/generation/length_m/
    point_count/start_yaw) に合わせつつ、記録に必要な frame_id / recorded_at_ms /
    points も付与する。points は [[x, y, yaw], ...] のリスト。
    """
    return {
        'id': route.id,
        'name': route.name,
        'generation': route.generation,
        'length_m': polyline_length(route.points),
        'point_count': len(route.points),
        'start_yaw': route.start_yaw,
        'recorded_at_ms': route.recorded_at_ms,
        'frame_id': route.frame_id,
        'points': [[p[0], p[1], p[2]] for p in route.points],
    }


def route_from_dict(d: dict) -> RouteData:
    """dict を RouteData へ逆変換する。points はタプル化する。"""
    points = [tuple(p) for p in d.get('points', [])]  # type: ignore[arg-type]
    return RouteData(
        id=d['id'],
        name=d['name'],
        generation=d.get('generation', 1),
        start_yaw=d['start_yaw'],
        recorded_at_ms=d.get('recorded_at_ms', 0),
        frame_id=d.get('frame_id', 'odom'),
        points=points,
    )


# ──────────────────────────────────────────────────────────────────
# 記録状態マシン
# ──────────────────────────────────────────────────────────────────
class RouteRecorderCore:
    """手動運転中の姿勢列を間引いて RouteData に記録する純ロジック。"""

    def __init__(self, params: RouteRecordParams = None):
        self.params = params or RouteRecordParams()
        self._points: List[Pose2D] = []
        self._start_yaw: float = 0.0

    def start(self, x: float, y: float, yaw: float) -> None:
        """記録開始。最初の姿勢を点列の先頭に入れ、start_yaw を確定する。

        既に記録中でも再 start() は状態をリセットする。
        """
        self._points = [(x, y, yaw)]
        self._start_yaw = yaw

    def resume(self) -> None:
        """一時停止からの再開。

        点列・start_yaw は保持し、次の add_pose の間引き基準（最終点）に
        戻すだけ。実質何もしなくても成立する。
        """

    def add_pose(self, x: float, y: float, yaw: float) -> bool:
        """間引き判定。点を追加したら True を返す。"""
        if not self._points:
            self._points.append((x, y, yaw))
            return True

        last_x, last_y, last_yaw = self._points[-1]
        dist = math.hypot(x - last_x, y - last_y)
        yaw_diff = abs(normalize_angle(yaw - last_yaw))

        if dist >= self.params.sample_min_dist_m or yaw_diff >= self.params.sample_min_yaw_rad:
            self._points.append((x, y, yaw))
            return True
        return False

    @property
    def recorded_m(self) -> float:
        """点列の折れ線長 [m]"""
        return polyline_length(self._points)

    @property
    def point_count(self) -> int:
        return len(self._points)

    @property
    def start_yaw(self) -> float:
        return self._start_yaw

    def finalize(
        self,
        route_id: str,
        name: str,
        recorded_at_ms: int,
        frame_id: str = "odom",
        generation: int = 1,
    ) -> RouteData:
        """記録を RouteData に落として返す。"""
        return RouteData(
            id=route_id,
            name=name,
            generation=generation,
            start_yaw=self._start_yaw,
            recorded_at_ms=recorded_at_ms,
            frame_id=frame_id,
            points=list(self._points),
        )
