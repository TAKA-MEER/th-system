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

import json
import math
import os
import re
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


def decimate_polyline(points, max_points):
    """表示用に点列を間引く。順序を保ち、始点と終点は必ず残す。

    points: [(x, y, yaw), ...] のような列（中身の型には触らない。要素をそのまま返す）
    max_points: 残す最大点数

    - len(points) <= max_points なら中身を変えずにそのまま返す
    - max_points <= 0 は間引かない（そのまま返す）とみなす
    - max_points == 1 なら始点だけ
    - それ以外は等間隔ストライドで選び、**必ず最後の要素を含める**
    - 空列は空列

    /route/preview は記録済みの全点を 2Hz で毎回 publish するため、長距離（100m 級、
    1000 点前後）になると数 Mbps の帯域を食い、直したばかりのロボット無線を圧迫する
    （WS-9F）。表示専用の間引きで帯域を抑える。順序は変えず、始点・終点は必ず残す。
    元のリストは破壊しない。
    """
    n = len(points)
    if n <= max_points or max_points <= 0:
        return points
    if max_points == 1:
        return [points[0]]
    out = []
    last_idx = -1
    for k in range(max_points):
        idx = int(round(k * (n - 1) / (max_points - 1)))
        if idx != last_idx:   # 重複を返さない（最後の点が終点と同一でも二重に入れない）
            out.append(points[idx])
            last_idx = idx
    return out


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
# 経路ファイルの保存（WS-9H。ROS2 非依存・os/json のみで完結）
# ──────────────────────────────────────────────────────────────────
# 教示は 10 分前後かかる。いまは finalize（画面「保存」）の 1 回だけしかファイルに
# 書かず、その間の点列はメモリにしか無い。ルートが落ちる・コンテナ再起動・電源断で
# 10 分の成果が丸ごと消える。さらに同名で保存し直すと旧版 <id>.json が消える
# （EXCEPTION-LEDGER W-04）。ここで「途中経過の定期保存」と「旧版を 1 世代残す
# 世代管理」を担う純関数をまとめる。
#
# 命名: 完成品は <id>.json、途中経過は <id>.wip、1 世代前は <id>.prev。
# /route/catalog は routes_dir を舐めて `.json` だけを拾うので、.wip/.prev は
# 一覧に出ない（除外判定は list_finalized_route_files に一元化）。
_FILENAME_UNSAFE_RE = re.compile(r'[\\/]+')


def _safe_id(route_id):
    """経路名をファイル名用に無害化する。`/` `\\` を `_` に置換する。

    経路名に `../` や `a/b` が混じっても、routes_dir の外へ書かないための防御。
    `../evil` は `.._evil` になりディレクトリを越えられない（既存保存の
    `f'{id}.json'` を直接 join していた扱いを安全側へ揃える）。
    """
    return _FILENAME_UNSAFE_RE.sub('_', route_id)


def finalized_path(routes_dir, route_id):
    """記録が確定した経路（完成品）の保存先 <id>.json。"""
    return os.path.join(routes_dir, _safe_id(route_id) + '.json')


def autosave_path(routes_dir, route_id):
    """記録中の途中経過を書く先。完成品(.json)と混ざらない .wip 名。"""
    return os.path.join(routes_dir, _safe_id(route_id) + '.wip')


def previous_path(routes_dir, route_id):
    """1 世代前を退避する先。完成品と区別できる .prev 名。"""
    return os.path.join(routes_dir, _safe_id(route_id) + '.prev')


def save_route_atomic(path, route_dict):
    """同じディレクトリの一時ファイルへ書いてから os.replace で差し替える。

    途中で落ちても、読み手が「半分だけ書かれた JSON」を掴まないようにする。
    一時ファイルは隠し名 `.<basename>.tmp` で本体と異なる名前にする。
    """
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    tmp = os.path.join(directory, f'.{os.path.basename(path)}.tmp')
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(route_dict, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)
    return path


def finalize_route_file(routes_dir, route_id, route_dict):
    """完成した経路を保存する。戻り値は保存先のパス。

    1. 既存の <id>.json があれば previous_path へ退避する（1 世代だけ。
       既に前の世代があれば上書きしてよい）
    2. <id>.json を save_route_atomic で書く
    3. 途中経過ファイル（autosave_path）が残っていれば消す
    """
    os.makedirs(routes_dir, exist_ok=True)
    dest = finalized_path(routes_dir, route_id)
    prev = previous_path(routes_dir, route_id)
    if os.path.exists(dest):
        os.replace(dest, prev)   # 旧版を 1 世代前へ退避
    save_route_atomic(dest, route_dict)
    wip = autosave_path(routes_dir, route_id)
    if os.path.exists(wip):
        os.remove(wip)
    return dest


def list_finalized_route_files(routes_dir):
    """/route/catalog に出す経路一覧を返す。routes_dir 内の `.json` だけを拾う。

    途中経過(.wip)・1 世代前(.prev)は一覧に出してはならない。この除外判定を
    純関数として一元化し、route_recorder 側とテスト両方から直接叩けるようにする。
    """
    if not os.path.isdir(routes_dir):
        return []
    return sorted(f for f in os.listdir(routes_dir) if f.endswith('.json'))


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
    def points(self) -> List[Pose2D]:
        """記録中の点列のコピー（プレビュー描画用）。"""
        return list(self._points)

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
