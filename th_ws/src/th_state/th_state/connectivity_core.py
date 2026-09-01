"""connectivity_core.py — connectivity_checker の ROS2 非依存の純粋コア（WP-STATE-03）

rclpy を import しない（`th_testing` から直接 pytest できる。CLAUDE.md／
DetailedDesign-state.md §1 と同じ制約。state_core.py / zones.py と同じ流儀）。

DetailedDesign-state.md §12.2 の4行がここでの合否判定そのもの（この表が正）。
「リンクしている」だけでは足りない ―― 各項目とも「実際にデータが行き来していること」を
受信間隔で確かめる（Spec-ops.md §2.2）。

数値リテラルを書かない（R2）。しきい値・期待点数・必須ノード名はすべて呼び出し側
（connectivity_checker.py。registry.yaml 由来の ROS2 パラメータ）から `Params` として渡す。
"""
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


@dataclass(frozen=True)
class Params:
    """DetailedDesign-wp1.md WP-STATE-03 §3.3 のパラメータのうち `evaluate()` が使うもの。

    `sim`: §3.3 の表そのものには無い、このパケットの §8（Gazebo シナリオ）を満たすための
    追加フィールド（判断は WP-STATE-03 完了報告に明記）。Gazebo には `esp32_bridge` が
    居ないため、ESP32 の2項目（`esp32_feedback`/`esp32_loopback`）と `required_nodes` の
    判定を除外する。「除外の事実を黙って通さない」（§8）はログで表現する
    （呼び出し側 = connectivity_checker.py の責務）。
    """
    esp32_alive_timeout_ms: int
    scan_expected_points: int
    required_nodes: Tuple[str, ...]
    sim: bool = False


@dataclass(frozen=True)
class LinkReport:
    esp32_feedback: bool
    esp32_loopback: bool
    lidar: bool
    nodes: bool
    missing_nodes: Tuple[str, ...]

    def all_ok(self) -> bool:
        return self.esp32_feedback and self.esp32_loopback and self.lidar and self.nodes


def _fresh(now_ms: int, last_ms: Optional[int], timeout_ms: int) -> bool:
    """6.2 フェイルセーフ既定: 一度も受信していない（`None`）は不合格として扱う。"""
    if last_ms is None:
        return False
    return (now_ms - last_ms) <= timeout_ms


def evaluate(now_ms: int, last_fb_ms: Optional[int], last_cmd_ms: Optional[int],
             last_scan_ms: Optional[int], scan_points: int,
             present_nodes: Iterable[str], p: Params) -> LinkReport:
    """DetailedDesign-state.md §12.2 の4行を判定する。

    | 対象 | 実装 |
    | --- | --- |
    | 行1 ESP32 | `/esp32/wheel_feedback` の受信間隔が `esp32_alive_timeout_ms` 以内 |
    | 行2 ESP32 | `/esp32/wheel_cmd_speed` の受信間隔が同時間内（キープアライブの折り返し） |
    | 行3 RaspberryPi4 | `/scan` の受信間隔が同時間内 かつ `ranges` の点数が `scan_expected_points` に一致 |
    | 行4 PC | `required_nodes` が全て `present_nodes` に含まれる |

    L-1: Wi-Fi AP はどの項目にも現れない（`LinkReport` のフィールド集合そのものが判定項目）。
    `p.sim` が真のとき、行1・行2・行4（ESP32 の2項目と required_nodes）を除外する（§8）。
    """
    if p.sim:
        esp32_feedback = True
        esp32_loopback = True
        missing_nodes: Tuple[str, ...] = ()
        nodes = True
    else:
        esp32_feedback = _fresh(now_ms, last_fb_ms, p.esp32_alive_timeout_ms)
        esp32_loopback = _fresh(now_ms, last_cmd_ms, p.esp32_alive_timeout_ms)
        present = set(present_nodes)
        missing_nodes = tuple(n for n in p.required_nodes if n not in present)
        nodes = len(missing_nodes) == 0

    lidar = (_fresh(now_ms, last_scan_ms, p.esp32_alive_timeout_ms)
              and scan_points == p.scan_expected_points)

    return LinkReport(
        esp32_feedback=esp32_feedback,
        esp32_loopback=esp32_loopback,
        lidar=lidar,
        nodes=nodes,
        missing_nodes=missing_nodes,
    )
