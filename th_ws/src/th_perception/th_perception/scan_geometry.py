"""
scan_geometry.py — 角度 → LaserScan 添字の変換規約（正本）
================================================================
`lidar_filter`（th_perception）と `obstacle_limiter`（th_safety。今後実装、
C++ 移植版 `scan_geometry.hpp`）が同じ `blind_angle_ranges` を同じセクタ
として解釈するための変換規約を、ここ 1 か所に集約する
（docs/plan/detailed/DetailedDesign-wp2.md WP-CALIB-01 §4.1・B-3）。

ROS2 非依存の純粋 Python（follow_planner_core.py / keepalive_core.py と
同じ二層構造の流儀。CLAUDE.md「追従ロジックの二層構造」参照）。`scan`
引数は sensor_msgs/LaserScan そのものでなくてよく、`angle_min` /
`angle_increment` / `ranges`（長さのみ使用）を持つダックタイピングで足りる
（pytest から ROS2 なしでテストできるようにするため）。

規約（§4.1 原文。番号はそのまま踏襲する）:
  ① 角度は laser_link 基準・ラジアン・**反時計回り正**。
     sensor_msgs/LaserScan の標準どおり angle = angle_min + i *
     angle_increment の向きに一致する（ROS の LaserScan は反時計回りが
     正の標準）。
     ※ 旧 lidar_filter.py のコメントは「(0=前方 右回り正)」と書かれて
     いたが、実装（angle = msg.angle_min + i * msg.angle_increment を
     そのまま比較）は反時計回り正で動いており、コメントと実装が矛盾して
     いた。実装のほうを正としてコメントを規約①に合わせて修正する
     （このパッケージの後続コミットで対応）。
  ② registry.yaml の blind_angle_ranges は度で持ち、ロード時に 1 度だけ
     ラジアンへ変換する。本モジュールでは sector_indices() が a0_deg /
     a1_deg という「度」の引数を受け取り、内部で angle_to_index() に渡す
     直前の 1 回だけ math.radians() で変換する。
  ③ 角度は [-pi, pi) に正規化してから比較する。
  ④ 添字は floor((angle - angle_min) / angle_increment)。
  ⑤ 範囲外は clamp せず「そのセクタは存在しない」として扱う。
     → 本モジュールでは **None** を返す。理由:
       - Python では「値なし」を表す最も自然な表現であり、呼び出し側は
         `if idx is None:` で素直に分岐できる。
       - 例外にすると sector_indices() のように 2 つの角度のうち片方だけ
         範囲外、というケースの扱いが煩雑になる（呼び出し側で
         try/except を 2 回書くか、部分的な結果を例外の中から取り出す
         必要が出る）。
       - 番兵値（例: -1）は「本物の添字 -1」との区別が付けられず事故の
         もとになるため避けた。
       C++ には None が無いが、C++17 の `std::optional<std::size_t>` が
       同じ意味（「値があるかもしれないし無いかもしれない」を型で表す）
       を持つため、C++ 移植版 `scan_geometry.hpp` は
       `std::optional<std::size_t>` を返す形でこの選択を踏襲する。

浮動小数点の注意（B-3。過去に同種の 1 ULP 問題が実バグとして見つかっている）:
  floor((angle - angle_min) / angle_increment) は角度が格子点ちょうどの
  ときに丸め誤差で隣の添字へ落ちることがある。実際、`angle_min + k *
  angle_increment` で作った角度を [-pi, pi) へ正規化してから逆算すると、
  fmod の丸めにより `(角度 - angle_min) / increment` が `k` ではなく
  `k - 1e-14` 程度になることがあり、素の floor() では意図した k ではなく
  k-1 を返してしまう（本モジュール実装時に test_scan_geometry.py で実際に
  再現・確認済み）。これを放置すると「境界ちょうどの角度が隣のセクタに
  誤分類される」という実害になるため、floor の直前に十分小さい
  epsilon（`_FLOOR_EPS = 1e-9`）を加算して丸め誤差を吸収する。
  1e-9 は実運用の angle_increment（数 mrad〜数十 mrad）に対して角度換算で
  1e-11 rad 程度の補正にしかならず、意図的な半端角度の floor 挙動
  （規約④）には影響しない大きさに選んである。

  Python と C++ が bit-for-bit 同じ結果を返すことが等価性テスト
  （test_scan_geometry_equivalence、B-3）の要件なので、両言語とも
    正規化 → 減算 → 除算 → epsilon 加算 → floor → 整数キャスト
  の演算順序と epsilon の値（1e-9）を厳密に揃えてある。度→ラジアン変換も
  `度 * (pi / 180.0)`（Python の math.radians() の内部実装と同じ式）で
  揃えており、pi の値そのものも C++ 側で Python の math.pi と同じ最近接
  倍精度表現を明示的な定数として使う（scan_geometry.hpp 参照）。
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

# floor() 直前に加える微小補正。角度が格子点ちょうどのとき丸め誤差で
# 1 つ下の添字へ落ちるのを防ぐ（モジュール docstring「浮動小数点の注意」参照）。
# C++ 側 (scan_geometry.hpp) も同じ値を使うこと（B-3）。
_FLOOR_EPS = 1e-9


def _normalize_angle(angle_rad: float) -> float:
    """角度を [-pi, pi) に正規化する（規約③）。

    C++ 側 (scan_geometry.hpp の normalize_angle()) も std::fmod を使い
    まったく同じ演算順序で実装してある（B-3）。
    """
    normalized = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if normalized < 0.0:
        normalized += 2.0 * math.pi
    return normalized - math.pi


def angle_to_index(angle_rad: float, scan) -> Optional[int]:
    """`scan` の angle_min / angle_increment から LaserScan の添字を返す。

    規約①〜⑤（モジュール docstring 参照）。`scan` は `angle_min` /
    `angle_increment` / `ranges` を持つオブジェクト（sensor_msgs/LaserScan
    でもダミーでもよい）。範囲外（0 <= idx < len(scan.ranges) を満たさない）
    なら None（規約⑤。clamp しない）。
    """
    n = len(scan.ranges)
    if n == 0:
        return None

    angle_norm = _normalize_angle(angle_rad)
    idx_f = math.floor((angle_norm - scan.angle_min) / scan.angle_increment + _FLOOR_EPS)
    idx = int(idx_f)

    if idx < 0 or idx >= n:
        return None
    return idx


def sector_indices(a0_deg: float, a1_deg: float, scan) -> Tuple[Optional[int], Optional[int]]:
    """[a0_deg, a1_deg]（度）を添字の (i0, i1) に変換する。

    a0_deg <= a1_deg（正規化後）であれば [i0, i1] が連続区間を意味する。
    a0_deg > a1_deg のときは 0 をまたぐ 2 区間（[i0, len-1] と [0, i1]）を
    意味する。**連結（実際に添字集合を組み立てる処理）は呼び出し側の責務**
    （§4.1）。i0 / i1 のどちらかが範囲外なら None（規約⑤）。片方だけ
    None になった場合の扱い（そのセクタ全体を諦めるか等）も呼び出し側の
    責務とする。
    """
    i0 = angle_to_index(math.radians(a0_deg), scan)
    i1 = angle_to_index(math.radians(a1_deg), scan)
    return (i0, i1)
