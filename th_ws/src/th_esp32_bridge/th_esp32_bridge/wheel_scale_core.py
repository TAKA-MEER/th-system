"""
wheel_scale_core.py — esp32_bridge の wheel_radius_scale（車輪半径スケール）適用
============================================================================
WP-ESP32-02 (O-d10): ESP32 ファームの公称半径 R_fw と真の半径 R_true の比
k = R_true / R_fw を esp32_bridge の両方向にかけて、直進校正（測距）の出力を
閉じる。ファームは触らない（docs/plan/detailed/DetailedDesign-maintenance.md §4）。

  | 方向 | 変換 |
  | --- | --- |
  | 指令（PC → ESP32） | v_send = v_desired / k |
  | 実測（ESP32 → PC） | v_actual = v_report × k |

A10: |wheel_radius_scale − 1| ≤ wheel_radius_scale_max_dev（既定 0.1）。超えたら
**起動を拒否**（10% を超えるずれは校正ではなく機械的な異常。同 §4.3）。判定は
`th_params.assertions.a10_wheel_radius_scale` を再利用する（単一の正本）。
ただし同断言は浮動小数の `<=` をそのまま使うため、境界ちょうど
（k = 1.0 ± max_dev。真の値は |k−1| == max_dev）が 2 進数表現の丸めで
僅か（≈1e-16）に「超える」と判定される。ここではその人工物だけを
`_A10_BOUND_TOLERANCE`（1e-9。丸め誤差より十分大きく、判定に実影響を
与え得る差 0.0001 より十分小さい）で吸収する。

テスト容易性のため ROS2 インポートを一切含まない。ノード側
（scripts/esp32_bridge.py）はこのモジュールを import して使うだけにする。
"""
from __future__ import annotations

from th_params.assertions import a10_wheel_radius_scale as _a10_wheel_radius_scale

# A10 境界の浮動小数丸め吸収（上記モジュール docstring 参照。1e-9）。
_A10_BOUND_TOLERANCE = 1e-9


def scale_wheel_command(v_desired: float, k: float) -> float:
    """指令側: ESP32 へ送る車輪速度 [m/s] = 望む速度 / k。

    ESP32 の PID は「自分の報告値が指令に一致するよう駆動する」ので、真の速度は
    指令の k 倍になる（maintenance.md §4.2）。望む速度 v_desired を得るには、
    その k 分の 1 を送る。PC 側の速度上限（obstacle_limiter）は v_desired に、
    ESP32 内部の速度制限は v_send に効くため、この変換は安全上限を狂わせない。
    """
    return v_desired / k


def scale_wheel_feedback(v_report: float, k: float) -> float:
    """実測側: 真の車輪速度 [m/s] = ESP32 の報告値 × k。

    ESP32 は公称半径 R_fw で速度を計算するため、真の半径 R_true との比 k 分だけ
    実測が短く（長く）出る。オドメトリ積分・/esp32/wheel_feedback・/odom へ
    使う前に ×k して真の単位へそろえる。
    """
    return v_report * k


def a10_violations(wheel_radius_scale: float, max_dev: float) -> list[str]:
    """A10 判定: |wheel_radius_scale − 1| ≤ max_dev。

    戻り値は違反メッセージのリスト（**空 = 合格**）。`th_params.assertions` の
    `a10_wheel_radius_scale` を再利用する（起動時と実行中 `ros2 param set` の
    両方でこの関数を呼ぶ）。負・0・NaN はいずれも「1 からの差が max_dev を
    超える」ため不合格になる（abs() の性質）。
    """
    violations = _a10_wheel_radius_scale(wheel_radius_scale, max_dev)
    if violations and abs(wheel_radius_scale - 1.0) <= max_dev + _A10_BOUND_TOLERANCE:
        # 境界ちょうど（|k−1| が真には max_dev）の丸め人工物のみ合格にする。
        # 真正の違反（|k−1| ＞ max_dev）はこの分岐に入らない。
        return []
    return violations