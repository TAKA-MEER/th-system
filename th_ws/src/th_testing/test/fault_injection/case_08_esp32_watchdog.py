"""
case_08_esp32_watchdog.py — 故障注入 8「ESP32 ウォッチドッグ」（ctest 登録名: fault_injection_08）
====================================================================================
`DetailedDesign-safety.md` §10 #8: ROS2 側（PC）を落とし、ESP32 が独立ウォッチドッグ
（`config.h` の `WATCHDOG_MS`。registry.yaml の `esp32_watchdog_ms`＝600ms）だけで
停止できることを確認する。自動化: **実機のみ**（ROS2 プロセスを本当に落とす必要があり、
Gazebo では ESP32 ファーム自体が存在しないため代替できない）。

T-3 の理由・逃げ道を作らなかった理由は `case_04_estop_physical.py` の docstring と同じ
（重複を避けるためここでは繰り返さない）。
"""
import pytest

_PROCEDURE_NOTE = (
    "実機専用（通電必要。ROS2 プロセスを実際に落とす）。"
    "手順書: docs/実機作業キュー.md §1 の `WP-TEST-01` 行を参照。"
    "この項目（#8 ESP32ウォッチドッグ）単体の実施手順はまだ同ファイルに個別記載が無いため、"
    "実施時に手順と結果をそこへ追記すること。"
    "仕様: docs/plan/detailed/DetailedDesign-safety.md §10 の表 #8 行。"
)


def test_fault_injection_08_esp32_watchdog():
    pytest.fail(
        "故障注入8『ESP32ウォッチドッグ』は実機のみで確認する項目であり自動化しない。"
        + _PROCEDURE_NOTE
    )
