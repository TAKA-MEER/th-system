"""
case_10_physical_button_at_boot.py — 故障注入 10「物理ボタン起動時押下」
（ctest 登録名: fault_injection_10）
========================================================================
`DetailedDesign-safety.md` §10 #10: 物理非常停止ボタンを押したまま起動し、
運用モードに入らせず解除を案内することを確認する。自動化: **実機のみ**
（物理ボタンの押下状態そのものが試験条件であり、Gazebo では再現できない）。

T-3 の理由・逃げ道を作らなかった理由は `case_04_estop_physical.py` の docstring と同じ
（重複を避けるためここでは繰り返さない）。
"""
import pytest

_PROCEDURE_NOTE = (
    "実機専用（通電必要。起動シーケンスと物理ボタンの押下タイミングの組み合わせが試験条件）。"
    "手順書: docs/実機作業キュー.md §1 の `WP-TEST-01` 行を参照。"
    "この項目（#10 物理ボタン起動時押下）単体の実施手順はまだ同ファイルに個別記載が無いため、"
    "実施時に手順と結果をそこへ追記すること。"
    "仕様: docs/plan/detailed/DetailedDesign-safety.md §10 の表 #10 行。"
)


def test_fault_injection_10_physical_button_at_boot():
    pytest.fail(
        "故障注入10『物理ボタン起動時押下』は実機のみで確認する項目であり自動化しない。"
        + _PROCEDURE_NOTE
    )
