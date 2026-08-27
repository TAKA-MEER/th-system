"""
case_09_person_detection_off.py — 故障注入 9「人検知 OFF とフォルトの区別」
（ctest 登録名: fault_injection_09）
============================================================================
`DetailedDesign-safety.md` §10 #9: 人検知を意図的に OFF にし、フォルトにならない・
強制遷移しないことを確認する（`CLAUDE.md` の「モード FSM」節にある
`PERSON_TRACKER_LOST` 例外の裏側の確認）。自動化: Gazebo。

**このパケットの範囲外。** 中身は `case_01_obstacle_limiter_forward.py` と同じ理由で
次パケットに送る。
"""
import pytest


def test_fault_injection_09_person_detection_off_is_not_a_fault():
    pytest.fail(
        "未実装。次パケットで実装する（Gazebo で人検知を意図的に OFF にし、"
        "`/safety/fault` が active にならないこと・`/system/state` の mode が "
        "強制遷移しないことを確認する）。"
        "DetailedDesign-safety.md §10 #9 / CLAUDE.md『モード FSM』節参照。"
    )
