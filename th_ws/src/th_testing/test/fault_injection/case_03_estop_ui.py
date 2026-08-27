"""
case_03_estop_ui.py — 故障注入 3「非常停止（UI）」（ctest 登録名: fault_injection_03）
====================================================================================
`DetailedDesign-safety.md` §10 #3: 走行中に UI から非常停止を押し、停止することを
確認する。自動化: Gazebo ＋ 実機（Gazebo 側がこのファイルの対象。実機側の確認は
別途 `docs/実機作業キュー.md` の運用に乗る）。

**このパケットの範囲外。** 中身は `case_01_obstacle_limiter_forward.py` と同じ理由で
次パケットに送る。
"""
import pytest


def test_fault_injection_03_estop_ui():
    pytest.fail(
        "未実装。次パケットで実装する（Gazebo で走行中に `/mode_manager` 等の "
        "非常停止相当サービスを呼び、`/cmd_vel` が 0 になることを "
        "`sim_stack` / `assert_stops_within` で確認する）。"
        "DetailedDesign-safety.md §10 #3 参照。"
    )
