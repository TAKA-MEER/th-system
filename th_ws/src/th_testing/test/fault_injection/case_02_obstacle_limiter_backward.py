"""
case_02_obstacle_limiter_backward.py — 故障注入 2「障害物リミッタ（後退）」
（ctest 登録名: fault_injection_02）
=========================================================================
`DetailedDesign-safety.md` §10 #2: 障害物へ向けて後退させ、停止することを確認する
（`CL-X-4`）。自動化: Gazebo。

**このパケットの範囲外。** 中身は `case_01_obstacle_limiter_forward.py` と同じ理由で
次パケットに送る。
"""
import pytest


def test_fault_injection_02_obstacle_backward():
    pytest.fail(
        "未実装。次パケットで実装する（Gazebo で障害物へ後退させ、`/cmd_vel` が "
        "0 になることを `sim_stack` / `assert_stops_within` で確認する）。"
        "DetailedDesign-safety.md §10 #2（`CL-X-4`）参照。"
    )
