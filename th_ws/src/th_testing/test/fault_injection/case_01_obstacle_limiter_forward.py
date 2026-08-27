"""
case_01_obstacle_limiter_forward.py — 故障注入 1「障害物リミッタ（前進）」
（ctest 登録名: fault_injection_01）
========================================================================
`DetailedDesign-safety.md` §10 #1: 障害物へ向けて前進させ、停止することを確認する。
自動化: Gazebo。

**このパケットの範囲外。** Gazebo を実際に起動し障害物へ走らせる試験本体は
別パケットで書く（`feat/wp-test-01-fault-injection` の作業指示より、
このパケットは「共通骨格 + 実機専用3本 + ctest登録」のみが範囲）。
ここでは ctest 登録（`fault_injection_01`）の実体として、意図的に fail する
プレースホルダを置く。

観測点（実装予定）: `/cmd_vel` が 0 になること（`conftest.py` の
`TopicWatcher` ＋ `assert_stops_within` フィクスチャを使う設計）。
"""
import pytest


def test_fault_injection_01_obstacle_forward():
    pytest.fail(
        "未実装。次パケットで実装する（Gazebo で障害物へ前進させ、`/cmd_vel` が "
        "0 になることを `sim_stack` / `assert_stops_within` で確認する）。"
        "DetailedDesign-safety.md §10 #1 / DetailedDesign-wp2.md WP-TEST-01 参照。"
    )
