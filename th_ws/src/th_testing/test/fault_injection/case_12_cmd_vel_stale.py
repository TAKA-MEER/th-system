"""
case_12_cmd_vel_stale.py — 故障注入 12「`/cmd_vel` の途絶」（ctest 登録名: fault_injection_12）
=================================================================================
`DetailedDesign-safety.md` §10 #12: `obstacle_limiter` を止めたまま非ゼロ指令を残し、
`cmd_vel_stale_ms` 以内に ESP32 への指令（`/esp32/wheel_cmd_speed`）がゼロになることを
確認する。自動化: **統合テスト ＋ 実機（Gazebo ではない）**。

**`esp32_bridge` は Gazebo で起動しない。** よってこの項目だけ `sim_stack`
（Gazebo 起動）を使わず、`launch_testing` で `esp32_bridge` ＋ WS モックサーバーを
直接立てる形になる。既存の `test_esp32_bridge_node.py`（`generate_test_description()`
で `esp32_bridge.py` を起動し、`_WsMockEsp32` で ESP32 の代わりに WS 接続する構造）が
そのまま手本になる。

**このパケットの範囲外。** 統合テスト本体は次パケットで書く。ここでは
`conftest.py` の `fault_params['cmd_vel_stale_ms']`（T-1）を使う設計だけを示す
プレースホルダを置く。`sim_stack` には依存しない（上記の理由により、この項目は
そもそも Gazebo を使わないため）。
"""
import pytest


def test_fault_injection_12_cmd_vel_stale(fault_params):
    pytest.fail(
        "未実装。次パケットで実装する（test_esp32_bridge_node.py と同様に "
        "launch_testing で esp32_bridge ＋ WS モックを起動し、`/cmd_vel` を途絶させた上で "
        "fault_params['cmd_vel_stale_ms'] 以内に `/esp32/wheel_cmd_speed` が 0 になることを"
        " 確認する。sim_stack は使わない — esp32_bridge は Gazebo で起動しないため）。"
        "DetailedDesign-safety.md §10 #12 参照。"
    )
