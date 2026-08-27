"""
case_05_link_loss_fault.py — 故障注入 5「通信断 → フォルト検知」
（ctest 登録名: fault_injection_05）
=====================================================================
`DetailedDesign-safety.md` §10 #5: LiDAR / ESP32 の通信を切り、`lidar_timeout_ms` /
`esp32_timeout_ms` 以内にフォルトが立つことを確認する。自動化: Gazebo ＋ 実機。

**T-2: これは 6（フォルト検知 → 停止）とは別のテストである。** 1本にまとめて
「通信を切ってから 100ms 以内に停止」と書くと、`lidar_timeout_ms` が現状 2000ms
前後であるため正常な実装でも必ず不合格になる（100ms は層3の応答時間であって
通信断からの合計ではない。`DetailedDesign-safety.md` §10 の注／`F-21`）。

**このパケットの範囲外。** Gazebo 本体は次パケットで書く。ここでは
`conftest.py` の `fault_params['lidar_timeout_ms']` / `['esp32_timeout_ms']`
（T-1: registry.yaml から解決した値であり、このテストに数値を直書きしない）を使う
設計だけを示すプレースホルダを置く。
"""
import pytest


def test_fault_injection_05_link_loss_raises_fault(fault_params):
    pytest.fail(
        "未実装。次パケットで実装する（Gazebo で LiDAR/ESP32 通信を切り、"
        "fault_params['lidar_timeout_ms'] / ['esp32_timeout_ms'] 以内に "
        "`/safety/fault` で LIDAR_LOST / ESP32_DISCONNECTED が active になることを "
        "`sim_stack` / `assert_fault_within` で確認する）。"
        "DetailedDesign-safety.md §10 #5 参照。"
    )
