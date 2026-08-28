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
このファイルは「通信断 → フォルト検知」だけを測る。「フォルト検知 → 停止」は
`case_06_fault_to_stop.py` が別に測る。

【ESP32 は対象外】
`gazebo.launch.py` の `SAFETY_ENABLED_TARGETS_SIM = ['lidar', 'limiter']` に
`'esp32'` が入っていない（`esp32_bridge` が sim では起動しないため
`safety_monitor.cpp` の `targetEnabled("esp32")` が常に False——`checkTimeout`
自体が呼ばれない）。したがって sim では `ESP32_DISCONNECTED` を発火させる
方法がそもそも存在しない。このテストは **LIDAR_LOST のみ** を検証する
（作業指示にある「ESP32側は sim に esp32_bridge が居ないので使えない」を
実装しながら確認した）。

【`/scan` をどう止めたか（/clock を止めない）】
`conftest.py` の `cut_lidar_and_keep_clock_alive()` を使う。gzserver/gzclient
を殺して `/scan`（Gazebo の LiDAR センサプラグインが同一プロセス内で発行）を
止めつつ、`SyntheticClock` で `/clock` を実時間で発行し続けることで、
use_sim_time=True な全ノードの ROS 時刻が凍結しないようにする。
理由・検討した代替案・既知のリスク（Docker 未実行のため未検証）は
`conftest.py` の `cut_lidar_and_keep_clock_alive` 付近のモジュール docstring
に詳しく書いた。**本番の `gazebo_plugins.xacro` / `gazebo.launch.py` は
一切変更していない**（試験のためだけに本番launch構造を変える案は
実装せず作業報告で扱いを確認する対象にした）。

【走行させていない】
このテストは「フォルトが検知されるか」だけを見るので、`enter_manual_mode()`
でロボットを実際に走らせる必要はない（safety_monitor のタイムアウト判定は
`state_manager` のモードに依存しない）。走行を伴う検証（「フォルト検知後に
本当に止まるか」）は `case_06_fault_to_stop.py` が担当する。

【観測窓に `safety_monitor_sim_startup_grace_sec` を足す理由
（コーディネーターとの共同調査で判明。2026-08-28）】
`conftest.py` の `safety_monitor_sim_startup_grace_sec` フィクスチャの
docstring に詳細を書いたが、要点だけここにも記す: `safety_monitor.cpp` は
起動から `startup_grace_sec`（sim 秒。既定7秒）経つまで `checkTimeout()`
（LIDAR_LOST 検知本体）を一切呼ばない。ところが `/safety/fault_lock` の
10Hz発行はこの grace 期間中も止まらないため、外部からの観測だけでは
grace 中かどうか分からない。`sim_stack` の準備完了条件（`/scan` を1件受信）
は Gazebo の物理が動き始めてすぐ満たされうるため、このテストは
**sim 時刻がまだ grace 期間内のうちに `/scan` を止めうる**
（実測: cut時 sim=2.8s）。観測窓が `lidar_timeout_ms` 由来の余裕だけだと
grace を抜けられないまま打ち切られる（実測: 2464ms 窓で LIDAR_LOST が
一度も active にならなかった failure）。そのため、`cut` の時点で sim 時刻が
最悪 0 秒（grace の起点）だったと仮定しても grace を抜けられるだけの
時間（`startup_grace_sec` 秒ぶん）を観測窓に必ず含める。
"""
from __future__ import annotations

import pytest
from th_system_msgs.msg import FaultStatus

from fault_injection.conftest import TopicWatcher, cut_lidar_and_keep_clock_alive

# scaffolding（上のモジュールdocstring参照。T-1の対象外）。
# safety_monitor の check_period_ms(100ms) 周期のポーリング・SyntheticClock の
# 発見/購読の成立に要する遅延を吸収するための余裕（`case_11` の
# `dead_ms * 3` と同じ流儀）。
_FAULT_WINDOW_MULTIPLIER = 8

_SIM_STACK_PARAM = {'launch_args': {'obstacle': 'false'}}


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_05_link_loss_raises_fault(
        sim_stack, ros_node, fault_params, safety_monitor_sim_startup_grace_sec,
        assert_fault_within):
    fault_watcher = TopicWatcher(ros_node, '/safety/fault', FaultStatus)
    synthetic_clock = None
    try:
        synthetic_clock, _cut_time = cut_lidar_and_keep_clock_alive(ros_node)

        # 上のモジュールdocstring参照: cut時のsim時刻が最悪0秒(grace起点)
        # だった場合でもgraceを抜けられるよう、startup_grace_sec分を
        # 必ず観測窓に含める。
        window_ms = (
            safety_monitor_sim_startup_grace_sec * 1000
            + fault_params['lidar_timeout_ms'] * _FAULT_WINDOW_MULTIPLIER)
        assert_fault_within(fault_watcher, 'LIDAR_LOST', ms=window_ms)
    finally:
        if synthetic_clock is not None:
            synthetic_clock.stop()
        fault_watcher.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
