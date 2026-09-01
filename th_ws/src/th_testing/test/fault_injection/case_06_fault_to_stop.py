"""
case_06_fault_to_stop.py — 故障注入 6「フォルト検知 → 停止」
（ctest 登録名: fault_injection_06）
=================================================================
`DetailedDesign-safety.md` §10 #6: 5 の続きを測る。フォルトから
`conftest.FAULT_TO_STOP_LAYER3_MS`（＝100ms。層3の応答時間そのものであり
パラメータではなく設計上の定数。T-1 の唯一の例外）以内に速度指令が 0 になる
ことを確認する。自動化: Gazebo ＋ 実機。

**T-2: これは 5（通信断 → フォルト検知）とは別のテストである。** `sim_stack` を
独立に立て、`case_05_link_loss_fault.py` と同じ手段（`/scan` を止めつつ
`/clock` を止めない）で自前で LIDAR_LOST を発生させる。5 のプロセスや状態を
共有しない。

【観測点を `/cmd_vel` ではなく `/cmd_vel_muxed` にした理由（実装しながら判明・
このパケットで自分で判断した点）】
`DetailedDesign-safety.md` §1.1「層3は `obstacle_limiter` を通らない」の図では、
層3（`safety_monitor` → `/safety/estop` `/safety/fault_lock` → `twist_mux` の
ロック）の実体は `twist_mux` と（実機のみの）`esp32_bridge` の2本腕であり、
`obstacle_limiter` は含まれない。ところが `obstacle_limiter.cpp` は実装上
`/safety/fault_lock` を直接購読し（tier1・`obstacle_limiter_core.cpp:163`）、
さらに `/scan` 自体の途絶も独立に検知して停止させる（tier3・`scan_stale_ms`。
`obstacle_limiter_core.cpp:185`）。

registry.yaml を実際に解決すると
`lidar_timeout_ms = 308ms`（`resolve_registry()` 実測。`safety_monitor` が
LIDAR_LOST を検知するまでの時間）に対し `scan_stale_ms = 300ms`（`given`。
`obstacle_limiter` が tier3 で `/scan` 途絶それ自体を検知してゼロを出すまでの
時間）と、**ほぼ同じ大きさ**であることが分かった。つまり `/scan` を止めると、
`obstacle_limiter` の tier3（scan 途絶。safety_monitor のフォルト検知より前に、
ほぼ同時か早いタイミングで独立に発火しうる）によって `/cmd_vel` が**フォルトが
実際に検知されるより前に、あるいはほぼ同時に**ゼロになってしまう。この状態で
`/cmd_vel` を「フォルト検知からの応答時間」の観測点にすると、フォルトが
発火する前からすでにゼロだったものを「100ms以内にゼロになった」と誤判定
しかねない——これは `control.py`（FMEA①「テストが誤って通る」）が守ろうと
している問題そのものであり、`/scan` 途絶を使う限り LIDAR_LOST では `/cmd_vel`
を観測点にできないと判断した。

`/cmd_vel_muxed`（`twist_mux` の出力。`th_safety/config/twist_mux.yaml` の
`cmd_vel_out:=/cmd_vel_muxed` remap）は `/scan` の鮮度に一切依存せず、
`safety_monitor` が発行する `/safety/fault_lock`（bool）だけを見て
ロックするため、この confound を避けられる。`/cmd_vel_muxed` は**既存の
トピック**（新設ではない。§3「新しいトピック・サービス・パラメータを
作らない」に抵触しない）で、`DetailedDesign-safety.md` §1.1 の層3の定義
（「`twist_mux` のロック」）にちょうど対応する観測点でもある。

【フォルト検知時刻の起点】
`assert_fault_within` で LIDAR_LOST の active を確認したあと、
`conftest.first_match_time()` で `fault_watcher.records` から実際に
「active かつ fault_type == LIDAR_LOST」な最初のメッセージの受信時刻
（`time.monotonic()`）を取り出し、それを `assert_zero_within` の `since` に渡す。
`muxed_watcher` は `/scan` を止める**前**（=フォルトより十分前）から購読を
開始しておく（`TopicWatcher` の docstring どおり。取りこぼしによる
偽陽性を避けるため）。

【走行させている理由】
`case_01`/`case_11` と同じ理由——「止まっていたものが見かけ上ゼロだった」
ではなく、「実際に動いていたものがフォルトを境に止まった」ことを示すため、
`enter_manual_mode()` で `MANUAL` に入り `DriveController` で走らせ続けた
状態でフォルトを注入する。

【観測窓に `safety_monitor_sim_startup_grace_sec` を足す理由
（コーディネーターとの共同調査で判明。2026-08-28）】
`case_05_link_loss_fault.py` と同じ理由・同じ対処。詳細は
`conftest.py` の `safety_monitor_sim_startup_grace_sec` フィクスチャの
docstring参照。要点: `safety_monitor.cpp` は起動から `startup_grace_sec`
（sim秒。既定7秒）経つまで `checkTimeout()` を呼ばず、`/safety/fault_lock`
の10Hz発行はこの間も止まらないため外部から grace 中かどうか分からない。
`cut_lidar_and_keep_clock_alive()` を呼ぶ時点の sim 時刻がまだ grace 期間内
（実測: `case_05` で cut時 sim=2.8s）だと、`lidar_timeout_ms` 由来の余裕
だけでは grace を抜けられないまま観測窓が尽きる。`enter_manual_mode()` の
分だけ `case_05` より cut が遅れるため grace を抜けている可能性はあるが、
保証はできないので同じ安全側の余裕を入れる。
"""
from __future__ import annotations

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from th_system_msgs.msg import FaultStatus

from fault_injection.conftest import (
    FAULT_TO_STOP_LAYER3_MS, DriveController, TopicWatcher,
    cut_lidar_and_keep_clock_alive, enter_manual_mode, first_match_time,
)

# scaffolding（case_05 と同じ流儀。T-1の対象外）。
_FAULT_WINDOW_MULTIPLIER = 8

_SIM_STACK_PARAM = {'launch_args': {'obstacle': 'false'}}


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_06_fault_to_stop(
        sim_stack, ros_node, fault_params, limiter_param,
        safety_monitor_sim_startup_grace_sec,
        assert_fault_within, assert_zero_within):
    bootstrap, state_watcher = enter_manual_mode(ros_node)
    fault_watcher = TopicWatcher(ros_node, '/safety/fault', FaultStatus)
    muxed_watcher = TopicWatcher(ros_node, '/cmd_vel_muxed', Twist)
    drive = DriveController(ros_node, linear_x=limiter_param('v_max'))
    synthetic_clock = None
    try:
        # ウォームアップ: 実際に走り出してから /scan を止める。
        warmup_deadline = time.monotonic() + 1.0
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(ros_node, timeout_sec=0.05)
        muxed_watcher.records.clear()

        synthetic_clock, _cut_time = cut_lidar_and_keep_clock_alive(ros_node)

        # 上のモジュールdocstring参照: cut時のsim時刻が最悪0秒(grace起点)
        # だった場合でもgraceを抜けられるよう、startup_grace_sec分を
        # 必ず観測窓に含める。
        window_ms = (
            safety_monitor_sim_startup_grace_sec * 1000
            + fault_params['lidar_timeout_ms'] * _FAULT_WINDOW_MULTIPLIER)
        assert_fault_within(fault_watcher, 'LIDAR_LOST', ms=window_ms)
        t_fault = first_match_time(
            fault_watcher, lambda m: m.active and m.fault_type == 'LIDAR_LOST')

        assert_zero_within(
            muxed_watcher, 'linear.x', since=t_fault, ms=FAULT_TO_STOP_LAYER3_MS)
    finally:
        if synthetic_clock is not None:
            synthetic_clock.stop()
        drive.stop()
        fault_watcher.destroy()
        muxed_watcher.destroy()
        bootstrap.stop()
        state_watcher.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
