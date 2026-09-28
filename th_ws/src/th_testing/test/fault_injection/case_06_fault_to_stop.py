"""
case_06_fault_to_stop.py — 故障注入 6「フォルト検知 → 停止」
（ctest 登録名: fault_injection_06）
=================================================================
`DetailedDesign-safety.md` §10 #6: 5 の続きを測る。フェルトから
`conftest.FAULT_TO_STOP_LAYER3_MS`（＝100ms。層3の応答時間そのものであり
パラメータではなく設計上の定数。T-1 の唯一の例外）**経過した以降、
`twist_mux` の出力 `/cmd_vel_muxed` に非ゼロが 1 件も出ない**ことを確認する。
自動化: Gazebo ＋ 実機。（合格条件の変更理由と 100ms の根拠は下の
【`twist_mux` のロックは 0 を出さず黙る】を参照。2026-09-28 ユーザー決定）

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

`/cmd_vel_muxed`（`twist_mux` の出力。`th_bringup/launch/gazebo.launch.py` の
`cmd_vel_out:=/cmd_vel_muxed` remap）は `/scan` の鮮度に一切依存せず、
`safety_monitor` が発行する `/safety/fault_lock`（bool）だけを見て
ロックするため、この confound を避けられる。`/cmd_vel_muxed` は**既存の
トピック**（新設ではない。§3「新しいトピック・サービス・パラメータを
作らない」に抵触しない）で、`DetailedDesign-safety.md` §1.1 の層3の定義
（「`twist_mux` のロック」）にちょうど対応する観測点でもある。

【`twist_mux` のロックは 0 を出さず黙る（2026-09-28。判定基準を差し替えた理由）】
`twist_mux` 4.3.0（`ros-teleop/twist_mux` humble ブランチ）のソースで確かめた：
`LockTopicHandle::callback` は `stamp_` と `msg_` を覚えるだけで何も publish
せず、`VelocityTopicHandle::callback` は `mux_->hasPriority(*this)` が真のとき
だけ `publishTwist()` する。ロックの priority（estop 255 / fault 254）は
velocity 側の priority（behavior 20 / nav 10 / manual_joy 30）より全部高く、
ロック中はどの入力も `isMasked()` になる。**つまり `/cmd_vel_muxed` は「0 に
なる」のではなく「無音になる」。**
したがって「`/cmd_vel_muxed` に 0 が届く」を待つ判定は構造的に通らない
（本試験が一度も緑になったことがなかったのはこれが原因）。
`DetailedDesign-safety.md` §1.1 の注記と §10 #6 の表は 2026-09-28 にこの内容へ
更新済みで、判定もそれに合わせている（100ms は緩めていない）。
実際に 0 を送って機体を止めるのはロックを直接見る後段（`obstacle_limiter` と
`esp32_bridge`）だが、本試験が Gazebo で測るのはその**前段＝`twist_mux` の
ロック**そのものである。

【フェルト検知時刻の起点】
`assert_fault_within` で LIDAR_LOST の active を確認したあと、
`conftest.first_match_time()` で `fault_watcher.records` から実際に
「active かつ fault_type == LIDAR_LOST」な最初のメッセージの受信時刻
（`time.monotonic()`）を取り出し、それを `assert_no_nonzero_after` の `since`
に `FAULT_TO_STOP_LAYER3_MS / 1000` を足した値を渡す。
`/safety/fault_lock` は `safety_monitor.cpp` の `publishFault()` が
`/safety/fault` の直後に同じコールバック内で publish するので、ロックが
twist_mux に届くのは `t_fault` から数 ms 後である。加えて twist_mux は
入力の受信でしか出力を再評価しないため、最後の非ゼロが publish されるのは
twist_mux が `fault_lock` を受け取ってから次の `/cmd_vel_nav`
（`DriveController` の 50Hz なら最悪 20ms）まで。合計でも 100ms 予算に
十分収まる。
`muxed_watcher` は `/scan` を止める**前**（=フェルトより十分前）から購読を
開始しておく（`TopicWatcher` の docstring どおり。取りこぼしによる
偽陽性を避けるため）。

【走行させている理由】
`case_01`/`case_11` と同じ理由——「止まっていたものが見かけ上ゼロだった」
ではなく、「実際に動いていたものがフェルトを境に止まった」ことを示すため、
`enter_manual_mode()` で `MANUAL` に入り `DriveController` で走らせ続けた
状態でフェルトを注入する。twist_mux のロックは「0 を出さず黙る」だけなので、
この「実際に走っていた」ことを書くだけでは縛にならない。試験本体で
「フェルト直前の `_PRE_FAULT_WINDOW_SEC` 秒に `/cmd_vel_muxed` の非ゼロが
1 件以上ある」ことを assert している（無いと「最初から止まっていた」状態を
合格してしまう）。

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

# 「実際に走っていた」ことを確認する遡及窓（秒）。フェルト検知の直前は
# twist_mux がまだロックしていないので、この区間に非ゼロが1件以上あること
# が「走っていた」証拠になる。`lidar_timeout_ms`（=308ms）より長い 0.5 秒を
# 取ることで、LIDAR_LOST が正式に立つ前（/scan を止めた直後）も含めて見る。
_PRE_FAULT_WINDOW_SEC = 0.5

# フェルトから100ms（層3の予算）を過ぎたあと、どれだけ長く無音を確かめるか。
# twist_mux のロックは `/safety/fault_lock` の publisher（safety_monitor）が
# 生きている間だけ続く（timeout 0.5s は「0.5s 以内に更新がなければロック
# 解除」）。safety_monitor は 10Hz で publish し続けるので 0.5s より長い
# 観測が安全側で成立する。100ms だけだと「たまたまその 100ms のうちに
# 沈黙していた」可能性が残るので、余裕を持たせて 800ms 見る。
_POST_FAULT_SILENCE_MS = 800

# 「非ゼロ」の判定しきい値（conftest の assert_* 系と同じ）。
_NONZERO_ATOL = 1e-6

_SIM_STACK_PARAM = {'launch_args': {'obstacle': 'false'}}


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_06_fault_to_stop(
        sim_stack, ros_node, fault_params, limiter_param,
        safety_monitor_sim_startup_grace_sec,
        assert_fault_within, assert_no_nonzero_after):
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

        # 前提条件: フェルト直前まで実際に走っていたこと（モジュールdocstring
        # の【走行させている理由】）。twist_mux はロックすると 0 ではなく
        # 黙るだけなので、これが無いと「最初から止まっていた」状態を合格
        # にしてしまう。購読不成立（QoS不一致・トピック名誤り）でも
        # 非ゼロは1件も取れないので、この assert がその偽の合格も弾く。
        in_window = [m for t, m in muxed_watcher.records
                     if t_fault - _PRE_FAULT_WINDOW_SEC <= t < t_fault]
        pre_fault_nonzero = [m for m in in_window if abs(m.linear.x) > _NONZERO_ATOL]
        assert pre_fault_nonzero, (
            f"フェルト（t_fault={t_fault:.3f}）の {_PRE_FAULT_WINDOW_SEC} 秒前に "
            f"/cmd_vel_muxed の linear.x が非ゼロだった記録が1件も無い"
            f"（同窓の受信 {len(in_window)} 件"
            f"、直近の値: {in_window[-1].linear.x if in_window else '受信なし'}"
            f"）。走っていない・ twist_mux が既にロックしていた・購読不成立の"
            f"いずれか。合格にしてはいけない。")

        # 合格条件（2026-09-28 ユーザー決定。DetailedDesign-safety.md §10 #6）:
        # フェルトから層3の予算 100ms が過ぎた以降、twist_mux の出力に
        # 非ゼロが1件も出ないこと。twist_mux はロックすると 0 を出すのではなく
        # 下位入力を捨てて黙る（モジュールdocstring 参照）ため、「0 が出る」を
        # 待つ `assert_zero_within` では判定できない。
        assert_no_nonzero_after(
            muxed_watcher, 'linear.x',
            since=t_fault + FAULT_TO_STOP_LAYER3_MS / 1000.0,
            ms=_POST_FAULT_SILENCE_MS)
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
