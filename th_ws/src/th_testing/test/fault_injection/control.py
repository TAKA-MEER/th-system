"""
control.py — 対照ケース（ctest 登録名: fault_injection_control）
=======================================================================
故障を注入しなければ止まらないことを確認する 1 本。

【なぜ必要か】
故障注入 1〜13 の合格条件は共通して「（何らかの契機で）止まること」である。
もし観測点の選び方や `assert` の実装を間違えていたら
（例: 購読するトピック名が違う・フィールド名が違う・許容誤差が緩すぎる・
そもそも常に真になる条件を書いてしまった等）、そのテストは「故障を注入して
いなくても常に合格してしまう」壊れた検証になり得る。これは
`DetailedDesign-wp2.md` `WP-TEST-01` §6.3 の FMEA①「テストが誤って通る
（観測点が間違っている）」そのものであり、対策として「各テストに『壊さない
場合は失敗すること』の対照ケースを付ける」ことが指定されている。

この対照ケースが無いと、**常に止まっている実装（例えば `/cmd_vel` を
恒常的に 0 しか出さない壊れた実装）でも 13 本全部が「停止する」という
合格条件を満たしてしまい、見かけ上パスする。** 対照ケースは、その裏返し ──
**意図的に故障を注入しないシナリオ**で「ロボットは走り続ける（`/cmd_vel` が
意図せず 0 に張り付かない）」ことを確認することで、1〜13 のアサーション自体が
「常に合格する」壊れたロジックになっていないかを担保する。

【(C)(D) と同じ判定関数を使う】
このファイルは `case_01_obstacle_limiter_forward.py`（故障注入1）が使うのと
**同じ** `assert_stops_within` フィクスチャを、**故障を注入していない入力**
（障害物なし・弱い駆動指令を出し続ける）に対して呼び出す。ただし期待する
結果は逆転する: 「`ms` 以内に 0 になる」ことを `pytest.raises(AssertionError)`
で包み、**その例外が実際に投げられること**を確認する。つまり
`assert_stops_within` が「0 を見つけられずに `AssertionError` を投げる」のが
正常系であり、もし `assert_stops_within` 自身が壊れていて（購読トピックが
違う・常に真になる等）ここで例外を投げずに通ってしまったら、この対照ケース
自体が失敗する——FMEA①をそのまま検出できる設計。

【何を publish して走らせるか】
`DetailedDesign-wp2.md` `WP-TEST-01` §3 は「新しいトピック・サービス・
パラメータを作らない」と定めるが、**駆動のさせ方**（既存のどの入力に何を
publish するか）までは規定していない。ここでは `conftest.py` の
`DriveController` を使い、既存の入力トピック `/cmd_vel_nav`（Nav2 が使う、
twist_mux priority 10 の入力）に一定の前進速度を発行し続ける。

【`/system/state` は偽装しない（コーディネーターの指示・2026-08-27 に方針転換）】
以前のバージョンは `DriveController` が `/system/state` も直接発行していたが、
これは「故障注入試験の目的は実スタックの挙動を確かめること。`/system/state`
をテストが作ってしまうと `state_manager` を経路から外した別物を試験する
ことになる」という指摘を受けて撤回した。代わりに `conftest.py` の
`enter_manual_mode()` が、実際に起動している `state_manager` /
`connectivity_checker` の新FSMを `/ui/active_screen`（画面 S-11・手動走行）
経由で `IDLE` → `MANUAL` へ進め、`state_manager` 自身に
`/system/state.speed_limit` を計算・発行させる。この経路上で
`scan_expected_points` の実機値と Gazebo の LiDAR センサ設定の不一致が
新たに判明したが、`gazebo_plugins.xacro` の `<samples>` を実機値へ合わせて
解消済み（`DetailedDesign-open.md` N-22。詳細は `enter_manual_mode()`
docstring）。
"""
from __future__ import annotations

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist

from fault_injection.conftest import DriveController, TopicWatcher, enter_manual_mode

# obstacle:=false: このテストは障害物の有無を検証しないので、
# wanderer のランダム歩行(obstacle_mover.py)を止めて再現性を上げる。
# (wanderer 自体は世界に残るが、既定位置(1.5,-1.5)は本テストの走行経路
#  (spawn (-4,-3) から +x 方向)から外れているので干渉しない)
_SIM_STACK_PARAM = {'launch_args': {'obstacle': 'false'}}

# ロボットが本当に動いているかを見るための観測窓。安全パラメータの
# しきい値ではなく、このテスト自身のscaffolding値(T-1の対象外)。
# muxed_stale_ms(twist_mux→obstacle_limiterの鮮度基準)の15倍を取り、
# 「twist_mux/obstacle_limiterの通常のタイムアウト挙動では絶対に
# 説明がつかない長さ」であることをregistry値から示す。
_WINDOW_MULTIPLIER = 15


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_control_runs_without_fault(
        sim_stack, ros_node, limiter_param, assert_stops_within):
    bootstrap, state_watcher = enter_manual_mode(ros_node)
    watcher = TopicWatcher(ros_node, '/cmd_vel', Twist)
    # v_max の半分程度で走らせる(v_maxちょうどだとクランプ境界に近すぎて
    # 判定が微妙になるのを避けるための余裕)。
    drive = DriveController(ros_node, linear_x=limiter_param('v_max') * 0.5)
    try:
        # ウォームアップ: /cmd_vel_nav の発行が twist_mux/obstacle_limiter
        # 側に反映されるまで待ってから測定窓をクリアする
        # (起動直後の過渡的なゼロを測定に含めない)。
        warmup_deadline = time.monotonic() + 2.0
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(ros_node, timeout_sec=0.05)
        watcher.records.clear()

        window_ms = limiter_param('muxed_stale_ms') * _WINDOW_MULTIPLIER

        # (C)(D) と同じ判定関数を、故障を注入していない入力に対して呼ぶ。
        # 正常系なら「0 が見つからず AssertionError を投げる」——それを
        # pytest.raises で捕まえること自体がこのテストの合格条件。
        with pytest.raises(AssertionError):
            assert_stops_within(watcher, 'linear.x', window_ms)
    finally:
        drive.stop()
        watcher.destroy()
        bootstrap.stop()
        state_watcher.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
