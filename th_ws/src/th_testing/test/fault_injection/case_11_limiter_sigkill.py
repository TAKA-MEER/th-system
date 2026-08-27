"""
case_11_limiter_sigkill.py — 故障注入 11「リミッタの死（SIGKILL）」
（ctest 登録名: fault_injection_11）
========================================================================
`DetailedDesign-safety.md` §10 #11: `obstacle_limiter` を SIGKILL し、
重大フォルト → `ESTOP` → 駆動ゼロとなることを確認する（`DEBT-4`）。
自動化: Gazebo ＋ 実機。

────────────────────────────────────────────────────────────
【実装を読んで判明した2つの事実（作業指示・設計書のいずれにも無かった）】
────────────────────────────────────────────────────────────

(1) `gazebo.launch.py` の `SAFETY_ENABLED_TARGETS_SIM` に `"limiter"` が
    無かった。`safety_monitor.cpp` の `LIMITER_DEAD` 検出は
    `targetEnabled("limiter")` でゲートされており（F-5・O-7。有効化するまで
    監視しない設計）、これが欠けていると **`obstacle_limiter` を殺しても
    safety_monitor は一切気づかない**。obstacle_limiter は実際には
    `/safety/limiter_status` を20Hzで発行している（heartbeat 実装済み）ので、
    検出ロジック自体は揃っている。単に有効化リストへの追加漏れと判断し、
    このパケットで `gazebo.launch.py` に `"limiter"` を追加した（別コミット。
    obstacle_limiter が実際に publish している heartbeat を、既に実装済みの
    監視ロジックに繋ぐだけの1行修正であり、新しい安全機能の実装ではない
    ため、このパケットのテスト実装に必要な最小限の修正として扱った）。

(2) `mode_manager.cpp` は `/safety/fault` の `severity` を見ていない。
    `sub_fault_` のコールバックは `msg->fault_type` だけで分岐し、
    アクティブなモードから常に `IDLE` へ強制遷移させる
    （`PERSON_TRACKER_LOST` の narrower gating のみ例外）。`ESTOP` へ
    遷移するのは `/safety/estop`（ハード/UI E-Stop）だけ。これは
    `DetailedDesign-safety.md` §5.1 の「`CRITICAL` → `ESTOP`」という表と
    食い違う——**現状の実装は `LIMITER_DEAD` のような CRITICAL フォルトでも
    `IDLE` にしか遷移しない**。これは mode_manager.cpp 側の実装ギャップで、
    このパケットの範囲（test ファイル + 1行の launch 修正）を超える
    修正が必要なため、ここでは直さない。

    §5.2.1 は「`th_state` の `ESTOP` 遷移は表示と復帰のためだけであり、
    駆動を止める役目は持たない（駆動を止めるのは層3=`/safety/fault_lock`）」
    と明記しているため、**この2番目のギャップは「駆動ゼロ」という
    安全上の本質的な結果には影響しない**（`/safety/fault_lock` は
    severity==CRITICAL を正しく見ており、こちらは実装済み。
    `safety_monitor_core.cpp::compute_fault_lock()` 参照）。だが
    §10 #11 が明示的に要求する「`ESTOP`」という表示状態を実際に検証すると
    **現状は失敗する**。これは実装管理者に必ず報告すべき既知のギャップと
    判断し、テストとしては「本来満たすべき仕様」をそのまま書いた
    （ごまかして緩めていない）。mode_manager.cpp が修正されれば、この
    テストはコード変更なしでそのまま合格するようにしてある。

(3) このテストは `conftest.py` の `enter_manual_mode()` に依存しており、
    そちらにさらに未解決の疑わしいブロッカーがある（`scan_expected_points`
    の実機値=1080 と Gazebo LiDAR センサの `<samples>720` の不一致。
    `connectivity_checker` が `evt.link_ok` を出せず `state_manager` が
    `INIT` から進めない可能性）。①〜⑤の手前でこれに引っかかった場合、
    エラーメッセージにその旨が出る（`enter_manual_mode()` docstring参照）。

────────────────────────────────────────────────────────────
【「駆動ゼロ」をどう判定したか】
────────────────────────────────────────────────────────────
`obstacle_limiter` は `/cmd_vel` の唯一の publisher であり、死ぬと
**誰も `/cmd_vel` を publish しなくなる**（新しいメッセージが来なくなる
だけで、明示的なゼロ Twist が飛ぶわけではない）。さらに Gazebo の
`libgazebo_ros_diff_drive` プラグインには実機の ESP32（600ms
ウォッチドッグ）のような「指令が来なくなったら自動でゼロにする」仕組みが
無いと考えられる（一般的な gazebo_ros 系プラグインの実装を踏まえた判断。
Docker で実際に確認できていない）。つまり **Gazebo 上では
「/cmd_vel が沈黙する」ことと「モータが実際にゼロになる」ことは
別の事象**であり、後者は sim 特有の忠実度の限界で確認できない可能性がある
（実機では ESP32 のウォッチドッグが別途担保する。§10 #11 の自動化欄が
「Gazebo ＋ 実機」なのはこのため——sim だけでは検証が完結しない）。

この限界を踏まえ、このテストは **「obstacle_limiter を殺す前に、実際に
障害物へ向けて走らせて安全に停止させておく」**（故障注入1と同じ手順）
という前提を作った上で SIGKILL する設計にした。これにより:
  - kill 後に `/cmd_vel` が沈黙しても、それは「元々動いていなかった」
    ことの見かけ上の停止ではなく、実際に安全側で止まっていた状態から
    さらに操作不能になったことを意味する。
  - kill 後に **万が一何らかの理由で新しい非ゼロ指令が観測されたら**
    （例えば别のノードが誤って `/cmd_vel` に書き込むような回帰）、
    それは明確な異常として検出できる（`assert_no_nonzero_after`）。
`/esp32/wheel_cmd_speed`（実機観測点）は sim では publisher が存在しない
（`esp32_bridge` は `condition=UnlessCondition(sim)` で sim では起動しない。
故障注入12がまさにこの理由で「Gazeboではない」統合テストになっている）
ため、sim 環境のこのテストでは使えない。

────────────────────────────────────────────────────────────
【DDS discovery 破損への対策】
────────────────────────────────────────────────────────────
CLAUDE.md「`kill -9` を繰り返すとコンテナ内の DDS discovery が壊れる」を
踏まえ、`sim_stack` に `cleanup_dds_shm=True` を渡す。これにより後始末で
`/dev/shm/fastrtps_*` のうち **どのプロセスも開いていないもの**だけを
削除する（`conftest.py::_cleanup_idle_dds_shm()`）。「掃除だけでは直らない
ことがある」（CLAUDE.md）ため確実な対策ではないが、ベストエフォートとして
入れる判断をした。計画にあった「11・13は毎回コンテナを作り直す」は
このテストファイル単体では実施できない（コンテナのライフサイクルは
Docker 側の話であり、pytest の中からは制御できない）。作業指示にある
「ctest のエントリが1つずつ別プロセスなので緩和される」を踏まえ、
コンテナ再作成は Docker 実行側（ユーザー）の運用判断に委ね、
プロセス内でできる緩和策（shm 掃除）だけをここで入れた。
"""
from __future__ import annotations

import os
import signal
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from th_system_msgs.msg import FaultStatus, LimiterStatus, RobotMode

from fault_injection.conftest import (
    DriveController, TopicWatcher, enter_manual_mode, find_pids_by_exe_basename,
    place_entity_state,
)

_ROBOT_SPAWN_X = -4.0
_ROBOT_SPAWN_Y = -3.0
_OBSTACLE_AHEAD_M = 2.5
_STOP_WINDOW_MS = 30_000

# `/safety/limiter_status` は best_effort publisher（obstacle_limiter.cpp）。
# reliability を合わせないと購読自体が成立しない。
_LIMITER_STATUS_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

_SIM_STACK_PARAM = {
    'launch_args': {'obstacle': 'false'},
    'cleanup_dds_shm': True,  # SIGKILL テストにつき /dev/shm を後始末する。
}


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_11_limiter_sigkill(
        sim_stack, ros_node, limiter_param,
        assert_stops_within, assert_fault_within, assert_true_within,
        assert_equals_within, assert_silent_within, assert_no_nonzero_after):
    place_entity_state(
        ros_node, 'wanderer',
        x=_ROBOT_SPAWN_X + _OBSTACLE_AHEAD_M, y=_ROBOT_SPAWN_Y)

    bootstrap, state_watcher = enter_manual_mode(ros_node)
    cmd_watcher = TopicWatcher(ros_node, '/cmd_vel', Twist)
    limiter_hb_watcher = TopicWatcher(
        ros_node, '/safety/limiter_status', LimiterStatus, qos=_LIMITER_STATUS_QOS)
    fault_watcher = TopicWatcher(ros_node, '/safety/fault', FaultStatus)
    fault_lock_watcher = TopicWatcher(ros_node, '/safety/fault_lock', Bool)
    mode_watcher = TopicWatcher(ros_node, '/robot/mode', RobotMode)

    drive = DriveController(ros_node, linear_x=limiter_param('v_max'))
    try:
        # ── 1) 故障注入1と同じ手順で、実際に障害物の手前で安全停止させる ──
        warmup_deadline = time.monotonic() + 1.0
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(ros_node, timeout_sec=0.05)
        cmd_watcher.records.clear()
        assert_stops_within(cmd_watcher, 'linear.x', _STOP_WINDOW_MS)

        # ── 2) obstacle_limiter だけを SIGKILL する(§10 #11の仕様どおり) ──
        pids = find_pids_by_exe_basename('obstacle_limiter')
        assert len(pids) == 1, (
            f"obstacle_limiter の PID が一意に特定できない: {pids} "
            "(0件=既に死んでいる、複数件=多重起動の可能性)")
        kill_time = time.monotonic()
        os.kill(pids[0], signal.SIGKILL)

        dead_ms = limiter_param('limiter_dead_ms')

        # ── 3) heartbeat (/safety/limiter_status, 20Hz) が途絶えたことを
        #        直接確認する(プロセスが本当に死んだことの一次証拠) ──
        assert_silent_within(limiter_hb_watcher, since=kill_time, ms=dead_ms * 3)

        # ── 4) safety_monitor が LIMITER_DEAD(重大)を立てる ──
        assert_fault_within(fault_watcher, 'LIMITER_DEAD', ms=dead_ms * 3)

        # ── 5) fault_lock が立つ(severity==CRITICALのOR項。§5.2.1・層3) ──
        assert_true_within(fault_lock_watcher, 'data', ms=dead_ms * 3)

        # ── 6) 表示・復帰用の ESTOP モード遷移 ──
        # 既知のギャップ(本ファイル冒頭docstring(2))により、現状の
        # mode_manager.cpp では**ここで失敗する**想定。ギャップが解消されたら
        # このアサーションはコード変更なしで合格するようにしてある。
        assert_equals_within(mode_watcher, 'mode', RobotMode.ESTOP, ms=dead_ms * 10)

        # ── 7) 駆動ゼロ: 既に停止済みの状態から、kill後に非ゼロ指令が
        #        再び現れないこと(本ファイル冒頭docstringの限界の説明どおり、
        #        「沈黙の継続」を「駆動ゼロの継続」の代替証拠として扱う) ──
        assert_no_nonzero_after(cmd_watcher, 'linear.x', since=kill_time, ms=dead_ms * 10)
    finally:
        drive.stop()
        cmd_watcher.destroy()
        limiter_hb_watcher.destroy()
        fault_watcher.destroy()
        fault_lock_watcher.destroy()
        mode_watcher.destroy()
        bootstrap.stop()
        state_watcher.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
