"""
case_03_estop_ui.py — 故障注入 3「非常停止（UI）」（ctest 登録名: fault_injection_03）
====================================================================================
`DetailedDesign-safety.md` §10 #3: 走行中に UI から非常停止を押し、停止することを
確認する。自動化: Gazebo ＋ 実機（Gazebo 側がこのファイルの対象。実機側の確認は
別途 `docs/実機作業キュー.md` の運用に乗る）。

【`/safety/estop_ui` を試験が発行する判断の根拠】
`safety_monitor.cpp:19` のコメントのとおり、UI 非常停止（`/safety/estop_ui`）と
物理 E-Stop（`/safety/estop_hw`）は `enabled_targets` の対象外（常時監視）で、
**実機ではこれを出すのは WebUI（rosbridge 経由）**であり、sim にはその WebUI 役の
ノードが存在しない。これは `conftest.py` の `_UiInputBootstrap` が
`/safety/estop_hw` と `/ui/active_screen` を発行しているのと**同じ性質**——
「実機では別ノードが供給するはずの生の入力信号を、その実機ノードが sim に
存在しないためテストが代わりに供給する」ケースである。`/safety/estop_ui` は
safety_monitor から見て「押されたか否か」という raw な入力事実そのものであり、
それを受けて `/safety/estop` を立てる・`fault_lock` に波及させる・twist_mux が
ロックする、という**計算・配線は safety_monitor / twist_mux の実装がそのまま
行う**（テストは計算結果である `/safety/estop` や `/system/state` を偽装しない）。
よって前パケットで確立した区別（「生の入力は肩代わりしてよいが、計算された
出力は偽装しない」）に沿っている。

【`DetailedDesign-safety.md` §6.2「状態機械を安全経路から外す」との対応】
UI 非常停止は `th_state`（新FSM）を経由せず、`safety_monitor` → `/safety/estop`
→ `twist_mux` のロックで止まる設計（§6.2）。このテストは `enter_manual_mode()`
で `state_manager` を実際に `MANUAL` へ進めた上で `/safety/estop_ui` を発行し、
**`/cmd_vel` が止まることを、`state_manager` の遷移を待たずに（=state_manager
が MANUAL のまま/ESTOPへ変化しようとしまいと関係なく）確認する**——これにより
「状態機械を経由しない安全経路」が実際にその設計どおり機能していることを
確かめる。

【観測点】
`/cmd_vel`（WP-TEST-01 §3 の観測点表）。estop はスキャン鮮度に依存しないので、
故障注入6で問題になった「obstacle_limiter tier3 の scan_stale による独立した
停止」との混同は起きない（obstacle_limiter は `/safety/estop` を tier1 で
直接見ており、こちらも同じ入力に対する反応。`case_06_fault_to_stop.py` の
docstring 参照）。

【合格条件の時間】
`assert_stops_within` の `ms` は `_STOP_WINDOW_MS`（モジュール定数。T-1 の対象外
=このテスト自身の観測窓というscaffolding。`case_01` と同じ扱い）。
`DetailedDesign-safety.md` §10 #3 の合格条件は「停止する」のみで、5・6のような
明示的な時間しきい値（`lidar_timeout_ms` 等）を持たない。
"""
from __future__ import annotations

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from fault_injection.conftest import DriveController, TopicWatcher, enter_manual_mode

# 観測窓（scaffolding。T-1の対象外・上のモジュールdocstring参照）。
# case_01/02 と同じ余裕を取る。
_STOP_WINDOW_MS = 30_000

_SIM_STACK_PARAM = {'launch_args': {'obstacle': 'false'}}


class _EstopUiPublisher:
    """`/safety/estop_ui` を一定周期で True 発行し続けるヘルパー。
    `UiEstopLatch::on_true()` は1件受信すれば即座にラッチするが（safety_monitor_core.hpp）、
    reliable QoS のマッチング遅延を吸収するため数百ms連続で送る。"""

    def __init__(self, node, period_sec: float = 0.05):
        self._node = node
        self._pub = node.create_publisher(Bool, '/safety/estop_ui', 10)
        self._timer = node.create_timer(period_sec, self._tick)

    def _tick(self) -> None:
        self._pub.publish(Bool(data=True))

    def stop(self) -> None:
        self._node.destroy_timer(self._timer)
        self._node.destroy_publisher(self._pub)


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_03_estop_ui(sim_stack, ros_node, limiter_param, assert_stops_within):
    bootstrap, state_watcher = enter_manual_mode(ros_node)
    watcher = TopicWatcher(ros_node, '/cmd_vel', Twist)
    drive = DriveController(ros_node, linear_x=limiter_param('v_max'))
    estop_ui = None
    try:
        # ウォームアップ: 実際に走り出してから estop を押す
        # (case_01/control と同じ理由。起動直後の過渡的なゼロを測定に含めない)。
        warmup_deadline = time.monotonic() + 1.0
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(ros_node, timeout_sec=0.05)
        watcher.records.clear()

        # UI 非常停止を「押す」(上のモジュールdocstring参照。生の入力信号)。
        estop_ui = _EstopUiPublisher(ros_node)

        assert_stops_within(watcher, 'linear.x', _STOP_WINDOW_MS)
    finally:
        if estop_ui is not None:
            estop_ui.stop()
        drive.stop()
        watcher.destroy()
        bootstrap.stop()
        state_watcher.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
