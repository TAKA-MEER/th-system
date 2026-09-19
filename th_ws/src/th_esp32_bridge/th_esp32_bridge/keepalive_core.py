"""
keepalive_core.py — esp32_bridge のキープアライブ判定 (ROS2 非依存)
========================================================================
DEBT-4: 旧実装は /cmd_vel が途絶しても、キープアライブで最後の非ゼロ指令を
20Hz で送り続けていた (esp32_bridge.py 旧 237-238 行付近)。ESP32 側の
ウォッチドッグも WHEEL_CMD が来ている限り発火しないため、上流 (ROS2 側) が
死んでも誰も止めない単一障害点になっていた
(docs/plan/detailed/DetailedDesign-safety.md §0 DEBT-4)。

ここでは /cmd_vel の途絶 (stale) を判定し、参照値をゼロへ書き換える。
**送信そのものは止めない** (K-1)。「送るのをやめる」のではなく「ゼロを送る」。
送信を止めるとウォッチドッグ経由の停止 (最大 esp32_watchdog_ms = 600ms) に
なり、PC 側の対処 (cmd_vel_stale_ms < esp32_watchdog_ms、A7) より遅れて
しまう (docs/plan/detailed/DetailedDesign-wp2.md WP-SAFE-02 §4.2)。

テスト容易性のため ROS2 インポートを一切含まない。ノード側
(scripts/esp32_bridge.py) はこのモジュールを import して使うだけにする。
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Twist2:
    """geometry_msgs/Twist のうち linear.x / angular.z のみを持つ最小表現。"""
    linear: float = 0.0
    angular: float = 0.0


ZERO_TWIST2 = Twist2(0.0, 0.0)


def keepalive_value(last_cmd_ms: float, now_ms: float, last_cmd: Twist2,
                     stale_ms: float, locked: bool) -> Twist2:
    """ロック中 or stale ならゼロ。それ以外は last_cmd をそのまま返す。

    K-2: ロック (/safety/estop または /safety/fault_lock) は stale 判定より
    先に評価する。ロック中に「たまたま鮮度は十分」な指令が通ってしまうことを
    避けるため (独立ロック層は twist_mux 等の上位が死んでも効く最後の砦)。
    """
    if locked:
        return ZERO_TWIST2
    if (now_ms - last_cmd_ms) > stale_ms:
        return ZERO_TWIST2
    return last_cmd
