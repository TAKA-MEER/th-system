#!/usr/bin/env python3
"""single_controller_guard.py — 別の PC の制御系が同時に動いていたら起動を中止する。

背景（Spec-safety.md §6.4「制御系は同時に 1 つ」・2026-11-19 ユーザー決定）:
ロボットのネットワーク（ROS_DOMAIN_ID=10）に 2 台の PC が bringup を起動すると、
2 台目の `/cmd_vel`・`/safety/estop`・`/safety/fault_lock` が 1 台目の
`esp32_bridge` に混ざる（`esp32_bridge._cb_estop` は最後に届いた値で決まるので、
2 台目の「解除」が 1 台目の非常停止を上書きしうる）。
`prelaunch_guard.py` は同じ機械の /proc しか見ないので、別の PC は検知できない。

このモジュールは新しい bringup の開始時に、同じドメインにこのスタックの制御系の
ノード（CONTROL_NODE_NAMES）が既にいないかを数秒見る。いたら起動を中止し、
「別の PC の制御系が動いています」を出して launch を終わらせる
（OpaqueFunction から例外を上げる）。

位置づけは prelaunch_guard と同じ: ノードを 1 つも起動する前に、最初の
OpaqueFunction として同期実行する（bringup.launch.py）。

決まりごと:
* グラフの問い合わせが失敗したときは止めない（不明を他者ありと決めない）が警告を出す。
* `dev_mode` の項目は作らない（開発モードでも外さない）。
* `launch` / `launch_ros` / `rclpy` はモジュール読み込み時には import しない
  （prelaunch_guard.py と同じ理由。host の pytest から純関数だけテストできる）。
"""
from __future__ import annotations

import time
from typing import Callable, Sequence

# このスタックの制御系のノード名。同じドメインにこれらが既にいれば別の PC の
# 制御系が動いている（本チェックは自スタックのノードを 1 つも起動する前に走るため）。
# 制御系の「誰が居るべきか」の定義はここ 1 か所にまとめる。
# safety_monitor 側の動作中検知（CONTROLLER_CONFLICT）も同じ 4 名を見る
# （C++ 側の写し: th_safety/safety_monitor_core.hpp の controller_node_names()）。
CONTROL_NODE_NAMES: tuple[str, ...] = (
    "obstacle_limiter",
    "safety_monitor",
    "state_manager",
    "esp32_bridge",
)

# 見る時間（秒）。DDS の discovery が他 PC のノードを届けるまでの余裕。
# 短すぎると他 PC の制御系を見逃す（起動直前まで生きていた残骸も含む）。
DEFAULT_WATCH_SEC = 3.0
POLL_INTERVAL_SEC = 0.5


def find_other_controllers(node_names: Sequence[str]) -> list[str]:
    """グラフにいるノード名の列から、制御系のノードを返す（純関数）。

    本チェックは自スタックのノードを 1 つも起動する前に走るので、制御系の
    ノードが 1 つでも見えればそれは別の PC の制御系である。
    """
    return sorted({n for n in node_names if n in CONTROL_NODE_NAMES})


def check_single_controller(
    *,
    watch_sec: float = DEFAULT_WATCH_SEC,
    poll_interval_sec: float = POLL_INTERVAL_SEC,
    node_names_fn: Callable[[], list[str]],
    sleep_fn: Callable[[float], None] = time.sleep,
    now_fn: Callable[[], float] = time.monotonic,
) -> tuple[list[str], str | None]:
    """watch_sec の間グラフを数秒見て、別の制御系がいればその名前を返す。

    戻り値: (found, warning)。found が非空なら他者あり。warning はグラフの
    問い合わせに失敗したときの警告文（found は空＝止めない）。
    """
    deadline = now_fn() + watch_sec
    found: list[str] = []
    while True:
        try:
            names = node_names_fn()
        except Exception as e:  # noqa: BLE001 — 不明を他者ありと決めない
            return [], (
                f"[single_controller_guard] 警告: グラフの問い合わせに失敗しました"
                f"（{e}）。別の PC の制御系がいるか確認できませんでしたが、"
                f"起動を続けます。"
            )
        found = find_other_controllers(names)
        if found:
            return found, None
        if now_fn() >= deadline:
            return [], None
        sleep_fn(poll_interval_sec)


def refusal_message(found: Sequence[str]) -> str:
    return (
        "別の PC の制御系が動いています"
        f"（ノード名: {', '.join(found)}）。"
        "そちらを止めてから起動してください"
    )


def run_startup_check(
    *,
    watch_sec: float = DEFAULT_WATCH_SEC,
) -> list[str]:
    """実 rclpy でグラフを問い合わせる本体。bringup の OpaqueFunction から呼ぶ。

    他者がいれば RuntimeError を上げる（launch を中止する）。
    グラフ照会失敗時は警告の行を返す（止めない）。
    """
    import rclpy  # noqa: PLC0415 — モジュール読み込み時に import しない

    initialized_here = False
    if not rclpy.ok():
        rclpy.init()
        initialized_here = True
    try:
        node = rclpy.create_node("single_controller_guard")
        try:
            def _node_names() -> list[str]:
                # discovery の受信を進める（spin しないと他 PC のノードが
                # グラフのキャッシュに載らない）。
                rclpy.spin_once(node, timeout_sec=0.0)
                return node.get_node_names()

            found, warning = check_single_controller(
                watch_sec=watch_sec, node_names_fn=_node_names)
        finally:
            node.destroy_node()
    finally:
        if initialized_here:
            rclpy.shutdown()

    if found:
        raise RuntimeError(refusal_message(found))
    return [warning] if warning else []


def make_single_controller_opaque_function(**check_kwargs) -> Callable:
    """`OpaqueFunction(function=...)` に渡すコールバック。bringup.launch.py の
    prelaunch_guard の直後（ノードを 1 つも起動する前）に登録すること。
    """
    def _opaque(context, *args, **kwargs):
        from launch.actions import LogInfo

        lines = run_startup_check(**check_kwargs)
        return [LogInfo(msg=line) for line in lines]

    return _opaque
