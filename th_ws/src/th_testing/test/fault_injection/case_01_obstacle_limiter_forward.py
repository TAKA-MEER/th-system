"""
case_01_obstacle_limiter_forward.py — 故障注入 1「障害物リミッタ（前進）」
（ctest 登録名: fault_injection_01）
========================================================================
`DetailedDesign-safety.md` §10 #1: 障害物へ向けて前進させ、停止することを確認する。
自動化: Gazebo。

【障害物をどう置いたか】
`th_perception/scripts/obstacle_mover.py` が動かす "wanderer"（人間に模した
円柱。`panel_room_no_actor.world` に静的モデルとして定義済み）を、
`conftest.py` の `place_entity_state()`（obstacle_mover.py と同じ
`/gazebo/set_entity_state` サービスを直接呼ぶ）でロボットの正面に固定する。

obstacle_mover.py 自体は使わない（`sim_stack` には `obstacle:=false` を渡し、
ノード自体を起動しない）。obstacle_mover.py はランダム歩行が目的で、
毎回の実行で障害物の位置が変わってしまい、このテストが求める
「決まった距離で決まって停止する」という再現性のある検証に向かないため。
`panel_room.launch` のシナリオプリセット（`config/scenarios/`）も検討したが、
いずれも人物・障害物の経路が試験員追従の検証用に作り込まれており、単純な
直進試験には迂遠（`obstacle:=false` + 素の legacy デフォルト + `wanderer` の
直接配置のほうが依存が少なく検証しやすいと判断した）。

【合格条件の時間】
`assert_stops_within` の `ms` は `_STOP_WINDOW_MS`（モジュール定数）。
これは「観測にどれだけ待つか」というテスト側の scaffolding であり、
T-1 が禁じる「安全パラメータのしきい値をテストに直書きする」こととは別。
実際の停止判定そのもの（`obstacle_floor_distance_m` 手前で止まるか）は
obstacle_limiter 側の registry 値で決まっており、このテストはその値を
一切書き換えない。

【前提: 速度上限を実際に流すために state_manager の新FSMを通す】
`conftest.py` の `enter_manual_mode()` docstring 参照。`gazebo.launch.py` に
`state_manager`（＋ `connectivity_checker`）を追加した（このパケットで、
コーディネーターの指摘を受けて追加）。何もしないと `/system/state` の
publisher が無く obstacle_limiter は障害物の有無に関わらず常に速度上限 0 を
出してしまう（＝停止試験として無意味になる）ため、`enter_manual_mode()` で
実際に `IDLE` → `MANUAL` へ進めてから駆動する。当初 `scan_expected_points`
の実機値と Gazebo の LiDAR センサ設定の不一致で `state_manager` が `IDLE`
に到達できない問題があったが、`gazebo_plugins.xacro` の `<samples>` を
実機値へ合わせて解消済み（`DetailedDesign-open.md` N-22。詳細は
`enter_manual_mode()` docstring）。
"""
from __future__ import annotations

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist

from fault_injection.conftest import (
    DriveController, TopicWatcher, enter_manual_mode, place_entity_state,
)

# gazebo.launch.py の legacy デフォルト spawn（scenario 未指定時）。
# _scenario_setup() の _LEGACY 定数と同じ値（このテストは意図的に scenario を
# 指定せずデフォルトのまま使う——余計な人物・障害物経路を持ち込まないため）。
_ROBOT_SPAWN_X = -4.0
_ROBOT_SPAWN_Y = -3.0

# spawn から前方(+x, yaw=0)へこの距離だけ進んだ位置に障害物を置く。
# panel_room_no_actor.world の室内は x∈[-5,5]・y∈[-4,4]（壁の内側）なので、
# 2.5m 前方(x=-1.5)は壁・仕切り・配電盤のいずれとも干渉しない
# （仕切り partition_west は y≈1.0 付近、配電盤群は y=3.55 付近。
#   本テストの走行経路は y=-3.0 のまま）。
_OBSTACLE_AHEAD_M = 2.5

# 観測窓（scaffolding。T-1の対象外・上のモジュールdocstring参照）。
# obstacle_floor_distance_m 手前でのヒステリシス込みクランプは漸近的に
# 速度を落とすため、単純な距離/速度の見積りより長めに余裕を取った。
_STOP_WINDOW_MS = 30_000

_SIM_STACK_PARAM = {'launch_args': {'obstacle': 'false'}}


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_01_obstacle_forward(
        sim_stack, ros_node, limiter_param, assert_stops_within):
    place_entity_state(
        ros_node, 'wanderer',
        x=_ROBOT_SPAWN_X + _OBSTACLE_AHEAD_M, y=_ROBOT_SPAWN_Y)

    bootstrap, state_watcher = enter_manual_mode(ros_node)
    watcher = TopicWatcher(ros_node, '/cmd_vel', Twist)
    # v_max で前進を指令する。obstacle_limiter 側の最終出力は
    # min(この指令, 上限, 障害物クランプ) で決まるため、指令自体の大きさは
    # 「上限に張り付かない程度に十分大きい」以外の意味を持たない。
    drive = DriveController(ros_node, linear_x=limiter_param('v_max'))
    try:
        warmup_deadline = time.monotonic() + 1.0
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(ros_node, timeout_sec=0.05)
        watcher.records.clear()

        assert_stops_within(watcher, 'linear.x', _STOP_WINDOW_MS)
    finally:
        drive.stop()
        watcher.destroy()
        bootstrap.stop()
        state_watcher.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
