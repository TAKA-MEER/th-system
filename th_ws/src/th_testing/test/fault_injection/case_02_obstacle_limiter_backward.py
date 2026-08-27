"""
case_02_obstacle_limiter_backward.py — 故障注入 2「障害物リミッタ（後退）」
（ctest 登録名: fault_injection_02）
=========================================================================
`DetailedDesign-safety.md` §10 #2: 障害物へ向けて後退させ、停止することを確認する
（`CL-X-4`）。自動化: Gazebo。

【重要: このテストは現時点で意図的に fail する（実装のバグではない）】

`obstacle_limiter_core.cpp` の `update()` は後退時
（`in.muxed.value.linear_x < 0`）、`applied_limit` に必ず
`p.v_reverse` を `min` で重ねる（§3.3.1・L4）。

`v_reverse` は `registry.yaml` 上 `status: derived`（`derived_from:
[blind_clearance_m, ...]`）だが、その依存先 `blind_clearance_m` 自体が
`status: placeholder`（コメント:「`v_reverse` の逆算元。**LiDAR死角**。
`O-a2` が決まるまで placeholder」）。`resolve_registry()` は依存が
placeholder のまま解決できない値を呼び出し側に placeholder として
伝播するため、`v_reverse` も実際には未解決（実測済み。
`th_params.export.resolve_registry()` を素で呼ぶと
`('placeholder', 'TBD_MEASURE')` が返る）。

**さらに実行時の挙動として**: `th_params.export.sanitize_node_params()` は
placeholder（= None）な値を生成物から**キー自体を落とす**ため、
`obstacle_limiter.cpp` の `declare_parameter("v_reverse", 0.0)` の
**宣言時デフォルト 0.0 がそのまま使われる**。つまり **現状の Gazebo 実行では、
後退コマンドは障害物の有無に関わらず常に出力ゼロになる**
（`applied_limit = min(..., 0.0) = 0.0` が無条件に効くため）。

この状態で「障害物へ後退→停止する」を検証しても、障害物を無くした
対照シナリオ（`control.py` と同種の入力）でも**同じ結果（常にゼロ）**に
なってしまい、両者を区別できない。これは `control.py` が守ろうとしている
FMEA①（テストが誤って通る）と同種の問題が、`v_reverse` の未測定という
**データ側の理由**でこのテスト個別に発生している状態であり、
「障害物を検知して止めた」ことの証明にならない。

このパケットの他のフィクスチャ（`fault_params`）が確立した規約
（`registry.yaml` が placeholder のままの値は黙って進めず
`pytest.fail` で明示的に落とす）に合わせ、本テストは
`limiter_param('v_reverse')` を呼んで**意図的にここで落とす**。
`v_reverse` が測定されて実際の値が入れば、このテストはコード変更なしで
そのまま意味のある検証になるように書いてある（下の
`test_fault_injection_02_obstacle_backward` の本体を参照）。

`_v_reverse_precondition` を `sim_stack` より前の引数に置いているのは、
どうせ後で失敗するテストのために Gazebo を毎回 40〜45 秒かけて起動する
無駄を避けるため（pytest は同スコープ・無依存のフィクスチャを引数の
出現順に近い順序でセットアップする。保証ではなくベストエフォートの最適化
であり、万一順序が変わっても正しさには影響しない——いずれにせよ最終的に
同じ `pytest.fail` で終わる）。
"""
from __future__ import annotations

import math
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist

from fault_injection.conftest import DriveController, TopicWatcher, place_entity_state

_ROBOT_SPAWN_X = -4.0
_ROBOT_SPAWN_Y = -3.0
_OBSTACLE_BEHIND_M = 2.5
_STOP_WINDOW_MS = 30_000

# ロボットを yaw=π（world -x 向き）でスポーンさせる。
# legacy デフォルト spawn (-4,-3) は西壁 (x≈-5.0) まで約1mしか無く、
# yaw=0 のまま後退（-x）させると壁との干渉のほうが先に効いてしまい、
# 障害物リミッタ由来の停止を検証できない。yaw=π にすると
# 「後退（ロボット座標系の -x）」が world +x 方向になり、
# spawn から東側の壁(x≈+5.0)まで約9mの余裕があるので、その途中
# （spawn+2.5m の位置）に置いた障害物で無理なく検証できる。
_ROBOT_SPAWN_YAW = math.pi

_SIM_STACK_PARAM = {'launch_args': {
    'obstacle': 'false',
    'robot_yaw': str(_ROBOT_SPAWN_YAW),
}}


@pytest.fixture
def _v_reverse_precondition(limiter_param):
    """本ファイル冒頭 docstring 参照。v_reverse が placeholder な間はここで
    pytest.fail する（sim_stack のセットアップより前に評価させたい）。"""
    return limiter_param('v_reverse')


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_02_obstacle_backward(
        _v_reverse_precondition, sim_stack, ros_node, limiter_param, assert_stops_within):
    # yaw=π なので、world 座標で見て spawn の "front"(+x, 進行方向)側に
    # 障害物を置く。ロボット座標系ではこれは "後方"(reversing 時の正面)になる。
    place_entity_state(
        ros_node, 'wanderer',
        x=_ROBOT_SPAWN_X + _OBSTACLE_BEHIND_M, y=_ROBOT_SPAWN_Y)

    watcher = TopicWatcher(ros_node, '/cmd_vel', Twist)
    # 後退コマンド(linear_x < 0)。v_reverse キャップの実測値がいくつであれ、
    # コマンド自体はそれより十分大きい大きさで出しておけば
    # (min(コマンド, 上限, 障害物クランプ) で決まる出力の性質上)問題ない。
    drive = DriveController(ros_node, linear_x=-limiter_param('v_max'), mode='IDLE')
    try:
        warmup_deadline = time.monotonic() + 1.0
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(ros_node, timeout_sec=0.05)
        watcher.records.clear()

        assert_stops_within(watcher, 'linear.x', _STOP_WINDOW_MS)
    finally:
        drive.stop()
        watcher.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
