"""
case_09_person_detection_off.py — 故障注入 9「人検知 OFF とフォルトの区別」
（ctest 登録名: fault_injection_09）
============================================================================
`DetailedDesign-safety.md` §10 #9: 人検知を意図的に OFF にし、フォルトにならない・
強制遷移しないことを確認する（`CLAUDE.md` の「モード FSM」節にある
`PERSON_TRACKER_LOST` 例外の裏側の確認）。自動化: Gazebo。

**このパケットで実装しようとして判明した事実誤認・アーキテクチャ上の欠落**
（作業指示の「事実誤認があれば指摘すること」に該当するため詳しく書く）。

作業指示は「`/system/state` の `tracker_enabled` が false のとき、
`PERSON_TRACKER_LOST` フォルトにならず、強制遷移も起きないことを確認する」
という枠組みを与えていたが、コードを読むと **`tracker_enabled` と
`PERSON_TRACKER_LOST` の間には現状なんの結線も無い**。これは1つの欠落では
なく、独立した2つの欠落が重なっている:

1. **`tracker_enabled` はどこからも消費されていない。**
   `state_manager.py` の `_SETTABLE_FLAGS` に含まれ `/system/set_flag` で
   設定でき、`/system/state.tracker_enabled` に反映される配線はあるが、
   それを**読む**側が無い。`th_state/th_state/guards.py` の
   `_mode_entry_allowed`（`mode_entry_allowed` ガード）は `mode_entry.yaml`
   の表だけを見ており、`tracker_enabled` は見ていない——これは見落としでは
   なく、`mode_entry.yaml:8-9` に明記された既知の未実装
   （「前提条件（tracker_enabled 含む）は WP-STATE-01 の対象外」）である。
   `safety_monitor.cpp` も `tracker_enabled` を一切購読しない
   （そもそも `th_state` 側だけが持つフラグで、C++ 側の `Context` には
   存在しない概念）。

2. **`PERSON_TRACKER_LOST` 自体が sim でも実機でも発火し得ない。**
   `safety_monitor.cpp:270` の `person` ターゲットは `targetEnabled("person")`
   でゲートされているが、`gazebo.launch.py` の
   `SAFETY_ENABLED_TARGETS_SIM = ['lidar', 'limiter']` にも
   `SAFETY_ENABLED_TARGETS_REAL = ['lidar', 'esp32', 'runaway', 'state',
   'firmware', 'limiter']` にも `'person'` が無い。**さらに**、たとえ
   `enabled_targets` に加えても、`/person/targets`（`PersonTargets` 型）を
   publish するノードがリポジトリ全体を検索して1つも存在しない
   （`grep -rl PersonTargets` で `safety_monitor.cpp` と
   `test_msg_definitions.py` しかヒットしない）。`th_perception/scripts/
   gazebo_person_relay.py` は廃止予定の `PersonStatus`（`/person/status`）を
   出しており、`PersonTargets` への移行は `DetailedDesign-reuse.md` §2.3 が
   `person_tracker_bridge.py`（**実機専用**。`real_tracker_expr` 条件）の
   改修として予定しているだけで、sim 側の中継はまだ移行されていない。
   `DetailedDesign-safety.md` §5.4 も `tracker_lost_grace_ms` を
   `blocking_from_stage: 4` の placeholder としており、「人物ロスト」の
   仕組み自体が意図的にまだ完成していない設計であることが読み取れる。

**結論**: 現状の実装では `tracker_enabled` を false にしようが true にしようが、
`PERSON_TRACKER_LOST` は sim でも実機でも一切発火しない（`person` が
`enabled_targets` に無いことに加え、そもそも publisher が存在しない）。
したがって「OFF にしたらフォルトにならない」を確認しても**必ず無条件に
成立してしまい**、§9 が確かめたい「OFF は意図的な区別であって偶然の沈黙では
ない」という主張を何一つ検証できない——作業指示自身が警告している
「居ないから当然フォルトにならない、では試験の意味がない」の状態そのもの。

**このパケットでは実装しない。** 有意な試験にするには次のいずれか
（両方必要な可能性が高い）が要る:
  - `gazebo.launch.py` の `SAFETY_ENABLED_TARGETS_SIM` に `'person'` を追加
    する（`case_11` で見つけた `'limiter'` 追加漏れと同種だが、こちらは
    「追加しても誰も `/person/targets` を出さない」ため単独では効果が無い）
  - sim 側で `/person/targets` を発行する経路を用意する（`gazebo_person_relay.py`
    の改修、または `person_tracker_bridge.py` 相当を sim 向けに用意する）
  - `th_state` 側で `tracker_enabled` を実際に読んで `PERSON_TRACKER_LOST` の
    強制遷移を抑制する結線（`guards.py` の `_fault_stops_mode` 拡張）を追加する
どれも**本番の launch/description/知覚ノードの構造**を変える話であり、
作業指示の「本番launchの構造を変える必要が出た場合は実装せず報告する」に
該当するため、コーディネーターの判断を仰ぐ。
"""
import ast
import os

import pytest

_GAZEBO_LAUNCH_PY = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'th_bringup', 'launch', 'gazebo.launch.py'))


def _sim_enabled_targets() -> list[str]:
    """`gazebo.launch.py` の `SAFETY_ENABLED_TARGETS_SIM = [...]` を静的に読む
    （ROS2 import 不要。Gazebo を起動する前に機械的に判定してここで落とすため。
    `case_02` の `_v_reverse_precondition` と同じ設計判断——どうせ落ちるテストの
    ために Gazebo を毎回 40〜45 秒かけて起動する無駄を避ける）。"""
    with open(_GAZEBO_LAUNCH_PY, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=_GAZEBO_LAUNCH_PY)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(t, ast.Name) and t.id == 'SAFETY_ENABLED_TARGETS_SIM'
                   for t in node.targets):
            continue
        return ast.literal_eval(node.value)
    pytest.fail(
        f"{_GAZEBO_LAUNCH_PY} に SAFETY_ENABLED_TARGETS_SIM の代入が見つからない "
        "(定数名が変わった? ファイル構造が変わった?)")


@pytest.fixture
def _person_target_precondition():
    """上のモジュール docstring 参照。'person' が SAFETY_ENABLED_TARGETS_SIM に
    無い間はここで明示的に fail する。"""
    targets = _sim_enabled_targets()
    if 'person' not in targets:
        pytest.fail(
            "未実装（このパケットの範囲外。詳細は本ファイル冒頭 docstring）。"
            f"gazebo.launch.py の SAFETY_ENABLED_TARGETS_SIM={targets!r} に "
            "'person' が無いため、sim では PERSON_TRACKER_LOST が構造的に "
            "発火しない（safety_monitor.cpp の targetEnabled('person') ゲート）。"
            "加えて /person/targets (PersonTargets) を発行するノードが sim に "
            "1つも存在しないため、'person' を追加するだけでも解決しない。"
            "DetailedDesign-safety.md §10 #9 参照。")
    return targets


def test_fault_injection_09_person_detection_off_is_not_a_fault(_person_target_precondition):
    pytest.fail(
        "SAFETY_ENABLED_TARGETS_SIM に 'person' が入ったので、この先の本体"
        "（tracker_enabled=false を /system/set_flag で設定 → PERSON_TRACKER_LOST "
        "を発生させうる状況を作って /safety/fault が active にならないこと・"
        "/system/state.mode が強制遷移しないことを確認する）を実装すること。"
        "実装されていないのでここで明示的に fail する。")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
