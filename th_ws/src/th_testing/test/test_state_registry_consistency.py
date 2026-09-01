"""test_state_registry_consistency.py — th_state.zones.LIMIT_STRICTNESS と
registry.yaml の speed_limit_strictness_rank が一致することを機械的に検査する
（N-15 レビュー・WP-SAFE-03 の派生作業）。

背景: A13（`th_params/assertions.py::a13_speed_limit_strictness_order()`）は
「speed_limit 名の厳しさ順」が registry.yaml の解決済みの数値の昇順と一致することを
起動時に検査するアサーションだが、その並びをどこから読むかにはレビューで指摘が入った。

`th_params` は registry.yaml から全ノード分の設定を生成する**上流**（`REGISTRY_NODES` に
`state_manager` を含む）であり、下流の `th_state` を import すると層が逆転する。加えて
`export.py` はモジュール先頭 import に失敗すると `python3 -m th_params.export` が丸ごと
起動不能になる（`params_generation.py` が launch のたびにこれをサブプロセスで呼ぶ）。

そのため `th_params/export.py::speed_limit_strictness_order()` は `th_state` を import
せず、registry.yaml 自身が持つ `speed_limit_strictness_rank` フィールドから並びを組み立てる。
一方 `th_state/zones.py::LIMIT_STRICTNESS` は `combine_speed_limits()` が実際にモード由来・
画面由来の速度上限を合成する際に使う実体であり、これは変更していない。

結果として同じ「厳しさ順」を 2 か所（registry.yaml と th_state/zones.py）で宣言する重複が
残るが、**このテストが両者の一致を機械的に照合する**ことで、N-14（`auto_brake` の定義が
3 箇所で食い違っていた）のような静かな乖離を防ぐ。このファイルだけは起動経路ではなく
テストコードなので、`th_state` と `th_params` の両方を import してよい。
"""
from th_params import export
from th_state.zones import LIMIT_STRICTNESS


def test_registry_rank_order_matches_th_state_limit_strictness(registry_rows):
    order_from_registry = export.speed_limit_strictness_order(registry_rows)
    assert order_from_registry == LIMIT_STRICTNESS, (
        "registry.yaml の speed_limit_strictness_rank から組み立てた並び "
        f"{order_from_registry} が th_state/zones.py の LIMIT_STRICTNESS "
        f"{LIMIT_STRICTNESS} と一致しない。どちらかを直すときは両方揃えること（N-15）")


def test_registry_rank_order_starts_with_stop_and_has_no_gaps(registry_rows):
    """rank の付け方そのものが壊れていないことの補助検査: 先頭は "stop"（暗黙の rank 0）で、
    rank に飛びや重複が無い（＝連番）こと。上のテストが LIMIT_STRICTNESS との一致で
    実質的に同じことを保証するが、こちらは registry.yaml 側だけで完結させて
    「rank を書き間違えた」原因を素早く切り分けられるようにする。"""
    order = export.speed_limit_strictness_order(registry_rows)
    assert order[0] == "stop"

    ranks = sorted(row["speed_limit_strictness_rank"] for row in registry_rows
                    if "speed_limit_strictness_rank" in row)
    assert ranks == list(range(1, len(ranks) + 1)), (
        f"speed_limit_strictness_rank が 1 から連番になっていない: {ranks}")
