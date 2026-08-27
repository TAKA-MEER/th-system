"""
case_06_fault_to_stop.py — 故障注入 6「フォルト検知 → 停止」
（ctest 登録名: fault_injection_06）
=================================================================
`DetailedDesign-safety.md` §10 #6: 5 の続きを測る。フォルトから
`conftest.FAULT_TO_STOP_LAYER3_MS`（＝100ms。層3の応答時間そのものであり
パラメータではなく設計上の定数。T-1 の唯一の例外）以内に速度指令が 0 になる
ことを確認する。自動化: Gazebo ＋ 実機。

**T-2: これは 5（通信断 → フォルト検知）とは別のテストである。** 理由は
`case_05_link_loss_fault.py` の docstring と同じ（`F-21`）。

**このパケットの範囲外。** Gazebo 本体は次パケットで書く。ここでは
`conftest.py` の `FAULT_TO_STOP_LAYER3_MS` を使う設計だけを示すプレースホルダを置く。
"""
import pytest

# 定数の値そのものは conftest.py の FAULT_TO_STOP_LAYER3_MS が正本。
# このファイルからは `import conftest` しない — pytest はテストファイルごとの
# ディレクトリを sys.path に足す都合上、リポジトリ内の複数の conftest.py
# （例: th_testing/test/conftest.py と fault_injection/conftest.py）が
# 同じモジュール名 "conftest" を取り合い、意図しない方が import される
# 事故につながりうるため（__init__.py が無いディレクトリでの既知の落とし穴）。
# 値が必要な次パケットの実装では、素朴な `import conftest` ではなく
# fixture 経由（例えば conftest.py に `fault_to_stop_layer3_ms` fixture を
# 足す）で受け渡す設計にすること。


def test_fault_injection_06_fault_to_stop():
    pytest.fail(
        "未実装。次パケットで実装する（5 でフォルトが立った時刻を基準に、"
        "conftest.FAULT_TO_STOP_LAYER3_MS（層3の応答時間。100ms。T-1 の唯一の例外）"
        "以内に `/cmd_vel` が 0 になることを `sim_stack` / `assert_zero_within` で"
        "確認する）。DetailedDesign-safety.md §10 #6 参照。"
    )
