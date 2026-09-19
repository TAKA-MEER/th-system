"""
case_07_summon_retreat_wait.py — 故障注入 7「呼び寄せの退避待ち」
（ctest 登録名: fault_injection_07）
=====================================================================
`DetailedDesign-safety.md` §10 #7: 人が退かずに待っているとき発進せず、
タイムアウトで中止することを確認する。自動化: Gazebo。

**段階6で有効化予定（`DetailedDesign-wp2.md` `WP-TEST-01` §11「7（退避待ち）・
13（自己位置喪失）は段階6・5の機能に依存する。段階2では skip マーク付きで置き、
該当段階で有効化する」）。** SUMMONING モード自体が現時点（段階2）でまだ
存在しない/有効化されていないため、いま Gazebo テストを書いても検証対象が無い。

【`pytest.fail` ではなく `pytest.skip` を使う理由】
実機専用3本（T-3）とは性質が異なる。T-3 が「自動化できないこと自体が仕様」で
恒常的に赤であるべきなのに対し、この項目は「まだ対象の機能が存在しない」
ことによる一時的な保留であり、設計書 §11 で明示的に skip 運用が指定されている。
段階6で `SUMMONING` の退避待ちロジックが入ったら、このテストを実装した上で
skip を外すこと。
"""
import pytest


def test_fault_injection_07_summon_retreat_wait():
    pytest.skip(
        "段階6で有効化予定（呼び寄せ SUMMONING の退避待ちロジックは現時点の段階2では"
        "未実装）。DetailedDesign-wp2.md WP-TEST-01 §11 参照。"
    )
