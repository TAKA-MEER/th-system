"""
case_13_localization_loss.py — 故障注入 13「自己位置喪失（SIGKILL）」
（ctest 登録名: fault_injection_13）
==========================================================================
`DetailedDesign-safety.md` §10 #13: `slam_toolbox` を SIGKILL し、
重大フォルト → `ESTOP` となることを確認する。自動化: Gazebo（段階5で有効化）。

**段階5で有効化予定** — 理由・`pytest.skip` を使う理由は `case_07_summon_retreat_wait.py`
の docstring と同じ（`DetailedDesign-wp2.md` `WP-TEST-01` §11）。

【`SIGKILL` を使うテストについての既知の注意】`case_11_limiter_sigkill.py` と同じ
（`kill -TERM` を既定にし、SIGKILL が要る 11・13 は毎回コンテナを作り直す）。
"""
import pytest


def test_fault_injection_13_localization_loss():
    pytest.skip(
        "段階5で有効化予定（自己位置喪失時の重大フォルト化ロジックは現時点の段階2では"
        "未実装）。DetailedDesign-wp2.md WP-TEST-01 §11 参照。"
    )
