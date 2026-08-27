"""
case_11_limiter_sigkill.py — 故障注入 11「リミッタの死（SIGKILL）」
（ctest 登録名: fault_injection_11）
========================================================================
`DetailedDesign-safety.md` §10 #11: `obstacle_limiter` を SIGKILL し、
重大フォルト → `ESTOP` → 駆動ゼロとなることを確認する（`DEBT-4`）。
自動化: Gazebo ＋ 実機。

**このパケットの範囲外。** 中身は `case_01_obstacle_limiter_forward.py` と同じ理由で
次パケットに送る。

【`SIGKILL` を使うテストについての既知の注意（`CLAUDE.md` の環境の癖）】
`kill -9` を繰り返すとコンテナ内の DDS discovery が壊れることがある
（`DetailedDesign-wp2.md` WP-TEST-01 §6.3 ②）。次パケットでこのテストを実装する際は
`kill -TERM` を既定にする他のテストと違い、SIGKILL が要る 11・13 は**毎回コンテナを
作り直す**運用にすること。
"""
import pytest


def test_fault_injection_11_limiter_sigkill():
    pytest.fail(
        "未実装。次パケットで実装する（`obstacle_limiter` を SIGKILL し、"
        "重大フォルト → `/system/state` の mode が ESTOP → `/cmd_vel` が 0 になることを "
        "確認する。SIGKILL テストにつき毎回コンテナを作り直すこと）。"
        "DetailedDesign-safety.md §10 #11（`DEBT-4`）参照。"
    )
