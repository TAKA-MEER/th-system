"""
test_keepalive_core.py
=========================
keepalive_core.py (esp32_bridge の /cmd_vel stale タイムアウト判定) の
単体テスト。ROS2 なし・純粋 Python で実行可能。

満たす仕様: docs/plan/detailed/DetailedDesign-wp2.md WP-SAFE-02 §4.1・§4.3
（K-1・K-2）。
"""

from keepalive_core import Twist2, ZERO_TWIST2, keepalive_value


class TestKeepaliveValue:

    def test_stale_returns_zero(self):
        """/cmd_vel が stale_ms を超えて途絶していればゼロを返す (DEBT-4)"""
        last_cmd = Twist2(0.3, 0.1)
        out = keepalive_value(
            last_cmd_ms=0, now_ms=500, last_cmd=last_cmd,
            stale_ms=400, locked=False)
        assert out == ZERO_TWIST2

    def test_locked_returns_zero(self):
        """ロック中は鮮度に関わらずゼロを返す (K-2)"""
        last_cmd = Twist2(0.3, 0.1)
        out = keepalive_value(
            last_cmd_ms=1000, now_ms=1000, last_cmd=last_cmd,
            stale_ms=400, locked=True)
        assert out == ZERO_TWIST2

    def test_fresh_returns_last(self):
        """stale でも locked でもなければ last_cmd をそのまま返す（既存挙動の維持）"""
        last_cmd = Twist2(0.3, 0.1)
        out = keepalive_value(
            last_cmd_ms=1000, now_ms=1200, last_cmd=last_cmd,
            stale_ms=400, locked=False)
        assert out is last_cmd

    def test_boundary_exactly_stale_ms_is_not_yet_stale(self):
        """経過時間がちょうど stale_ms のときはまだ stale ではない（`>` であって `>=` でない）"""
        last_cmd = Twist2(0.2, 0.0)
        out = keepalive_value(
            last_cmd_ms=0, now_ms=400, last_cmd=last_cmd,
            stale_ms=400, locked=False)
        assert out is last_cmd

    def test_boundary_just_over_stale_ms_is_stale(self):
        last_cmd = Twist2(0.2, 0.0)
        out = keepalive_value(
            last_cmd_ms=0, now_ms=400.001, last_cmd=last_cmd,
            stale_ms=400, locked=False)
        assert out == ZERO_TWIST2

    def test_locked_overrides_fresh_input(self):
        """K-2: ロックは stale 判定より先に評価する。
        鮮度が十分でもロック中ならゼロ（ロック優先の明示的確認）"""
        last_cmd = Twist2(0.5, 0.5)
        out = keepalive_value(
            last_cmd_ms=1000, now_ms=1000, last_cmd=last_cmd,
            stale_ms=400, locked=True)
        assert out == ZERO_TWIST2

    def test_zero_last_cmd_when_fresh_returns_zero_unchanged(self):
        """既にゼロ指令が新鮮な場合はゼロのまま（回帰確認）"""
        out = keepalive_value(
            last_cmd_ms=1000, now_ms=1050, last_cmd=ZERO_TWIST2,
            stale_ms=400, locked=False)
        assert out == ZERO_TWIST2
