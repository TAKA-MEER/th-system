"""
test_wheel_scale_core.py
=========================
wheel_scale_core.py (esp32_bridge の wheel_radius_scale 双方向適用) の単体テスト。
ROS2 なし・純粋 Python で実行可能。

満たす仕様: docs/plan/detailed/DetailedDesign-maintenance.md §4（O-d10）
　指令（PC → ESP32）: v_send = v_desired / k
　実測（ESP32 → PC）: v_actual = v_report × k
　A10: |k − 1| ≤ wheel_radius_scale_max_dev（既定 0.1）。超えたら起動を拒否
完了条件: 指令と実測の往復で元に戻る。
"""

import math

from wheel_scale_core import a10_violations, scale_wheel_command, scale_wheel_feedback


class TestWheelScaleConversions:

    def test_k_is_identity(self):
        """k=1 では変換しない（恒等）"""
        for v in (-1.2, -0.0, 0.0, 0.3, 2.5):
            assert scale_wheel_command(v, 1.0) == v == scale_wheel_feedback(v, 1.0)

    def test_cmd_is_divided_by_k(self):
        """指令は / k（k=1.05 で 0.3 → 0.3/1.05）"""
        k = 1.05
        for v in (-0.6, 0.0, 0.3, 1.2):
            assert abs(scale_wheel_command(v, k) - v / k) < 1e-12

    def test_feedback_is_multiplied_by_k(self):
        """実測は × k（k=1.05 で 0.3 → 0.315）"""
        k = 1.05
        for v in (-0.6, 0.0, 0.3, 1.2):
            assert abs(scale_wheel_feedback(v, k) - v * k) < 1e-12

    def test_roundtrip_restores_original(self):
        """往復: 指令で / k し、その値をそのまま実測に流すと元の速度に戻る"""
        for k in (0.90, 0.95, 1.0, 1.05, 1.10):
            for v in (-1.2, -0.4, 0.0, 0.3, 1.5):
                sent = scale_wheel_command(v, k)
                measured = scale_wheel_feedback(sent, k)
                assert abs(measured - v) < 1e-12, f"k={k} v={v}: {measured} != {v}"


class TestA10Boundaries:

    def test_boundary_exact_is_ok(self):
        """A10: 境界ちょうど（±0.10）は可"""
        assert a10_violations(1.0 + 0.10, 0.10) == []
        assert a10_violations(1.0 - 0.10, 0.10) == []

    def test_over_boundary_is_rejected(self):
        """A10: 0.1001 は不可"""
        assert a10_violations(1.0 + 0.1001, 0.10) != []
        assert a10_violations(1.0 - 0.1001, 0.10) != []

    def test_negative_zero_nan_rejected(self):
        """負・0・NaN は不可"""
        assert a10_violations(-1.0, 0.10) != []
        assert a10_violations(-0.05, 0.10) != []
        assert a10_violations(0.0, 0.10) != []
        assert a10_violations(float('nan'), 0.10) != []

    def test_error_message_contains_a10(self):
        """違反メッセージが A10 であること（呼び出し側のログの識別子）"""
        msgs = a10_violations(1.5, 0.10)
        assert msgs and 'A10' in msgs[0]