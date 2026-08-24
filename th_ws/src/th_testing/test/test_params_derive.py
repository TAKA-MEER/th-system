"""test_params_derive.py — th_params/derive.py の 12 関数を検証する（WP-PARAM-01 §7）。"""
import math

import pytest

from th_params import derive


def test_braking_roundtrip():
    """speed_from_braking_distance(braking_distance(v)) == v（往復）。"""
    a = 0.8
    t_delay = 0.2
    for v in (0.1, 0.3, 0.5, 1.0, 1.5):
        d = derive.braking_distance(v, a, t_delay)
        v_back = derive.speed_from_braking_distance(d, a, t_delay)
        assert v_back == pytest.approx(v, rel=1e-9)


def test_floor_is_speed_independent():
    """floor_distance は速度を引数に取らない（signature に v が無いこと自体で保証されるが、
    ここでは複数の呼び出しが常に同じ式で計算されることを確認する）。"""
    import inspect
    sig = inspect.signature(derive.floor_distance)
    assert "v" not in sig.parameters
    assert list(sig.parameters) == ["body_half_length_m", "floor_margin_m"]
    assert derive.floor_distance(0.35, 0.15) == pytest.approx(0.5)


def test_braking_distance_formula():
    # v^2/(2a) + v*t_delay
    assert derive.braking_distance(1.0, 0.5, 0.2) == pytest.approx(1.0 / 1.0 + 0.2)


def test_v_max_from_ceiling():
    assert derive.v_max_from_ceiling(1.5, 0.8) == pytest.approx(1.2)


def test_person_backstop_ms_takes_the_max():
    # max(grace*factor, p99*factor)
    assert derive.person_backstop_ms(500, 100, 2.0) == pytest.approx(1000)
    assert derive.person_backstop_ms(100, 800, 2.0) == pytest.approx(1600)


def test_clear_distance():
    assert derive.clear_distance(0.35, 0.3) == pytest.approx(0.65)


def test_hysteresis_band_uses_floor_distance_not_stop_distance():
    """P-4: hysteresis_band は floor_distance_m を引数に取る（obstacle_stop_distance_m ではない）。"""
    import inspect
    sig = inspect.signature(derive.hysteresis_band)
    assert list(sig.parameters) == ["floor_distance_m", "ratio"]
    assert derive.hysteresis_band(0.5, 0.2) == pytest.approx(0.1)


def test_combined_heading_error():
    rss, worst = derive.combined_heading_error(3.0, 4.0)
    assert rss == pytest.approx(5.0)
    assert worst == pytest.approx(7.0)


def test_two_point_angle_error_deg():
    # atan2(sigma*sqrt(2), spacing) in degrees
    result = derive.two_point_angle_error_deg(1.0, 0.1)
    expected = math.degrees(math.atan2(0.1 * math.sqrt(2.0), 1.0))
    assert result == pytest.approx(expected)


def test_timeout_lower_bound_ms():
    assert derive.timeout_lower_bound_ms(500, 1.0) == pytest.approx(1000)


def test_timeout_upper_bound_ms():
    assert derive.timeout_upper_bound_ms(1.0, 0.5) == pytest.approx(500.0)


def test_deviation_budget_m():
    assert derive.deviation_budget_m(2.0, 0.5, 0.1) == pytest.approx((2.0 - 0.5) / 2.0 - 0.1)


def test_all_twelve_functions_exist_with_expected_signatures():
    """DetailedDesign-wp0.md WP-PARAM-01 §4.1 の 12 関数がすべて実装されていること。"""
    import inspect
    expected = {
        "braking_distance": ["v", "a", "t_delay"],
        "speed_from_braking_distance": ["d_allow", "a", "t_delay"],
        "v_max_from_ceiling": ["ceiling_mps", "headroom_ratio"],
        "floor_distance": ["body_half_length_m", "floor_margin_m"],
        "person_backstop_ms": ["grace_ms", "link_p99_ms", "factor"],
        "clear_distance": ["body_half_length_m", "clear_margin_m"],
        "hysteresis_band": ["floor_distance_m", "ratio"],
        "combined_heading_error": ["two_point_deg", "nav_tolerance_deg"],
        "two_point_angle_error_deg": ["spacing_m", "sigma_m"],
        "deviation_budget_m": ["corridor_width_m", "body_width_m", "margin_m"],
        "timeout_lower_bound_ms": ["p99_gap_ms", "margin_ratio"],
        "timeout_upper_bound_ms": ["v_max", "intrusion_budget_m"],
    }
    assert len(expected) == 12
    for func_name, params in expected.items():
        assert hasattr(derive, func_name), f"derive.{func_name} が無い"
        sig = inspect.signature(getattr(derive, func_name))
        assert list(sig.parameters) == params, f"derive.{func_name} の引数が一致しない"
