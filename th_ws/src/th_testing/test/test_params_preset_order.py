"""test_params_preset_order.py — SG-C14 速度プリセット順序（低速≦中速≦高速）。

`th_params/preset_order.py` の純粋関数の試験（ROS2 不要・ホストで走る）。
サーバ側（`params_audit` の `/params/set`）の二重ガードがこの関数を使う。
"""
from th_params import preset_order


def test_ok_when_ordered():
    assert preset_order.check_preset_order(
        {"speed_preset_low": 0.27, "speed_preset_mid": 0.55,
         "speed_preset_high": 1.0}) is None
    # 等しいのも可（各段は隣の段の範囲内）。
    assert preset_order.check_preset_order(
        {"speed_preset_low": 0.5, "speed_preset_mid": 0.5,
         "speed_preset_high": 0.5}) is None


def test_rejects_inversion_with_reason():
    reason = preset_order.check_preset_order(
        {"speed_preset_low": 0.6, "speed_preset_mid": 0.55,
         "speed_preset_high": 1.0})
    assert "低速" in reason and "中速" in reason
    reason = preset_order.check_preset_order(
        {"speed_preset_low": 0.27, "speed_preset_mid": 1.0,
         "speed_preset_high": 0.9})
    assert "中速" in reason and "高速" in reason


def test_rejects_over_ceiling_and_below_min():
    reason = preset_order.check_preset_order(
        {"speed_preset_low": 0.27, "speed_preset_mid": 0.55,
         "speed_preset_high": 1.2})
    assert "100%" in reason
    reason = preset_order.check_preset_order(
        {"speed_preset_low": 0.01, "speed_preset_mid": 0.55,
         "speed_preset_high": 1.0})
    assert "0.05" in reason


def test_skips_when_not_all_numeric():
    # 型の検査は schema の責務。ここでは判定しない。
    assert preset_order.check_preset_order({}) is None
    assert preset_order.check_preset_order(
        {"speed_preset_low": 0.27, "speed_preset_mid": "x",
         "speed_preset_high": 1.0}) is None
    assert preset_order.check_preset_order(
        {"speed_preset_low": True, "speed_preset_mid": 0.55,
         "speed_preset_high": 1.0}) is None


def test_registry_defaults_are_ordered():
    """現行 registry の既定値（0.27/0.55/1.0）は順序を満たす。
    満たさないと無関係の /params/set まで通らなくなる（fail-closed の回帰防止）。"""
    import yaml
    import os
    path = os.path.join(os.path.dirname(__file__), "..", "..",
                        "th_params", "config", "registry.yaml")
    rows = yaml.safe_load(open(path, encoding="utf-8"))
    values = {r["name"]: r["value"] for r in rows
              if r["name"] in preset_order.PRESET_ORDER_NAMES}
    assert len(values) == 3
    assert preset_order.check_preset_order(values) is None
