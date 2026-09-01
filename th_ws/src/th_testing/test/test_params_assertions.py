"""test_params_assertions.py — th_params/assertions.py の A1〜A11 を検証する（WP-PARAM-01 §7）。

各 A<N> は違反ケースと合格ケースの両方を持つ。
"""
from th_params import assertions


def test_a1_intrusion_budget():
    # 違反: v_max(1.0) * timeout(1000ms)/1000 = 1.0 > budget(0.5)
    assert assertions.a1_intrusion_budget(1.0, 1000, 0.5) != []
    # 合格: 1.0 * 200/1000 = 0.2 <= 0.5
    assert assertions.a1_intrusion_budget(1.0, 200, 0.5) == []


def test_a2a_floor_before_behavior():
    # 違反: floor(0.6) >= stop(0.5)
    assert assertions.a2a_floor_before_behavior(0.6, 0.5) != []
    # 合格: floor(0.4) < stop(0.5)
    assert assertions.a2a_floor_before_behavior(0.4, 0.5) == []


def test_a2a_floor_before_behavior_axes_only_autonomous_axes():
    # v_jog_panel は対象外（極低速軸）。floor > jog 軸の値でも違反にならない
    stop_by_axis = {"v_max": 0.6, "v_slow": 0.55, "v_reverse": 0.55, "v_jog_panel": 0.1}
    errors = assertions.a2a_floor_before_behavior_axes(0.5, stop_by_axis)
    # v_max/v_slow/v_reverse は floor(0.5) < stop なので合格。v_jog_panel は対象外
    assert errors == []

    # 違反ケース: v_max 軸で floor が stop 以上
    stop_by_axis_bad = {"v_max": 0.4, "v_slow": 0.55, "v_reverse": 0.55, "v_jog_panel": 0.1}
    errors_bad = assertions.a2a_floor_before_behavior_axes(0.5, stop_by_axis_bad)
    assert any("v_max" in e for e in errors_bad)


def test_a2b_floor_before_follow():
    # 違反: floor(0.6) >= follow_stop(0.5)
    assert assertions.a2b_floor_before_follow(0.6, 0.5) != []
    # 合格
    assert assertions.a2b_floor_before_follow(0.4, 0.5) == []


def test_a3_tracker_lost_grace():
    # 違反: grace(0) は 0 より大きくない
    assert assertions.a3_tracker_lost_grace(0, 1000) != []
    # 違反: grace >= person_timeout
    assert assertions.a3_tracker_lost_grace(1000, 1000) != []
    # 合格
    assert assertions.a3_tracker_lost_grace(500, 1000) == []


def test_a4_jog_lease_vs_manual_joy():
    # 違反: jog_lease(900ms) < manual_joy_timeout(1.0s=1000ms)
    assert assertions.a4_jog_lease_vs_manual_joy(900, 1.0) != []
    # 合格: jog_lease(1200ms) >= 1000ms
    assert assertions.a4_jog_lease_vs_manual_joy(1200, 1.0) == []


def test_a5_speed_ordering():
    # 違反: v_reverse > v_slow
    assert assertions.a5_speed_ordering(0.6, 0.5, 1.0, 0.1) != []
    # 違反: v_jog_panel > v_reverse
    assert assertions.a5_speed_ordering(0.3, 0.5, 1.0, 0.4) != []
    # 合格
    assert assertions.a5_speed_ordering(0.3, 0.5, 1.0, 0.1) == []


def test_a6_esp32_timeout_vs_watchdog():
    # 違反(警告): esp32_timeout(500) <= watchdog(600)
    assert assertions.a6_esp32_timeout_vs_watchdog(500, 600) != []
    # 合格
    assert assertions.a6_esp32_timeout_vs_watchdog(800, 600) == []


def test_a7_cmd_vel_stale_vs_watchdog():
    # 違反: cmd_vel_stale(600) >= watchdog(600)
    assert assertions.a7_cmd_vel_stale_vs_watchdog(600, 600) != []
    # 合格: cmd_vel_stale(400) < watchdog(600)
    assert assertions.a7_cmd_vel_stale_vs_watchdog(400, 600) == []


def test_a8_blocking_placeholders_violation_and_pass():
    rows = [
        {"name": "brake_accel_mps2", "status": "placeholder", "blocking": True,
         "blocking_from_stage": 0, "consumers": ["obstacle_limiter"]},
        {"name": "replay_drift_m_per_100m", "status": "placeholder", "blocking": True,
         "blocking_from_stage": 5, "consumers": ["replay_runner"]},
    ]
    # 違反: stage=0 で brake_accel_mps2 が該当
    errors = assertions.a8_blocking_placeholders(rows, stage=0)
    assert any("brake_accel_mps2" in e for e in errors)
    # replay_drift は blocking_from_stage=5 > stage(0) なので該当しない
    assert not any("replay_drift" in e for e in errors)

    # 合格: すべて measured になった（status が placeholder でない）
    good_rows = [
        {"name": "brake_accel_mps2", "status": "measured", "blocking": True,
         "blocking_from_stage": 0, "consumers": ["obstacle_limiter"]},
    ]
    assert assertions.a8_blocking_placeholders(good_rows, stage=0) == []


def test_a8_respects_consumers():
    """起動しないノードのパラメータで止まらない（§7 の必須テスト）。"""
    rows = [
        {"name": "brake_accel_mps2", "status": "placeholder", "blocking": True,
         "blocking_from_stage": 0, "consumers": ["obstacle_limiter"]},
    ]
    # obstacle_limiter を起動しない launch では止まらない
    errors = assertions.a8_blocking_placeholders(rows, stage=0, nodes=["follow_runner"])
    assert errors == []
    # obstacle_limiter を起動する launch では止まる
    errors2 = assertions.a8_blocking_placeholders(rows, stage=0, nodes=["obstacle_limiter"])
    assert errors2 != []


def test_a9_consumers_nonempty():
    # 違反: consumers が空
    rows_bad = [{"name": "dead_param", "consumers": []}]
    assert assertions.a9_consumers_nonempty(rows_bad) != []
    # 合格
    rows_good = [{"name": "alive_param", "consumers": ["some_node"]}]
    assert assertions.a9_consumers_nonempty(rows_good) == []


def test_a10_wheel_radius_scale():
    # 違反: |1.2 - 1| = 0.2 > 0.10
    assert assertions.a10_wheel_radius_scale(1.2, 0.10) != []
    # 合格: |1.05 - 1| = 0.05 <= 0.10
    assert assertions.a10_wheel_radius_scale(1.05, 0.10) == []


def test_a11_hysteresis_band():
    # 違反: floor(0.5) + hysteresis(0.2) = 0.7 >= stop(0.6)
    assert assertions.a11_hysteresis_band(0.5, 0.2, 0.6) != []
    # 合格: floor(0.5) + hysteresis(0.05) = 0.55 < stop(0.6)
    assert assertions.a11_hysteresis_band(0.5, 0.05, 0.6) == []


def test_a13_speed_limit_strictness_order_pass():
    # LIMIT_STRICTNESS そのものの並び。stop は 0.0 扱い、数値は昇順で一致している
    order = ("stop", "v_check", "v_calib", "v_jog_panel", "v_slow", "v_leash", "v_max")
    values = {"v_check": 0.05, "v_calib": 0.15, "v_jog_panel": 0.3,
              "v_slow": 0.6, "v_leash": 1.0, "v_max": 1.2}
    assert assertions.a13_speed_limit_strictness_order(order, values) == []


def test_a13_speed_limit_strictness_order_detects_inversion():
    # v_jog_panel が並びでは v_calib の後ろ（＝より厳しい）だが、実際の値は
    # v_calib(0.15) より小さい(0.1) ではなく大きい(0.5) と仮定 → まだ合格のはず。
    # 逆に、並びで v_calib の後ろに置かれているのに実際の値が v_calib より小さい
    # ケース（当初の誤り: stop の直後＝v_check や v_calib より前に置いてしまった状態
    # の再現）を検出できることを確認する。
    order = ("stop", "v_check", "v_calib", "v_jog_panel", "v_slow", "v_leash", "v_max")
    values = {"v_check": 0.05, "v_calib": 0.15, "v_jog_panel": 0.1,  # v_calib より小さい
              "v_slow": 0.6, "v_leash": 1.0, "v_max": 1.2}
    errors = assertions.a13_speed_limit_strictness_order(order, values)
    assert errors != []
    assert any("v_jog_panel" in e and "v_calib" in e for e in errors)


def test_a13_speed_limit_strictness_order_excludes_placeholders():
    # v_jog_panel が values_by_name に無い（＝placeholder。未測定）なら比較対象から
    # 除外され、他が正しい昇順であれば合格する。
    order = ("stop", "v_check", "v_calib", "v_jog_panel", "v_slow", "v_leash", "v_max")
    values = {"v_check": 0.05, "v_calib": 0.15, "v_slow": 0.6, "v_leash": 1.0, "v_max": 1.2}
    assert assertions.a13_speed_limit_strictness_order(order, values) == []


def test_a13_speed_limit_strictness_order_stop_is_zero():
    # stop は常に 0.0 として扱われる。名前の直後に値 0.0 未満のものは存在しえないので、
    # stop の次に来る名前が正の値ならどれでも合格する（stop が最も厳しいという定義）。
    order = ("stop", "v_check")
    assert assertions.a13_speed_limit_strictness_order(order, {"v_check": 0.05}) == []
