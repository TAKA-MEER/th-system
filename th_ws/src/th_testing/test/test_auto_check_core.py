"""test_auto_check_core.py — auto_check_core（起動時の自動点検の判定）のホスト単体テスト。

`auto_check_core` は ROS2 非依存の純粋関数なので、ホストの素の pytest だけで
回せる（test_opcheck_core.py と同じ流儀）。
"""
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "th_maintenance"))
from th_maintenance.auto_check_core import (  # noqa: E402
    AUTO_ITEMS,
    AutoItemVerdict,
    combine_overall,
    is_suppressed,
    judge_estop_auto,
    judge_imu_auto,
    judge_lidar_auto,
)
from th_maintenance.check_core import CheckParams  # noqa: E402


def make_params(**overrides):
    base = dict(
        motor_deadband_mps=0.03,
        motor_follow_min_ratio=0.5,
        imu_bias_max_rad_s=0.2,
        imu_wz_implausible_rad_s=10.0,
        opcheck_imu_window_s=2.0,
        opcheck_deadman_timeout_s=0.5,
        opcheck_spin_w_rad_s=0.3,
        opcheck_blind_tolerance_deg=5.0,
        opcheck_scan_coverage_gap_deg=2.0,
        v_check=0.05,
        scan_stale_ms=300.0,
    )
    base.update(overrides)
    return CheckParams(**base)


def flat_scan(n=360, value=5.0):
    return [value] * n


# ── ESTOP（今の状態だけ。押す・離すの確認は人の点検） ──

class TestEstopAuto:
    def test_released_and_alive_is_ok(self):
        v = judge_estop_auto(True, False)
        assert (v.result, v.reason) == ("OK", "")

    def test_pressed_is_ng(self):
        v = judge_estop_auto(True, True)
        assert v.result == "NG"
        assert v.reason == "pressed"

    def test_no_data_is_ng(self):
        v = judge_estop_auto(False, False)
        assert v.result == "NG"
        assert v.reason == "no_data"


# ── IMU（check_core の合成） ──

class TestImuAuto:
    def test_healthy_is_ok(self):
        v = judge_imu_auto(0xFF, 0.01, 0.05, True, make_params())
        assert v.result == "OK"

    def test_no_data_is_ng(self):
        v = judge_imu_auto(0xFF, 0.01, 0.05, False, make_params())
        assert v.result == "NG"
        assert v.reason == "no_data"

    def test_bias_too_large_is_ng(self):
        v = judge_imu_auto(0xFF, 0.5, 0.5, True, make_params())
        assert v.result == "NG"

    def test_needs_calibration_is_warn(self):
        # sys/gyro/accel/mag のいずれかが 3 未満 → WARN（故障扱いにしない）
        v = judge_imu_auto(0x00, 0.01, 0.05, True, make_params())
        assert v.result == "WARN"
        assert v.reason == "needs_calibration"

    def test_implausible_wz_is_ng(self):
        # dps のまま等の単位取り違え → NG
        v = judge_imu_auto(0xFF, 0.01, 57.3, True, make_params())
        assert v.result == "NG"
        assert v.reason == "wz_implausible"


# ── LIDAR（check_core の写し） ──

class TestLidarAuto:
    def test_healthy_is_ok(self):
        v = judge_lidar_auto(True, 0.1, flat_scan(), 1.0, [], make_params())
        assert v.result == "OK"

    def test_no_scan_is_ng(self):
        v = judge_lidar_auto(False, None, [], 1.0, [], make_params())
        assert v.result == "NG"
        assert v.reason == "no_data"

    def test_stale_period_is_ng(self):
        v = judge_lidar_auto(True, 1.0, flat_scan(), 1.0, [], make_params())
        assert v.result == "NG"
        assert v.reason == "scan_stale"

    def test_blind_mismatch_is_ng(self):
        # 近距離帯（死角）を推定できるスキャンに対し、設定が食い違う → NG
        ranges = [5.0] * 360
        for i in range(10, 20):
            ranges[i] = 0.3
        v = judge_lidar_auto(True, 0.1, ranges, 1.0, [100.0, 110.0], make_params())
        assert v.result == "NG"
        assert v.reason == "blind_mismatch"


# ── 総合 ──

class TestCombineOverall:
    def ok(self):
        return AutoItemVerdict("OK")

    def test_all_checking_is_checking(self):
        c = AutoItemVerdict("CHECKING")
        assert combine_overall(c, c, c).overall == "CHECKING"

    def test_all_ok_is_ok(self):
        assert combine_overall(self.ok(), self.ok(), self.ok()).overall == "OK"

    def test_one_ng_is_ng(self):
        o = combine_overall(self.ok(), AutoItemVerdict("NG", "no_data"), self.ok())
        assert o.overall == "NG"

    def test_warn_without_ng_is_warn(self):
        o = combine_overall(
            self.ok(), AutoItemVerdict("WARN", "needs_calibration"), self.ok())
        assert o.overall == "WARN"

    def test_partial_checking_is_checking(self):
        o = combine_overall(self.ok(), self.ok(), AutoItemVerdict("CHECKING"))
        assert o.overall == "CHECKING"

    def test_items_are_kept(self):
        o = combine_overall(self.ok(), self.ok(), self.ok())
        assert sorted(o.items.keys()) == ["ESTOP", "IMU", "LIDAR"]
        assert AUTO_ITEMS == ("ESTOP", "IMU", "LIDAR")


# ── 開発モードの抑制判定（dev_mode_core.hpp と同じ作法） ──

def dev_json(opcheck: bool, dev_mode: bool = True) -> str:
    return json.dumps({"dev_mode": dev_mode, "effective": {"opcheck": opcheck}})


class TestIsSuppressed:
    def test_effective_opcheck_suppresses(self):
        assert is_suppressed(dev_json(True), 1000.0, 900.0) is True

    def test_not_selected_does_not_suppress(self):
        assert is_suppressed(dev_json(False), 1000.0, 900.0) is False

    def test_master_off_does_not_suppress(self):
        assert is_suppressed(dev_json(True, dev_mode=False), 1000.0, 900.0) is False

    def test_unreceived_does_not_suppress(self):
        assert is_suppressed(None, 1000.0, None) is False
        assert is_suppressed(dev_json(True), 1000.0, None) is False

    def test_stale_does_not_suppress(self):
        # 3 秒超の古い effective でも消してはいけない（変異チェック②の標的）
        assert is_suppressed(dev_json(True), 5000.0, 1000.0) is False
        assert is_suppressed(dev_json(True), 4000.0, 1000.0) is False
        assert is_suppressed(dev_json(True), 3999.0, 1000.0) is True

    def test_broken_json_does_not_suppress(self):
        assert is_suppressed("not-json", 1000.0, 900.0) is False
        assert is_suppressed('{"dev_mode": true}', 1000.0, 900.0) is False
        assert is_suppressed('[1,2]', 1000.0, 900.0) is False
