"""test_opcheck_core.py — check_core（始業点検の判定ロジック）のホスト単体テスト。

`check_core` は ROS2 非依存の純粋関数なので、ホストの素の pytest だけで回せる
（`th_maintenance/check_core.py` の docstring 参照）。判定のしきい値は registry.yaml
（正本）の opcheck_runner 向け行と 1 対 1 で一致させること（test_maintenance_registry_driven
がその対応を機械的に検証する）。
"""
import math
import sys
from pathlib import Path

import pytest

_PARAMS_SRC = Path(__file__).resolve().parents[2] / "th_params"
REGISTRY_YAML = _PARAMS_SRC / "config" / "registry.yaml"

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "th_maintenance"))
from th_maintenance.check_core import (  # noqa: E402
    BLIND_MIN_RUN_DEG,
    CheckParams,
    NG,
    OK,
    WARN,
    blind_offset_deg,
    estimate_blind_sectors,
    judge_estop,
    judge_gyro_unit,
    judge_imu,
    judge_lidar,
    judge_motor,
    judge_motor_samples,
    pairs_from_flat,
    scan_coverage_gap_deg,
    unpack_calib,
    verdict_to_fsm_result,
)


def make_params(**overrides):
    """registry.yaml（WP-MAINT-01 ブロック）の値を既定値にした CheckParams を作る。"""
    base = dict(
        motor_deadband_mps=0.03,
        motor_follow_min_ratio=0.5,
        imu_bias_max_rad_s=0.2,
        imu_wz_implausible_rad_s=10.0,
        opcheck_imu_window_s=2.0,
        opcheck_deadman_timeout_s=0.5,
        opcheck_spin_w_rad_s=0.3,
        opcheck_blind_tolerance_deg=5.0,
        opcheck_scan_coverage_gap_deg=20.0,
        v_check=0.05,
        scan_stale_ms=300.0,
    )
    base.update(overrides)
    return CheckParams(**base)


# ── CheckVerdict / 写像 ─────────────────────────────────────────────────────

class TestVerdict:
    def test_verdict_values_and_reason(self):
        assert OK().result == "OK"
        assert NG("no_press").result == "NG"
        assert NG("no_press").reason == "no_press"
        assert WARN("needs_calibration").result == "WARN"
        assert WARN("needs_calibration").reason == "needs_calibration"

    def test_ok_has_empty_reason(self):
        assert OK().reason == ""

    def test_fsm_mapping_ok(self):
        assert verdict_to_fsm_result(OK()) == "OK"

    def test_fsm_mapping_ng_and_warn(self):
        assert verdict_to_fsm_result(NG("any")) == "NG"
        assert verdict_to_fsm_result(WARN("needs_calibration")) == "NG"


# ── 非常停止ボタン ──────────────────────────────────────────────────────────

class TestJudgeEStop:
    def test_ok_when_press_and_release_seen(self):
        assert judge_estop(True, True, True) == OK()

    def test_no_data_when_estop_not_alive(self):
        v = judge_estop(False, True, True)
        assert v.result == "NG" and v.reason == "no_data"

    def test_no_press(self):
        v = judge_estop(True, False, True)
        assert v.reason == "no_press"

    def test_stuck_release(self):
        v = judge_estop(True, True, False)
        assert v.reason == "stuck_release"


# ── モーター・エンコーダ ────────────────────────────────────────────────────

class TestJudgeMotor:
    def test_ok_when_follows_both(self):
        p = make_params()
        # 指令 ±v_check(0.05)、実測がよく追従
        assert judge_motor(-0.05, -0.045, 0.05, 0.048, p) == OK()

    def test_deadband_skip(self):
        p = make_params(motor_deadband_mps=0.03)
        # |指令| < deadband では符号不一致 / 低実測でも判定しない
        assert judge_motor(0.02, -0.05, -0.02, 0.0, p) == OK()

    def test_sign_mismatch_left(self):
        p = make_params()
        v = judge_motor(-0.05, 0.04, 0.05, 0.048, p)
        assert v.reason == "sign_mismatch_L"

    def test_sign_mismatch_right_direction_flip(self):
        p = make_params()
        v = judge_motor(0.05, 0.04, 0.05, -0.048, p)
        assert v.reason == "sign_mismatch_R"

    def test_zero_measured_means_mismatch(self):
        # 指令を出しているのに実測が 0（動かない）は sign_mismatch として NG
        p = make_params()
        assert judge_motor(-0.05, 0.0, 0.05, 0.048, p).reason == "sign_mismatch_L"

    def test_no_follow_below_ratio(self):
        p = make_params(motor_follow_min_ratio=0.5)
        # |実測| = 0.01 < 0.05 * 0.5 = 0.025 → NG
        v = judge_motor(-0.05, -0.01, 0.05, 0.048, p)
        assert v.reason == "no_follow_L"

    def test_no_follow_right(self):
        p = make_params()
        v = judge_motor(0.05, 0.048, 0.05, 0.01, p)
        assert v.reason == "no_follow_R"

    def test_marginal_follow_passes(self):
        p = make_params(motor_follow_min_ratio=0.5)
        # 境界 ±ε は NG ではない（< で比較）
        assert judge_motor(-0.05, -0.025, 0.05, 0.025, p) == OK()


class TestJudgeMotorSamples:
    def test_no_samples(self):
        assert judge_motor_samples([], make_params()).reason == "no_samples"

    def test_any_sign_mismatch_fails(self):
        p = make_params()
        # 追従したあとで逆向きに動いた（立ち上がりの遅れではない）
        samples = [(-0.05, -0.048, 0.05, 0.047), (-0.05, 0.048, 0.05, 0.047)]
        v = judge_motor_samples(samples, p)
        assert v.result == "NG" and v.reason == "sign_mismatch_L"

    def test_returns_ng_for_follow_failure_on_peak(self):
        p = make_params(motor_follow_min_ratio=0.5)
        # 最良サンプル（左右実測絶対値和が最大）自体が R 追従不足 → NG。
        # s1 の L 追従不足は「非 peak」なので無視される（立ち上がり中の一時的な
        # 低実測で誤 NG しない設計）。
        samples = [
            (-0.05, -0.02, 0.05, 0.048),  # peak（sum=0.068）: L が no_follow
            (-0.05, -0.049, 0.05, 0.001),  # sum=0.050: R が no_follow
        ]
        v = judge_motor_samples(samples, p)
        assert v.result == "NG"
        assert v.reason == "no_follow_L"

    def test_all_following_passes(self):
        p = make_params()
        samples = [
            (-0.05, -0.04, 0.05, 0.041),
            (-0.05, -0.049, 0.05, 0.05),
            (0.0, 0.0, 0.0, 0.0),
        ]
        assert judge_motor_samples(samples, p) == OK()

    def test_deadband_row_never_ng(self):
        p = make_params()
        # 指令 0 の行は非ゼロ実測でも判定しない（指令が届いた行があれば OK）
        samples = [(0.0, 0.5, 0.0, -0.5), (-0.05, -0.049, 0.05, 0.05)]
        assert judge_motor_samples(samples, p) == OK()

    def test_command_never_reached_is_ng(self):
        # 押している間ずっと指令が 0（安全装置が止めていて ESP32 まで届かない）。
        # 判定しない行だけで OK にしてはいけない。
        p = make_params()
        samples = [(0.0, 0.0, 0.0, 0.0)] * 30
        v = judge_motor_samples(samples, p)
        assert v.result == "NG" and v.reason == "no_command"

    def test_command_below_deadband_is_ng(self):
        p = make_params(motor_deadband_mps=0.03)
        samples = [(0.02, 0.02, -0.02, -0.02)] * 5
        assert judge_motor_samples(samples, p).reason == "no_command"


class TestJudgeMotorSamplesWithLag:
    """実機（2026-10-10 T-8）: 指令に対し実測が数百 ms 遅れて追従する機体で OK になること。

    以前は押下直後の「指令は出たが実測がまだ 0」のサンプルを符号不一致
    （`cmd * meas <= 0`）と見て、正常な機体を毎回 sign_mismatch で NG にしていた。
    """

    @staticmethod
    def _press(cmd_l, cmd_r, lag_samples=12, hold_samples=60, tau=4.0):
        """20Hz。指令は 1 サンプル目から階段、実測は lag_samples 個 0 のあと一次遅れで追従。"""
        out = []
        ml = mr = 0.0
        for i in range(hold_samples):
            if i >= lag_samples:
                ml += (cmd_l - ml) / tau
                mr += (cmd_r - mr) / tau
            out.append((cmd_l, ml, cmd_r, mr))
        return out

    @pytest.mark.parametrize("name,cl,cr", [
        ("FORWARD", 0.05, 0.05),
        ("BACK", -0.05, -0.05),
        ("LEFT", -0.06, 0.06),
        ("RIGHT", 0.06, -0.06),
    ])
    @pytest.mark.parametrize("lag", [0, 6, 12, 20])
    def test_lagging_healthy_machine_is_ok(self, name, cl, cr, lag):
        v = judge_motor_samples(self._press(cl, cr, lag_samples=lag), make_params())
        assert v == OK(), f"{name} lag={lag}: {v}"

    def test_lag_with_previous_residue_is_ok(self):
        # 前の押下の残り（逆符号の小さな実測）が先頭に残っていても追従すれば OK
        p = make_params()
        samples = [(-0.06, 0.01, 0.06, -0.01)] * 3 + self._press(-0.06, 0.06, 8)
        assert judge_motor_samples(samples, p) == OK()

    def test_truly_reversed_wheel_still_ng(self):
        # 左輪が逆に回る（極性逆）は、遅れがあっても NG
        samples = self._press(-0.06, 0.06, 8)
        samples = [(cl, -ml, cr, mr) for cl, ml, cr, mr in samples]
        v = judge_motor_samples(samples, make_params())
        assert v.result == "NG" and v.reason == "sign_mismatch_L"

    def test_never_moves_is_ng(self):
        samples = [(-0.06, 0.0, 0.06, 0.0)] * 60
        v = judge_motor_samples(samples, make_params())
        assert v.result == "NG" and v.reason in ("sign_mismatch_L", "no_follow_L")

    def test_only_one_wheel_moves_is_ng(self):
        samples = [(0.05, 0.05, 0.05, 0.0)] * 60
        assert judge_motor_samples(samples, make_params()).result == "NG"


# ── IMU ─────────────────────────────────────────────────────────────────────

class TestUnpackCalib:
    def test_all_zero(self):
        assert unpack_calib(0x00) == (0, 0, 0, 0)

    def test_all_three(self):
        assert unpack_calib(0xFF) == (3, 3, 3, 3)

    def test_bit_assignment(self):
        # 0b01_11_01_10 → sys=01, gyro=11, accel=01, mag=10
        assert unpack_calib(0b01110110) == (1, 3, 1, 2)

    def test_each_field_isolated(self):
        assert unpack_calib(0x04) == (0, 0, 1, 0)   # accel のみ
        assert unpack_calib(0x10) == (0, 1, 0, 0)   # gyro のみ
        assert unpack_calib(0x40) == (1, 0, 0, 0)   # sys のみ


class TestJudgeImu:
    def test_ok_when_calibrated_and_small_bias(self):
        p = make_params()
        assert judge_imu(0xFF, 0.02, True, p) == OK()

    def test_no_data(self):
        p = make_params()
        v = judge_imu(0xFF, 0.02, False, p)
        assert v.reason == "no_data"

    def test_bias_too_large(self):
        p = make_params(imu_bias_max_rad_s=0.2)
        v = judge_imu(0xFF, 0.35, True, p)
        assert v.reason == "bias_too_large"

    def test_bias_at_threshold_passes(self):
        p = make_params(imu_bias_max_rad_s=0.2)
        assert judge_imu(0xFF, 0.2, True, p) == OK()

    def test_needs_calibration_is_warn(self):
        p = make_params()
        v = judge_imu(0b11_01_11_11, 0.02, True, p)  # accel のみ未校正
        assert v.result == "WARN" and v.reason == "needs_calibration"

    def test_accel_uncalibrated_still_warns(self):
        # 磁気コンパス未使用でも、慣性系として使う accel の未校正は WARN 対象
        p = make_params()
        v = judge_imu(0b00_11_01_10, 0.02, True, p)
        assert v.reason == "needs_calibration"

    def test_bias_priority_over_calib(self):
        # 校正済みでもバイアス大は NG（人に直してもらう故障）
        p = make_params()
        assert judge_imu(0xFF, 3.0, True, p).reason == "bias_too_large"


class TestJudgeGyroUnit:
    def test_ok(self):
        assert judge_gyro_unit(0.4, make_params()) == OK()
        assert judge_gyro_unit(-0.4, make_params()) == OK()

    def test_implausible(self):
        # 10 deg/s 誤配線（dps のまま）は 10 rad/s 近辺の値を出す
        v = judge_gyro_unit(10.1, make_params())
        assert v.reason == "wz_implausible"

    def test_near_threshold_passes(self):
        assert judge_gyro_unit(-10.0, make_params()) == OK()

    def test_no_samples_warn(self):
        v = judge_gyro_unit(None, make_params())
        assert v.result == "WARN" and v.reason == "no_samples"


# ── LiDAR ───────────────────────────────────────────────────────────────────

class TestPairsFromFlat:
    def test_even_pairs(self):
        assert pairs_from_flat([10.0, 20.0, 350.0, 10.0]) == [(10.0, 20.0), (350.0, 10.0)]

    def test_odd_tail_dropped(self):
        assert pairs_from_flat([10.0, 20.0, 30.0]) == [(10.0, 20.0)]

    def test_empty(self):
        assert pairs_from_flat([]) == []


def _scan_array(valid_mask, inc=0.25):
    """valid_mask の True を有効（1.0m）、False を無効（inf）にした角度配列を返す。"""
    return [1.0 if v else math.inf for v in valid_mask]


class TestScanCoverageGap:
    def test_all_valid(self):
        assert scan_coverage_gap_deg([1.0] * 100, 3.6) == 0.0

    def test_single_gap(self):
        # 4 ビーム連続 invalid × 5deg = 20deg
        mask = [True] * 90 + [False] * 4 + [True] * 6
        assert scan_coverage_gap_deg(_scan_array(mask, inc=5.0), 5.0) == pytest.approx(20.0)

    def test_wrap_around_gap_counted(self):
        # 末尾 3 + 先頭 2 = 5 ビームが連続（巡回で 25deg）
        mask = [False] * 2 + [True] * 8 + [False] * 3
        assert scan_coverage_gap_deg(_scan_array(mask, inc=5.0), 5.0) == pytest.approx(25.0)

    def test_all_invalid(self):
        assert scan_coverage_gap_deg([math.inf] * 10, 5.0) == 360.0

    def test_non_finite_variants(self):
        arr = [1.0, 0.0, float("nan"), -1.0, 1.0]  # 0 / NaN / 負も無効
        assert scan_coverage_gap_deg(arr, 1.0) == pytest.approx(3.0)
        arr = [1.0, 0.0, float("nan"), -1.0, 1.0]  # 0 / NaN / 負も無効
        assert scan_coverage_gap_deg(arr, 1.0) == pytest.approx(3.0)

    def test_empty(self):
        assert scan_coverage_gap_deg([], 5.0) == 360.0


def _near_arr(mask, inc=0.25, near=0.2):
    return [near if v else 3.0 for v in mask]


def _real_band_scan(bands_real, angle_min_deg=-180.0, inc=1.0, n=360,
                    near=0.2, far=5.0):
    """実角度の帯 `[a0, a1)` に近距離を置いたスキャン配列を返す。

    ビーム i の実角度は `angle_min + i * inc`。帯をまたぐ添字（負・n 超え）
    は全周で折り返す。
    """
    ranges = [far] * n
    for a0, a1 in bands_real:
        i0 = int(round((a0 - angle_min_deg) / inc))
        i1 = int(round((a1 - angle_min_deg) / inc))
        for i in range(i0, i1):
            ranges[i % n] = near
    return ranges


class TestEstimateBlindSectors:
    def test_no_near_points(self):
        assert estimate_blind_sectors([3.0] * 40, 9.0) == []

    def test_all_near(self):
        assert estimate_blind_sectors([0.2] * 10, 36.0) == [(0.0, 360.0)]

    def test_one_sector(self):
        inc = 3.0
        mask = [False] * 20 + [True] * 10 + [False] * 10  # 30deg の死角
        sectors = estimate_blind_sectors(_near_arr(mask, inc), inc)
        assert len(sectors) == 1
        start, end = sectors[0]
        assert start == pytest.approx(60.0)
        assert end == pytest.approx(90.0)

    def test_wrap_around_split_into_two(self):
        # 近距離帯が角度 0（配列末尾→先頭）をまたぐ形。False 帯 20..65deg の反対側、
        # すなわち 70..90(末尾) と 0..20(先頭) の 2 帯に分解して返す
        inc = 5.0
        mask = [True] * 4 + [False] * 10 + [True] * 4
        sectors = estimate_blind_sectors(_near_arr(mask, inc), inc)
        assert len(sectors) == 2
        assert sectors[0] == pytest.approx((70.0, 90.0))
        assert sectors[1] == pytest.approx((0.0, 20.0))

    def test_run_below_min_width_discarded(self):
        # 2deg の帯 ≈ 0.2m の細い柱 → ノイズとして捨てる
        inc = 1.0
        mask = [True] * 2 + [False] * 8
        assert estimate_blind_sectors(_near_arr(mask, inc), inc) == []

    def test_min_run_deg_boundary_included(self):
        inc = BLIND_MIN_RUN_DEG  # 幅がちょうど min_run なら採用
        mask = [True] * 1 + [False] * 3
        assert len(estimate_blind_sectors(_near_arr(mask, inc), inc)) == 1

    def test_empty_ranges(self):
        assert estimate_blind_sectors([], 0.25) == []

    def test_zero_and_inf_are_not_near(self):
        # 無効値（inf）・0 は死角（近距離有効値）ではない。5 本 (0..5deg) の
        # 近距離帯だけが残る
        arr = [0.2] * 5 + [math.inf] + [0.0] + [3.0] * 3
        assert estimate_blind_sectors(arr, 1.0) == [(0.0, 5.0)]


class TestBlindOffsetDeg:
    _configured = [0.0, 20.0, 340.0, 360.0]

    def test_match(self):
        est = [(0.0, 20.0), (340.0, 360.0)]
        assert blind_offset_deg(est, self._configured) == pytest.approx(0.0)

    def test_shift(self):
        # 車体の旋回で死角が 3deg ずれている
        est = [(3.0, 23.0), (343.0, 363.0)]
        assert blind_offset_deg(est, self._configured) == pytest.approx(3.0)

    def test_only_configured_side_empty(self):
        assert blind_offset_deg([(10.0, 20.0)], []) is None

    def test_only_estimated_side_empty(self):
        assert blind_offset_deg([], self._configured) is None


class TestJudgeLidar:
    def test_ok(self):
        p = make_params()
        ranges = [1.0] * 1440
        assert judge_lidar(True, 0.2, ranges, 0.25, [], p, angle_min_deg=0.0) == OK()

    def test_no_data(self):
        p = make_params()
        v = judge_lidar(False, None, [], 0.25, [], p, angle_min_deg=0.0)
        assert v.reason == "no_data"

    def test_stale_period(self):
        p = make_params(scan_stale_ms=300.0)
        v = judge_lidar(True, 0.4, [1.0] * 10, 1.0, [], p, angle_min_deg=0.0)
        assert v.reason == "scan_stale"

    def test_period_at_threshold_ok(self):
        p = make_params(scan_stale_ms=300.0)
        assert judge_lidar(True, 0.3, [1.0] * 10, 1.0, [], p, angle_min_deg=0.0) == OK()

    def test_period_none_skips_stale_check(self):
        p = make_params()
        assert judge_lidar(True, None, [1.0] * 10, 1.0, [], p, angle_min_deg=0.0) == OK()

    def test_coverage_gap(self):
        p = make_params(opcheck_scan_coverage_gap_deg=2.0)
        mask = [True] * 8 + [False] * 1 + [True] * 1  # 3deg の欠損 > 2deg
        v = judge_lidar(True, 0.2, _scan_array(mask, inc=3.0), 3.0, [], p,
                        angle_min_deg=0.0)
        assert v.reason == "coverage_gap"

    def test_coverage_under_threshold_ok(self):
        p = make_params(opcheck_scan_coverage_gap_deg=2.0)
        mask = [True] * 8 + [False] * 1 + [True] * 1  # 1deg の欠損 < 2deg
        assert judge_lidar(True, 0.2, _scan_array(mask, inc=1.0), 1.0, [], p,
                           angle_min_deg=0.0) == OK()

    def test_blind_mismatch(self):
        p = make_params(opcheck_blind_tolerance_deg=5.0)
        inc = 5.0
        # 死角は実測 25..45deg（単一帯）。設定が 0..20 で 25deg ズレ → NG
        mask = [False] * 5 + [True] * 4 + [False] * 5
        v = judge_lidar(True, 0.2, _near_arr(mask, inc, near=0.2), inc, [0.0, 20.0], p,
                        angle_min_deg=0.0)
        assert v.reason == "blind_mismatch"

    def test_blind_within_tolerance_ok(self):
        p = make_params(opcheck_blind_tolerance_deg=5.0)
        inc = 5.0
        mask = [False] * 5 + [True] * 4 + [False] * 5  # 単一帯 25..45deg
        config = [25.0, 45.0]
        assert judge_lidar(True, 0.2, _near_arr(mask, inc, near=0.2), inc, config, p,
                           angle_min_deg=0.0) == OK()

    def test_no_blind_map_never_ng(self):
        # blind_angle_ranges が空（未校正）なら盲点は比較しない
        p = make_params()
        inc = 5.0
        mask = [True] * 5 + [False] * 5 + [True] * 5
        assert judge_lidar(True, 0.2, _near_arr(mask, inc, near=0.2), inc, [], p,
                           angle_min_deg=0.0) == OK()

    def test_stale_beats_gap_order(self):
        # 周期異常と欠損は周期を先に判定
        p = make_params()
        v = judge_lidar(True, 9.9, [], 0.25, [], p, angle_min_deg=0.0)
        assert v.reason == "scan_stale"


class TestJudgeLidarAngleMin:
    """`angle_min != 0` のスキャンでも実角度で比べること（実機は -π）。

    回帰の由来: `estimate_blind_sectors` は添字 × 刻み＝0..360° で帯を返す
    ため、`angle_min` を見ずに比べると実機（`angle_min=-π`）では推定帯が
    180° ずれた座標で比較されていた。
    """

    def test_match_at_minus_pi_is_ok(self):
        # 実角度 10..20° の帯が設定どおり → OK（直さないと 180° ずれて NG）
        p = make_params(opcheck_blind_tolerance_deg=5.0)
        ranges = _real_band_scan([(10.0, 20.0)], angle_min_deg=-180.0)
        v = judge_lidar(True, 0.1, ranges, 1.0, [10.0, 20.0], p,
                        angle_min_deg=-180.0)
        assert v == OK()

    def test_shifted_10deg_at_minus_pi_is_ng(self):
        # 実角度 10..20° に対し設定が 10° ずれて 20..30° → 許容 5° 超で NG
        p = make_params(opcheck_blind_tolerance_deg=5.0)
        ranges = _real_band_scan([(10.0, 20.0)], angle_min_deg=-180.0)
        v = judge_lidar(True, 0.1, ranges, 1.0, [20.0, 30.0], p,
                        angle_min_deg=-180.0)
        assert v.reason == "blind_mismatch"

    def test_zero_angle_min_unchanged(self):
        # angle_min=0 では添字空間＝実角度。従来どおり OK
        p = make_params(opcheck_blind_tolerance_deg=5.0)
        ranges = _real_band_scan([(190.0, 200.0)], angle_min_deg=0.0)
        v = judge_lidar(True, 0.1, ranges, 1.0, [190.0, 200.0], p,
                        angle_min_deg=0.0)
        assert v == OK()

    def test_band_across_seam_is_merged(self):
        # 継ぎ目（angle_min＝-180°）をまたぐ実角度 -185..-175° の帯。
        # 推定は添字 0 で 2 つに割れるが、シフト後に 1 本へ戻して比べる。
        # 人工的な境界（±180°）が残ると許容 2° に対して 5° のズレに見える。
        p = make_params(opcheck_blind_tolerance_deg=2.0)
        ranges = _real_band_scan([(-185.0, -175.0)], angle_min_deg=-180.0)
        v = judge_lidar(True, 0.1, ranges, 1.0, [-185.0, -175.0], p,
                        angle_min_deg=-180.0)
        assert v == OK()

# ── 実機の形のスキャン（2026-10-13）と判定の根拠 ──────────────────────────────

import json  # noqa: E402
import math  # noqa: E402

from th_maintenance.check_core import basis_to_json  # noqa: E402

# 実機 2026-10-13 の生スキャン: 720 ビーム・angle_min = -3.124 rad（≒ -179°）・
# 刻み 0.5°。正常でも約 -32° に無効（∞）が 8° 連続する。
_REAL_N = 720
_REAL_INC_DEG = 0.5
_REAL_ANGLE_MIN_DEG = math.degrees(-3.124)


def _real_scan(gap_deg: float, start_idx: int = 295) -> list[float]:
    ranges = [4.0] * _REAL_N
    for i in range(start_idx, start_idx + int(round(gap_deg / _REAL_INC_DEG))):
        ranges[i] = float("inf")
    return ranges


def _basis_by_key(verdict):
    return {b["key"]: b for b in verdict.basis}


class TestJudgeLidarRealScan:
    """registry の既定（無効帯の許容 20°）で、実機の形のスキャンを判定する。"""

    def _p(self):
        return make_params(opcheck_scan_coverage_gap_deg=20.0)

    def test_8deg_invalid_band_is_ok(self):
        v = judge_lidar(True, 0.1, _real_scan(8.0), _REAL_INC_DEG, [], self._p(),
                        angle_min_deg=_REAL_ANGLE_MIN_DEG)
        assert v.result == "OK", v
        b = _basis_by_key(v)["coverage_gap_deg"]
        assert b["value"] == pytest.approx(8.0)
        assert b["threshold"] == 20.0 and b["ok"] is True

    def test_25deg_invalid_band_is_coverage_gap(self):
        v = judge_lidar(True, 0.1, _real_scan(25.0), _REAL_INC_DEG, [], self._p(),
                        angle_min_deg=_REAL_ANGLE_MIN_DEG)
        assert (v.result, v.reason) == ("NG", "coverage_gap")
        b = _basis_by_key(v)["coverage_gap_deg"]
        assert b["value"] == pytest.approx(25.0) and b["ok"] is False

    def test_boundary_just_under_and_at_threshold(self):
        # 19.5° は合格・20.0° ちょうどは「20° 未満」を満たさないので NG
        ok = judge_lidar(True, 0.1, _real_scan(19.5), _REAL_INC_DEG, [], self._p(),
                         angle_min_deg=_REAL_ANGLE_MIN_DEG)
        ng = judge_lidar(True, 0.1, _real_scan(20.0), _REAL_INC_DEG, [], self._p(),
                         angle_min_deg=_REAL_ANGLE_MIN_DEG)
        assert ok.result == "OK"
        assert (ng.result, ng.reason) == ("NG", "coverage_gap")

    def test_registry_default_is_20(self):
        # 値を 2.0 に戻す変異で赤にする（registry の値そのものを縛る）
        import yaml
        from pathlib import Path
        reg = Path(__file__).resolve().parents[2] / "th_params" / "config" / "registry.yaml"
        rows = yaml.safe_load(reg.read_text(encoding="utf-8"))
        rows = rows["params"] if isinstance(rows, dict) and "params" in rows else rows
        row = next(r for r in rows if r.get("name") == "opcheck_scan_coverage_gap_deg")
        assert row["value"] == 20.0


class TestJudgmentBasis:
    def test_lidar_basis_has_three_items_with_threshold(self):
        p = make_params(opcheck_scan_coverage_gap_deg=20.0, scan_stale_ms=2000.0,
                        opcheck_blind_tolerance_deg=5.0)
        v = judge_lidar(True, 0.1, _real_scan(8.0), _REAL_INC_DEG, [], p,
                        angle_min_deg=_REAL_ANGLE_MIN_DEG)
        by = _basis_by_key(v)
        assert set(by) == {"coverage_gap_deg", "scan_period_s", "blind_offset_deg"}
        assert by["scan_period_s"]["value"] == pytest.approx(0.1)
        assert by["scan_period_s"]["threshold"] == pytest.approx(2.0)
        assert by["blind_offset_deg"]["value"] is None  # 設定が空＝比較不能
        assert by["blind_offset_deg"]["threshold"] == 5.0
        assert all(b["ok"] for b in v.basis)

    def test_lidar_stale_marks_only_period_not_ok(self):
        p = make_params(opcheck_scan_coverage_gap_deg=20.0, scan_stale_ms=300.0)
        v = judge_lidar(True, 0.5, _real_scan(8.0), _REAL_INC_DEG, [], p,
                        angle_min_deg=_REAL_ANGLE_MIN_DEG)
        assert v.reason == "scan_stale"
        bad = [b["key"] for b in v.basis if not b["ok"]]
        assert bad == ["scan_period_s"]

    def test_lidar_no_data_basis(self):
        v = judge_lidar(False, None, [], 0.25, [], make_params(), angle_min_deg=0.0)
        assert [(b["key"], b["ok"]) for b in v.basis] == [("scan_alive", False)]

    def test_imu_needs_calibration_keeps_calib_state_in_basis(self):
        # 実機の WARN(needs_calibration) bias≈0.000 maxwz≈0.002
        p = make_params()
        v = judge_imu(0b11_11_10_00, 0.0004, True, p)  # sys3 gyro3 accel2 mag0
        assert (v.result, v.reason) == ("WARN", "needs_calibration")
        by = _basis_by_key(v)
        assert by["calib_sys"]["value"] == 3 and by["calib_sys"]["ok"] is True
        assert by["calib_accel"]["value"] == 2 and by["calib_accel"]["ok"] is False
        assert by["calib_mag"]["value"] == 0 and by["calib_mag"]["ok"] is False
        assert by["imu_bias"]["ok"] is True and by["imu_bias"]["threshold"] == 0.2

    def test_imu_bias_too_large_marks_bias(self):
        v = judge_imu(0xFF, 0.5, True, make_params())
        assert v.reason == "bias_too_large"
        assert [b["key"] for b in v.basis if not b["ok"]] == ["imu_bias"]

    def test_gyro_unit_basis(self):
        v = judge_gyro_unit(0.002, make_params())
        assert _basis_by_key(v)["imu_max_wz"]["value"] == pytest.approx(0.002)
        bad = judge_gyro_unit(15.0, make_params())
        assert bad.reason == "wz_implausible"
        assert _basis_by_key(bad)["imu_max_wz"]["ok"] is False
        none = judge_gyro_unit(None, make_params())
        assert _basis_by_key(none)["imu_max_wz"]["value"] is None

    def test_basis_does_not_affect_equality(self):
        assert judge_imu(0xFF, 0.0, True, make_params()) == OK()

    def test_basis_to_json_roundtrip_and_empty(self):
        v = judge_imu(0xFF, 0.0, True, make_params())
        parsed = json.loads(basis_to_json(v.basis))
        assert parsed[0].keys() >= {"key", "value", "threshold", "unit", "cmp", "ok"}
        assert basis_to_json(()) == ""
        assert basis_to_json(OK().basis) == ""
