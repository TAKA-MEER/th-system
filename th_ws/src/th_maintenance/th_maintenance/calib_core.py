"""calib_core.py — 校正（CALIB）の補正値の計算と合否判定（純粋。ROS2・ファイル非依存）。

`DetailedDesign-maintenance.md` §3.4・§3.5・§4 の写実。
`scripts/calib_runner.py` はこのモジュールを import して計算・判定に使う。
引数だけで完結する（ホストの素の pytest で直接走る。`check_core.py` と同じ作法）。

要点:
  - 直進（LINEAR）の適用先は車輪半径そのものではなく `wheel_radius_scale`（§4.2）。
    `k_new = k_current × (measured / commanded)`。
  - 旋回（ROTATION）の適用先は `wheel_base`。`base_new = base_current × (commanded / measured)`
    （設計書 §3.4 の式と**逆数**。理由は `corrected_wheel_base` の docstring）。
  - `sanity()` を通らない値は S3（確認）から S4（適用）へ進めない（T-CAL-04 の
    ガード `preview_sane`）。LINEAR はさらに **A10 を超える値を適用しない**。
  - 検証（S4）で測った誤差が許容範囲を超えたら適用前に戻す（T-CAL-06）。
    許容範囲が未確定（`status: placeholder`）のときは**合格にしない**（安全側）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# 項目名（`/calib/start` の item）。BLIND の純粋関数は `blind_core.py`。
ITEMS = ("LINEAR", "ROTATION", "IMU", "BLIND")

# sanity の範囲。補正係数（測定／指令）が 0.5〜2.0 を外れたら桁違い・取り違えとみなす。
# A10（|k−1| ≤ 0.10）よりずっと緩い「明らかな入力ミス」の弾き。
RATIO_LO = 0.5
RATIO_HI = 2.0


@dataclass(frozen=True)
class Verdict:
    ok: bool
    reason: str = ""


def sanity(value: float, lo: float, hi: float) -> bool:
    """ゼロ割れ・桁違いを弾く。Step 3 のプレビュー前に必ず通す。"""
    return math.isfinite(value) and lo <= value <= hi


def a10_ok(scale: float, max_dev: float) -> bool:
    """A10: |wheel_radius_scale − 1| ≤ max_dev。NaN・負・0 は不合格（abs の性質）。

    esp32_bridge 側にも同じ検査がある（二重化）。ここで先に弾くのは、
    プレビューの段階で「適用できない値」を人に見せて進ませないため。
    境界ちょうどは浮動小数の丸めで落とさない（1e-9 の許容。wheel_scale_core と同じ扱い）。
    """
    if not math.isfinite(scale) or not math.isfinite(max_dev):
        return False
    return abs(scale - 1.0) <= max_dev + 1e-9


def corrected_wheel_radius(nominal_r: float, commanded_m: float, measured_m: float) -> float:
    """走った距離が短ければ半径が過大に見積もられている（設計書 §3.4 の式）。"""
    return nominal_r * (measured_m / commanded_m)


def corrected_wheel_radius_scale(current_scale: float, commanded_m: float,
                                  measured_m: float) -> float:
    """直進校正の出力 `wheel_radius_scale`（§4.2）。

    ロボットは「現在のスケールを掛けたオドメトリ」で commanded_m を走ったつもりで止まる。
    実際に進んだのが measured_m なら、現在のスケールを measured/commanded 倍する。
    入力が不正（0 以下・非有限）なら NaN（`ratio_sane` が不合格にする）。
    """
    if not (math.isfinite(commanded_m) and math.isfinite(measured_m)
            and math.isfinite(current_scale)) or commanded_m <= 0.0 or measured_m <= 0.0:
        return math.nan
    return current_scale * (measured_m / commanded_m)


def corrected_wheel_base(current_base: float, commanded_deg: float, measured_deg: float) -> float:
    """旋回校正の出力 `wheel_base`。**設計書 §3.4 の式（current × measured/commanded）とは逆数**。

    ロボットは「エンコーダから計算したオドメトリ角」が commanded に達したら止まる。
    オドメトリ角 = 車輪の弧長差 / base_param、実際の回転角 = 弧長差 / base_real なので
    `measured / commanded = base_param / base_real`。よって
    `base_real = base_param × commanded / measured`。回りすぎた（measured > commanded）なら
    base_param が**過大**で、小さくする。設計書の式と既存の `th_calibration/rotation_calib.py`
    の補正は逆向き（式のコメント自身の等式 `実走行距離 = base_real × 実測角 = base_param × odom角`
    から導くと逆数になる）。`test_calib_core.py` の物理シミュレーションで収束を確かめてある。
    不正入力は NaN。
    """
    if not (math.isfinite(commanded_deg) and math.isfinite(measured_deg)
            and math.isfinite(current_base)) or commanded_deg <= 0.0 or measured_deg <= 0.0:
        return math.nan
    return current_base * (commanded_deg / measured_deg)


def ratio_sane(measured: float, commanded: float) -> bool:
    """測定／指令の比が桁違いでないか（S3 のプレビュー前の入力検査）。"""
    if not (math.isfinite(measured) and math.isfinite(commanded)) or commanded <= 0.0:
        return False
    return sanity(measured / commanded, RATIO_LO, RATIO_HI)


def linear_preview_sane(commanded_m: float, measured_m: float, new_scale: float,
                        max_dev: float) -> bool:
    """LINEAR のプレビューが S4 へ進めるか。比の sanity ＋ **A10**（超える値は適用しない）。"""
    return ratio_sane(measured_m, commanded_m) and a10_ok(new_scale, max_dev)


def rotation_preview_sane(commanded_deg: float, measured_deg: float, new_base: float,
                          current_base: float) -> bool:
    """ROTATION のプレビューが S4 へ進めるか。比の sanity ＋ 新しい車輪間距離が桁違いでない。"""
    if not ratio_sane(measured_deg, commanded_deg):
        return False
    return sanity(new_base, current_base * RATIO_LO, current_base * RATIO_HI)


def _tolerance_defined(tol: Optional[float]) -> bool:
    return tol is not None and math.isfinite(tol) and tol > 0.0


def verify_linear(commanded_m: float, measured_m: float,
                  tolerance_ratio: Optional[float]) -> Verdict:
    """検証走行の誤差（指定距離に対する相対誤差）が許容範囲か（`calib_linear_tolerance_ratio`）。"""
    if not _tolerance_defined(tolerance_ratio):
        return Verdict(False, "tolerance_undefined")
    if not (math.isfinite(commanded_m) and math.isfinite(measured_m)) or commanded_m <= 0.0 \
            or measured_m <= 0.0:
        return Verdict(False, "invalid_measurement")
    err = abs(measured_m - commanded_m) / commanded_m
    if err > tolerance_ratio:
        return Verdict(False, f"error_exceeds_tolerance:{err:.4f}>{tolerance_ratio:.4f}")
    return Verdict(True, f"error:{err:.4f}")


def verify_rotation(commanded_deg: float, measured_deg: float,
                    tolerance_deg: Optional[float]) -> Verdict:
    """検証旋回の誤差（絶対誤差 deg）が許容範囲か（`calib_rotation_tolerance_deg`）。"""
    if not _tolerance_defined(tolerance_deg):
        return Verdict(False, "tolerance_undefined")
    if not (math.isfinite(commanded_deg) and math.isfinite(measured_deg)) \
            or commanded_deg <= 0.0 or measured_deg <= 0.0:
        return Verdict(False, "invalid_measurement")
    err = abs(measured_deg - commanded_deg)
    if err > tolerance_deg:
        return Verdict(False, f"error_exceeds_tolerance:{err:.2f}>{tolerance_deg:.2f}")
    return Verdict(True, f"error:{err:.2f}")


def imu_all_calibrated(calib_status: int) -> bool:
    """BNO055 の sys/gyro/accel/mag が全て 3（完全）か。`/esp32/imu_calib_status` の値そのもの。"""
    return all(((calib_status >> shift) & 0x03) == 3 for shift in (0, 2, 4, 6))
