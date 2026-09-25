"""check_core.py — 始業点検（OPCHECK）の合否判定（純粋。ROS2 非依存）。

`DetailedDesign-maintenance.md` §2 と `Spec-checks.md` §2.4 の写実。
`scripts/opcheck_runner.py` はこのモジュールを import して判定に使う。

設計方針（この関数群が守ること）:
  - ファイル・環境・ROS2 を一切触らない。引数だけで完結する（P-1 と同じ不変条件。
    ユニットテストがホストの素の pytest で直接走れるようにするため）。
  - 判定は全て CheckVerdict（OK / WARN / NG）で返す。`verdict_to_fsm_result` が
    FSM のガード語彙（OK / NG）へ写像する（WARN は NG 扱い。FSM は OK/NG しか
    受け付けない。詳細は CheckStatus.detail に残す）。
  - 判定のしきい値は全て `CheckParams`（=
    registry.yaml の `opcheck_runner` 向け生成 YAML の中身）から取る。ハードコード
    しない。例外的に `near_m`（近距離帯の判定距離）だけは module 定数にする
    （`scripts/measure_blind_sectors.py` の `NEAR_M` と同じ値。registry に持つ
    までもない校正時の内部定数）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

# 近距離帯（死角として写る支柱等）の判定距離。measure_blind_sectors.py の NEAR_M と同値。
BLIND_NEAR_M = 0.6
# 死角帯として扱う最小幅（measure_blind_sectors.py の MARGIN に相当する
# ノイズ除去用の下限）。
BLIND_MIN_RUN_DEG = 5.0


@dataclass(frozen=True)
class CheckVerdict:
    """1 項目の合否。`result` は "OK" / "WARN" / "NG" の 3 値。"""

    result: str
    reason: str = ""


def OK() -> CheckVerdict:
    return CheckVerdict("OK")


def WARN(reason: str) -> CheckVerdict:
    return CheckVerdict("WARN", reason)


def NG(reason: str) -> CheckVerdict:
    return CheckVerdict("NG", reason)


def verdict_to_fsm_result(verdict: CheckVerdict) -> str:
    """CheckVerdict を FSM のガード語彙（2 値: "OK" / "NG"）へ写像する。

    WARN は NG として扱う（`evt.check_result` は OK/NG しか遷移条件に持たない
    [DetailedDesign-state.md]。WARN の内容は CheckStatus.detail に reason を残し、
    画面側が「校正に誘導する NG」と「故障の NG」を区別できるようにする）。
    """
    if verdict.result == "OK":
        return "OK"
    return "NG"


@dataclass(frozen=True)
class CheckParams:
    """registry.yaml の opcheck_runner 向け行の写し。

    `registry.yaml の該当行 <-> この dataclass <-> opcheck_runner のパラメータ宣言`
    の 3 点が一致していることは `test_maintenance_registry_driven` が機械的に
    突き合わせる（W-03 / W-13 と同じ流儀）。
    """

    motor_deadband_mps: float          # 指令がこれ未満なら追従判定しない
    motor_follow_min_ratio: float      # |実測| ≥ |指令| × この比率 を満たさないと NG
    imu_bias_max_rad_s: float          # 静止角速度（≒バイアス）の許容上限
    imu_wz_implausible_rad_s: float    # |wz| がこれを超えたらジャイロ単位の取り違えを疑う
    opcheck_imu_window_s: float        # IMU の静止観測窓（ジャイロの最大値・バイアス平均の区間）
    opcheck_deadman_timeout_s: float   # /opcheck/motor_hold の途絶を「離し」扱いにする時間
    opcheck_spin_w_rad_s: float        # モーター確認の超信地旋回角速度
    opcheck_blind_tolerance_deg: float # 死角帯のズレ許容（設定 ↔ 推定）
    opcheck_scan_coverage_gap_deg: float  # 無効ビーム連続帯の許容幅
    v_check: float                     # モーター確認の直進速度
    scan_stale_ms: float               # /scan の死活・周期（obstacle_limiter と同じ行）


def _circle_dist_deg(a: float, b: float) -> float:
    """角度差の最小値（0..180 度）。円周上の距離。"""
    d = abs((a - b) % 360.0)
    return min(d, 360.0 - d)


def unpack_calib(calib_status: int) -> tuple[int, int, int, int]:
    """BNO055 の校正ステータス（sys / gyro / accel / mag、各 2 bit）を分解する。

    `calib_status` は `/esp32/imu_calib_status`（std_msgs/UInt8）の値そのもの。
    bit 配分: [1:0]=mag / [3:2]=accel / [5:4]=gyro / [7:6]=sys（BNO055 の
    CALIB_STAT レジスタ準拠）。
    """
    mag = calib_status & 0x03
    accel = (calib_status >> 2) & 0x03
    gyro = (calib_status >> 4) & 0x03
    sys = (calib_status >> 6) & 0x03
    return (sys, gyro, accel, mag)


# ── 項目 1: 物理非常停止ボタン ──────────────────────────────────────────────

def judge_estop(estop_alive: bool, saw_press: bool, saw_release: bool) -> CheckVerdict:
    """物理非常停止ボタンがシステムに正しく届いているか（maintenance.md §2.2）。

    - `estop_alive`: 点検中に `/safety/estop_hw` が途絶していないか（届いていれば真）
    - `saw_press`: 押下フェーズで pressed=True を観測できたか
    - `saw_release`: 解除フェーズで pressed=False を観測できたか

    押す・離すの両方を試す（片方だけでは断線・短絡を区別できない。§2.2）。
    """
    if not estop_alive:
        return NG("no_data")           # ボタン状態自体が届いていない（GPIO・配線）
    if not saw_press:
        return NG("no_press")          # 押下が検出できない（導通・ボタン・バイパス）
    if not saw_release:
        return NG("stuck_release")     # 解除が検出できない（短絡・固着）
    return OK()


# ── 項目 2: モーター・エンコーダ ────────────────────────────────────────────

def judge_motor(cmd_l: float, meas_l: float, cmd_r: float, meas_r: float,
                p: CheckParams) -> CheckVerdict:
    """単一サンプルの符号一致と追従率（maintenance.md §2.3・`def judge_motor`）。

    左右それぞれ:
      - 指令がデッドバンド未満の間は判定しない
      - 符号が不一致（`cmd * meas <= 0`）なら NG（左右入れ替え／極性逆／動かない）
      - |実測| が |指令| × motor_follow_min_ratio 未満なら NG（片輪だけ追従しない）
    """
    for c, m, side in ((cmd_l, meas_l, "L"), (cmd_r, meas_r, "R")):
        if abs(c) < p.motor_deadband_mps:
            continue
        if c * m <= 0:
            return NG(f"sign_mismatch_{side}")
        if abs(m) < abs(c) * p.motor_follow_min_ratio:
            return NG(f"no_follow_{side}")
    return OK()


def judge_motor_samples(samples: Sequence[tuple[float, float, float, float]],
                        p: CheckParams) -> CheckVerdict:
    """押下期間に集めたサンプルの集合から、モーター項目を判定する。

    `samples` は `(cmd_l, meas_l, cmd_r, meas_r)` の列。1 回の押下中に複数
    サンプルが届くので、以下の要領で 1 個の verdict にまとめる:
      - サンプルが 1 つも無い（指令すら通らなかった）→ NG("no_samples")
      - いずれかのサンプルで符号不一致 → NG（左右入れ替え等の構造的なエラー）
      - 追従率は「最も良く動いた瞬間」（左右実測の絶対値和が最大のサンプル）で
        判定する。立ち上がり等の一時的な低実測で誤 NG しないため。
    """
    if not samples:
        return NG("no_samples")
    for cmd_l, meas_l, cmd_r, meas_r in samples:
        verdict = judge_motor(cmd_l, meas_l, cmd_r, meas_r, p)
        if verdict.result == "NG":
            return verdict
    peak = max(samples, key=lambda s: abs(s[1]) + abs(s[3]))
    return judge_motor(peak[0], peak[1], peak[2], peak[3], p)


# ── 項目 3: IMU ─────────────────────────────────────────────────────────────

def judge_imu(calib_status: int, gyro_bias_rad_s: float, alive: bool,
              p: CheckParams) -> CheckVerdict:
    """IMU データの生死・バイアス・校正状態（maintenance.md §2.4・`def judge_imu`）。

    「校正済みで今は正常」なのか「劣化していて校正が必要」なのかを分けるため、
    未校正（= needs_calibration）は WARN にする（画面が人に校正を促す。故障扱い
    にはしない）。
    """
    sys_, gyro, accel, mag = unpack_calib(calib_status)
    if not alive:
        return NG("no_data")
    if abs(gyro_bias_rad_s) > p.imu_bias_max_rad_s:
        return NG("bias_too_large")
    if min(sys_, gyro, accel, mag) < 3:
        return WARN("needs_calibration")
    return OK()


def judge_gyro_unit(max_wz_rad_s: float | None, p: CheckParams) -> CheckVerdict:
    """ジャイロの単位取り違え（dps のまま）を検知する（maintenance.md §2.4 注）。

    esp32_bridge の `|wz| > 10` 警告（`_IMU_WZ_IMPLAUSIBLE_RAD_S`）と同じ判定を
    始業点検でも行う。この機体の最大ヨーレートは 1 rad/s 程度なので、静止観測
    窓（opcheck_imu_window_s）内の |wz| 最大値が `imu_wz_implausible_rad_s` を
    超えたら、ファームが 2026-08-06 修正前の dps 送信のままの可能性が高い。

    `max_wz_rad_s` が None（窓内サンプル無し）なら WARN（生死は judge_imu が判定）。
    """
    if max_wz_rad_s is None:
        return WARN("no_samples")
    if abs(max_wz_rad_s) > p.imu_wz_implausible_rad_s:
        return NG("wz_implausible")
    return OK()


# ── 項目 4: LiDAR ───────────────────────────────────────────────────────────

def pairs_from_flat(flat: Sequence[float]) -> list[tuple[float, float]]:
    """registry の `blind_angle_ranges`（deg の平坦配列 `[a0,a1,a2,a3, ...]`）を
    (start, end) のペア列へ変換する。奇数個は末尾を無視する。"""
    pairs: list[tuple[float, float]] = []
    step = 2
    for i in range(0, len(flat) - 1, step):
        pairs.append((float(flat[i]), float(flat[i + 1])))
    return pairs


def scan_coverage_gap_deg(ranges: Sequence[float], angle_increment_deg: float) -> float:
    """全周のうち、無効（∞ / NaN / 0 / 負）なビームが連続する最大幅（度）。

    LiDAR が「届かない」角度帯を検出する（`/scan` の欠損）。先頭末尾の巡回
    連結も考慮する。死角（アルミパイプ等）は近距離**有効値**として写るため
    ここでは検出されない（死角は `estimate_blind_sectors` と
    `opcheck_blind_tolerance_deg` の比較で扱う）。
    """
    n = len(ranges)
    if n == 0 or angle_increment_deg <= 0.0:
        return 360.0
    valid = [math.isfinite(r) and r > 0.0 for r in ranges]
    if all(valid):
        return 0.0
    k = next(i for i, v in enumerate(valid) if not v)  # 無効の場所から回転
    rot = valid[k:] + valid[:k]
    # rot を 2 連結して 0..360 をまたぐ欠損（末尾→先頭）を 1 本の連続として数える。
    # 全周欠損は worst がビーム数 n に達するので、そこで 360.0 と確定する。
    worst = 0
    run = 0
    for v in rot + rot:
        if not v:
            run += 1
            if run > worst:
                worst = run
        else:
            run = 0
    if worst >= n:
        return 360.0
    return min(worst * angle_increment_deg, 360.0)


def estimate_blind_sectors(ranges: Sequence[float], angle_increment_deg: float,
                           near_m: float = BLIND_NEAR_M,
                           min_run_deg: float = BLIND_MIN_RUN_DEG) -> list[tuple[float, float]]:
    """恒常的に近距離（0 < r < near_m）を返す連続角度帯を推定する。

    `scripts/measure_blind_sectors.py`（NEAR_M / MARGIN）と同じ思想のスナップ
    ショット近似。`/scan` 1 フレームから近距離連続帯を列挙し、幅が
    `min_run_deg` 未満の帯はノイズとして捨てる。

    戻り値: `(start_deg, end_deg)` のリスト。0..360 をまたぐ帯は 2 つに分割して
    返す。死角なしは `[]`、全周死角は `[(0.0, 360.0)]`。
    """
    n = len(ranges)
    if n == 0 or angle_increment_deg <= 0.0:
        return []
    near = [0.0 < r < near_m for r in ranges]
    if not any(near):
        return []
    if all(near):
        return [(0.0, 360.0)]
    k = next(i for i, v in enumerate(near) if not v)  # False の場所から回転
    rot = near[k:] + near[:k]
    sectors: list[tuple[float, float]] = []
    i = 0
    while i < n:
        if not rot[i]:
            i += 1
            continue
        j = i
        while j < n and rot[j]:
            j += 1
        width = (j - i) * angle_increment_deg
        if width + 1e-9 >= min_run_deg:
            start = (((k + i) % n) * angle_increment_deg) % 360.0
            if (k + i) < n and (k + j - 1) >= n:
                # 帯が配列末尾→先頭（角度 0）をまたぐ。0 またぎ部分は lead 本が
                # 末尾側、残りが先頭側なので 2 つに分割して返す（設定
                # blind_angle_ranges の (start, end) 書式に合わせる）。
                lead_count = n - (k + i)
                end = start + lead_count * angle_increment_deg
                sectors.append((start, min(end, 360.0)))
                rest = width - lead_count * angle_increment_deg
                sectors.append((0.0, max(rest, 0.0)))
            else:
                end = start + width
                sectors.append((start, min(end, 360.0)))
        i = j
    return sectors


def blind_offset_deg(estimated: Sequence[tuple[float, float]],
                     configured_flat: Sequence[float]) -> float | None:
    """推定した死角帯と設定済み `blind_angle_ranges` のズレ（最大、度）。

    両者の境界角（start / end の全点）それぞれについて、「相手側の最近傍境界
    との円周距離」の最大値を返す。どちらか一方でも空なら比較不能 → None を返す
    （未校正のマスクを NG にしない。`blind_calibrated` が別にいる）。
    """
    estimated_bounds = {float(b) % 360.0 for sweep in estimated for b in sweep}
    configured_bounds = {float(b) % 360.0
                         for sweep in pairs_from_flat(configured_flat) for b in sweep}
    if not estimated_bounds or not configured_bounds:
        return None
    worst = 0.0
    for b in estimated_bounds:
        worst = max(worst, min(_circle_dist_deg(b, c) for c in configured_bounds))
    for c in configured_bounds:
        worst = max(worst, min(_circle_dist_deg(c, b) for b in estimated_bounds))
    return worst


def judge_lidar(alive: bool, period_s: float | None, ranges: Sequence[float],
                angle_increment_deg: float, configured_ranges: Sequence[float],
                p: CheckParams) -> CheckVerdict:
    """LiDAR の死活・周期・カバレッジ・死角マスクのズレ（maintenance.md §2.5）。

    順に: データが届いているか → 周期が `scan_stale_ms` 以内か → 全周に欠損が
    無いか → 推定した死角帯が設定 `blind_angle_ranges` と `opcheck_blind_tolerance`
    以上ズレていないか。
    """
    if not alive:
        return NG("no_data")
    if period_s is not None and period_s > (p.scan_stale_ms / 1000.0):
        return NG("scan_stale")
    if scan_coverage_gap_deg(ranges, angle_increment_deg) >= p.opcheck_scan_coverage_gap_deg:
        return NG("coverage_gap")
    estimated = estimate_blind_sectors(ranges, angle_increment_deg)
    offset = blind_offset_deg(estimated, configured_ranges)
    if offset is not None and offset > p.opcheck_blind_tolerance_deg:
        return NG("blind_mismatch")
    return OK()