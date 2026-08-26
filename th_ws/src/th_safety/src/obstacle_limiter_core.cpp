// ============================================================
// obstacle_limiter_core.cpp — obstacle_limiter_core.hpp の実装
// ============================================================
#include "th_safety/obstacle_limiter_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace th_safety {

namespace {

constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// [0, 2*pi) に正規化する（arcs_overlap の相対角比較専用。normalize_angle
// （scan_geometry.hpp・[-pi,pi) 版）とは値域が違うだけで意味は同じ）。
double norm_0_2pi(double x) {
  double r = std::fmod(x, kTwoPi);
  if (r < 0.0) {
    r += kTwoPi;
  }
  return r;
}

// 円周上の 2 つの弧 A=[a_start, a_start+a_len), B=[b_start, b_start+b_len)
// が重なっているか（弧の長さの向きは反時計回り正で統一する）。
bool arcs_overlap(double a_start, double a_len, double b_start, double b_len) {
  if (a_len <= 0.0 || b_len <= 0.0) {
    return false;  // 幅ゼロの区間は「死角なし」と同義（現構成の blind_angle_ranges と同じ扱い）
  }
  if (a_len >= kTwoPi || b_len >= kTwoPi) {
    return true;
  }
  const double d_ab = norm_0_2pi(b_start - a_start);
  const double d_ba = norm_0_2pi(a_start - b_start);
  return d_ab <= a_len || d_ba <= b_len;
}

}  // namespace

SourceClass compute_source_class(const ObstacleLimiterInputs& in, const ObstacleLimiterParams& p) {
  const bool manual_fresh =
      in.manual.received && (in.now_sec - in.manual.stamp_sec) <= p.manual_joy_timeout_sec;
  const bool state_fresh =
      in.state.received && (in.now_sec - in.state.stamp_sec) <= p.state_stale_sec;
  const bool manual_like =
      in.state.jog_active || in.state.mode == "MANUAL" || in.state.mode == "TEACH_MANUAL";
  // L7: manual_fresh / state_fresh のどちらか欠けても、あるいは manual_like で
  // なければ AUTO（厳しい側）に倒す。緩める方向には絶対に倒さない。
  return (manual_fresh && state_fresh && manual_like) ? SourceClass::MANUAL : SourceClass::AUTO;
}

bool policy_stops(SourceClass source_class, Zone zone, bool auto_brake) {
  if (source_class == SourceClass::AUTO) {
    return true;  // 無効化不可（L7・Spec-safety.md §2.1）
  }
  if (zone == Zone::NA) {
    return true;  // OPCHECK/CALIB は極低速で人が張り付いている
  }
  // IN/OUT は式としては同じ（auto_brake で決まる）。既定値が違うだけ（§3.3 注記）。
  return auto_brake;
}

double v_allow(double nearest_m, double obstacle_floor_distance_m, double brake_accel_mps2) {
  const double margin = nearest_m - obstacle_floor_distance_m;
  if (margin <= 0.0) {
    return 0.0;
  }
  return std::sqrt(2.0 * brake_accel_mps2 * margin);
}

double clamp_toward_zero(double val, double max_abs) {
  const double capped = std::min(std::fabs(val), std::max(0.0, max_abs));
  return std::copysign(capped, val);
}

bool blind_direction_overlap(double direction_rad, double half_width_rad,
                              const std::vector<std::pair<double, double>>& blind_angle_ranges_deg) {
  const double a_start = normalize_angle(direction_rad - half_width_rad);
  const double a_len = 2.0 * half_width_rad;
  for (const auto& range : blind_angle_ranges_deg) {
    const double b_start = normalize_angle(range.first * kDegToRad);
    const double b_end = normalize_angle(range.second * kDegToRad);
    const double b_len = norm_0_2pi(b_end - b_start);
    if (arcs_overlap(a_start, a_len, b_start, b_len)) {
      return true;
    }
  }
  return false;
}

std::optional<double> nearest_in_cone(const ScanSnapshot& scan, double direction_rad,
                                       double half_width_rad) {
  if (!scan.received || scan.geometry.num_ranges == 0 ||
      scan.ranges.size() != scan.geometry.num_ranges) {
    return std::nullopt;
  }

  const double direction_deg = direction_rad * kRadToDeg;
  const double half_width_deg = half_width_rad * kRadToDeg;
  const auto bounds =
      sector_indices(direction_deg - half_width_deg, direction_deg + half_width_deg, scan.geometry);
  const auto& i0 = bounds.first;
  const auto& i1 = bounds.second;
  if (!i0.has_value() || !i1.has_value()) {
    // scan_geometry 規約⑤: 範囲外は「そのセクタは存在しない」。
    // lidar_filter の既定（安全側＝判定できない区間は無視する）を踏襲し、
    // 「その方向は観測なし」として nullopt を返す（呼び出し側で空き扱い）。
    return std::nullopt;
  }

  const std::size_t n = scan.geometry.num_ranges;
  std::optional<double> nearest;
  auto consider = [&](std::size_t idx) {
    const double r = scan.ranges[idx];
    if (!std::isfinite(r) || r < 0.0) {
      return;
    }
    if (!nearest.has_value() || r < *nearest) {
      nearest = r;
    }
  };

  if (*i0 <= *i1) {
    for (std::size_t i = *i0; i <= *i1; ++i) {
      consider(i);
    }
  } else {
    // 配列境界（angle_min/angle_max）をまたぐ区間（lidar_filter と同じ扱い）。
    for (std::size_t i = *i0; i < n; ++i) {
      consider(i);
    }
    for (std::size_t i = 0; i <= *i1; ++i) {
      consider(i);
    }
  }
  return nearest;
}

ObstacleLimiterOutput ObstacleLimiterCore::update(const ObstacleLimiterInputs& in,
                                                   const ObstacleLimiterParams& p) {
  ObstacleLimiterOutput out;

  // ── tier 0: 未校正ゲート（確認済みの事実③） ─────────────────
  // 死角セクタが 0 個かどうかではなく、明示フラグ blind_calibrated で判定する。
  if (!p.blind_calibrated) {
    out.action = LimiterAction::BLOCKED_UNCALIBRATED;
    out.nearest_obstacle_m = std::numeric_limits<double>::infinity();
    out.source_class = SourceClass::AUTO;
    out.applied_limit_mps = 0.0;
    return out;
  }

  // ── tier 1: ロック（L6・§1.2・§3.4.2）。stale／未受信もロック扱い ──
  const bool estop_locked = !in.estop.received ||
      (in.now_sec - in.estop.stamp_sec) > p.lock_stale_sec || in.estop.value;
  const bool fault_locked = !in.fault_lock.received ||
      (in.now_sec - in.fault_lock.stamp_sec) > p.lock_stale_sec || in.fault_lock.value;
  if (estop_locked || fault_locked) {
    out.action = LimiterAction::STOP;
    out.nearest_obstacle_m = std::numeric_limits<double>::infinity();
    out.source_class = SourceClass::AUTO;
    out.applied_limit_mps = 0.0;
    return out;
  }

  // ── tier 2: /cmd_vel_muxed の途絶（L2・ZERO_STALE） ─────────────
  const bool muxed_stale =
      !in.muxed.received || (in.now_sec - in.muxed.stamp_sec) > p.muxed_stale_sec;
  if (muxed_stale) {
    out.action = LimiterAction::ZERO_STALE;
    out.nearest_obstacle_m = std::numeric_limits<double>::infinity();
    out.source_class = SourceClass::AUTO;
    out.applied_limit_mps = 0.0;
    return out;
  }

  // ── tier 3: /scan の途絶（L2。§3.4.2「古いスキャンで空きと判定しない」） ─
  const bool scan_stale = !in.scan.received || (in.now_sec - in.scan.stamp_sec) > p.scan_stale_sec;
  if (scan_stale) {
    out.action = LimiterAction::STOP;
    out.nearest_obstacle_m = -1.0;  // 「不明」の明示値（+infinity＝空き確認済み、とは区別する）
    out.source_class = compute_source_class(in, p);
    out.applied_limit_mps = 0.0;
    return out;
  }

  // ── ここから通常処理 ────────────────────────────────────────
  const SourceClass source_class = compute_source_class(in, p);
  const bool state_fresh =
      in.state.received && (in.now_sec - in.state.stamp_sec) <= p.state_stale_sec;
  // §3.4.2: /system/state が途絶したら auto_brake = ON に倒す。
  const bool eff_auto_brake = state_fresh ? in.state.auto_brake : true;

  const bool reversing = in.muxed.value.linear_x < 0.0;
  const double direction_rad = reversing ? kPi : 0.0;
  const double half_width =
      reversing ? p.obstacle_cone_half_width_reverse_rad : p.obstacle_cone_half_width_rad;

  // §3.3.1: applied_limit_mps = min(画面由来, モード由来, v_reverse（後退時）)。
  // §3.4.2: /system/state 途絶時はゾーン＝最も低い上限（＝停止）に倒す。
  double applied_limit;
  if (!state_fresh) {
    applied_limit = 0.0;
  } else {
    applied_limit = std::min(in.screen_limit_mps, in.mode_limit_mps);
    if (reversing) {
      applied_limit = std::min(applied_limit, p.v_reverse);
    }
  }
  // L5: 死角セクタに向かう成分がある間だけ、その方向の上限を v_reverse にする
  // （全方向ではない。state の新鮮さに関わらず幾何的に成立する制約なので
  // state_fresh の分岐の外で常に適用する）。
  if (blind_direction_overlap(direction_rad, half_width, p.blind_angle_ranges_deg)) {
    applied_limit = std::min(applied_limit, p.v_reverse);
  }
  applied_limit = std::max(0.0, applied_limit);

  const std::optional<double> nearest_opt = nearest_in_cone(in.scan, direction_rad, half_width);
  const double nearest_m = nearest_opt.value_or(std::numeric_limits<double>::infinity());

  // ヒステリシス（§3.4.3）。stop_latched_ はインスタンスの状態として持ち、
  // nearest_m >= floor + band になるまで STOP を維持する。
  const bool below_floor = nearest_m < p.obstacle_floor_distance_m;
  const bool clear_of_band = nearest_m >= p.obstacle_floor_distance_m + p.hysteresis_band_m;
  if (below_floor) {
    stop_latched_ = true;
  } else if (clear_of_band) {
    stop_latched_ = false;
  }
  // else: floor と floor+band の間 → 直前の stop_latched_ を維持する。
  const double nearest_for_v_allow = stop_latched_ ? p.obstacle_floor_distance_m : nearest_m;

  const bool policy_active = policy_stops(source_class, in.state.zone, eff_auto_brake);
  double linear_ceiling = applied_limit;
  if (policy_active) {
    linear_ceiling =
        std::min(linear_ceiling, v_allow(nearest_for_v_allow, p.obstacle_floor_distance_m,
                                          p.brake_accel_mps2));
  }
  linear_ceiling = std::max(0.0, linear_ceiling);

  // L3: 前方障害物では linear.x だけをクランプし、angular.z は殺さない。
  // ただし d_floor を割っている間（below_floor）は angular.z にも w_align_max を掛ける。
  double angular_ceiling = std::numeric_limits<double>::infinity();
  if (below_floor) {
    angular_ceiling = p.w_align_max;
  }

  out.out.linear_x = clamp_toward_zero(in.muxed.value.linear_x, linear_ceiling);
  out.out.angular_z = clamp_toward_zero(in.muxed.value.angular_z, angular_ceiling);
  out.nearest_obstacle_m = nearest_m;
  out.source_class = source_class;
  out.applied_limit_mps = applied_limit;

  const bool unchanged = (out.out.linear_x == in.muxed.value.linear_x) &&
      (out.out.angular_z == in.muxed.value.angular_z);
  const bool linear_forced_stop =
      policy_active && in.muxed.value.linear_x != 0.0 && out.out.linear_x == 0.0;
  if (unchanged) {
    out.action = LimiterAction::PASS;
  } else if (linear_forced_stop) {
    out.action = LimiterAction::STOP;
  } else {
    out.action = LimiterAction::CLAMP;
  }
  return out;
}

}  // namespace th_safety
