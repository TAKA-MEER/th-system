// ============================================================
// test_obstacle_limiter_core.cpp — obstacle_limiter_core.hpp の property test
//
// 対応する仕様（docs/plan/detailed/DetailedDesign-safety.md §3.5）:
//   L1  RandomizedSingleCallInvariants  : リミッタは速度を絶対に上げない・符号保存
//   L2  RandomizedSingleCallInvariants  : muxed/scan の stale・ロックは無条件ゼロ
//   L3  RandomizedSingleCallInvariants  : d_floor 割れの間だけ angular に w_align_max
//   L4  RandomizedSingleCallInvariants  : 後退は |linear.x| <= v_reverse
//   L5  RandomizedSingleCallInvariants  : 死角セクタに向かう成分がある方向だけ v_reverse
//   L6  RandomizedSingleCallInvariants  : estop/fault_lock（stale・未受信含む）は無条件ゼロ
//   L7  RandomizedSingleCallInvariants + PolicyStopsAutoNeverRelaxes
//
// 乱数シードは固定（kSeed）。失敗した反復番号とその入力一式を
// SCOPED_TRACE / ADD_FAILURE のメッセージに出す（再現用）。
// ============================================================
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "th_safety/obstacle_limiter_core.hpp"

using namespace th_safety;

namespace {

constexpr std::uint32_t kSeed = 20260826u;  // 固定シード（今日の日付）。失敗時は同じ列を再現できる
constexpr int kMainIterations = 10000;
constexpr int kHysteresisIterations = 10000;
constexpr double kEps = 1e-9;

// ── ランダム入力の生成 ──────────────────────────────────────
struct GeneratedCase {
  ObstacleLimiterInputs in;
  ObstacleLimiterParams p;
};

// 鮮度バケツ: 0=fresh, 1=stale, 2=未受信。70/20/10 で重み付けする
// （全入力が独立に一様 1/3 だと「ロック無し かつ 全トピック新鮮」に到達する
// 確率が (1/3)^5 未満まで落ち、L3/L5/L7 が検証される「通常経路」の実効試行数が
// 10,000 通りのうち一桁台まで減ってしまうことを標準ドライバでの事前確認で
// 検出した。実運用でも「大半のサイクルは全入力が新鮮」が普通の状態なので、
// 70/20/10 の方が実態にも近い）。
int freshness_bucket(std::mt19937& rng) {
  std::discrete_distribution<int> bucket({0.7, 0.2, 0.1});
  return bucket(rng);
}

void make_stamped_bool(std::mt19937& rng, double now_sec, double stale_thresh, double active_prob_true,
                        Stamped<bool>* out) {
  std::uniform_real_distribution<double> frac(0.0, 1.0);
  std::bernoulli_distribution active_dist(active_prob_true);
  int b = freshness_bucket(rng);
  out->received = (b != 2);
  if (!out->received) {
    out->stamp_sec = 0.0;
  } else if (b == 0) {
    out->stamp_sec = now_sec - frac(rng) * stale_thresh * 0.9;  // fresh
  } else {
    out->stamp_sec = now_sec - stale_thresh * (1.1 + frac(rng));  // stale
  }
  out->value = active_dist(rng);
}

void make_stamped_twist(std::mt19937& rng, double now_sec, double stale_thresh, double v_range,
                         Stamped<Twist2D>* out) {
  std::uniform_real_distribution<double> frac(0.0, 1.0);
  std::uniform_real_distribution<double> v(-v_range, v_range);
  int b = freshness_bucket(rng);
  out->received = (b != 2);
  if (!out->received) {
    out->stamp_sec = 0.0;
  } else if (b == 0) {
    out->stamp_sec = now_sec - frac(rng) * stale_thresh * 0.9;
  } else {
    out->stamp_sec = now_sec - stale_thresh * (1.1 + frac(rng));
  }
  out->value.linear_x = v(rng);
  out->value.angular_z = v(rng);
}

// 720 点・全周をカバーする ScanGeometryInfo（angle_min=-pi, 0.5deg 刻み）。
ScanGeometryInfo fixed_geometry() {
  ScanGeometryInfo g;
  g.angle_min = -kPi;
  g.angle_increment = (2.0 * kPi) / 720.0;
  g.num_ranges = 720;
  return g;
}

// direction_rad 付近（cone 内）の添字に obstacle_range_m を書き込み、
// それ以外は far_range_m（届かない距離）で埋める。
void plant_obstacle(std::vector<double>* ranges, const ScanGeometryInfo& geom, double direction_rad,
                     double obstacle_range_m, double far_range_m) {
  ranges->assign(geom.num_ranges, far_range_m);
  const double center_idx =
      std::floor((normalize_angle(direction_rad) - geom.angle_min) / geom.angle_increment);
  for (int di = -3; di <= 3; ++di) {
    long idx = static_cast<long>(center_idx) + di;
    idx = ((idx % static_cast<long>(geom.num_ranges)) + static_cast<long>(geom.num_ranges)) %
        static_cast<long>(geom.num_ranges);
    (*ranges)[static_cast<std::size_t>(idx)] = obstacle_range_m;
  }
}

GeneratedCase generate_case(std::mt19937& rng) {
  GeneratedCase c;
  const double now_sec = 1000.0;
  c.in.now_sec = now_sec;

  // ── パラメータ ──
  std::uniform_real_distribution<double> floor_dist(0.2, 0.6);
  std::uniform_real_distribution<double> band_dist(0.02, 0.15);
  std::uniform_real_distribution<double> accel_dist(0.5, 3.0);
  std::uniform_real_distribution<double> cone_dist(0.2, 0.7);
  std::uniform_real_distribution<double> cone_rev_dist(0.2, 0.8);
  std::uniform_real_distribution<double> v_reverse_dist(0.05, 0.4);
  std::uniform_real_distribution<double> w_align_dist(0.1, 0.5);
  std::uniform_real_distribution<double> manual_to_dist(0.5, 1.5);
  std::uniform_real_distribution<double> state_stale_dist(0.5, 2.0);
  std::uniform_real_distribution<double> muxed_stale_dist(0.05, 0.5);
  std::uniform_real_distribution<double> scan_stale_dist(0.05, 0.5);
  std::uniform_real_distribution<double> lock_stale_dist(0.1, 1.0);

  c.p.obstacle_floor_distance_m = floor_dist(rng);
  c.p.hysteresis_band_m = band_dist(rng);
  c.p.brake_accel_mps2 = accel_dist(rng);
  c.p.obstacle_cone_half_width_rad = cone_dist(rng);
  c.p.obstacle_cone_half_width_reverse_rad = cone_rev_dist(rng);
  c.p.v_reverse = v_reverse_dist(rng);
  c.p.w_align_max = w_align_dist(rng);
  c.p.manual_joy_timeout_sec = manual_to_dist(rng);
  c.p.state_stale_sec = state_stale_dist(rng);
  c.p.muxed_stale_sec = muxed_stale_dist(rng);
  c.p.scan_stale_sec = scan_stale_dist(rng);
  c.p.lock_stale_sec = lock_stale_dist(rng);

  std::bernoulli_distribution calibrated_dist(0.9);
  c.p.blind_calibrated = calibrated_dist(rng);

  // ── 死角セクタの配置（0=無し, 1=前方cone内, 2=後方cone内, 3=無関係な位置） ──
  std::uniform_int_distribution<int> blind_bucket(0, 3);
  const int bb = blind_bucket(rng);
  if (bb == 1) {
    c.p.blind_angle_ranges_deg = {{-5.0, 5.0}};
  } else if (bb == 2) {
    c.p.blind_angle_ranges_deg = {{175.0, 185.0}};
  } else if (bb == 3) {
    c.p.blind_angle_ranges_deg = {{85.0, 95.0}};
  }
  // bb==0: 空のまま（死角なし）

  // ── 入力 ──
  make_stamped_twist(rng, now_sec, c.p.muxed_stale_sec, 2.0, &c.in.muxed);
  make_stamped_twist(rng, now_sec, c.p.manual_joy_timeout_sec, 1.0, &c.in.manual);
  make_stamped_bool(rng, now_sec, c.p.lock_stale_sec, 0.1, &c.in.estop);
  make_stamped_bool(rng, now_sec, c.p.lock_stale_sec, 0.1, &c.in.fault_lock);

  // /system/state
  {
    std::uniform_real_distribution<double> frac(0.0, 1.0);
    int b = freshness_bucket(rng);
    c.in.state.received = (b != 2);
    if (!c.in.state.received) {
      c.in.state.stamp_sec = 0.0;
    } else if (b == 0) {
      c.in.state.stamp_sec = now_sec - frac(rng) * c.p.state_stale_sec * 0.9;
    } else {
      c.in.state.stamp_sec = now_sec - c.p.state_stale_sec * (1.1 + frac(rng));
    }
    static const std::vector<std::string> kModes = {
        "MANUAL", "TEACH_MANUAL", "FOLLOW", "IDLE", "AT_PANEL", "OPCHECK", "CALIB"};
    std::uniform_int_distribution<std::size_t> mode_idx(0, kModes.size() - 1);
    c.in.state.mode = kModes[mode_idx(rng)];
    std::bernoulli_distribution jog_dist(0.3);
    c.in.state.jog_active = jog_dist(rng);
    std::uniform_int_distribution<int> zone_idx(0, 2);
    static const Zone kZones[3] = {Zone::IN, Zone::OUT, Zone::NA};
    c.in.state.zone = kZones[zone_idx(rng)];
    std::bernoulli_distribution auto_brake_dist(0.5);
    c.in.state.auto_brake = auto_brake_dist(rng);
  }

  // /scan
  {
    std::uniform_real_distribution<double> frac(0.0, 1.0);
    int b = freshness_bucket(rng);
    c.in.scan.received = (b != 2);
    if (!c.in.scan.received) {
      c.in.scan.stamp_sec = 0.0;
    } else if (b == 0) {
      c.in.scan.stamp_sec = now_sec - frac(rng) * c.p.scan_stale_sec * 0.9;
    } else {
      c.in.scan.stamp_sec = now_sec - c.p.scan_stale_sec * (1.1 + frac(rng));
    }
    c.in.scan.geometry = fixed_geometry();

    std::uniform_int_distribution<int> scan_bucket(0, 3);
    const bool reversing_hint = c.in.muxed.value.linear_x < 0.0;
    const double direction_rad = reversing_hint ? kPi : 0.0;
    std::uniform_real_distribution<double> dist_around_floor(
        c.p.obstacle_floor_distance_m - 0.3, c.p.obstacle_floor_distance_m + 0.5);
    switch (scan_bucket(rng)) {
      case 0:  // 空き（十分遠い）
        c.in.scan.ranges.assign(c.in.scan.geometry.num_ranges, 30.0);
        break;
      case 1: {  // 進行方向 cone 内、floor 付近にプラント
        double d = dist_around_floor(rng);
        if (d < 0.0) d = 0.01;
        plant_obstacle(&c.in.scan.ranges, c.in.scan.geometry, direction_rad, d, 30.0);
        break;
      }
      case 2:  // 全 inf（センサ無反応）
        c.in.scan.ranges.assign(c.in.scan.geometry.num_ranges,
                                 std::numeric_limits<double>::infinity());
        break;
      case 3: {  // 進行方向 cone 内、床よりかなり近い
        std::uniform_real_distribution<double> very_near(0.01, c.p.obstacle_floor_distance_m * 0.5);
        plant_obstacle(&c.in.scan.ranges, c.in.scan.geometry, direction_rad, very_near(rng), 30.0);
        break;
      }
      default:
        c.in.scan.ranges.assign(c.in.scan.geometry.num_ranges, 30.0);
    }
  }

  std::uniform_real_distribution<double> limit_dist(0.0, 1.5);
  c.in.screen_limit_mps = limit_dist(rng);
  c.in.mode_limit_mps = limit_dist(rng);

  return c;
}

std::string describe(const GeneratedCase& c, int iter) {
  std::ostringstream os;
  os << "iter=" << iter << " seed=" << kSeed << "\n"
     << "  in.linear_x=" << c.in.muxed.value.linear_x
     << " in.angular_z=" << c.in.muxed.value.angular_z
     << " muxed.received=" << c.in.muxed.received << " muxed.stamp=" << c.in.muxed.stamp_sec << "\n"
     << "  manual.received=" << c.in.manual.received << " manual.stamp=" << c.in.manual.stamp_sec
     << "\n"
     << "  state.received=" << c.in.state.received << " state.stamp=" << c.in.state.stamp_sec
     << " state.mode=" << c.in.state.mode << " jog_active=" << c.in.state.jog_active
     << " zone=" << static_cast<int>(c.in.state.zone) << " auto_brake=" << c.in.state.auto_brake
     << "\n"
     << "  scan.received=" << c.in.scan.received << " scan.stamp=" << c.in.scan.stamp_sec << "\n"
     << "  estop.received=" << c.in.estop.received << " estop.stamp=" << c.in.estop.stamp_sec
     << " estop.value=" << c.in.estop.value << "\n"
     << "  fault_lock.received=" << c.in.fault_lock.received
     << " fault_lock.stamp=" << c.in.fault_lock.stamp_sec << " fault_lock.value=" << c.in.fault_lock.value
     << "\n"
     << "  screen_limit_mps=" << c.in.screen_limit_mps << " mode_limit_mps=" << c.in.mode_limit_mps
     << "\n"
     << "  p.obstacle_floor_distance_m=" << c.p.obstacle_floor_distance_m
     << " p.hysteresis_band_m=" << c.p.hysteresis_band_m
     << " p.brake_accel_mps2=" << c.p.brake_accel_mps2 << " p.v_reverse=" << c.p.v_reverse
     << " p.w_align_max=" << c.p.w_align_max << " p.blind_calibrated=" << c.p.blind_calibrated
     << " p.blind_angle_ranges_deg.size=" << c.p.blind_angle_ranges_deg.size();
  return os.str();
}

}  // namespace

// ── L1〜L2, L4〜L7: 単発呼び出しの不変条件（フレッシュな core を毎回使う） ──
TEST(ObstacleLimiterCoreProperty, RandomizedSingleCallInvariants) {
  std::mt19937 rng(kSeed);
  int failures = 0;

  for (int iter = 0; iter < kMainIterations; ++iter) {
    GeneratedCase c = generate_case(rng);
    ObstacleLimiterCore core;  // 呼び出しごとに新規（ヒステリシス状態を持ち込まない）
    ObstacleLimiterOutput out = core.update(c.in, c.p);

    const std::string ctx = describe(c, iter);

    // ── L1: 出力は入力を絶対に超えない。符号は保存する ──
    if (std::fabs(out.out.linear_x) > std::fabs(c.in.muxed.value.linear_x) + kEps) {
      ADD_FAILURE() << "L1 linear exceeded input\n" << ctx;
      ++failures;
    }
    if (std::fabs(out.out.angular_z) > std::fabs(c.in.muxed.value.angular_z) + kEps) {
      ADD_FAILURE() << "L1 angular exceeded input\n" << ctx;
      ++failures;
    }
    if (out.out.linear_x != 0.0 &&
        (out.out.linear_x > 0) != (c.in.muxed.value.linear_x > 0)) {
      ADD_FAILURE() << "L1 linear sign not preserved\n" << ctx;
      ++failures;
    }
    if (out.out.angular_z != 0.0 &&
        (out.out.angular_z > 0) != (c.in.muxed.value.angular_z > 0)) {
      ADD_FAILURE() << "L1 angular sign not preserved\n" << ctx;
      ++failures;
    }

    // ── L4: 後退時は v_reverse 以内 ──
    if (out.out.linear_x < 0.0 && std::fabs(out.out.linear_x) > c.p.v_reverse + kEps) {
      ADD_FAILURE() << "L4 exceeded v_reverse while reversing\n" << ctx;
      ++failures;
    }

    // ── §3.4.2 の鮮度を仕様どおりに独立再計算する（実装内部は見ない） ──
    const bool estop_locked = !c.in.estop.received ||
        (c.in.now_sec - c.in.estop.stamp_sec) > c.p.lock_stale_sec || c.in.estop.value;
    const bool fault_locked = !c.in.fault_lock.received ||
        (c.in.now_sec - c.in.fault_lock.stamp_sec) > c.p.lock_stale_sec || c.in.fault_lock.value;
    const bool locked = estop_locked || fault_locked;
    const bool muxed_stale =
        !c.in.muxed.received || (c.in.now_sec - c.in.muxed.stamp_sec) > c.p.muxed_stale_sec;
    const bool scan_stale =
        !c.in.scan.received || (c.in.now_sec - c.in.scan.stamp_sec) > c.p.scan_stale_sec;

    if (!c.p.blind_calibrated) {
      // ── 未校正ゲート ──
      if (out.action != LimiterAction::BLOCKED_UNCALIBRATED || out.out.linear_x != 0.0 ||
          out.out.angular_z != 0.0) {
        ADD_FAILURE() << "BLOCKED_UNCALIBRATED not enforced\n" << ctx;
        ++failures;
      }
      continue;
    }

    if (locked) {
      // ── L6 ──
      if (out.out.linear_x != 0.0 || out.out.angular_z != 0.0) {
        ADD_FAILURE() << "L6 locked but output nonzero\n" << ctx;
        ++failures;
      }
      continue;
    }

    if (muxed_stale) {
      // ── L2 (muxed) ──
      if (out.action != LimiterAction::ZERO_STALE || out.out.linear_x != 0.0 ||
          out.out.angular_z != 0.0) {
        ADD_FAILURE() << "L2 muxed stale but not ZERO_STALE/zero\n" << ctx;
        ++failures;
      }
      continue;
    }

    if (scan_stale) {
      // ── L2 (scan) ──
      if (out.action != LimiterAction::STOP || out.out.linear_x != 0.0 || out.out.angular_z != 0.0) {
        ADD_FAILURE() << "L2 scan stale but not STOP/zero\n" << ctx;
        ++failures;
      }
      continue;
    }

    // ── ここから通常経路。L7: source_class を仕様どおり独立再計算して照合 ──
    const SourceClass expected_source_class = compute_source_class(c.in, c.p);
    if (out.source_class != expected_source_class) {
      ADD_FAILURE() << "L7 source_class mismatch\n" << ctx;
      ++failures;
    }

    const bool reversing = c.in.muxed.value.linear_x < 0.0;
    const double direction_rad = reversing ? kPi : 0.0;
    const double half_width =
        reversing ? c.p.obstacle_cone_half_width_reverse_rad : c.p.obstacle_cone_half_width_rad;

    // ── L5: 死角セクタに向かう成分がある方向だけ v_reverse キャップ ──
    const bool overlap = blind_direction_overlap(direction_rad, half_width, c.p.blind_angle_ranges_deg);
    const bool state_fresh =
        c.in.state.received && (c.in.now_sec - c.in.state.stamp_sec) <= c.p.state_stale_sec;
    double non_blind_ceiling;
    if (!state_fresh) {
      non_blind_ceiling = 0.0;
    } else {
      non_blind_ceiling = std::min(c.in.screen_limit_mps, c.in.mode_limit_mps);
      if (reversing) non_blind_ceiling = std::min(non_blind_ceiling, c.p.v_reverse);
    }
    if (overlap) {
      const double expected_ceiling = std::max(0.0, std::min(non_blind_ceiling, c.p.v_reverse));
      if (out.applied_limit_mps > expected_ceiling + kEps) {
        ADD_FAILURE() << "L5 blind overlap but applied_limit_mps not capped to v_reverse\n" << ctx;
        ++failures;
      }
    } else {
      const double expected_ceiling = std::max(0.0, non_blind_ceiling);
      if (std::fabs(out.applied_limit_mps - expected_ceiling) > 1e-6) {
        ADD_FAILURE() << "L5 non-overlapping direction unexpectedly capped\n"
                       << "  expected_ceiling=" << expected_ceiling
                       << " got=" << out.applied_limit_mps << "\n"
                       << ctx;
        ++failures;
      }
    }

    // ── L3: d_floor を割っている間だけ angular に w_align_max ──
    // フレッシュな core なので below_floor は生の nearest_m 比較と一致する
    // （ヒステリシスは前歴に依存するのでこのテストでは対象外。別テストで検証）。
    const std::optional<double> nearest_opt = nearest_in_cone(c.in.scan, direction_rad, half_width);
    const double nearest_m = nearest_opt.value_or(std::numeric_limits<double>::infinity());
    const bool below_floor = nearest_m < c.p.obstacle_floor_distance_m;
    if (below_floor) {
      if (std::fabs(out.out.angular_z) > c.p.w_align_max + kEps) {
        ADD_FAILURE() << "L3 angular not capped to w_align_max while below floor\n"
                       << "  nearest_m=" << nearest_m << " out.angular_z=" << out.out.angular_z
                       << "\n"
                       << ctx;
        ++failures;
      }
    } else {
      if (out.out.angular_z != c.in.muxed.value.angular_z) {
        ADD_FAILURE() << "L3 angular unexpectedly clamped while clear of floor\n"
                       << "  nearest_m=" << nearest_m << "\n"
                       << ctx;
        ++failures;
      }
    }
  }

  EXPECT_EQ(failures, 0) << kMainIterations << " 通り中 " << failures << " 件失敗";
}

// ── L7 補強: policy_stops(AUTO, ...) はどんな zone/auto_brake でも必ず true ──
TEST(ObstacleLimiterCoreProperty, PolicyStopsAutoNeverRelaxes) {
  const Zone zones[] = {Zone::IN, Zone::OUT, Zone::NA};
  for (Zone z : zones) {
    EXPECT_TRUE(policy_stops(SourceClass::AUTO, z, true));
    EXPECT_TRUE(policy_stops(SourceClass::AUTO, z, false));
  }
  // NA は MANUAL でも常に true（OPCHECK/CALIB）。
  EXPECT_TRUE(policy_stops(SourceClass::MANUAL, Zone::NA, true));
  EXPECT_TRUE(policy_stops(SourceClass::MANUAL, Zone::NA, false));
  // IN/OUT は auto_brake で決まる。
  EXPECT_TRUE(policy_stops(SourceClass::MANUAL, Zone::IN, true));
  EXPECT_FALSE(policy_stops(SourceClass::MANUAL, Zone::IN, false));
  EXPECT_TRUE(policy_stops(SourceClass::MANUAL, Zone::OUT, true));
  EXPECT_FALSE(policy_stops(SourceClass::MANUAL, Zone::OUT, false));
}

// ── ヒステリシス（§3.4.3）: 一度 STOP に入ったら floor+band を超えるまで維持する ──
TEST(ObstacleLimiterCoreProperty, HysteresisHoldsUntilClearOfBand) {
  std::mt19937 rng(kSeed + 1);
  int failures = 0;

  ObstacleLimiterParams p;
  p.obstacle_floor_distance_m = 0.4;
  p.hysteresis_band_m = 0.1;
  p.brake_accel_mps2 = 1.0;
  p.obstacle_cone_half_width_rad = 0.5;
  p.obstacle_cone_half_width_reverse_rad = 0.6;
  p.v_reverse = 0.2;
  p.w_align_max = 0.3;
  p.manual_joy_timeout_sec = 1.0;
  p.state_stale_sec = 1.5;
  p.muxed_stale_sec = 0.2;
  p.scan_stale_sec = 0.3;
  p.lock_stale_sec = 0.5;
  p.blind_calibrated = true;

  std::uniform_real_distribution<double> below_dist(0.01, p.obstacle_floor_distance_m - 0.01);
  std::uniform_real_distribution<double> in_band_dist(
      p.obstacle_floor_distance_m + 0.001, p.obstacle_floor_distance_m + p.hysteresis_band_m - 0.001);
  std::uniform_real_distribution<double> clear_dist(
      p.obstacle_floor_distance_m + p.hysteresis_band_m + 0.01, 5.0);

  ObstacleLimiterInputs base;
  base.now_sec = 1000.0;
  base.muxed.received = true;
  base.muxed.stamp_sec = base.now_sec;
  base.muxed.value.linear_x = 1.0;
  base.muxed.value.angular_z = 0.0;
  base.manual.received = false;  // AUTO 固定（ジョグ介入を混ぜない）
  base.state.received = true;
  base.state.stamp_sec = base.now_sec;
  base.state.mode = "FOLLOW";
  base.state.jog_active = false;
  base.state.zone = Zone::OUT;
  base.state.auto_brake = true;
  base.estop = Stamped<bool>{true, base.now_sec, false};
  base.fault_lock = Stamped<bool>{true, base.now_sec, false};
  base.scan.received = true;
  base.scan.stamp_sec = base.now_sec;
  base.scan.geometry = fixed_geometry();
  base.screen_limit_mps = 1.2;
  base.mode_limit_mps = 1.2;

  for (int iter = 0; iter < kHysteresisIterations; ++iter) {
    ObstacleLimiterCore core;
    ObstacleLimiterInputs in = base;

    // ステップ1: floor を割る → STOP に入るはず
    double d1 = below_dist(rng);
    plant_obstacle(&in.scan.ranges, in.scan.geometry, 0.0, d1, 30.0);
    ObstacleLimiterOutput o1 = core.update(in, p);
    if (o1.out.linear_x != 0.0) {
      ADD_FAILURE() << "hysteresis iter=" << iter << " d1=" << d1 << " STOP に入らなかった";
      ++failures;
    }

    // ステップ2: floor と floor+band の間へ回復 → STOP を維持するはず
    double d2 = in_band_dist(rng);
    plant_obstacle(&in.scan.ranges, in.scan.geometry, 0.0, d2, 30.0);
    ObstacleLimiterOutput o2 = core.update(in, p);
    if (o2.out.linear_x != 0.0) {
      ADD_FAILURE() << "hysteresis iter=" << iter << " d1=" << d1 << " d2=" << d2
                     << " バンド内なのに STOP を解除した";
      ++failures;
    }
    if (!core.stop_latched()) {
      ADD_FAILURE() << "hysteresis iter=" << iter << " stop_latched が false になった（バンド内）";
      ++failures;
    }

    // ステップ3: floor+band を超えて回復 → STOP が解除されるはず
    double d3 = clear_dist(rng);
    plant_obstacle(&in.scan.ranges, in.scan.geometry, 0.0, d3, 30.0);
    ObstacleLimiterOutput o3 = core.update(in, p);
    if (core.stop_latched()) {
      ADD_FAILURE() << "hysteresis iter=" << iter << " d1=" << d1 << " d2=" << d2 << " d3=" << d3
                     << " floor+band を超えても stop_latched が解除されない";
      ++failures;
    }
    if (o3.out.linear_x <= 0.0) {
      ADD_FAILURE() << "hysteresis iter=" << iter << " d3=" << d3
                     << " 解除後も linear_x が 0 のまま（速度が戻らない）";
      ++failures;
    }
  }

  EXPECT_EQ(failures, 0) << kHysteresisIterations << " 通り中 " << failures << " 件失敗";
}
