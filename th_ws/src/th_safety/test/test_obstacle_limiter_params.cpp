// ============================================================
// test_obstacle_limiter_params.cpp — obstacle_limiter_params.hpp の単体試験
//
// 対応する仕様（DetailedDesign-wp2.md WP-SAFE-03 §3.3・「確認済みの事実①②」）:
//   ・flat_to_range_pairs        : 平坦配列 → ペア列（空配列は「死角なし」）
//   ・resolve_speed_limit_name   : 名前 → 数値。"stop" と未知の名前の扱い
// ============================================================
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "th_safety/obstacle_limiter_params.hpp"

using th_safety::flat_to_range_pairs;
using th_safety::resolve_speed_limit_name;

// ── flat_to_range_pairs ──────────────────────────────────────

TEST(FlatToRangePairs, EmptyIsNoBlindSectors) {
  auto pairs = flat_to_range_pairs({});
  EXPECT_TRUE(pairs.empty());
}

TEST(FlatToRangePairs, SinglePair) {
  auto pairs = flat_to_range_pairs({10.0, 20.0});
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_DOUBLE_EQ(pairs[0].first, 10.0);
  EXPECT_DOUBLE_EQ(pairs[0].second, 20.0);
}

TEST(FlatToRangePairs, MultiplePairsPreserveOrder) {
  auto pairs = flat_to_range_pairs({170.0, 190.0, -10.0, 10.0});
  ASSERT_EQ(pairs.size(), 2u);
  EXPECT_DOUBLE_EQ(pairs[0].first, 170.0);
  EXPECT_DOUBLE_EQ(pairs[0].second, 190.0);
  EXPECT_DOUBLE_EQ(pairs[1].first, -10.0);
  EXPECT_DOUBLE_EQ(pairs[1].second, 10.0);
}

// lidar_filter.py::_build_blind_ranges() と同じ規約: 末尾の半端な 1 個は切り捨てる。
TEST(FlatToRangePairs, TrailingOddElementIsDropped) {
  auto pairs = flat_to_range_pairs({10.0, 20.0, 30.0});
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_DOUBLE_EQ(pairs[0].first, 10.0);
  EXPECT_DOUBLE_EQ(pairs[0].second, 20.0);
}

TEST(FlatToRangePairs, SingleElementProducesNoPairs) {
  auto pairs = flat_to_range_pairs({10.0});
  EXPECT_TRUE(pairs.empty());
}

// ── resolve_speed_limit_name ────────────────────────────────

TEST(ResolveSpeedLimitName, StopIsAlwaysZeroRegardlessOfTable) {
  std::map<std::string, double> table{{"stop", 99.0}};  // table にあっても無視する
  auto r = resolve_speed_limit_name("stop", table);
  EXPECT_DOUBLE_EQ(r.value_mps, 0.0);
  EXPECT_FALSE(r.unknown);
}

TEST(ResolveSpeedLimitName, KnownNameResolves) {
  std::map<std::string, double> table{{"v_max", 1.2}, {"v_slow", 0.4}};
  auto r = resolve_speed_limit_name("v_slow", table);
  EXPECT_DOUBLE_EQ(r.value_mps, 0.4);
  EXPECT_FALSE(r.unknown);
}

// 未知の名前は安全側（0.0=停止）に倒し、呼び出し側が警告を出せるよう unknown=true を返す。
TEST(ResolveSpeedLimitName, UnknownNameFallsBackToZero) {
  std::map<std::string, double> table{{"v_max", 1.2}};
  auto r = resolve_speed_limit_name("v_typo", table);
  EXPECT_DOUBLE_EQ(r.value_mps, 0.0);
  EXPECT_TRUE(r.unknown);
}

TEST(ResolveSpeedLimitName, EmptyTableFallsBackToZero) {
  std::map<std::string, double> table;
  auto r = resolve_speed_limit_name("v_max", table);
  EXPECT_DOUBLE_EQ(r.value_mps, 0.0);
  EXPECT_TRUE(r.unknown);
}

// ── validate_blind_ranges（校正 BLIND の実行中更新の上限。2026-10-01）──

using th_safety::BlindLimits;
using th_safety::validate_blind_ranges;

TEST(ValidateBlindRanges, EmptyIsValid) {
  EXPECT_TRUE(validate_blind_ranges({}, BlindLimits{}).ok);
}

TEST(ValidateBlindRanges, ShippedMaskIsWithinLimits) {
  EXPECT_TRUE(validate_blind_ranges(
      {-132.8, -117.5, -59.5, -38.9, 45.9, 61.5, 130.3, 143.5}, BlindLimits{}).ok);
}

TEST(ValidateBlindRanges, SectorLimitBoundary) {
  EXPECT_TRUE(validate_blind_ranges({0.0, 30.0}, BlindLimits{}).ok);
  const auto v = validate_blind_ranges({0.0, 30.5}, BlindLimits{});
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason, "sector_too_wide");
}

TEST(ValidateBlindRanges, WrapAroundWidthIsCounted) {
  EXPECT_TRUE(validate_blind_ranges({170.0, -170.0}, BlindLimits{}).ok);   // 20°
  EXPECT_FALSE(validate_blind_ranges({170.0, -130.0}, BlindLimits{}).ok);  // 60°
}

TEST(ValidateBlindRanges, TotalLimit) {
  EXPECT_TRUE(validate_blind_ranges(
      {0.0, 22.5, 60.0, 82.5, 120.0, 142.5, -60.0, -37.5}, BlindLimits{}).ok);  // 90°
  const auto v = validate_blind_ranges(
      {0.0, 23.0, 60.0, 83.0, 120.0, 143.0, -60.0, -37.0}, BlindLimits{});     // 92°
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason, "total_too_wide");
}

TEST(ValidateBlindRanges, SectorCountLimit) {
  std::vector<double> nine;
  for (int i = 0; i < 9; ++i) {
    nine.push_back(-170.0 + i * 40.0);
    nine.push_back(-165.0 + i * 40.0);
  }
  const auto v = validate_blind_ranges(nine, BlindLimits{});
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason, "too_many_sectors");
  nine.resize(16);
  EXPECT_TRUE(validate_blind_ranges(nine, BlindLimits{}).ok);
}

TEST(ValidateBlindRanges, MalformedRejected) {
  EXPECT_EQ(validate_blind_ranges({1.0, 2.0, 3.0}, BlindLimits{}).reason, "odd_length");
  EXPECT_EQ(validate_blind_ranges({5.0, 5.0}, BlindLimits{}).reason, "zero_width");
  EXPECT_EQ(validate_blind_ranges({0.0, std::nan("")}, BlindLimits{}).reason, "non_finite");
}
