// ============================================================
// test_obstacle_limiter_params.cpp — obstacle_limiter_params.hpp の単体試験
//
// 対応する仕様（DetailedDesign-wp2.md WP-SAFE-03 §3.3・「確認済みの事実①②」）:
//   ・flat_to_range_pairs        : 平坦配列 → ペア列（空配列は「死角なし」）
//   ・resolve_speed_limit_name   : 名前 → 数値。"stop" と未知の名前の扱い
// ============================================================
#include <gtest/gtest.h>

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
