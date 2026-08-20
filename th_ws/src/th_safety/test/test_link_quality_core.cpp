// ============================================================
// test_link_quality_core.cpp — link_quality_core.hpp の単体試験
//
// 対応する仕様（DetailedDesign-wp0.md WP-SAFE-00 §7）:
//   ・test_link_quality_core            : 既知の系列で p50/p99/max が一致
//   ・test_link_quality_core::EmptyWindow : Q-2。受信 0 でゼロ値
// ============================================================
#include <gtest/gtest.h>

#include "th_safety/link_quality_core.hpp"

using th_safety::GapTracker;
using th_safety::Quantiles;

// 受信が 1 件も無い場合は p50=p99=max=0, window_sec=0（Q-2: 沈黙しない）。
TEST(LinkQualityCore, EmptyWindowReturnsZero) {
  GapTracker tracker;

  Quantiles q = tracker.compute(/*now_sec=*/100.0, /*window_sec=*/30.0);

  EXPECT_DOUBLE_EQ(q.p50_ms, 0.0);
  EXPECT_DOUBLE_EQ(q.p99_ms, 0.0);
  EXPECT_DOUBLE_EQ(q.max_ms, 0.0);
  EXPECT_EQ(q.window_sec, 0u);
}

// 最初の push() はギャップを生成しない（基準点が無いため）。
TEST(LinkQualityCore, SinglePushProducesNoGap) {
  GapTracker tracker;
  tracker.push(0.0);

  Quantiles q = tracker.compute(/*now_sec=*/0.0, /*window_sec=*/30.0);

  EXPECT_DOUBLE_EQ(q.p50_ms, 0.0);
  EXPECT_DOUBLE_EQ(q.p99_ms, 0.0);
  EXPECT_DOUBLE_EQ(q.max_ms, 0.0);
  EXPECT_EQ(q.window_sec, 0u);
}

// 既知の系列（ギャップ 100/200/300/400/500 ms）で p50/p99/max が一致する。
// 線形補間（numpy 既定と同じ方式）: rank = p * (n-1)
//   p50: rank = 0.5*4 = 2.0        → 300 ms（ちょうど中央値）
//   p99: rank = 0.99*4 = 3.96      → 400 + 0.96*(500-400) = 496 ms
//   max:                            → 500 ms
TEST(LinkQualityCore, KnownSeriesMatchesQuantiles) {
  GapTracker tracker;

  // stamps: 0.0, 0.1, 0.3, 0.6, 1.0, 1.5
  // gaps  :      0.1, 0.2, 0.3, 0.4, 0.5  (秒)
  tracker.push(0.0);
  tracker.push(0.1);
  tracker.push(0.3);
  tracker.push(0.6);
  tracker.push(1.0);
  tracker.push(1.5);

  Quantiles q = tracker.compute(/*now_sec=*/1.5, /*window_sec=*/30.0);

  EXPECT_NEAR(q.p50_ms, 300.0, 1e-6);
  EXPECT_NEAR(q.p99_ms, 496.0, 1e-6);
  EXPECT_NEAR(q.max_ms, 500.0, 1e-6);
  EXPECT_EQ(q.window_sec, 30u);
}

// 窓の外（now_sec - window_sec より前）のギャップは集計から除外される。
TEST(LinkQualityCore, ExcludesGapsOutsideWindow) {
  GapTracker tracker;

  // ギャップは「受信した時刻（区間の終端）」を基準に窓内/窓外を判定する
  // （ギャップの大きさそのものではない）。区間終端の時刻が古ければ、
  // ギャップがどれだけ大きくても窓の外になる。
  //
  // 1,2,...,10 秒と徐々に大きくなるギャップを積む。
  //   stamps: 0,1,3,6,10,15,21,28,36,45,55 (区間終端の累積)
  //   gaps  :   1,2,3,4, 5, 6, 7, 8, 9,10 (秒)
  tracker.push(0.0);
  tracker.push(1.0);   // gap=1s,  終端 t=1
  tracker.push(3.0);   // gap=2s,  終端 t=3
  tracker.push(6.0);   // gap=3s,  終端 t=6
  tracker.push(10.0);  // gap=4s,  終端 t=10
  tracker.push(15.0);  // gap=5s,  終端 t=15
  tracker.push(21.0);  // gap=6s,  終端 t=21
  tracker.push(28.0);  // gap=7s,  終端 t=28
  tracker.push(36.0);  // gap=8s,  終端 t=36
  tracker.push(45.0);  // gap=9s,  終端 t=45
  tracker.push(55.0);  // gap=10s, 終端 t=55

  // now=55, window=30 → 窓は終端時刻 [25, 55] のギャップだけを含む。
  // 該当するのは終端 28/36/45/55（ギャップ 7/8/9/10 秒）の 4 件。
  // 終端 1/3/6/10/15/21（ギャップ 1〜6 秒）は窓の外で除外される。
  Quantiles q = tracker.compute(/*now_sec=*/55.0, /*window_sec=*/30.0);

  // 窓内の 4 件 [7000, 8000, 9000, 10000] ms から:
  //   p50: rank=0.5*3=1.5   → 8000 + 0.5*(9000-8000)  = 8500
  //   p99: rank=0.99*3=2.97 → 9000 + 0.97*(10000-9000) = 9970
  //   max: 10000
  // もし窓外のギャップ（1〜6 秒）が除外されずに混ざっていれば、
  // 中央値は全 10 件 [1000..10000] の中央（5500）になり、この期待値と一致しない。
  EXPECT_NEAR(q.p50_ms, 8500.0, 1e-6);
  EXPECT_NEAR(q.p99_ms, 9970.0, 1e-6);
  EXPECT_NEAR(q.max_ms, 10000.0, 1e-6);
  EXPECT_EQ(q.window_sec, 30u);
}

// main() は定義しない。ament_add_gtest は GTEST_LIBRARIES に gtest_main を
// 含めてリンクするため、ここで main() を定義すると多重定義になる。
