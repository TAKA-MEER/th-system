// test_obstacle_min_points.cpp — WS-9P (2026-09-04)
//
// 実機フィードバック「何も無いところで止まってしばらくしてまた走り出した」。
// observe_cone() が円錐内の**最小値をそのまま**採用していたため、スキャン 1 枚の
// ノイズ 1 点で停止距離を割り、次のスキャンで消えるので独りでに再開していた。
//
// 実測 (2026-09-04): /scan_filtered は 720 点・分解能 0.499°、前方円錐 (±28.6°) に
// 105 点。停止距離 0.425 m では 1 ビームの弧長が 3.7 mm なので、幅 2 cm の実体は
// 5 点以上に映る。したがって「3 点そろわないと障害物とみなさない」は実体を
// 取りこぼさず、孤立した 1〜2 点のノイズだけを落とす。
//
// ここで固定するのは次の 4 つ:
//   1. 孤立したノイズ 1 点では最近傍が下がらない
//   2. 実体（連続した複数点）はちゃんと最近傍になる
//   3. 有効点が min_points に満たないときは最小値へ退避する（安全側）
//   4. min_points=1 は従来どおりの最小値そのもの
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "th_safety/obstacle_limiter_core.hpp"

namespace {

// 720 点 / 360°（実機の /scan_filtered と同じ）。全点を far で埋める。
th_safety::ScanSnapshot MakeScan(double far_m) {
  th_safety::ScanSnapshot scan;
  scan.received = true;
  scan.stamp_sec = 100.0;
  scan.geometry.angle_min = -th_safety::kPi;
  scan.geometry.angle_increment = (2.0 * th_safety::kPi) / 720.0;
  scan.geometry.num_ranges = 720;
  scan.ranges.assign(720, far_m);
  return scan;
}

// 正面（index 358 付近が 0°）に n 点だけ近い値を置く。
void PutNear(th_safety::ScanSnapshot& scan, std::size_t start, std::size_t n, double near_m) {
  for (std::size_t i = 0; i < n; ++i) {
    scan.ranges[(start + i) % scan.ranges.size()] = near_m;
  }
}

constexpr double kHalfWidth = 0.5;   // rad（実機の obstacle_cone_half_width_rad）

}  // namespace

TEST(ObstacleMinPoints, IsolatedSingleNoisePointIsIgnored) {
  auto scan = MakeScan(3.0);
  PutNear(scan, 358, 1, 0.20);   // ノイズ 1 点だけ 0.20m
  const auto cone = th_safety::observe_cone(scan, 0.0, kHalfWidth, 3);
  EXPECT_TRUE(cone.covered);
  EXPECT_DOUBLE_EQ(cone.nearest_m, 3.0)
      << "ノイズ 1 点で最近傍が下がってはいけない（WS-9P）";
}

TEST(ObstacleMinPoints, TwoNoisePointsAreStillIgnored) {
  auto scan = MakeScan(3.0);
  PutNear(scan, 358, 2, 0.20);
  const auto cone = th_safety::observe_cone(scan, 0.0, kHalfWidth, 3);
  EXPECT_DOUBLE_EQ(cone.nearest_m, 3.0);
}

TEST(ObstacleMinPoints, RealObstacleWithThreePointsIsDetected) {
  auto scan = MakeScan(3.0);
  PutNear(scan, 358, 3, 0.20);
  const auto cone = th_safety::observe_cone(scan, 0.0, kHalfWidth, 3);
  EXPECT_DOUBLE_EQ(cone.nearest_m, 0.20)
      << "3 点そろえば実体とみなして停止できなければならない";
}

TEST(ObstacleMinPoints, TwoCentimeterObstacleAtStoppingDistanceIsDetected) {
  // 停止距離 0.425m で幅 2cm の実体は 5 点以上に映る（実測に基づく見積り）。
  // 既定の 3 点では確実に検知できることを固定する。
  auto scan = MakeScan(3.0);
  PutNear(scan, 358, 5, 0.425);
  const auto cone = th_safety::observe_cone(scan, 0.0, kHalfWidth, 3);
  EXPECT_DOUBLE_EQ(cone.nearest_m, 0.425);
}

TEST(ObstacleMinPoints, FallsBackToMinimumWhenTooFewValidPoints) {
  // 円錐内の有効点が min_points に満たない（LiDAR 異常）。
  // ここで「空き」に倒すと危険側なので、最小値へ退避する。
  auto scan = MakeScan(std::numeric_limits<double>::infinity());
  PutNear(scan, 358, 2, 0.20);   // 有効点は 2 点だけ
  const auto cone = th_safety::observe_cone(scan, 0.0, kHalfWidth, 3);
  EXPECT_TRUE(cone.covered);
  EXPECT_DOUBLE_EQ(cone.nearest_m, 0.20)
      << "有効点が足りないときは最小値へ退避すること（安全側）";
}

TEST(ObstacleMinPoints, MinPointsOneKeepsLegacyMinimumBehaviour) {
  auto scan = MakeScan(3.0);
  PutNear(scan, 358, 1, 0.20);
  const auto cone = th_safety::observe_cone(scan, 0.0, kHalfWidth, 1);
  EXPECT_DOUBLE_EQ(cone.nearest_m, 0.20);
}

TEST(ObstacleMinPoints, DefaultArgumentIsLegacyMinimum) {
  // 既存の 2 引数呼び出し（幾何の等価性テスト等）が意味を変えないこと。
  auto scan = MakeScan(3.0);
  PutNear(scan, 358, 1, 0.20);
  const auto cone = th_safety::observe_cone(scan, 0.0, kHalfWidth);
  EXPECT_DOUBLE_EQ(cone.nearest_m, 0.20);
}

namespace {

th_safety::ObstacleLimiterParams MakeParams(std::size_t min_points) {
  th_safety::ObstacleLimiterParams p;
  p.obstacle_cone_half_width_rad = kHalfWidth;
  p.obstacle_cone_half_width_reverse_rad = kHalfWidth;
  p.obstacle_floor_distance_m = 0.425;
  p.brake_accel_mps2 = 0.5;
  p.obstacle_min_points = min_points;
  p.v_reverse = 0.25;
  p.w_max = 0.6;
  p.w_align_max = 0.6;
  p.manual_joy_timeout_sec = 1.0;
  p.state_stale_sec = 1.0;
  p.muxed_stale_sec = 1.0;
  p.scan_stale_sec = 1.0;
  p.lock_stale_sec = 1.0;
  p.blind_calibrated = true;
  return p;
}

th_safety::ObstacleLimiterInputs MakeInputs(const th_safety::ScanSnapshot& scan) {
  th_safety::ObstacleLimiterInputs in;
  in.now_sec = 100.0;
  in.muxed.received = true;
  in.muxed.stamp_sec = 100.0;
  in.muxed.value.linear_x = 0.3;
  in.scan = scan;
  in.state.received = true;
  in.state.stamp_sec = 100.0;
  in.estop.received = true;
  in.estop.stamp_sec = 100.0;
  in.estop.value = false;
  in.fault_lock.received = true;
  in.fault_lock.stamp_sec = 100.0;
  in.fault_lock.value = false;
  in.screen_limit_mps = 1.0;
  in.mode_limit_mps = 1.0;
  return in;
}

}  // namespace

TEST(ObstacleMinPoints, NoiseDoesNotStopTheRobotThroughUpdate) {
  // observe_cone 単体でなく update() まで通して、ノイズ 1 点で速度がゼロに
  // 落ちないことを確かめる（実機で起きた事象そのもの）。
  auto scan = MakeScan(3.0);
  PutNear(scan, 358, 1, 0.20);   // ノイズ 1 点

  th_safety::ObstacleLimiterCore core;
  const auto out = core.update(MakeInputs(scan), MakeParams(3));
  EXPECT_GT(out.out.linear_x, 0.0)
      << "ノイズ 1 点で機体を止めてはいけない（WS-9P）";
}

TEST(ObstacleMinPoints, RealObstacleStillStopsTheRobotThroughUpdate) {
  // 対照。実体（5 点）ならこれまでどおり止まること。
  auto scan = MakeScan(3.0);
  PutNear(scan, 358, 5, 0.20);

  th_safety::ObstacleLimiterCore core;
  const auto out = core.update(MakeInputs(scan), MakeParams(3));
  EXPECT_DOUBLE_EQ(out.out.linear_x, 0.0)
      << "実体の前では止まらなければならない";
}
