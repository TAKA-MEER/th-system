// ============================================================
// test_dev_mode_core.cpp — 開発モード（Spec-safety.md §10）の C++ 側
//
// 1. dev_mode_core.hpp: /system/dev_mode の JSON から effective だけを読み、
//    読めない・古い・未受信は「何も無視しない」に倒すこと。
// 2. obstacle_limiter_core の項目 scan_stop: /scan 途絶でも MANUAL だけ通し、
//    古い点群を観測として使わず v_reverse 上限に落ちること。AUTO は止める。
// ============================================================
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "th_safety/dev_mode_core.hpp"
#include "th_safety/obstacle_limiter_core.hpp"

using namespace th_safety;

namespace {

const char* kOnJson =
    R"({"dev_mode": true, "effective": {"link": true, "lidar_fault": true, "scan_stop": true},)"
    R"( "ignore": {"link": true, "lidar_fault": true, "scan_stop": true}})";

DevModeSnapshot snap_of(const std::string& json, double stamp_sec) {
  DevModeSnapshot s;
  s.received = true;
  s.stamp_sec = stamp_sec;
  s.effective = parse_dev_effective(json);
  return s;
}

}  // namespace

// ── 1. dev_mode_core ─────────────────────────────────────────

TEST(DevModeCore, ReadsEffectiveItems) {
  const auto s = snap_of(kOnJson, 100.0);
  EXPECT_TRUE(dev_item_effective(s, kDevItemLidarFault, 100.5));
  EXPECT_TRUE(dev_item_effective(s, kDevItemScanStop, 100.5));
}

TEST(DevModeCore, IgnoresSelectionThatIsNotEffective) {
  // ignore（選択）だけ真で effective が偽 → 無視しない。
  const auto s = snap_of(
      R"({"dev_mode": true, "effective": {"scan_stop": false},)"
      R"( "ignore": {"scan_stop": true, "lidar_fault": true}})", 100.0);
  EXPECT_FALSE(dev_item_effective(s, kDevItemScanStop, 100.0));
  EXPECT_FALSE(dev_item_effective(s, kDevItemLidarFault, 100.0));
}

TEST(DevModeCore, MasterOffMeansNothing) {
  const auto s = snap_of(R"({"dev_mode": false, "effective": {"scan_stop": true}})", 100.0);
  EXPECT_FALSE(dev_item_effective(s, kDevItemScanStop, 100.0));
}

TEST(DevModeCore, MalformedOrWrongTypesMeanNothing) {
  for (const char* bad : {
           "", "not json", "[]", "{\"dev_mode\": true}",
           R"({"dev_mode": "true", "effective": {"scan_stop": true}})",
           R"({"dev_mode": true, "effective": {"scan_stop": "true"}})",
           R"({"dev_mode": true, "effective": {"scan_stop": 1}})",
           R"({"dev_mode": true, "effective": ["scan_stop"]})",
           R"({"dev_mode": true, "effective": {"scan_stop": true})",  // 閉じ括弧欠け
       }) {
    SCOPED_TRACE(bad);
    const auto s = snap_of(bad, 100.0);
    EXPECT_FALSE(dev_item_effective(s, kDevItemScanStop, 100.0));
  }
}

TEST(DevModeCore, NotReceivedMeansNothing) {
  DevModeSnapshot s;
  s.effective = parse_dev_effective(kOnJson);  // 中身があっても received=false なら無効
  EXPECT_FALSE(dev_item_effective(s, kDevItemScanStop, 0.0));
}

TEST(DevModeCore, StaleMeansNothing) {
  const auto s = snap_of(kOnJson, 100.0);
  EXPECT_TRUE(dev_item_effective(s, kDevItemScanStop, 100.0 + kDevStateStaleSec));
  EXPECT_FALSE(dev_item_effective(s, kDevItemScanStop, 100.0 + kDevStateStaleSec + 0.01));
}

// ── 2. obstacle_limiter_core の scan_stop ─────────────────────

namespace {

ScanGeometryInfo full_geometry() {
  ScanGeometryInfo g;
  g.angle_min = -kPi;
  g.angle_increment = (2.0 * kPi) / 720.0;
  g.num_ranges = 720;
  return g;
}

ObstacleLimiterParams make_params() {
  ObstacleLimiterParams p;
  p.obstacle_floor_distance_m = 0.4;
  p.hysteresis_band_m = 0.1;
  p.brake_accel_mps2 = 1.0;
  p.obstacle_cone_half_width_rad = 0.5;
  p.obstacle_cone_half_width_reverse_rad = 0.6;
  p.v_reverse = 0.2;
  p.w_max = 0.6;
  p.w_align_max = 0.3;
  p.manual_joy_timeout_sec = 1.0;
  p.state_stale_sec = 1.5;
  p.muxed_stale_sec = 0.2;
  p.scan_stale_sec = 0.3;
  p.lock_stale_sec = 0.5;
  p.blind_calibrated = true;
  return p;
}

// 手動走行（MANUAL・ジョイ新鮮）で 0.8 m/s 前進を要求。scan は未受信。
ObstacleLimiterInputs make_manual_no_scan(double now) {
  ObstacleLimiterInputs in;
  in.now_sec = now;
  in.muxed = Stamped<Twist2D>{true, now, Twist2D{0.8, 0.3}};
  in.manual = Stamped<Twist2D>{true, now, Twist2D{0.8, 0.3}};
  in.state.received = true;
  in.state.stamp_sec = now;
  in.state.mode = "MANUAL";
  in.state.zone = Zone::OUT;
  in.state.auto_brake = true;
  in.estop = Stamped<bool>{true, now, false};
  in.fault_lock = Stamped<bool>{true, now, false};
  in.screen_limit_mps = 1.0;
  in.mode_limit_mps = 1.0;
  return in;
}

}  // namespace

TEST(ObstacleLimiterDevScanStop, OffStopsManualWithoutScan) {
  ObstacleLimiterCore core;
  const auto out = core.update(make_manual_no_scan(1000.0), make_params());
  EXPECT_EQ(out.action, LimiterAction::STOP);
  EXPECT_EQ(out.out.linear_x, 0.0);
  EXPECT_EQ(out.out.angular_z, 0.0);
}

TEST(ObstacleLimiterDevScanStop, OnLetsManualMoveCappedAtVReverse) {
  const auto p = make_params();
  ObstacleLimiterCore core;
  auto in = make_manual_no_scan(1000.0);
  in.dev_ignore_scan_stop = true;
  const auto out = core.update(in, p);
  EXPECT_EQ(out.source_class, SourceClass::MANUAL);
  EXPECT_GT(out.out.linear_x, 0.0);
  EXPECT_LE(out.out.linear_x, p.v_reverse + 1e-9);
  EXPECT_DOUBLE_EQ(out.applied_limit_mps, p.v_reverse);
  EXPECT_DOUBLE_EQ(out.nearest_obstacle_m, -1.0) << "未観測は「不明」(-1) のまま";
}

TEST(ObstacleLimiterDevScanStop, OnStillStopsAuto) {
  ObstacleLimiterCore core;
  auto in = make_manual_no_scan(1000.0);
  in.dev_ignore_scan_stop = true;
  in.state.mode = "FOLLOW";   // 手動系モードでない → AUTO
  const auto out = core.update(in, make_params());
  EXPECT_EQ(out.source_class, SourceClass::AUTO);
  EXPECT_EQ(out.action, LimiterAction::STOP);
  EXPECT_EQ(out.out.linear_x, 0.0);
}

TEST(ObstacleLimiterDevScanStop, OnStillStopsWhenJoyIsStale) {
  // ジョイが途絶えたら MANUAL ではない（L7）→ AUTO 扱いで止める。
  ObstacleLimiterCore core;
  auto in = make_manual_no_scan(1000.0);
  in.dev_ignore_scan_stop = true;
  in.manual.stamp_sec = 1000.0 - 5.0;
  const auto out = core.update(in, make_params());
  EXPECT_EQ(out.action, LimiterAction::STOP);
  EXPECT_EQ(out.out.linear_x, 0.0);
}

TEST(ObstacleLimiterDevScanStop, OldScanIsNotTreatedAsClear) {
  // 一度は受信した「空き」の点群が途絶で古くなった状態。項目 ON でも古い点群を
  // 観測に使わず、上限は v_reverse に落ちること（§3.4.2）。
  const auto p = make_params();
  ObstacleLimiterCore core;
  auto in = make_manual_no_scan(1000.0);
  in.dev_ignore_scan_stop = true;
  in.scan.received = true;
  in.scan.stamp_sec = 1000.0 - 5.0;
  in.scan.geometry = full_geometry();
  in.scan.ranges.assign(in.scan.geometry.num_ranges, 30.0);
  const auto out = core.update(in, p);
  EXPECT_LE(out.out.linear_x, p.v_reverse + 1e-9);
  EXPECT_DOUBLE_EQ(out.applied_limit_mps, p.v_reverse);
  EXPECT_DOUBLE_EQ(out.nearest_obstacle_m, -1.0);
}

TEST(ObstacleLimiterDevScanStop, LocksStillWin) {
  ObstacleLimiterCore core;
  auto in = make_manual_no_scan(1000.0);
  in.dev_ignore_scan_stop = true;
  in.estop.value = true;
  EXPECT_EQ(core.update(in, make_params()).out.linear_x, 0.0);
  ObstacleLimiterCore core2;
  in.estop.value = false;
  in.fault_lock.value = true;
  EXPECT_EQ(core2.update(in, make_params()).out.linear_x, 0.0);
}

TEST(ObstacleLimiterDevScanStop, FreshScanIsUnaffected) {
  // scan が新鮮なら項目の有無で結果は変わらない（通常経路には触らない）。
  const auto p = make_params();
  auto in = make_manual_no_scan(1000.0);
  in.scan.received = true;
  in.scan.stamp_sec = 1000.0;
  in.scan.geometry = full_geometry();
  in.scan.ranges.assign(in.scan.geometry.num_ranges, 30.0);
  ObstacleLimiterCore off_core;
  const auto off = off_core.update(in, p);
  in.dev_ignore_scan_stop = true;
  ObstacleLimiterCore on_core;
  const auto on = on_core.update(in, p);
  EXPECT_DOUBLE_EQ(off.out.linear_x, on.out.linear_x);
  EXPECT_DOUBLE_EQ(off.applied_limit_mps, on.applied_limit_mps);
}
