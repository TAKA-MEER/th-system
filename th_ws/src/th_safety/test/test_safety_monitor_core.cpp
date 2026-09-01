// ============================================================
// test_safety_monitor_core.cpp — safety_monitor_core.hpp の単体試験
//
// 対応する仕様（DetailedDesign-wp2.md `WP-SAFE-01` §7）:
//   test_fault_severity                         : §5.1 の分類表を 1 行ずつ
//   test_fault_lock_includes_critical            : F-2（§5.2.1）
//   test_mux_dead_bidirectional                  : §4.1。ロック中を誤検知しない
//   test_runaway_zero_cmd                        : §4.1。停止指令中の非ゼロ実測
//   test_state_inconsistent                      : §4.1
//   test_ui_estop_latch                          : §4.2。途絶でラッチ維持＋UI_DISCONNECTED
//   test_clear_estop_ui_requires_hw_released      : §3.2。物理側が押されている間は拒否
//   test_clear_estop_ui_logs                      : 受理・拒否の両方が記録される
//   test_enabled_targets                          : F-5
// ============================================================
#include <gtest/gtest.h>

#include "th_safety/safety_monitor_core.hpp"

using namespace th_safety;

// ── test_fault_severity ─────────────────────────────────────────────────

TEST(SafetyMonitorCore, FaultSeverityRecoverable) {
  EXPECT_EQ(classify_severity("LIDAR_LOST"), Severity::RECOVERABLE);
  EXPECT_EQ(classify_severity("ESP32_DISCONNECTED"), Severity::RECOVERABLE);
  EXPECT_EQ(classify_severity("PERSON_TRACKER_LOST"), Severity::RECOVERABLE);
  EXPECT_EQ(classify_severity("UI_DISCONNECTED"), Severity::RECOVERABLE);
}

TEST(SafetyMonitorCore, FaultSeverityCritical) {
  EXPECT_EQ(classify_severity("LIMITER_DEAD"), Severity::CRITICAL);
  EXPECT_EQ(classify_severity("MUX_DEAD"), Severity::CRITICAL);
  EXPECT_EQ(classify_severity("DRIVE_RUNAWAY"), Severity::CRITICAL);
  EXPECT_EQ(classify_severity("STATE_INCONSISTENT"), Severity::CRITICAL);
  EXPECT_EQ(classify_severity("LOCALIZATION_LOST"), Severity::CRITICAL);
  EXPECT_EQ(classify_severity("ESTOP_BYPASS_ACTIVE"), Severity::CRITICAL);
}

TEST(SafetyMonitorCore, FaultSeverityUnknownDefaultsRecoverable) {
  EXPECT_EQ(classify_severity("NONE"), Severity::RECOVERABLE);
  EXPECT_EQ(classify_severity("SOMETHING_NEW"), Severity::RECOVERABLE);
}

// ── test_fault_lock_includes_critical（F-2） ──────────────────────────────

TEST(SafetyMonitorCore, FaultLockIncludesCritical) {
  // 必須通信系統の断だけ（従来どおり）
  EXPECT_TRUE(compute_fault_lock(/*lidar=*/true, /*esp32=*/false, /*critical=*/false));
  EXPECT_TRUE(compute_fault_lock(false, true, false));
  // どちらでもないが重大フォルトが立っている → F-2: それでも lock される
  EXPECT_TRUE(compute_fault_lock(false, false, /*critical=*/true));
  // 何も無ければ lock されない
  EXPECT_FALSE(compute_fault_lock(false, false, false));
}

// ── test_mux_dead_bidirectional ───────────────────────────────────────────

TEST(SafetyMonitorCore, MuxDeadDetectsMuxedFlowingButCmdStale) {
  // muxed が非ゼロで流れているのに cmd が途絶 → MUX_DEAD
  EXPECT_TRUE(detect_mux_dead(/*muxed_stale=*/false, /*muxed_nonzero=*/true,
                               /*cmd_stale=*/true, /*cmd_nonzero=*/false));
}

TEST(SafetyMonitorCore, MuxDeadDetectsMuxedStaleButCmdFlowing) {
  // muxed が途絶しているのに cmd が非ゼロのまま出続けている → MUX_DEAD
  EXPECT_TRUE(detect_mux_dead(/*muxed_stale=*/true, /*muxed_nonzero=*/false,
                               /*cmd_stale=*/false, /*cmd_nonzero=*/true));
}

TEST(SafetyMonitorCore, MuxDeadDoesNotMisfireWhenLocked) {
  // twist_mux がロック中（estop/fault_lock）: 両方とも途絶 or ゼロ。誤検知しない。
  EXPECT_FALSE(detect_mux_dead(/*muxed_stale=*/true, /*muxed_nonzero=*/false,
                                /*cmd_stale=*/true, /*cmd_nonzero=*/false));
  // 両方生きていてどちらもゼロ（停止中の正常状態）も誤検知しない
  EXPECT_FALSE(detect_mux_dead(/*muxed_stale=*/false, /*muxed_nonzero=*/false,
                                /*cmd_stale=*/false, /*cmd_nonzero=*/false));
  // 両方生きていて両方非ゼロ（正常走行中）も誤検知しない
  EXPECT_FALSE(detect_mux_dead(/*muxed_stale=*/false, /*muxed_nonzero=*/true,
                                /*cmd_stale=*/false, /*cmd_nonzero=*/true));
}

// ── test_runaway_zero_cmd ─────────────────────────────────────────────────

TEST(SafetyMonitorCore, RunawayZeroCmdNonzeroFeedbackIsCondition) {
  // 停止指令中(cmd≈0)に実測が動いている → 最優先ケース
  EXPECT_TRUE(is_runaway_condition(/*cmd=*/0.0, /*feedback=*/0.15,
                                    /*ratio=*/1.5, /*zero_threshold=*/0.02));
}

TEST(SafetyMonitorCore, RunawayZeroCmdZeroFeedbackIsNotCondition) {
  EXPECT_FALSE(is_runaway_condition(0.0, 0.0, 1.5, 0.02));
  EXPECT_FALSE(is_runaway_condition(0.01, 0.01, 1.5, 0.02));  // 両方閾値未満
}

TEST(SafetyMonitorCore, RunawayMovingRatioDeviationIsCondition) {
  // cmd=0.5 に対し ratio=1.5 の許容域は [0.333, 0.75]。1.0 は乖離。
  EXPECT_TRUE(is_runaway_condition(/*cmd=*/0.5, /*feedback=*/1.0, 1.5, 0.02));
  // 0.5 は許容域内 → 乖離ではない
  EXPECT_FALSE(is_runaway_condition(/*cmd=*/0.5, /*feedback=*/0.5, 1.5, 0.02));
}

TEST(SafetyMonitorCore, RunawayHoldTimerRequiresContinuousHold) {
  HoldTimer hold(/*hold_sec=*/0.5);
  // 250ms 継続では未確定
  EXPECT_FALSE(hold.update(true, 0.25));
  EXPECT_FALSE(hold.update(true, 0.20));
  // 550ms 目でようやく成立
  EXPECT_TRUE(hold.update(true, 0.10));
  // 条件が途切れたらリセットされる
  EXPECT_FALSE(hold.update(false, 0.10));
  EXPECT_FALSE(hold.update(true, 0.40));  // 再カウント開始、まだ 0.5s 未満
}

// ── test_state_inconsistent ───────────────────────────────────────────────

TEST(SafetyMonitorCore, StateInconsistentOnStaleness) {
  const auto& table = default_mode_states();
  EXPECT_TRUE(detect_state_inconsistent(/*stale=*/true, "IDLE", "NONE", table));
}

TEST(SafetyMonitorCore, StateInconsistentOnInvalidPair) {
  const auto& table = default_mode_states();
  // FOLLOW モードに "RUNNING_CHECK" という状態は存在しない（OPCHECK の状態）
  EXPECT_TRUE(detect_state_inconsistent(/*stale=*/false, "FOLLOW", "RUNNING_CHECK", table));
  // 未知のモードそのもの
  EXPECT_TRUE(detect_state_inconsistent(false, "NOT_A_MODE", "NONE", table));
}

TEST(SafetyMonitorCore, StateConsistentValidPairIsNotFault) {
  const auto& table = default_mode_states();
  EXPECT_FALSE(detect_state_inconsistent(false, "FOLLOW", "RUN", table));
  EXPECT_FALSE(detect_state_inconsistent(false, "IDLE", "NONE", table));
  EXPECT_FALSE(detect_state_inconsistent(false, "LINE", "ARRIVED", table));
}

// ── test_ui_estop_latch ───────────────────────────────────────────────────

TEST(SafetyMonitorCore, UiEstopLatchStaysLatchedOnStaleness) {
  UiEstopLatch latch;
  latch.on_true(/*stamp_sec=*/0.0);
  EXPECT_TRUE(latch.latched());

  // estop_ui_lease_ms（1.5s）を大きく超えて途絶しても、ラッチは維持される
  // （途絶では false にしない。押下側にラッチする — safety.md §6.3）。
  EXPECT_TRUE(latch.latched());
  EXPECT_FALSE(latch.is_disconnected(/*now_sec=*/1.0, /*lease_sec=*/1.5));
  EXPECT_TRUE(latch.is_disconnected(/*now_sec=*/3.0, /*lease_sec=*/1.5));
  // is_disconnected() は情報だけを返し、ラッチ自体は解除しない
  EXPECT_TRUE(latch.latched());
}

TEST(SafetyMonitorCore, UiEstopLatchClearsOnlyOnExplicitFalse) {
  UiEstopLatch latch;
  latch.on_true(0.0);
  latch.on_false();
  EXPECT_FALSE(latch.latched());
  EXPECT_FALSE(latch.is_disconnected(100.0, 1.5));  // 未受信状態は UI_DISCONNECTED にしない
}

// ── test_clear_estop_ui_requires_hw_released ──────────────────────────────

TEST(SafetyMonitorCore, ClearEstopUiRejectsWhileHwPressed) {
  auto d = decide_clear_estop_ui(/*estop_hw=*/true, /*has_critical_fault=*/false);
  EXPECT_FALSE(d.success);
}

TEST(SafetyMonitorCore, ClearEstopUiRejectsOnCriticalFault) {
  auto d = decide_clear_estop_ui(/*estop_hw=*/false, /*has_critical_fault=*/true);
  EXPECT_FALSE(d.success);
}

TEST(SafetyMonitorCore, ClearEstopUiAcceptsWhenHwReleasedAndNoCritical) {
  auto d = decide_clear_estop_ui(/*estop_hw=*/false, /*has_critical_fault=*/false);
  EXPECT_TRUE(d.success);
}

// ── test_clear_estop_ui_logs ──────────────────────────────────────────────
// 実際の RCLCPP_INFO/WARN によるログ出力（who/時刻/mode/state 付き）は
// safety_monitor.cpp（ROS ノード側）の責務であり gtest では検証できない。
// ここでは「受理・拒否のどちらも呼び出し側がログへ落とせる情報
// （message が空でない）を返す」ことだけを純粋関数側の契約として確認する。

TEST(SafetyMonitorCore, ClearEstopUiDecisionAlwaysCarriesMessage) {
  auto rejected_hw = decide_clear_estop_ui(true, false);
  auto rejected_fault = decide_clear_estop_ui(false, true);
  auto accepted = decide_clear_estop_ui(false, false);

  EXPECT_FALSE(rejected_hw.message.empty());
  EXPECT_FALSE(rejected_fault.message.empty());
  EXPECT_FALSE(accepted.message.empty());
  // 拒否理由が区別できる（同じ文言に潰れていない）
  EXPECT_NE(rejected_hw.message, rejected_fault.message);
  EXPECT_NE(rejected_hw.message, accepted.message);
}

// ── test_enabled_targets（F-5） ────────────────────────────────────────────

TEST(SafetyMonitorCore, EnabledTargetsGatesUnknownTargets) {
  const std::vector<std::string> enabled = {"lidar", "esp32"};
  EXPECT_TRUE(is_target_enabled("lidar", enabled));
  EXPECT_TRUE(is_target_enabled("esp32", enabled));
  // 段階 4 まで無効な対象（publisher が居ない）は false のまま
  EXPECT_FALSE(is_target_enabled("person", enabled));
  EXPECT_FALSE(is_target_enabled("limiter", enabled));
}

TEST(SafetyMonitorCore, EnabledTargetsEmptyMeansNothingMonitored) {
  const std::vector<std::string> enabled = {};
  EXPECT_FALSE(is_target_enabled("lidar", enabled));
  EXPECT_FALSE(is_target_enabled("esp32", enabled));
}
