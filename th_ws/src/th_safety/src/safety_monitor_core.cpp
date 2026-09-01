// ============================================================
// safety_monitor_core.cpp — safety_monitor_core.hpp の実装
// ============================================================
#include "th_safety/safety_monitor_core.hpp"

#include <algorithm>
#include <cmath>

namespace th_safety {

Severity classify_severity(const std::string& fault_type) {
  static const std::set<std::string> kCritical = {
      "LIMITER_DEAD",
      "MUX_DEAD",
      "DRIVE_RUNAWAY",
      "STATE_INCONSISTENT",
      "LOCALIZATION_LOST",
      "ESTOP_BYPASS_ACTIVE",
  };
  return kCritical.count(fault_type) != 0 ? Severity::CRITICAL : Severity::RECOVERABLE;
}

bool compute_fault_lock(bool lidar_lost_active, bool esp32_disconnected_active,
                         bool any_critical_fault_active) {
  return lidar_lost_active || esp32_disconnected_active || any_critical_fault_active;
}

bool detect_mux_dead(bool muxed_stale, bool muxed_last_nonzero, bool cmd_stale,
                      bool cmd_last_nonzero) {
  const bool muxed_flowing_nonzero = !muxed_stale && muxed_last_nonzero;
  const bool cmd_flowing_nonzero = !cmd_stale && cmd_last_nonzero;

  // muxed が流れている（非ゼロ）のに cmd が途絶
  const bool case_a = muxed_flowing_nonzero && cmd_stale;
  // muxed が途絶しているのに cmd が非ゼロのまま出続けている
  const bool case_b = muxed_stale && cmd_flowing_nonzero;

  return case_a || case_b;
}

bool is_runaway_condition(double cmd_speed_abs, double feedback_speed_abs, double runaway_ratio,
                           double runaway_zero_threshold) {
  if (cmd_speed_abs <= runaway_zero_threshold) {
    // 停止指令中: 実測が閾値を超えて動いていたら乖離（最優先ケース）
    return feedback_speed_abs > runaway_zero_threshold;
  }
  // 走行中: 比の乖離判定（加減速中の一時的な乖離は HoldTimer が吸収する）
  const double lo = cmd_speed_abs / runaway_ratio;
  const double hi = cmd_speed_abs * runaway_ratio;
  return feedback_speed_abs < lo || feedback_speed_abs > hi;
}

bool detect_state_inconsistent(bool state_stale, const std::string& mode, const std::string& state,
                                const std::map<std::string, std::set<std::string>>& mode_states) {
  if (state_stale) {
    return true;
  }
  auto it = mode_states.find(mode);
  if (it == mode_states.end()) {
    return true;  // 未知のモード
  }
  return it->second.find(state) == it->second.end();  // モードに存在しない状態
}

const std::map<std::string, std::set<std::string>>& default_mode_states() {
  // DetailedDesign-names.md §3 の写し。th_state/th_state/state_core.py の
  // MODE_STATES と同期させること（同期がずれても検出漏れになるだけで
  // 誤検知の方向にはならないよう、両者は独立に §3 の表から書き起こしてある）。
  static const std::map<std::string, std::set<std::string>> kModeStates = {
      {"INIT", {"CHECK"}},
      {"IDLE", {"NONE"}},
      {"ESTOP", {"NONE"}},
      {"CARRY", {"NONE"}},
      {"FOLLOW", {"SELECT", "CONFIRM", "RUN", "PAUSE"}},
      {"MANUAL", {"RUN", "PAUSE"}},
      {"TEACH_FOLLOW", {"ROUTE_SEL", "REC", "PAUSE", "SAVED"}},
      {"TEACH_MANUAL", {"ROUTE_SEL", "REC", "PAUSE", "SAVED"}},
      {"REPLAY", {"ROUTE_SEL", "LOCALIZE", "READY", "RUN", "PAUSE", "SAVED"}},
      {"LINE", {"SETUP", "PLANNED", "RUN", "PAUSE", "ARRIVED"}},
      {"LEASH", {"DEV_CHECK", "READY", "RUN", "HOLD", "PAUSE"}},
      {"PREP", {"MAPPING", "REGISTER", "RETURN", "EDIT", "PAUSE", "SAVED"}},
      {"PANEL_NAV", {"NAV", "BLOCKED", "PAUSE", "ALIGN"}},
      {"AT_PANEL", {"IDLE_P", "WORKING", "PAUSE"}},
      {"SUMMON", {"POINT", "WAIT_CLEAR", "NAV", "BLOCKED", "PAUSE", "ALIGN"}},
      {"HOME_NAV", {"NAV", "BLOCKED", "PAUSE"}},
      {"OPCHECK", {"LIST", "RUNNING_CHECK", "REPAIR"}},
      {"CALIB", {"LIST", "S1", "S2", "S3", "S4"}},
      // 旧 FSM（th_mode_manager）は "NONE" を状態として使わないモード名の
      // 集合が異なる。旧 FSM 由来の SystemState が来ることは無い想定だが、
      // 空文字状態のモードも起動直後は存在しうるため INIT だけは明示している。
  };
  return kModeStates;
}

ClearEstopUiDecision decide_clear_estop_ui(bool estop_hw, bool has_critical_fault) {
  if (estop_hw) {
    return {false, "estop_hw が押されたままのため拒否した"};
  }
  if (has_critical_fault) {
    return {false, "重大フォルトが発生中のため拒否した"};
  }
  return {true, "UI 非常停止ラッチを解除した"};
}

bool is_target_enabled(const std::string& target, const std::vector<std::string>& enabled_targets) {
  return std::find(enabled_targets.begin(), enabled_targets.end(), target) != enabled_targets.end();
}

}  // namespace th_safety
