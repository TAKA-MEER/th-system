// ============================================================
// safety_monitor_core.hpp — safety_monitor の判定ロジックの純粋コア
//
// ROS2 非依存（rclcpp を include しない）。follow_planner_core.py と同じ
// 二層構造の思想（CLAUDE.md「追従ロジックの二層構造」）。判定そのものは
// ここに置き、safety_monitor.cpp は ROS2 との配線（sub/pub/timer）だけを行う。
//
// 対応する仕様（DetailedDesign-wp2.md `WP-SAFE-01`）:
//   §5.1（DetailedDesign-safety.md）  重大フォルトの分類    → classify_severity()
//   §5.2.1（同上）                    fault_lock の合成式    → compute_fault_lock()
//   §4.1（wp2 WP-SAFE-01）            MUX_DEAD（双方向）     → detect_mux_dead()
//   §4.1（同上）                      DRIVE_RUNAWAY          → is_runaway_condition() + HoldTimer
//   §4.1（同上）                      STATE_INCONSISTENT     → detect_state_inconsistent()
//   §4.2（同上）                      UI 非常停止の生存確認  → UiEstopLatch
//   §3.2 / §6.3.1（同上 / safety.md） clear_estop_ui の受理  → decide_clear_estop_ui()
//   F-5（wp2 WP-SAFE-01 §4.3・O-7）   enabled_targets ゲート → is_target_enabled()
// ============================================================
#ifndef TH_SAFETY_SAFETY_MONITOR_CORE_HPP_
#define TH_SAFETY_SAFETY_MONITOR_CORE_HPP_

#include <map>
#include <set>
#include <string>
#include <vector>

namespace th_safety {

// ── §5.1 フォルトの 2 階級 ────────────────────────────────────
enum class Severity { RECOVERABLE, CRITICAL };

// DetailedDesign-safety.md §5.1 の分類表。
// RECOVERABLE: LIDAR_LOST / ESP32_DISCONNECTED / PERSON_TRACKER_LOST / UI_DISCONNECTED
// CRITICAL:    LIMITER_DEAD / MUX_DEAD / DRIVE_RUNAWAY / STATE_INCONSISTENT /
//              LOCALIZATION_LOST（検出は WP-SAFE-05。ここでは分類表にのみ登場）/
//              ESTOP_BYPASS_ACTIVE（DEBT-1 バイパス検出。safety.md §11.1）
// 未知の fault_type は RECOVERABLE 側にフォールバックする
// （fault_type はこのノード自身が閉じた集合で生成する文字列であり、外部入力ではない）。
Severity classify_severity(const std::string& fault_type);

inline const char* severity_to_string(Severity s) {
  return s == Severity::CRITICAL ? "CRITICAL" : "RECOVERABLE";
}

// ── §5.2.1 重大フォルトは層 3 で止める ────────────────────────
// /safety/fault_lock = LIDAR_LOST || ESP32_DISCONNECTED || (severity == CRITICAL)
bool compute_fault_lock(bool lidar_lost_active, bool esp32_disconnected_active,
                         bool any_critical_fault_active);

// ── §4.1 MUX_DEAD（双方向） ───────────────────────────────────
// muxed_stale / cmd_stale: 各トピックが mux_dead_ms 途絶しているか。
// muxed_last_nonzero / cmd_last_nonzero: 途絶していない場合の直近値が非ゼロか。
//
// twist_mux がロック中（lock 255/254）で両方とも「途絶」または「ゼロ」になる
// 正常時は誤検知しない（どちらの case も「途絶していない側」を要求するため）。
bool detect_mux_dead(bool muxed_stale, bool muxed_last_nonzero,
                      bool cmd_stale, bool cmd_last_nonzero);

// ── §4.1 DRIVE_RUNAWAY ────────────────────────────────────────
// 「停止指令中に動いている」を最優先で拾う（cmd がゼロ近傍なら feedback の
// 絶対値だけで判定する）。走行中は比の乖離（runaway_ratio）で判定する。
// 戻り値は「今この瞬間、乖離条件が成立しているか」であり、保持時間の判定は
// HoldTimer が別途行う（呼び出し側が runaway_hold_ms 継続を確認する）。
bool is_runaway_condition(double cmd_speed_abs, double feedback_speed_abs,
                           double runaway_ratio, double runaway_zero_threshold);

// 条件が連続して真である時間を積算する、ROS2 非依存の保持時間トラッカー。
// dt_sec は呼び出し側の周期（safety_monitor は check_period_ms 固定周期で
// 呼ぶ想定）。条件が偽になった時点でリセットする。
class HoldTimer {
 public:
  explicit HoldTimer(double hold_sec) : hold_sec_(hold_sec) {}

  // 戻り値: 今回の更新後、継続保持時間が hold_sec 以上になったら true。
  bool update(bool condition_active, double dt_sec) {
    if (condition_active) {
      held_sec_ += dt_sec;
    } else {
      held_sec_ = 0.0;
    }
    return held_sec_ >= hold_sec_;
  }

  void reset() { held_sec_ = 0.0; }
  double held_sec() const { return held_sec_; }

 private:
  double hold_sec_;
  double held_sec_ = 0.0;
};

// ── §4.1 STATE_INCONSISTENT ───────────────────────────────────
// state_stale: /system/state が state_stale_ms 途絶しているか。
// mode_states: モード → 有効な状態集合（起動時に読み込む。th_state/state_core.py
//              の MODE_STATES と names.md §3 の写し。default_mode_states() 参照）。
bool detect_state_inconsistent(bool state_stale, const std::string& mode,
                                const std::string& state,
                                const std::map<std::string, std::set<std::string>>& mode_states);

// names.md §3 のモード×状態表の写し（th_state/th_state/state_core.py の
// MODE_STATES と同期させること。「状態集合を起動時に読み込む」の実体。
// th_state が壊れた形の状態を出したときの最後の網 — wp2 WP-SAFE-01 §4.1）。
const std::map<std::string, std::set<std::string>>& default_mode_states();

// ── §4.2 UI 非常停止の生存確認 ────────────────────────────────
// true を受けたらラッチし、false を明示受信するまで解除しない
// （途絶では解除しない。押下側にラッチする — safety.md §6.3）。
// ラッチ中に estop_ui_lease_ms 途絶したら UI_DISCONNECTED（回復フォルト）を
// 別途立てる（is_disconnected() が判定するのはそのための情報だけで、
// ラッチ自体は解除しない）。
class UiEstopLatch {
 public:
  void on_true(double stamp_sec) {
    latched_ = true;
    last_true_stamp_sec_ = stamp_sec;
    has_received_true_ = true;
  }

  void on_false() {
    latched_ = false;
    has_received_true_ = false;
  }

  bool latched() const { return latched_; }

  // ラッチ中に estop_ui_lease_ms（lease_sec）途絶しているか。
  bool is_disconnected(double now_sec, double lease_sec) const {
    if (!latched_ || !has_received_true_) {
      return false;
    }
    return (now_sec - last_true_stamp_sec_) > lease_sec;
  }

 private:
  bool latched_ = false;
  bool has_received_true_ = false;
  double last_true_stamp_sec_ = 0.0;
};

// ── §3.2・§6.3.1 clear_estop_ui の受理判定 ────────────────────
struct ClearEstopUiDecision {
  bool success;
  std::string message;
};

// 受理条件: estop_hw == false（物理側が押されていない）かつ重大フォルト無し。
ClearEstopUiDecision decide_clear_estop_ui(bool estop_hw, bool has_critical_fault);

// ── F-5 enabled_targets ゲート（O-7） ─────────────────────────
// enabled_targets に無い対象は監視しない（フォルトを立てない）。
bool is_target_enabled(const std::string& target, const std::vector<std::string>& enabled_targets);

// ── 起動直後の未受信を「途絶」と誤判定しないための判定 ────────
//
// 起動直後は DDS のマッチングが終わるまで 1 通も届かない。これを途絶として
// 扱うと、publisher が正常でも CRITICAL フォルト → ESTOP になる。
// 実機 2026-09-03: safety_monitor 起動 331.082 / grace 明け 334.082 に対し、
// obstacle_limiter は 331.35 から 20Hz で /safety/limiter_status を出して
// いたのに、safety_monitor が実際に受信し始めたのが 334.5 頃で grace を
// 0.4 秒超過。LIMITER_DEAD(CRITICAL) と ESP32_DISCONNECTED が同時に誤発火し
// mode が ESTOP に落ちた（操作者が毎回「確認」を押す羽目になった）。
//
// ただし「いつまで経っても来ない」＝本当に居ない場合は検知しなければ
// ならないので、未受信の猶予は startup_deadline_sec で頭打ちにする
// （grace を延ばす対処は監視が盲目になる時間を延ばすので採らない）。
//
//   ever_received       : そのトピックを一度でも受信したか
//   since_last_sec      : 直近受信からの経過 [s]
//   timeout_sec         : 途絶とみなすしきい値 [s]（従来値。緩めない）
//   since_start_sec     : ノード起動からの経過 [s]
//   startup_deadline_sec: 未受信を許す上限 [s]。これを超えても 1 通も
//                         来なければ途絶として検知する
inline bool is_timeout_fault(bool ever_received,
                             double since_last_sec,
                             double timeout_sec,
                             double since_start_sec,
                             double startup_deadline_sec) {
  if (!ever_received) return since_start_sec > startup_deadline_sec;
  return since_last_sec > timeout_sec;
}

}  // namespace th_safety

#endif  // TH_SAFETY_SAFETY_MONITOR_CORE_HPP_
