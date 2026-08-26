// ============================================================
// obstacle_limiter_core.hpp — obstacle_limiter の判定ロジックの純粋コア
//
// ROS2 非依存（rclcpp / geometry_msgs / sensor_msgs を include しない）。
// follow_planner_core.py / safety_monitor_core.hpp と同じ二層構造の思想
// （CLAUDE.md「追従ロジックの二層構造」）。ROS ノード側（未実装。別作業
// パケットで配線する）が本ヘッダを呼び出し、購読／配信だけを担当する。
//
// 対応する仕様（docs/plan/detailed/DetailedDesign-safety.md）:
//   §3.2      自律／手動の判定             → compute_source_class()
//   §3.3      政策表                       → policy_stops()
//   §3.3.1    速度上限の合成               → ObstacleLimiterCore::update() 内
//   §3.4.2    入力の同期・stale 扱い       → ObstacleLimiterCore::update() 内
//   §3.4.3    距離→許容速度・ヒステリシス → v_allow() / stop_latched_
//   §3.5      不変条件 L1〜L7              → test/test_obstacle_limiter_core.cpp
//
// 角度→LaserScan 添字の変換は th_safety/scan_geometry.hpp に委譲する
// （Python/C++ 等価性が確認済みの唯一の正本。自前で floor() を書き直さない）。
//
// このコアは「画面由来の上限」「モード由来の上限」を数値として受け取る
// 設計にしている（screen_limit_mps / mode_limit_mps）。names.md §4 の
// SCREEN_ZONE 表や attributes.yaml の speed_limit をどの画面／モードに
// 対応させるかという「テーブル参照」はロジック層の責務ではなく、呼び出し側
// （ROS ノード。別パケット）が解決してから渡す設計にした。理由と、これが
// 設計書に明記されていない自己判断であることは実装報告に記載する。
// ============================================================
#ifndef TH_SAFETY_OBSTACLE_LIMITER_CORE_HPP_
#define TH_SAFETY_OBSTACLE_LIMITER_CORE_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "th_safety/scan_geometry.hpp"

namespace th_safety {

// ── 出力の 5 状態（LimiterStatus.msg の action 文字列と 1:1） ──────────
enum class LimiterAction { PASS, CLAMP, STOP, ZERO_STALE, BLOCKED_UNCALIBRATED };

inline const char* limiter_action_to_string(LimiterAction a) {
  switch (a) {
    case LimiterAction::PASS: return "PASS";
    case LimiterAction::CLAMP: return "CLAMP";
    case LimiterAction::STOP: return "STOP";
    case LimiterAction::ZERO_STALE: return "ZERO_STALE";
    case LimiterAction::BLOCKED_UNCALIBRATED: return "BLOCKED_UNCALIBRATED";
  }
  return "STOP";  // 到達しない想定（防御的デフォルト）
}

// ── §3.2 の 2 値 ────────────────────────────────────────────
enum class SourceClass { AUTO, MANUAL };

// ── §3.3 の 3 値（SystemState.msg の zone フィールドと 1:1） ───────────
enum class Zone { IN, OUT, NA };

// ── 速度指令の素の構造体（geometry_msgs::Twist を持ち込まない） ────────
struct Twist2D {
  double linear_x = 0.0;
  double angular_z = 0.0;
};

// ── 「鮮度」を持つトピック値の共通表現 ──────────────────────────
// received == false は「一度も受信していない」（§3.4.2: stale と同じ扱いにする）。
template <typename T>
struct Stamped {
  bool received = false;
  double stamp_sec = 0.0;
  T value{};
};

// /scan のうち判定に要る最小限の情報。
struct ScanSnapshot {
  bool received = false;
  double stamp_sec = 0.0;
  ScanGeometryInfo geometry;   // angle_min / angle_increment / num_ranges
  std::vector<double> ranges;  // geometry.num_ranges と同じ長さを期待
};

// /system/state のうち判定に要るフィールドの写し（SystemState.msg の部分集合）。
struct SystemStateSnapshot {
  bool received = false;
  double stamp_sec = 0.0;
  std::string mode;
  bool jog_active = false;
  Zone zone = Zone::NA;
  bool auto_brake = false;
};

// ── obstacle_limiter_core.hpp が呼び出し側に要求する入力 ──────────────
struct ObstacleLimiterInputs {
  double now_sec = 0.0;

  Stamped<Twist2D> muxed;     // /cmd_vel_muxed
  Stamped<Twist2D> manual;    // /cmd_vel_manual（値は使わず鮮度だけ見る。§3.2）
  ScanSnapshot scan;          // /scan
  SystemStateSnapshot state;  // /system/state
  Stamped<bool> estop;        // /safety/estop
  Stamped<bool> fault_lock;   // /safety/fault_lock

  double screen_limit_mps = 0.0;  // 画面由来の上限（呼び出し側で解決済み。ヘッダ冒頭コメント参照）
  double mode_limit_mps = 0.0;    // モード由来の上限（同上）
};

// ── 静的パラメータ（registry.yaml 由来。起動時に一度だけ決まる） ───────
struct ObstacleLimiterParams {
  double obstacle_floor_distance_m = 0.0;  // d_floor
  double hysteresis_band_m = 0.0;
  double brake_accel_mps2 = 0.0;
  double obstacle_cone_half_width_rad = 0.0;
  double obstacle_cone_half_width_reverse_rad = 0.0;
  double v_reverse = 0.0;
  double w_align_max = 0.0;

  double manual_joy_timeout_sec = 0.0;
  double state_stale_sec = 0.0;
  double muxed_stale_sec = 0.0;
  double scan_stale_sec = 0.0;
  double lock_stale_sec = 0.0;

  bool blind_calibrated = true;
  // [(a0_deg, a1_deg), ...]。lidar_filter / scan_geometry と同じ規約
  // （laser_link 基準・反時計回り正。度で持ち、使う直前にラジアンへ変換する）。
  std::vector<std::pair<double, double>> blind_angle_ranges_deg;
};

// ── 出力 ────────────────────────────────────────────────────
struct ObstacleLimiterOutput {
  LimiterAction action = LimiterAction::PASS;
  Twist2D out;
  double nearest_obstacle_m = 0.0;  // 不明（scan stale 等）なら -1.0、空き確認済みなら +infinity
  SourceClass source_class = SourceClass::AUTO;
  double applied_limit_mps = 0.0;   // 障害物クランプ「前」の速度上限（screen/mode/reverse/blind の合成）
};

// ── 純粋関数群（テストから直接呼べるように公開する） ────────────────

// §3.2。AUTO/MANUAL の分類。鮮度だけで決めない（L7: 迷ったら AUTO）。
SourceClass compute_source_class(const ObstacleLimiterInputs& in, const ObstacleLimiterParams& p);

// §3.3 の 4 行そのもの。AUTO は常に true（無効化不可・L7）。
// MANUAL の IN/OUT は auto_brake で決まる（既定値が違うだけで式は同じ。§3.3 注記）。
bool policy_stops(SourceClass source_class, Zone zone, bool auto_brake);

// §3.4.3。braking_distance の逆関数（mapless_target_speed() と同型）。
// ヒステリシスは含まない（ObstacleLimiterCore::update() が別途適用する）。
double v_allow(double nearest_m, double obstacle_floor_distance_m, double brake_accel_mps2);

// |val| を [0, max_abs] にクランプする。符号は保存する（L1・L4 の実装基盤）。
double clamp_toward_zero(double val, double max_abs);

// L5。direction_rad ± half_width_rad の扇が blind_angle_ranges_deg のいずれかと
// 重なっているか（角度だけの幾何判定。実スキャンは見ない）。
bool blind_direction_overlap(double direction_rad, double half_width_rad,
                              const std::vector<std::pair<double, double>>& blind_angle_ranges_deg);

// §3.4.3 判定コーン。direction_rad ± half_width_rad 内にある有限レンジの最小値。
// 見つからない場合・scan が cone を全くカバーしていない場合（scan_geometry 規約⑤）は
// std::nullopt（＝「観測なし」。呼び出し側は空きとして扱う）。
std::optional<double> nearest_in_cone(const ScanSnapshot& scan, double direction_rad,
                                       double half_width_rad);

// ── ヒステリシス・角度判定を含む本体（状態を持つのでクラス） ───────────
class ObstacleLimiterCore {
 public:
  ObstacleLimiterOutput update(const ObstacleLimiterInputs& in, const ObstacleLimiterParams& p);

  // テスト・再起動用。
  void reset() { stop_latched_ = false; }
  bool stop_latched() const { return stop_latched_; }

 private:
  bool stop_latched_ = false;
};

}  // namespace th_safety

#endif  // TH_SAFETY_OBSTACLE_LIMITER_CORE_HPP_
