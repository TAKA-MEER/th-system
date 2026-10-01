// ============================================================
// obstacle_limiter_params.hpp — obstacle_limiter ノードが行う
// 「ROS パラメータ表 → obstacle_limiter_core.hpp の入力値」変換のうち、
// テストできる部分だけを純粋関数として切り出したもの。
//
// ROS2 非依存（rclcpp を include しない。obstacle_limiter_core.hpp と同じ
// 二層構造の思想 — CLAUDE.md「追従ロジックの二層構造」）。
//
// obstacle_limiter_core.hpp が要求する ObstacleLimiterInputs /
// ObstacleLimiterParams の一部フィールドは、ROS の世界からそのままでは
// 埋められない（型・単位・表引きが要る）。ここに切り出したのはその変換の
// うち副作用が無いもの 2 つ:
//
//   1. resolve_speed_limit_name() — SystemState.speed_limit（パラメータ名の
//      文字列 or "stop"）を数値へ引く。obstacle_limiter_core.hpp 冒頭コメント
//      が「テーブル参照はロジック層の責務ではなく、呼び出し側が解決してから
//      渡す設計にした」と明記している、その解決処理そのもの。
//   2. flat_to_range_pairs() — registry.yaml 由来の blind_angle_ranges は
//      平坦配列 [a0,a1,a2,a3,...] で ROS パラメータに届く。
//      th_perception/scripts/lidar_filter.py::_build_blind_ranges() と
//      同じ規約（2 個ずつ組にする。奇数個の余りは切り捨て）で
//      ObstacleLimiterParams::blind_angle_ranges_deg の形へ組み直す。
// ============================================================
#ifndef TH_SAFETY_OBSTACLE_LIMITER_PARAMS_HPP_
#define TH_SAFETY_OBSTACLE_LIMITER_PARAMS_HPP_

#include <cmath>
#include <cstddef>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace th_safety {

// blind_angle_ranges の平坦配列 [a0,a1, a2,a3, ...]（度）を
// [(a0,a1), (a2,a3), ...] に組み直す。
// th_perception/scripts/lidar_filter.py::_build_blind_ranges() と同じ規約
// （range(0, len-1, 2) と同じ。末尾に半端な 1 個が残っていたら切り捨てる）。
// 空配列は「死角なし」という正常な入力であり、空の結果を返す
// （DetailedDesign-wp2.md WP-SAFE-03「確認済みの事実②」）。
inline std::vector<std::pair<double, double>> flat_to_range_pairs(
    const std::vector<double>& flat) {
  std::vector<std::pair<double, double>> pairs;
  if (flat.size() < 2) {
    return pairs;
  }
  for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
    pairs.emplace_back(flat[i], flat[i + 1]);
  }
  return pairs;
}

// ── 死角マスクの実行中更新の検査（校正 BLIND。Spec-checks.md §3.5・2026-10-01）──
// 死角マスクは「その角度の点を障害物として見ない」設定なので、広げすぎると本物の障害物が
// 見えなくなる。校正（calib_runner）が上限を検査してから送るが、安全判定の最終段である
// obstacle_limiter 自身も、実行中に届いた値を**ここで独立に検査**し、超える値は拒否して
// 古い値のまま動く（二重化）。値は registry の blind_max_*（calib_runner と同じ）。
struct BlindLimits {
  double max_sector_deg = 30.0;
  double max_total_deg = 90.0;
  std::size_t max_sectors = 8;
};

struct BlindValidation {
  bool ok = true;
  std::string reason;
};

// 反時計回りに a0 から a1 までの幅（度）。[0, 360)。a0 == a1 は 0。
inline double blind_sector_width_deg(double a0, double a1) {
  double w = std::fmod(a1 - a0, 360.0);
  if (w < 0.0) {
    w += 360.0;
  }
  return w;
}

// 平坦配列 [a0,a1, a2,a3, ...]（度）が上限内か。空は有効（マスクなし＝広く見える側）。
// 奇数長・非有限・幅ゼロ・区間数/1 区間/総幅の超過は拒否。総幅は重なりを数え直さない
// （重なっていれば二重に数えるので厳しい側＝安全側に倒れる）。
inline BlindValidation validate_blind_ranges(const std::vector<double>& flat,
                                             const BlindLimits& lim) {
  BlindValidation v;
  if (flat.size() % 2 != 0) {
    v.ok = false;
    v.reason = "odd_length";
    return v;
  }
  const std::size_t count = flat.size() / 2;
  if (count > lim.max_sectors) {
    v.ok = false;
    v.reason = "too_many_sectors";
    return v;
  }
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
    if (!std::isfinite(flat[i]) || !std::isfinite(flat[i + 1])) {
      v.ok = false;
      v.reason = "non_finite";
      return v;
    }
    const double w = blind_sector_width_deg(flat[i], flat[i + 1]);
    if (w <= 1e-9) {
      v.ok = false;
      v.reason = "zero_width";
      return v;
    }
    if (w > lim.max_sector_deg + 1e-9) {
      v.ok = false;
      v.reason = "sector_too_wide";
      return v;
    }
    total += w;
  }
  if (total > lim.max_total_deg + 1e-9) {
    v.ok = false;
    v.reason = "total_too_wide";
    return v;
  }
  return v;
}

// SystemState.speed_limit（パラメータ名の文字列 or "stop"）の解決結果。
struct SpeedLimitResolution {
  double value_mps = 0.0;
  bool unknown = false;  // true: table に無い名前だった（安全側 0.0 に倒した）
};

// name を table から数値へ引く。"stop" は table を引かず常に 0.0。
// table に無い未知の名前は安全側（0.0 = 停止）に倒し、unknown=true を返す
// （呼び出し側がログ警告を出すかどうかを決める。ここではログを出さない
// ——ROS 非依存の純粋関数にログ出力を持ち込まないため）。
inline SpeedLimitResolution resolve_speed_limit_name(
    const std::string& name, const std::map<std::string, double>& table) {
  if (name == "stop") {
    return SpeedLimitResolution{0.0, false};
  }
  auto it = table.find(name);
  if (it == table.end()) {
    return SpeedLimitResolution{0.0, true};
  }
  return SpeedLimitResolution{it->second, false};
}

}  // namespace th_safety

#endif  // TH_SAFETY_OBSTACLE_LIMITER_PARAMS_HPP_
