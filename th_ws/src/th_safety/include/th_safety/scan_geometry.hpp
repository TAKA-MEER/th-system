// ============================================================
// scan_geometry.hpp — 角度 → LaserScan 添字の変換規約（C++ 移植版）
//
// Python 正本 th_perception/th_perception/scan_geometry.py の C++ 移植。
// `lidar_filter`（th_perception。Python 正本を直接使う）と
// `obstacle_limiter`（th_safety。このヘッダを使う）が同じ
// blind_angle_ranges を同じセクタとして解釈するための共通規約
// （DetailedDesign-wp2.md WP-CALIB-01 §4.1・B-3）。
//
// ROS2 非依存のヘッダオンリー・純粋ロジック層（link_quality_core.hpp と
// 同じ INTERFACE ライブラリの流儀。sensor_msgs 等は一切 include しない）。
//
// 規約（Python 版 scan_geometry.py の docstring と同一。番号は§4.1のまま）:
//   ① 角度は laser_link 基準・ラジアン・反時計回り正
//      （sensor_msgs/LaserScan の angle = angle_min + i*angle_increment の
//      向きに一致）。
//   ② blind_angle_ranges は度で持ち、sector_indices() の中で 1 度だけ
//      ラジアンへ変換する。
//   ③ 角度は [-pi, pi) に正規化してから比較する。
//   ④ 添字は floor((angle - angle_min) / angle_increment)。
//   ⑤ 範囲外は clamp せず「そのセクタは存在しない」として扱う。
//      → Python 版は None を返す。C++ には None が無いが、C++17 の
//      std::optional<std::size_t> が同じ意味（「値があるかもしれないし
//      無いかもしれない」を型で表す）を持つため、これで同じ選択を踏襲する。
//
// 浮動小数点の注意（B-3。Python 版で実際に 1 ULP 問題を再現・確認済み）:
//   floor((angle - angle_min) / angle_increment) は角度が格子点ちょうどの
//   ときに丸め誤差で隣の添字へ落ちることがある。Python 版と bit-for-bit
//   同じ結果を返すことが等価性テスト（test_scan_geometry_equivalence、
//   B-3）の要件なので、
//     正規化 → 減算 → 除算 → epsilon 加算 → floor → 整数キャスト
//   の演算順序と epsilon の値（1e-9 = kFloorEps）を Python 版と厳密に
//   揃えてある。度→ラジアン変換も `度 * (pi / 180.0)`
//   （Python の math.radians() の内部実装と同じ式）で揃え、pi の値も
//   Python の math.pi と同じ最近接倍精度表現を kPi として明示的に使う
//   （プラットフォームの M_PI マクロには依存しない）。
// ============================================================
#ifndef TH_SAFETY_SCAN_GEOMETRY_HPP_
#define TH_SAFETY_SCAN_GEOMETRY_HPP_

#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

namespace th_safety {

// Python の math.pi と同じ最近接倍精度表現（B-3。プラットフォーム依存の
// M_PI マクロではなく明示的な定数を使う）。
inline constexpr double kPi = 3.14159265358979323846;

// floor() 直前に加える微小補正。角度が格子点ちょうどのとき丸め誤差で
// 1 つ下の添字へ落ちるのを防ぐ（scan_geometry.py の _FLOOR_EPS と同じ値）。
inline constexpr double kFloorEps = 1e-9;

// angle_to_index() / sector_indices() が要求する LaserScan の最小情報。
// sensor_msgs::msg::LaserScan と同じ意味のフィールドだけを持つ軽量構造体
// （このヘッダを ROS2 非依存に保つため sensor_msgs はリンクしない。
// 呼び出し側で LaserScan から詰め替える）。
struct ScanGeometryInfo {
  double angle_min = 0.0;
  double angle_increment = 0.0;
  std::size_t num_ranges = 0;
};

// 角度を [-pi, pi) に正規化する（規約③）。
// Python 版 _normalize_angle() と同じ演算順序（std::fmod ⇔ math.fmod）。
inline double normalize_angle(double angle_rad) {
  double normalized = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (normalized < 0.0) {
    normalized += 2.0 * kPi;
  }
  return normalized - kPi;
}

// `scan` の angle_min / angle_increment から LaserScan の添字を返す。
// 規約①〜⑤（ヘッダ冒頭コメント参照）。範囲外なら std::nullopt（規約⑤）。
inline std::optional<std::size_t> angle_to_index(
    double angle_rad, const ScanGeometryInfo& scan) {
  if (scan.num_ranges == 0) {
    return std::nullopt;
  }

  const double angle_norm = normalize_angle(angle_rad);
  const double idx_f = std::floor(
      (angle_norm - scan.angle_min) / scan.angle_increment + kFloorEps);

  if (idx_f < 0.0) {
    return std::nullopt;
  }
  const std::size_t idx = static_cast<std::size_t>(idx_f);
  if (idx >= scan.num_ranges) {
    return std::nullopt;
  }
  return idx;
}

// [a0_deg, a1_deg]（度）を添字の (i0, i1) に変換する。
// a0_deg <= a1_deg（正規化後）であれば [i0, i1] が連続区間を意味する。
// a0_deg > a1_deg のときは 0 をまたぐ 2 区間（[i0, len-1] と [0, i1]）を
// 意味する。連結（実際に添字集合を組み立てる処理）は呼び出し側の責務
// （§4.1）。i0 / i1 のどちらかが範囲外なら std::nullopt（規約⑤）。
inline std::pair<std::optional<std::size_t>, std::optional<std::size_t>>
sector_indices(double a0_deg, double a1_deg, const ScanGeometryInfo& scan) {
  // Python の math.radians(x) は x * (pi / 180.0) を 1 回の乗算で計算する
  // （degToRad を事前計算した定数として持つ）。同じ演算順序に揃える。
  static constexpr double kDegToRad = kPi / 180.0;
  const auto i0 = angle_to_index(a0_deg * kDegToRad, scan);
  const auto i1 = angle_to_index(a1_deg * kDegToRad, scan);
  return {i0, i1};
}

}  // namespace th_safety

#endif  // TH_SAFETY_SCAN_GEOMETRY_HPP_
