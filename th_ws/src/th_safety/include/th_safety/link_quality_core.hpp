// ============================================================
// link_quality_core.hpp — リンク品質（受信間隔）計測の純粋コア
//
// ROS2 非依存。GapTracker が受信のたびのタイムスタンプを受け取り、
// 直近の受信間隔（ギャップ）を保持する。compute() で窓内の全ギャップを
// 集計し、p50/p99/max を返す。
//
// 設計方針（DetailedDesign-wp0.md WP-SAFE-00 §4.1）:
//   「分位点は近似しない。窓内の全ギャップを保持してソートする
//    （1 Hz × 3 本なら十分安い）。」
//
// このヘッダは診断用の測定器であり、安全判定には一切使わない（Q-1）。
// 例外を投げない設計とし、呼び出し側（safety_monitor）が万一の例外に
// 備えて try/catch で囲む前提（FMEA ①）。
// ============================================================
#ifndef TH_SAFETY_LINK_QUALITY_CORE_HPP_
#define TH_SAFETY_LINK_QUALITY_CORE_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace th_safety {

// compute() の戻り値。受信が 1 件も無ければ全ゼロになる（Q-2。沈黙しない）。
struct Quantiles {
  double p50_ms = 0.0;
  double p99_ms = 0.0;
  double max_ms = 0.0;
  uint32_t window_sec = 0;
};

// 受信間隔（ギャップ）のトラッカー。
//
// push() は受信のたびに呼ぶ。2 回目以降の呼び出しで
// 「今回のタイムスタンプ − 前回のタイムスタンプ」をギャップとして記録する
// （初回はギャップが定義できないので記録しない）。
//
// compute() は窓内（now_sec - window_sec 以降）の全ギャップから
// 分位点（線形補間）を計算する。近似はしない。
class GapTracker {
 public:
  void push(double stamp_sec) {
    if (has_last_) {
      const double gap_sec = stamp_sec - last_stamp_sec_;
      // 時刻が逆行した場合（クロック調整・シム時刻リセット等）は
      // 負のギャップを記録しない。例外は投げず単に無視する。
      if (gap_sec >= 0.0) {
        gaps_.push_back(GapEntry{stamp_sec, gap_sec});
      }
    }
    last_stamp_sec_ = stamp_sec;
    has_last_ = true;

    // 無制限増加を避けるための防御的な上限（安全弁）。
    // registry.yaml の link_quality_window_sec（既定 30s）と実際の受信レート
    // （ESP32/LiDAR/UI いずれも数十 Hz 以下）を踏まえると窓内に必要な件数は
    // 数百〜数千件で足りるため、通常はこの上限に到達しない。
    while (gaps_.size() > kMaxRetainedGaps) {
      gaps_.pop_front();
    }
  }

  Quantiles compute(double now_sec, double window_sec) const {
    Quantiles result;

    std::vector<double> in_window;
    in_window.reserve(gaps_.size());
    for (const auto& g : gaps_) {
      const double age_sec = now_sec - g.stamp_sec;
      if (age_sec >= 0.0 && age_sec <= window_sec) {
        in_window.push_back(g.gap_sec);
      }
    }

    if (in_window.empty()) {
      // Q-2: 受信 0（または窓内に受信なし）でもゼロ値を返す。沈黙しない。
      return result;
    }

    std::sort(in_window.begin(), in_window.end());
    result.p50_ms = percentile(in_window, 0.50) * 1000.0;
    result.p99_ms = percentile(in_window, 0.99) * 1000.0;
    result.max_ms = in_window.back() * 1000.0;
    result.window_sec = static_cast<uint32_t>(window_sec);
    return result;
  }

 private:
  struct GapEntry {
    double stamp_sec;
    double gap_sec;
  };

  // sorted_vals は昇順ソート済み前提。線形補間による分位点（numpy 既定と同じ方式）。
  static double percentile(const std::vector<double>& sorted_vals, double p) {
    if (sorted_vals.size() == 1) {
      return sorted_vals.front();
    }
    const double rank = p * static_cast<double>(sorted_vals.size() - 1);
    const size_t lo = static_cast<size_t>(rank);
    const size_t hi = std::min(lo + 1, sorted_vals.size() - 1);
    const double frac = rank - static_cast<double>(lo);
    return sorted_vals[lo] + frac * (sorted_vals[hi] - sorted_vals[lo]);
  }

  static constexpr std::size_t kMaxRetainedGaps = 100000;

  std::deque<GapEntry> gaps_;
  double last_stamp_sec_ = 0.0;
  bool has_last_ = false;
};

}  // namespace th_safety

#endif  // TH_SAFETY_LINK_QUALITY_CORE_HPP_
