// ============================================================
// dev_mode_core.hpp — /system/dev_mode（std_msgs/String に JSON）の読み取り
//
// 開発モードの正本は connectivity_checker のパラメータで、状態は
// /system/dev_mode（transient_local・1 Hz）に JSON で出ている
// （DetailedDesign-names.md §6.2）。safety_monitor と obstacle_limiter は
// これを購読し、**effective（= dev_mode AND 項目別選択）に明示された項目だけ**
// に反応する（Spec-safety.md §10。2026-09-23 ユーザー決定で「開発モードでも
// 無視できない制限」を撤廃し、項目ごとに選ぶ方式にした）。
//
// フェイルセーフ（誤って ON と判定する方向を潰す）:
//   - 未受信・受信から kDevStateStaleSec 以上経過 → 何も無視しない
//   - JSON が壊れている・型が違う・キーが無い     → 何も無視しない
//   - トップレベルの dev_mode が true でない        → 何も無視しない
//   - `ignore` 側（選択だけで未実効）は読まない
//
// ROS2 非依存（obstacle_limiter_core.hpp と同じ流儀）。gtest で直接試験する。
// ============================================================
#pragma once

#include <nlohmann/json.hpp>

#include <map>
#include <string>

namespace th_safety {

// 項目名。connectivity_checker.py の _DEV_ITEMS・web_ui の DEV_ITEMS と揃える。
inline constexpr const char* kDevItemLidarFault = "lidar_fault";  // safety_monitor: LIDAR_LOST を出さない
inline constexpr const char* kDevItemScanStop   = "scan_stop";    // obstacle_limiter: /scan 途絶で手動を止めない（上限は通常と同じ）

// /system/dev_mode は 1 Hz。3 周期落ちたら発行者が居ないとみなす。
inline constexpr double kDevStateStaleSec = 3.0;

struct DevModeSnapshot {
  bool received = false;
  double stamp_sec = 0.0;                  // 受信した時刻（メッセージの中身ではない）
  std::map<std::string, bool> effective;   // parse_dev_effective() の結果
};

// JSON から「実効で無視している項目」だけを取り出す。読めなければ空。
inline std::map<std::string, bool> parse_dev_effective(const std::string& json_text) {
  std::map<std::string, bool> out;
  const auto j = nlohmann::json::parse(json_text, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object()) return out;
  const auto master = j.find("dev_mode");
  if (master == j.end() || !master->is_boolean() || !master->get<bool>()) return out;
  const auto eff = j.find("effective");
  if (eff == j.end() || !eff->is_object()) return out;
  for (auto it = eff->begin(); it != eff->end(); ++it) {
    // 真偽値の true だけを採る（"true" や 1 は採らない）。
    if (it.value().is_boolean() && it.value().get<bool>()) out[it.key()] = true;
  }
  return out;
}

// いまその項目を無視してよいか。鮮度切れ・未受信は false（通常運用と同じ）。
inline bool dev_item_effective(const DevModeSnapshot& snap, const std::string& item,
                               double now_sec, double stale_sec = kDevStateStaleSec) {
  if (!snap.received) return false;
  if ((now_sec - snap.stamp_sec) > stale_sec) return false;
  const auto it = snap.effective.find(item);
  return it != snap.effective.end() && it->second;
}

}  // namespace th_safety
