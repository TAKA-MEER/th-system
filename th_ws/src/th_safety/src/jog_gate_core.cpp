// ============================================================
// jog_gate_core.cpp — jog_gate_core.hpp の実装
// ============================================================
#include "th_safety/jog_gate_core.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <stdexcept>
#include <utility>

namespace th_safety {

JogLevel jog_level_from_string(const std::string& v) {
  if (v == "is_drive") return JogLevel::IS_DRIVE;
  if (v == "allowed") return JogLevel::ALLOWED;
  // "denied" を含め、不明・空・未定義の値は DENIED（安全側）。
  return JogLevel::DENIED;
}

JogLevel Attributes::jog_for(const std::string& mode) const {
  const auto it = jog.find(mode);
  if (it == jog.end()) {
    // 写像に無い未知のモードは DENIED（安全側。§4.3 の沈黙方針と一致）。
    return JogLevel::DENIED;
  }
  return it->second;
}

bool jog_passes(const JogGateStateView& st, const Attributes& attrs,
                const JogGateParams& p) {
  // 条件1: /system/state が新鮮か（未受信は不通過。§3.4.2）
  if (!st.received || st.state_age_sec > p.state_stale_sec) {
    return false;
  }
  // 条件2: attributes[mode].jog != "denied"（is_drive も通す）
  if (attrs.jog_for(st.mode) == JogLevel::DENIED) {
    return false;
  }
  // 条件3: 除外表（SUMMON / WAIT_CLEAR。F-28）
  if (st.mode == "SUMMON" && st.state == "WAIT_CLEAR") {
    return false;
  }
  return true;
}

// attributes.yaml を読み、jog 列だけを写像にする（J-4: th_state と同じファイル）。
// 各モード・エントリの jog 値が無い・読めない場合は DENIED（安全側）。
Attributes load_attributes_jog(const std::string& yaml_path) {
  YAML::Node root = YAML::LoadFile(yaml_path);

  if (!root.IsMap()) {
    throw std::runtime_error("attributes.yaml はマップでなければならない: " + yaml_path);
  }

  Attributes out;
  for (const auto& kv : root) {
    const std::string mode = kv.first.as<std::string>();
    const YAML::Node entry = kv.second;
    if (!entry.IsMap() || !entry["jog"]) {
      // jog 列が無いモードは DENIED（安全側）。
      out.jog.emplace(mode, JogLevel::DENIED);
      continue;
    }
    out.jog.emplace(mode, jog_level_from_string(entry["jog"].as<std::string>()));
  }
  return out;
}

// load_attributes_jog() のうち「yaml ファイルが無い」場合に安全側（空）で
// 継続する緩和版。jog_gate ノードでは一切使わず（起動失敗させる）、テスト
// の入口としてだけ用意する。空 = 全モード DENIED = 常に沈黙（安全側）。
Attributes load_attributes_jog_lenient(const std::string& yaml_path) {
  try {
    return load_attributes_jog(yaml_path);
  } catch (const std::exception&) {
    return Attributes{};
  }
}

}  // namespace th_safety
