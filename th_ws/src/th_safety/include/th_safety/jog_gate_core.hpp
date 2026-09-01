// ============================================================
// jog_gate_core.hpp — jog_gate の判定ロジックの純粋コア
//
// ROS2 非依存（rclcpp / geometry_msgs / th_system_msgs を include しない）。
// obstacle_limiter_core.hpp と同じ二層構造の思想（CLAUDE.md「追従ロジックの
// 二層構造」）。ROS ノード側（src/jog_gate.cpp）が本ヘッダを呼び出し、
// 購読／配信だけを担当する。
//
// 対応する仕様（docs/plan/detailed/DetailedDesign-wp2.md `WP-SAFE-04`）:
//   §4.1  純粋コア（3 条件の AND）   → jog_passes()
//   §3.4.2 / names §5.1  state_stale → JogGateParams::state_stale_sec
//   §4.3  不変条件 J-1〜J-4           → test/test_jog_gate_core.cpp
//
// attributes.yaml の読込は jog_gate_core.cpp の
// load_attributes_jog() / load_attributes_jog_lenient() に置く（yaml-cpp は
// ROS2 ではないので純粋層に含めてよい）。判定ロジックは写し取った写像
// （Attributes::jog）だけに依存し、ファイルや yaml に触れない。
//
// 安全の向き:
//   - 一度も /system/state を受信していない（st.state_received == false）は
//     stale と同じ扱い（未受信 = stale。§3.4.2）→ 不合格。
//   - 判定に失敗（不明な jog 値・モードが写像に無い）は denied 扱い =
//     沈黙側に倒す（§4.3 不変条件 J-1/J-2 と同じ安全側）。
// ============================================================
#ifndef TH_SAFETY_JOG_GATE_CORE_HPP_
#define TH_SAFETY_JOG_GATE_CORE_HPP_

#include <map>
#include <string>

namespace th_safety {

// attributes.yaml の jog 列の解釈（DetailedDesign-state.md §8.1・§8.2）。
// ALLOWED と IS_DRIVE は通す（jog != "denied"）。DENIED は塞ぐ。
enum class JogLevel { ALLOWED, DENIED, IS_DRIVE };

// 文字列 → JogLevel。未知・空文字は DENIED に倒す（安全側。§4.3 の沈黙方針と一致）。
// is_drive（MANUAL / TEACH_MANUAL）は IS_DRIVE。それ以外の非 denied は ALLOWED。
JogLevel jog_level_from_string(const std::string& v);

// attributes.yaml の jog 列の写像（mode → jog 値）。
struct Attributes {
  // 写像から引けない（未知の）モードは DENIED（安全側）。
  JogLevel jog_for(const std::string& mode) const;
  std::map<std::string, JogLevel> jog;
};

// 静的パラメータ（registry.yaml 由来。name: state_stale_ms）。
struct JogGateParams {
  double state_stale_sec = 0.0;
};

// /system/state のうち判定に要る写し（SystemState.msg の部分集合）。
struct JogGateStateView {
  bool received = false;        // 一度も受信していない（§3.4.2: stale と同じ扱い）
  std::string mode;             // 18 モード（文字列）
  std::string state;            // サブステート（状態が無ければ "NONE"）
  double state_age_sec = 0.0;   // 受信から経過した時間（now - 受信時刻）
};

// attributes.yaml を読み、jog 列だけを写像にする（J-4: th_state と同じファイル）。
// ファイルが読めない／形式が不正は std::runtime_error を投げる（jog_gate ノードは
// 起動失敗させる＝通しっぱなしにしない）。yaml-cpp は ROS2 ではないので
// この純粋層（jog_gate_core.cpp）に含める。
Attributes load_attributes_jog(const std::string& yaml_path);

// load_attributes_jog() の緩和版。ファイルが無い／読めない場合は安全側の
// 空写像（全モード denied = 常に沈黙）を返す。jog_gate ノードでは呼ばない
// （起動失敗させる方を使う）。テストの入口として用意する。
Attributes load_attributes_jog_lenient(const std::string& yaml_path);

// WP-SAFE-04 §4.1 の 3 条件の AND で判定する。
//   1. state_stale_sec 以内に /system/state を受信している（未受信は不合格）
//   2. attrs[st.mode].jog != "denied"（is_drive は通す）
//   3. (st.mode, st.state) が除外表に当たらない（SUMMON/WAIT_CLEAR）
//
// 戻り値 true = 通す（そのまま転送）、false = 沈黙する（publish しない）。
// 不変条件 J-1（通さないときはゼロを撃たず沈黙する）の判定の本体。
bool jog_passes(const JogGateStateView& st, const Attributes& attrs,
                const JogGateParams& p);

}  // namespace th_safety

#endif  // TH_SAFETY_JOG_GATE_CORE_HPP_
