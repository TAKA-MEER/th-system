// ============================================================
// test_jog_gate_core.cpp — jog_gate_core.hpp の単体試験
//
// 対応する仕様（docs/plan/detailed/DetailedDesign-wp2.md `WP-SAFE-04` §7）:
//   SilentWhenBlocked         : 不変条件 J-1（通さないときは沈黙＝publish 0）
//   SilentWhenStateStale      : 不変条件 J-2（/system/state 途絶・未受信は沈黙）
//   PassthroughUnchanged      : 不変条件 J-3（ゲートであってリミッタではない。
//                                判定が通ったら速度を変えず転送）
//   IsDrivePasses             : MANUAL / TEACH_MANUAL（is_drive）を通す
//   WaitClearBlocked          : F-28（SUMMON / WAIT_CLEAR は塞ぐ）
//   AllModesFromAttributes    : 18 モードを attributes.yaml から回す
//
// テスト名は設計書 §7 に固定（変更不能）。登録名（ctest）は
// test_jog_gate_core（th_safety）。
// ============================================================
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "th_safety/jog_gate_core.hpp"

using namespace th_safety;

namespace {

// 判定のための最小ヘルパー。
JogGateParams params(double stale_sec) {
  JogGateParams p;
  p.state_stale_sec = stale_sec;
  return p;
}

JogGateStateView fresh_state(std::string mode, std::string state,
                             double age_sec = 0.0) {
  JogGateStateView st;
  st.received = true;
  st.mode = std::move(mode);
  st.state = std::move(state);
  st.state_age_sec = age_sec;
  return st;
}

// attributes.yaml の実値の写し。**判定ロジックだけを見るテスト用**で、
// ファイル読込そのものは AllModesFromAttributes が実ファイルで検証する
// （こちらを唯一の入口にすると J-4 が無検証になる。§7 の経緯を参照）。
Attributes real_attributes() {
  Attributes a;
  // denied: INIT, IDLE, ESTOP, CARRY, OPCHECK, CALIB
  for (const char* m : {"INIT", "IDLE", "ESTOP", "CARRY", "OPCHECK", "CALIB"}) {
    a.jog.emplace(m, JogLevel::DENIED);
  }
  // is_drive: MANUAL, TEACH_MANUAL
  a.jog.emplace("MANUAL", JogLevel::IS_DRIVE);
  a.jog.emplace("TEACH_MANUAL", JogLevel::IS_DRIVE);
  // allowed: 残り 10 モード
  for (const char* m : {"FOLLOW", "TEACH_FOLLOW", "REPLAY", "LINE", "LEASH",
                        "PREP", "PANEL_NAV", "AT_PANEL", "SUMMON", "HOME_NAV"}) {
    a.jog.emplace(m, JogLevel::ALLOWED);
  }
  return a;
}

// ── J-1: 通さないときは沈黙する（ゼロを撃たない） ──────────────
// jog_passes() が false を返す限り、jog_gate ノードは何も publish しない
// （ノード側は false のとき return するだけ。実機確認は §10-④）。
TEST(JogGateCore, SilentWhenBlocked) {
  Attributes a = real_attributes();
  const JogGateParams p = params(1.5);

  // IDLE は denied → 沈黙
  EXPECT_FALSE(jog_passes(fresh_state("IDLE", "NONE"), a, p));
  // OPCHECK / CALIB も denied → 沈黙
  EXPECT_FALSE(jog_passes(fresh_state("OPCHECK", "LIST"), a, p));
  EXPECT_FALSE(jog_passes(fresh_state("CALIB", "LIST"), a, p));
}

// ── J-2: /system/state が途絶・未受信なら沈黙する ──────────────
TEST(JogGateCore, SilentWhenStateStale) {
  Attributes a = real_attributes();
  const JogGateParams p = params(1.5);

  // 未受信（received == false）は stale と同じ扱い → 沈黙
  JogGateStateView unreceived = fresh_state("FOLLOW", "RUN");
  unreceived.received = false;
  unreceived.state_age_sec = 0.0;
  EXPECT_FALSE(jog_passes(unreceived, a, p));

  // 受信したが閾値を超えて古い → 沈黙
  EXPECT_FALSE(jog_passes(fresh_state("FOLLOW", "RUN", 2.0), a, p));

  // 閾値ちょうどは通る（<=）
  EXPECT_TRUE(jog_passes(fresh_state("FOLLOW", "RUN", 1.5), a, p));
}

// ── J-3: 判定が通ったら速度を変えずそのまま転送 ────────────────
// 判定コアは速度の値を一切持たない（ゲートであってリミッタではない）。
// 通ったら true（ノードが *msg を変更せず publish する）。値そのものの
// 不変性はノード側の `pub_manual_->publish(*msg)`（コピー転送）で担保される。
TEST(JogGateCore, PassthroughUnchanged) {
  Attributes a = real_attributes();
  const JogGateParams p = params(1.5);

  // allowed かつ新鮮なら通る。速度値はコアでは見ない。
  EXPECT_TRUE(jog_passes(fresh_state("FOLLOW", "RUN"), a, p));
  EXPECT_TRUE(jog_passes(fresh_state("MANUAL", "RUN"), a, p));
  // 速度の大きさはコアに含まれない＝それが証拠
  // （J-3 は「クランプしない」。リミッタは obstacle_limiter の仕事）。
}

// ── is_drive（MANUAL / TEACH_MANUAL）は通す（FMEA③を避ける） ──
TEST(JogGateCore, IsDrivePasses) {
  Attributes a = real_attributes();
  const JogGateParams p = params(1.5);

  EXPECT_TRUE(jog_passes(fresh_state("MANUAL", "RUN"), a, p));
  EXPECT_TRUE(jog_passes(fresh_state("MANUAL", "PAUSE"), a, p));
  EXPECT_TRUE(jog_passes(fresh_state("TEACH_MANUAL", "REC"), a, p));
}

// ── F-28: SUMMON / WAIT_CLEAR は塞ぐ（ジョグ禁止の除外） ──────
TEST(JogGateCore, WaitClearBlocked) {
  Attributes a = real_attributes();
  const JogGateParams p = params(1.5);

  // SUMMON は attributes では allowed だが、WAIT_CLEAR は除外表で塞ぐ
  EXPECT_FALSE(jog_passes(fresh_state("SUMMON", "WAIT_CLEAR"), a, p));
  // WAIT_CLEAR 以外の SUMMON 状態（POINT / NAV 等）は通る
  EXPECT_TRUE(jog_passes(fresh_state("SUMMON", "POINT"), a, p));
  EXPECT_TRUE(jog_passes(fresh_state("SUMMON", "NAV"), a, p));
}

// ── 18 モードを attributes.yaml から回す（§7） ────────────────
// **実ファイル（th_state/config/attributes.yaml）を読む。**
// 表をテストに書き写すと J-4（th_state と同じファイルを読む）が無検証になる。
// パスは CMake の TH_STATE_ATTRIBUTES_YAML が渡す
// （test_scan_geometry_equivalence の TH_SAFETY_TEST_DATA_DIR と同じ流儀）。
//
// **この形にした経緯**: 初版は 18 行の写像をテスト内にハードコードしており、
// `attributes.yaml` の `MANUAL: jog` を `is_drive` → `denied` に書き換える
// 変異を入れても**テストが緑のまま通った**（2026-09-01 に実測）。
// 名前が `AllModesFromAttributes` でありながら attributes を読んでいなかった。
TEST(JogGateCore, AllModesFromAttributes) {
  const Attributes a = load_attributes_jog(TH_STATE_ATTRIBUTES_YAML);
  const JogGateParams p = params(1.5);

  // 18 モード（DetailedDesign-state.md §8.2）。数が変わったら設計と実装の
  // どちらかがずれているので、まずここで気づけるようにする。
  ASSERT_EQ(a.jog.size(), static_cast<std::size_t>(18))
      << "attributes.yaml のモード数が 18 でない: " << TH_STATE_ATTRIBUTES_YAML;

  // yaml の jog 列そのものを読み直して期待値にする（写像を経由しない）。
  // load_attributes_jog() が全部 DENIED に潰しても気づけるように、
  // ここでは「denied は 6 モード」「is_drive は MANUAL/TEACH_MANUAL の 2」を表明する。
  int n_denied = 0, n_is_drive = 0;
  for (const auto& kv : a.jog) {
    if (kv.second == JogLevel::DENIED) ++n_denied;
    if (kv.second == JogLevel::IS_DRIVE) ++n_is_drive;
    const JogGateStateView st = fresh_state(kv.first, "NONE");
    EXPECT_EQ(jog_passes(st, a, p), kv.second != JogLevel::DENIED)
        << "mode=" << kv.first;
  }
  EXPECT_EQ(n_denied, 6) << "denied は INIT/IDLE/ESTOP/CARRY/OPCHECK/CALIB の 6 つ";
  EXPECT_EQ(n_is_drive, 2) << "is_drive は MANUAL/TEACH_MANUAL の 2 つ";

  // 名指しの確認（表を読み違えていないこと）。
  EXPECT_EQ(a.jog_for("MANUAL"), JogLevel::IS_DRIVE);
  EXPECT_EQ(a.jog_for("TEACH_MANUAL"), JogLevel::IS_DRIVE);
  EXPECT_EQ(a.jog_for("IDLE"), JogLevel::DENIED);
  EXPECT_EQ(a.jog_for("FOLLOW"), JogLevel::ALLOWED);
}

// ── 読み込みが壊れたときに安全側（DENIED）へ倒れること ────────────
// load_attributes_jog() は例外を投げ、ノードは起動失敗する（素通しにしない）。
TEST(JogGateCore, MissingAttributesIsSilent) {
  EXPECT_THROW(load_attributes_jog("/nonexistent/attributes.yaml"), std::exception);

  // 緩和版は空写像＝全モード DENIED＝常に沈黙（安全側）。
  const Attributes empty = load_attributes_jog_lenient("/nonexistent/attributes.yaml");
  const JogGateParams p = params(1.5);
  EXPECT_FALSE(jog_passes(fresh_state("MANUAL", "RUN"), empty, p));
  EXPECT_FALSE(jog_passes(fresh_state("FOLLOW", "RUN"), empty, p));
}

}  // namespace
