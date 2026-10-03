// ============================================================
// test_jog_gate_core.cpp — jog_gate_core.hpp の単体試験
//
// 対応する仕様（docs/plan/detailed/DetailedDesign-wp2.md `WP-SAFE-04` §7）:
//   SilentWhenBlocked         : 不変条件 J-1（通さないときは沈黙＝publish 0）
//   SilentWhenStateStale      : 不変条件 J-2（/system/state 途絶・未受信は沈黙）
//   ScaledByLimits            : W-07（判定が通ったら比率に上限を掛けて転送。
//                                旧 J-3「そのまま転送」からの仕様変更）
//   ClampedToLimits           : W-07（範囲外の比率は ±1 に丸める。上限を超えない）
//   NonFiniteBecomesZero      : W-07 受け入れ（NaN・±inf は 0 に倒す。素通しにしない）
//   IsDrivePasses             : MANUAL / TEACH_MANUAL（is_drive）を通す
//   WaitClearBlocked          : F-28（SUMMON / WAIT_CLEAR は塞ぐ）
//   AllModesFromAttributes    : 18 モードを attributes.yaml から回す
//
// テスト名は設計書 §7 に固定（変更不能）。登録名（ctest）は
// test_jog_gate_core（th_safety）。
// （W-07 で PassthroughUnchanged を ScaledByLimits / ClampedToLimits に
// 置き換えた。設計書 §7 も同時に更新）
// ============================================================
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
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

// ── W-07: 判定が通ったら比率に上限を掛けて転送 ────────────────
// 入力は -1〜1 の比率。linear.x × v_jog_max、angular.z × w_jog_max。
// 旧 J-3「速度の大きさは変えない（そのまま転送）」からの仕様変更。
TEST(JogGateCore, ScaledByLimits) {
  JogSpeedLimits lim;
  lim.v_jog_max = 0.55;
  lim.w_jog_max = 1.0;

  // 比率 1.0 → 上限そのまま
  JogRatio full{1.0, 1.0};
  const JogCmd full_out = jog_apply_limits(full, lim);
  EXPECT_DOUBLE_EQ(full_out.vx, 0.55);
  EXPECT_DOUBLE_EQ(full_out.wz, 1.0);

  // 比率 0.5 → 上限の半分（前進・旋回の両方）
  JogRatio half{0.5, -0.5};
  const JogCmd half_out = jog_apply_limits(half, lim);
  EXPECT_DOUBLE_EQ(half_out.vx, 0.275);
  EXPECT_DOUBLE_EQ(half_out.wz, -0.5);

  // ゼロ → ゼロ（離したら出す明示ゼロがそのまま 0 になる）
  JogRatio zero{0.0, 0.0};
  const JogCmd zero_out = jog_apply_limits(zero, lim);
  EXPECT_DOUBLE_EQ(zero_out.vx, 0.0);
  EXPECT_DOUBLE_EQ(zero_out.wz, 0.0);

  // 判定コア自体は速度を見ない（ゲートの開閉だけが仕事）。
  // 通す／通さないの判定は jog_passes() が担う。
  Attributes a = real_attributes();
  const JogGateParams p = params(1.5);
  EXPECT_TRUE(jog_passes(fresh_state("FOLLOW", "RUN"), a, p));
  EXPECT_TRUE(jog_passes(fresh_state("MANUAL", "RUN"), a, p));
}

// ── W-07: 範囲外の比率は ±1 に丸める（上限を超えない） ─────────
TEST(JogGateCore, ClampedToLimits) {
  JogSpeedLimits lim;
  lim.v_jog_max = 0.55;
  lim.w_jog_max = 1.0;

  // 2.0（範囲外）→ 上限×1.0 に丸まる。上限を超えない
  JogRatio over{2.0, 2.0};
  const JogCmd over_out = jog_apply_limits(over, lim);
  EXPECT_DOUBLE_EQ(over_out.vx, 0.55);
  EXPECT_DOUBLE_EQ(over_out.wz, 1.0);

  // 負の範囲外も対称に丸まる
  JogRatio under{-3.0, -1.5};
  const JogCmd under_out = jog_apply_limits(under, lim);
  EXPECT_DOUBLE_EQ(under_out.vx, -0.55);
  EXPECT_DOUBLE_EQ(under_out.wz, -1.0);
}

// ── W-07 受け入れ: 非有限（NaN・±inf）は 0 に倒す ─────────────
// NaN は > も < も偽なので、明示的に弾かないと上限掛けが NaN のまま
// /cmd_vel_manual に載る。±inf は ±1 丸めでなく 0 に倒す（壊れた入力の
// 兆候なので上限いっぱいで走らせない）。
TEST(JogGateCore, NonFiniteBecomesZero) {
  JogSpeedLimits lim;
  lim.v_jog_max = 0.55;
  lim.w_jog_max = 1.0;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  JogRatio nan_in{nan, nan};
  const JogCmd nan_out = jog_apply_limits(nan_in, lim);
  EXPECT_DOUBLE_EQ(nan_out.vx, 0.0);
  EXPECT_DOUBLE_EQ(nan_out.wz, 0.0);

  JogRatio inf_in{std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity()};
  const JogCmd inf_out = jog_apply_limits(inf_in, lim);
  EXPECT_DOUBLE_EQ(inf_out.vx, 0.0);
  EXPECT_DOUBLE_EQ(inf_out.wz, 0.0);
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

  // 19 モード（DetailedDesign-state.md §8.2 / Spec-modes.md §2.3）。数が変わったら
  // 設計と実装のどちらかがずれているので、まずここで気づけるようにする。
  // 2026-09-08: AT_HOME（待機場所での待機。jog: allowed）を足して 18 → 19。
  ASSERT_EQ(a.jog.size(), static_cast<std::size_t>(19))
      << "attributes.yaml のモード数が 19 でない: " << TH_STATE_ATTRIBUTES_YAML;

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
  // 2026-09-08: 当日の待機場所でジョグできること（AT_PANEL と同じ扱い）。
  // ここが DENIED に戻ると「当日は行き先を選ぶまで機体を動かせない」に逆戻りする。
  EXPECT_EQ(a.jog_for("AT_HOME"), JogLevel::ALLOWED);
  EXPECT_EQ(a.jog_for("AT_PANEL"), JogLevel::ALLOWED);
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
