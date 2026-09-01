// parts/stickGeometry.js — pure stick-command geometry, extracted verbatim
// from App.jsx (L126-146) so it can be unit-tested on Node with no React and
// no ROS in scope (DetailedDesign-wp3.md WP-UI-03 §4.1).
//
// These constants and the 8-direction sector logic are the values tuned on
// the real robot; this packet only relocates and tests them (the design
// explicitly forbids changing the behavior: "stickToCmd() の中身は変えない").
// They live module-local here instead of in a screen so every caller (the
// perpetual stick, the W-6 panel) shares one definition.

// 緩旋回 (直進 + 旋回同時) 時の旋回レート。crawler_teleop の緩旋回と同じ 0.5 倍。
const JOG_ARC_ANG_SCALE = 0.5
// スティックのデッドゾーン。中心からの倒し量がこの割合未満なら停止。
const STICK_DEADZONE = 0.15

// current を target へ maxDelta の範囲内で近づける (加減速レート制限)。
// 既存 App.jsx:127 をそのまま移しただけで、挙動は変えない。
export function rampToward(current, target, maxDelta) {
  if (target > current + maxDelta) return current + maxDelta
  if (target < current - maxDelta) return current - maxDelta
  return target
}

// スティック位置 → 正規化コマンド。
// dx: 右+ / dy: 上+ (いずれも -1..1)。8方向セクター (45°幅) で判定:
//   上下 = 直進 / 真横 = 超信地旋回 / 斜め = 緩旋回 (旋回 0.5 倍)
// 倒し量 (デッドゾーン超過分を 0→1 に正規化) が速度の比率になる。
// 戻り値 { vn, wn, label } の vn / wn は正規化比率 (-1..1) であって物理速度
// そのものではない。物理速度への換算は下流 (obstacle_limiter) が持つ上限に
// 対する「割合」として UI 側で speed preset を掛けるだけ
// (DetailedDesign-wp3.md WP-UI-03 §3.3: ブラウザに上限を持たせない)。
// label は表示用の機械キー (parts/ は日本語リテラル禁止: c4)。日本語表示への
// 写像は i18n/screens.js の STICK_LABELS に置く。
export function stickToCmd(dx, dy, len) {
  if (len < STICK_DEADZONE) return { vn: 0, wn: 0, label: null }
  const m = Math.min(1, (len - STICK_DEADZONE) / (1 - STICK_DEADZONE))
  const deg = Math.abs(Math.atan2(dx, dy)) * 180 / Math.PI  // 0=上, 90=横, 180=下
  const turn = dx > 0 ? -1 : 1  // 右に倒す = 右旋回 (wz 負)
  if (deg < 22.5)  return { vn:  m, wn: 0,                            label: 'forward' }
  if (deg < 67.5)  return { vn:  m, wn: turn * JOG_ARC_ANG_SCALE * m, label: 'arc' }
  if (deg < 112.5) return { vn:  0, wn: turn * m,                     label: 'turn' }
  if (deg < 157.5) return { vn: -m, wn: turn * JOG_ARC_ANG_SCALE * m, label: 'rev_arc' }
  return { vn: -m, wn: 0, label: 'reverse' }
}
