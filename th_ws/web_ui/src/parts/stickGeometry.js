// parts/stickGeometry.js — pure stick-command geometry, extracted verbatim
// from App.jsx (L126-146) so it can be unit-tested on Node with no React and
// no ROS in scope (DetailedDesign-wp3.md WP-UI-03 §4.1).
//
// These constants and the 8-direction sector logic are the values tuned on
// the real robot; this packet only relocates and tests them (the design
// explicitly forbids changing the behavior: "stickToCmd() の中身は変えない").
// They live module-local here instead of in a screen so every caller (the
// perpetual stick, the W-6 panel) shares one definition.

// 緩旋回 (直進 + 旋回同時) 時の旋回レート。
// W-07 受け入れで 0.5 → 0.275 に換算した。旧 0.5 は「速度プリセットが m/s だった
// 時代」の定数で、緩旋回の実旋回速度は wn(=0.5×m)×speedPct [rad/s] だった。
// いま画面が送るのは比率で、機体側の jog_gate が前進に v_jog_max=0.55 を、
// 旋回に w_jog_max=1.0 を掛ける。緩旋回の実速度（vx・wz とも。曲率半径を含む）を
// 変えないには、wn の係数を 0.5 × v_jog_max / w_jog_max = 0.5×0.55/1.0 = 0.275
// にする（stickToCmd の形は変えず定数だけ。後退緩旋回も同じ定数なので同じ）。
const JOG_ARC_ANG_SCALE = 0.275
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
//   上下 = 直進 / 真横 = 超信地旋回 / 斜め = 緩旋回 (旋回 0.275 倍。W-07 受け入れの換算)
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

// 正規化コマンド { vn, wn } を速度プリセット比率で**比率指令 { vx, wz }** に換算する。
// 出力は m/s・rad/s ではなく -1〜1 の比率であり、実速度への換算（上限掛け）は
// 機体側の jog_gate（v_jog_max / w_jog_max）が持つ（W-07。画面に m/s の上限を置かない）。
// WS-9T: **その場旋回（前進成分ゼロ・旋回成分あり）は speedPct を掛けない。**
// 教示は前進を低速にしたいが、曲がり角のその場旋回まで一緒に遅くなると 90° 回るのに
// 10 秒かかって使い物にならない（2026-09-04 実機報告）。その場旋回のときは wn（倒し量）
// をそのまま比率として送り、実 rad/s 上限は下流の jog_gate（w_jog_max）が握る。
// 前進・後退・緩旋回（arc / rev_arc、前進しながら曲がる）は従来どおり speedPct で絞る。
// stickToCmd() は設計で凍結なので触らず、換算だけをここに足す。
export function scaleJogCmd({ vn, wn }, speedPct) {
  const pureTurn = vn === 0 && wn !== 0
  return {
    vx: vn * speedPct,
    wz: wn * (pureTurn ? 1 : speedPct),
  }
}
