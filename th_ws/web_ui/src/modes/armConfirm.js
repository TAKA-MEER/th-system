// modes/armConfirm.js — 腕の格納の確認（SM-3.1.1-18）を出すかを決める純関数。
//
// 本システムは腕の状態を持たないので、AUTO_MEASURE（自律測定）から人が
// IDLE へ抜ける前に毎回 W-4「測定機構の腕が格納位置にあることを確かめましたか」
// を出す。その判定を 1 か所に集めてテストで縛る（画面側は表示だけ）。
//
// - ESTOP / CARRY の間は使うのは prev_mode（押下前モード）。現在モードだけ
//   を見ると自律測定からの非常停止・手押しを拾えない。
// - ESTOP / CARRY 以外は現在モードで判定する。AUTO_MEASURE 自身の W-1
//   （フォルトで PAUSE）の「確認」(ui.resume_ack) が該当する。
// - P2 / P3（AT_PANEL の WORKING / ADJUST）は後で同じ部品を使うため、
//   AT_PANEL の拡張点も aspirational ではなく条件側に書ける形にする。
//   （このパケットでは AUTO_MEASURE だけが条件になる。AT_PANEL の条件は
//   P2 で足す。）

export const ARM_CONFIRM_TRIGGERS = ['ui.resume_no', 'ui.resume_ack', 'ui.finish']

// needsArmConfirmForExit({ trigger, currentMode, prevMode }) -> bool
//
// trigger: 送ろうとしている ui.* （IDLE へ抜けるものだけ。ui.resume_yes /
// ui.carry_resume は元のモードへ戻るので対象外）。
// currentMode: /system/state の mode。prevMode: 同 prev_mode。
export function needsArmConfirmForExit({ trigger, currentMode, prevMode }) {
  if (!ARM_CONFIRM_TRIGGERS.includes(trigger)) return false
  if (currentMode === 'ESTOP' || currentMode === 'CARRY') {
    return prevMode === 'AUTO_MEASURE'
  }
  return currentMode === 'AUTO_MEASURE'
}
