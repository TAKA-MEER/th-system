// modes/armConfirm.js — 腕の格納の確認（SM-3.1.1-18）を出すかを決める純関数。
//
// 本システムは腕の状態を持たないので、AUTO_MEASURE（自律測定）から人が
// IDLE へ抜ける前に毎回 W-4「測定機構の腕が格納位置にあることを確かめましたか」
// を出す。その判定を 1 か所に集めてテストで縛る（画面側は表示だけ）。
//
// - ESTOP / CARRY の間は使うのは prev_mode（押下前モード）。現在モードだけ
//   を見ると自律測定からの非常停止・手押しを拾えない。
// - ESTOP / CARRY 以外は常に false。AUTO_MEASURE の回復フォルトで PAUSE に
//   落ちたときの W-1「確認」(ui.resume_ack) は resume_state（PAUSE）に留まり
//   IDLE へ抜けないので、SM-3.1.1-18 の対象外。S-60 の「自律測定を終了」は
//   S-60 自身が W-4 を出す。
// - P2 / P3（AT_PANEL の WORKING / ADJUST）は後で同じ部品を使うため、
//   AT_PANEL の拡張点はコメントで残す（このパケットでは条件に含めない）。

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
  return false
}
