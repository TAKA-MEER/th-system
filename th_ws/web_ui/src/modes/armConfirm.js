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
// - 8-g P2-ui: ESTOP / CARRY では prev_mode が AUTO_MEASURE のほかに
//   /system/state.working が true のときも出す（AT_PANEL の WORKING から
//   非常停止・手押しに入った場合。WORKING の間は ESTOP / CARRY でも
//   working は外れない）。AT_PANEL / WORKING からの「終了」(ui.finish) も
//   出す（working が true のときだけ。AT_PANEL の IDLE_P 由来は出さない）。
//   AT_PANEL / WORKING の W-1「確認」(ui.resume_ack) は WORKING に留まる
//   ので対象外（trigger が ui.resume_ack のとき AT_PANEL では出さない）。

export const ARM_CONFIRM_TRIGGERS = ['ui.resume_no', 'ui.resume_ack', 'ui.finish']

// needsArmConfirmForExit({ trigger, currentMode, prevMode, working }) -> bool
//
// trigger: 送ろうとしている ui.* （IDLE へ抜けるものだけ。ui.resume_yes /
// ui.carry_resume は元のモードへ戻るので対象外）。
// currentMode: /system/state の mode。prevMode: 同 prev_mode。
// working: 同 working（停止命令の間。ESTOP / CARRY の間も押下前が
// AT_PANEL / WORKING なら true）。
export function needsArmConfirmForExit({ trigger, currentMode, prevMode, working = false }) {
  if (!ARM_CONFIRM_TRIGGERS.includes(trigger)) return false
  if (currentMode === 'ESTOP' || currentMode === 'CARRY') {
    return prevMode === 'AUTO_MEASURE' || working === true
  }
  if (currentMode === 'AT_PANEL' && trigger === 'ui.finish') {
    return working === true
  }
  return false
}
