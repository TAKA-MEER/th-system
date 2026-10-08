// shell/autoBrake.js — 自動ブレーキの表示・切替の判断（1b-15 SG-B10）。
//
// 画面は表示と要求だけを持つ。状態の正本は機体側（state_manager の
// /system/state.auto_brake）で、受理も機体が決める（自律系は auto_brake_locked で拒否）。
// ここは「押せる見た目にするか」「確認ウィンドウ（W-4）を挟むか」「接近警告を出すか」を
// 決めるだけの純関数。権威ではない（DetailedDesign-webui.md §4.1）。

// 切替ボタンを押せる状態か。手動系のモード（attributes.json の auto_brake_default が
// on_locked でない）か、ジョグ介入中だけ。通信が古い（stale）ときは押せない。
export function canToggleAutoBrake(state, attributes, stale = false) {
  if (stale || !state || state.mode == null) return false
  if (state.jog_active === true) return true
  const def = attributes?.[state.mode]?.auto_brake_default
  return def != null && def !== 'on_locked'
}

// ON → OFF のときだけ確認（W-4）を出す。OFF → ON は確認なし（安全側への切替）。
export function autoBrakeNeedsConfirm(currentOn, nextOn) {
  return currentOn === true && nextOn === false
}

// 接近警告を出すか。自動ブレーキが OFF のときだけ、機体（obstacle_limiter）が
// 出した approach_warning を映す。ON のときは減速するので警告ではなく減速で伝わる。
// 未受信（null）は出さない（障害物なしとも出さない。呼び出し側が「不明」を出す）。
export function showApproachWarning(limiterStatus, autoBrakeOn) {
  if (autoBrakeOn !== false) return false
  return limiterStatus?.approach_warning === true
}

// W-6（ジョグ中）に出す自動ブレーキの状態。正本は機体側の /system/state.auto_brake。
// 未受信（null / undefined）は ON とも OFF とも言わず 'unknown'（隠さない・断言しない）。
export function autoBrakeLabelKey(autoBrake) {
  if (autoBrake === true) return 'on'
  if (autoBrake === false) return 'off'
  return 'unknown'
}
