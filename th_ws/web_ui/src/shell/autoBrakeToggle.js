// shell/autoBrakeToggle.js — W-6 と自律系の画面に出す自動ブレーキ切替の判断（A-15）。
//
// 画面は表示と要求だけを持つ（受理は機体。state_manager の set_flag）。ここは
// 「開発モードの項目 auto_brake が実効か」「この画面で切替を出すか・押せるか」を
// 決める純関数。shell/autoBrake.js の canToggleAutoBrake（手動系・ジョグ中）に
// 開発モードの例外（Spec-safety §10）を足した形。

// /system/dev_mode の effective に auto_brake が明示されているときだけ真。
// 機体側（th_state/auto_brake.py の dev_item_effective）と同じく、未受信・null は偽。
export function devAutoBrakeEffective(dev) {
  return dev?.effective?.auto_brake === true
}

// 自律系の画面（S-14 / S-21）に切替を出すか。開発モードで項目を選んでいるときだけ
// （Spec-webui §3.5。選んでいなければ出さない）。
export function autoBrakeCardVisible(dev) {
  return devAutoBrakeEffective(dev)
}

// 押せるか。手動系・ジョグ中（canManual は shell/autoBrake.js の結果）か、
// 開発モードの項目が実効のとき。通信が古い・モード未受信は押せない。
export function canToggleWithDev(canManual, state, stale, devOn) {
  if (stale || !state || state.mode == null) return false
  return canManual === true || devOn === true
}
