// i18n/guides.js — guide{key} -> Japanese (W-3 banner text).
// Source of truth: docs/plan/detailed/DetailedDesign-names.md §4.2.
// Kept short (one line, §7.2 U-11): "what can be done next", not an
// explanation of design intent.
export const GUIDES = {
  estop_held_at_boot: '起動時に非常停止ボタンが押されたままです。戻してください',
  route_end: '経路の終点に到着しました',
  home_arrived: '待機場所に到着しました',
  leash_not_connected: 'リードデバイスが接続されていません',
  target_lost: '対象を見失いました。立ってほしい位置に立ってください',
  register_rejected: '対象の登録が拒否されました',
  line_lost: '誘導線を見失いました',
  wait_clear_timeout: '退避待ちが時間切れになりました',
  blind_mask_uncalibrated: '死角マスクが未校正です。校正してください',
  params_placeholder_blocking: '起動を止める暫定値が残っています',
}

export function guideLabel(key) {
  if (!key) return null
  return GUIDES[key] ?? null
}
