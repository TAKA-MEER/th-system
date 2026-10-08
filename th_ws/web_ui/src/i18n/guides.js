// i18n/guides.js — guide{key} -> Japanese (W-3 banner text).
// Source of truth: th_ws/src/th_state/config/transitions.yaml の guide の
// `key` (遷移表が正。DetailedDesign-names.md §4.2 の一覧は古い)。
// Kept short (one line, §7.2 U-11): "what can be done next", not an
// explanation of design intent.
export const GUIDES = {
  estop_held_at_boot: '起動時に非常停止ボタンが押されたままです。戻してください',
  route_end: '経路の終点に到着しました',
  home_arrived: '待機場所に到着しました',
  leash_not_connected: 'リードデバイスが接続されていません',
  target_lost: '対象を見失いました。立ってほしい位置に立ってください',
  register_rejected: '対象の登録が拒否されました',
  register_target_lost: '登録中に対象を見失いました。立ってほしい位置に立ってください',
  line_lost: '誘導線を見失いました',
  summon_clear_timeout: '退避待ちが時間切れになりました',
  summon_target_lost: '呼び寄せ中に対象を見失いました',
  // 1b-11 SG-B19: 地図・経路が読めないときの原因別の案内（Spec-transit.md §4.2.3）。
  route_no_map: 'この経路には使える地図がありません。別の経路を選ぶか、教示し直してください',
  route_unreadable: '経路ファイルが読み込めません。別の経路を選ぶか、教示し直してください',
  // 下の 2 つは guide effect としては出ない (reject_reason_key 側のキー)。
  // 消すかどうかは未確定のため残す (1b-6 報告参照)。
  blind_mask_uncalibrated: '死角マスクが未校正です。校正してください',
  params_placeholder_blocking: '起動を止める暫定値が残っています',
}

export function guideLabel(key) {
  if (!key) return null
  return GUIDES[key] ?? null
}
