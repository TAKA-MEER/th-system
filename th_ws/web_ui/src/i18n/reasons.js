// i18n/reasons.js — reject_reason_key -> Japanese.
// Source of truth: docs/plan/detailed/DetailedDesign-state.md §10.
// th_state never returns Japanese; the UI looks the key up here (only
// when a button is actually pressed and rejected — §7.2 U-11).
export const REJECT_REASONS = {
  not_allowed: '今の状態ではこの操作はできません',
  mode_entry_denied: '今のモードからは移動できません',
  tracker_disabled: '人物追跡がOFFです',
  tracker_lost: '対象を見失っています',
  no_target_selected: '対象が選ばれていません',
  working_in_progress: '作業中は行き先を選べません',
  estop_disabled_in_carry: '手押し中は非常停止を操作できません',
  estop_engaged: '非常停止が作動中です',
  hw_estop_still_pressed: '物理非常停止ボタンが押されたままです',
  route_not_found: '指定した経路が見つかりません',
  no_route_recorded: '教示済みの経路がありません',
  map_update_off: '地図更新がOFFのため保存できません',
  device_not_connected: '必要な機器が接続されていません',
  link_not_ready: '必須機器の疎通が取れていません',
  wait_clear_active: '退避待ち中は手動操作できません',
  calib_not_verified: '校正の検証が済んでいません',
  params_placeholder_blocking: '起動を止める暫定値が残っています',
  blind_mask_uncalibrated: '死角マスクが未校正です',
  unsaved_remains: '未保存のデータが残っています',
}

export const UNKNOWN_REASON_LABEL = '拒否されました（理由不明）'

export function reasonLabel(key) {
  if (!key) return null
  return REJECT_REASONS[key] ?? UNKNOWN_REASON_LABEL
}
