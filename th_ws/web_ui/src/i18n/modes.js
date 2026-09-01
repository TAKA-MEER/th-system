// i18n/modes.js — mode display names.
// Source of truth: docs/plan/spec/Spec-webui.md §8.1 (18 rows). Keep this
// object in the same order and wording as that table; test/unit/i18n-modes.test.js
// checks it verbatim.
export const MODE_NAMES = {
  INIT: '起動中',
  IDLE: '待機',
  ESTOP: '非常停止',
  CARRY: '手押し',
  FOLLOW: '追従走行',
  MANUAL: '手動走行',
  TEACH_FOLLOW: '教示（追従）',
  TEACH_MANUAL: '教示（手動）',
  REPLAY: '教示再生',
  LINE: 'ライン誘導',
  LEASH: '電子リード',
  PREP: '試験準備',
  PANEL_NAV: '配電盤へ移動',
  AT_PANEL: '配電盤前',
  SUMMON: '呼び寄せ',
  HOME_NAV: '待機場所へ移動',
  OPCHECK: '始業点検',
  CALIB: '校正',
}

// §6.2 fail-safe: when /system/state is stale, the mode display must not be
// left at its last value — it must show "unknown".
export const MODE_UNKNOWN_LABEL = '不明'

export function modeLabel(mode) {
  if (!mode) return MODE_UNKNOWN_LABEL
  return MODE_NAMES[mode] ?? MODE_UNKNOWN_LABEL
}
