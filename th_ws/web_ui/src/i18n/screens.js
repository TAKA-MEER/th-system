// i18n/screens.js — display strings for screens/ (S-00 / S-01, WP-UI-02).
// Source of truth: docs/plan/spec/Spec-webui.md §3.1 / §3.2 / §3.2.1,
// DetailedDesign-webui.md §8.3, DetailedDesign-state.md §12.1/§12.2/§12.5.
//
// screens/ isn't one of U-4's three machine-checked directories (shell/
// parts/ros), but the task's intent ("Japanese lives only under i18n/")
// applies here too -- see DetailedDesign-wp1.md WP-UI-02 instructions.
export const SCREEN_NAMES = {
  S00: '接続確認',
  S01: 'メインメニュー',
}

// ---------------------------------------------------------------- S-00 ----
export const S00_CHECK_TITLE = '疎通確認'
export const S00_COL_DEVICE = '機器'
export const S00_COL_REQ = '扱い'
export const S00_COL_STATUS = '状態'
export const S00_REQUIRED = '必須'
export const S00_MONITOR = '監視'

// DetailedDesign-state.md §12.2's four pass/fail rows. The WebUI has no
// per-item breakdown to show (only /system/state's single mode field is in
// this packet's interface contract -- see the completion report for why),
// so all four rows always share one status; the labels still name the four
// checks individually so the operator knows what "揃うと運用可" covers.
export const S00_ITEMS = [
  { key: 'esp32_feedback', label: 'ESP32（ホイールフィードバック）' },
  { key: 'esp32_loopback', label: 'ESP32（速度指令の折り返し）' },
  { key: 'lidar', label: 'RaspberryPi4（LiDAR）' },
  { key: 'nodes', label: 'PC（ノード起動）' },
]

export const S00_STATUS_CHECKING = '確認中'
export const S00_STATUS_OK = '疎通'

// DetailedDesign-safety.md §8.3 / this packet's brief: Wi-Fi AP is a single
// point of failure, shown here explicitly, and it is *not* one of the four
// required rows above (C-01).
export const S00_AP_LABEL = 'Wi-Fi AP'
export const S00_AP_NOTE = '単一障害点'

export const S00_OVERALL_TITLE = '総合'
export const S00_READY = '運用に入れます'
export const S00_CHECKING = '確認中'
export const S00_ADVANCE = 'メインメニュー画面へ'

// ---------------------------------------------------------------- S-01 ----
export const GROUP_MOVE_TITLE = '移動'
export const GROUP_FIELD_TITLE = '試験場'
export const GROUP_MAINT_TITLE = '保守'

// "この方式は使えません" (mockup's winDisabled) generalized: also used when
// an *enabled-looking* button is rejected by /system/trigger at the moment
// it's pressed (§4.1: the UI may pre-decide for convenience, but th_state
// is the authority and can still say no -- U-11 "拒否された理由 | UI
// (ウィンドウ。押されて初めて出す)").
export const WIN_REASON_TITLE = 'この操作はできません'
export const WIN_REASON_OK = '確認'

// 運用の終了 (DetailedDesign-webui.md §8.3 / Spec-webui.md §3.2.1).
export const SHUTDOWN_TITLE = '運用の終了'
export const SHUTDOWN_UNSAVED_LABEL = '未保存のデータ'
export const SHUTDOWN_NONE = 'なし'
export const SHUTDOWN_BUTTON = '制御系を停止する'
export const SHUTDOWN_HINT = '押すと一覧を表示します'

export const SHUTDOWN_WIN_TITLE = '制御系を停止します'
export const SHUTDOWN_WIN_INTRO = '停止する前に、未保存のデータの扱いを決めてください。'
export const SHUTDOWN_WIN_NONE = '未保存のデータはありません。'
export const SHUTDOWN_SAVE = '保存'
export const SHUTDOWN_DISCARD_IDLE = '破棄'
export const SHUTDOWN_DISCARD_ARMED = '本当に破棄'
export const SHUTDOWN_CANCEL = 'やめる'
export const SHUTDOWN_CONFIRM = '停止する'
export const SHUTDOWN_LOADING = '確認しています…'

export const SHUTDOWN_DONE_TITLE = '停止が完了しました'
export const SHUTDOWN_DONE_BODY = '電源を切って構いません。'
export const SHUTDOWN_RESOLVED = '処理済み'

// /shutdown/prepare itself failing to answer (rosbridge hiccup, not a
// th_state rejection) -- distinct from reject_reason_key, which is only for
// an actual accepted:false / success:false response.
export const SHUTDOWN_LOAD_ERROR = '確認できませんでした。もう一度お試しください。'

export function unsavedCountLabel(n) {
  return `${n} 件`
}

// §12.5's "unsaved になりうるもの" -- SystemState.unsaved / /shutdown/prepare
// carry these four type keys, never Japanese (th_state never returns
// Japanese -- DetailedDesign-webui.md §7).
export const UNSAVED_LABELS = {
  venue_map: '試験場内地図',
  route: '教示経路',
  calib: '校正の補正値',
  map_patch: '地図の書き足し',
}

export function unsavedLabel(key) {
  return UNSAVED_LABELS[key] ?? key
}
