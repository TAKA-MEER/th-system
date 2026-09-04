// i18n/states.js — sub-state and shell-chrome display text.
// Examples in DetailedDesign-webui.md §7 ("対象選択中" / "確認中" / "走行中" /
// "一時停止中") come from this table. Screen-level wording refinements land in
// WP-UI-02+; this covers the generic state tokens the shell itself can see.
export const STATE_NAMES = {
  NONE: '',
  CHECK: '確認中',
  PAUSE: '一時停止中',
  RUN: '走行中',
  REC: '記録中',
  NAV: '移動中',
  MAPPING: '地図作成中',
  SELECT: '対象選択中',
  ROUTE_SEL: '経路選択中',
  DEV_CHECK: '機器確認中',
  SETUP: '準備中',
  POINT: '位置指示中',
  LIST: '一覧確認中',
  IDLE_P: '待機中',
}

export const UNKNOWN_STATE_LABEL = '不明'

export function stateLabel(stateName) {
  if (!stateName) return UNKNOWN_STATE_LABEL
  return STATE_NAMES[stateName] ?? stateName
}

// §6.2 fail-safe banner: shown in the header when /system/state stops arriving.
export const DISCONNECTED_LABEL = '制御系と通信できません'
export const CONNECTED_LABEL = '正常'

// Project branding text (kept out of shell/ files per U-4).
export const PROJECT_NAME_JA = '高見舎プロジェクト'

// Zone pill text (DetailedDesign-names.md §4). Full zone derivation needs
// /ui/active_screen aggregation across clients, which is screen-level work
// not yet built (WP-UI-02+); these labels are ready for when that lands.
export const ZONE_LABELS = {
  IN:  { long: '試験場内・低速・自動ブレーキON', short: '場内・低速' },
  OUT: { long: '試験場外', short: '場外' },
  NA:  { long: '対象外', short: '対象外' },
}

export const DEV_MODE_LABEL = { long: '開発モード', short: '開発' }

// Marker substring slam_control embeds in its /slam_control/last_result
// text when a discard succeeded ("OK: ... 破棄 ..."). Kept here (not
// literally in ros/useRosbridge.js) so U-4's "no Japanese outside i18n/"
// check on src/ros/ stays true even though this is matched against another
// node's wire text, not shown in the UI.
export const SLAM_DISCARD_MARKER = '破棄'

// Header fault badge shown after W-1 is dismissed without being resolved
// (DetailedDesign-webui.md §6.2: "non-display doesn't require resolution").
export const REOPEN_WINDOW_LABEL = '再表示 ▲'
export const ESTOP_ACTIVE_LABEL = '非常停止中'

// DetailedDesign-safety.md §8.3 / WP-UI-02: Wi-Fi AP is a single point of
// failure; S-00 shows it as a device-list row, the header carries the same
// note as a tooltip (no extra layout width, so it can't break U-10's
// "header never wraps past 2 rows").
export const AP_SPOF_NOTE = 'Wi-Fi AP は単一障害点です'

// Release bar (bottom-fixed; distinct wording from the header estop button
// per §2.2 — it must not be mistaken for the same control).
export const ESTOP_RELEASE_NOTE = '非常停止中です。解除するには下のボタンを押してください。'
export const ESTOP_RELEASE_BUTTON = '非常停止を解除'
export const ESTOP_BUTTON_LABEL = '非常停止'

// W-1 / W-2 window chrome.
export const WIN_HIDE_LABEL = '非表示'
export const WIN_RESUME_YES = 'はい'
export const WIN_RESUME_NO = 'いいえ'
export const WIN_RESUME_ACK = '確認'
// UI 非常停止（重大フォルト無し）解除後の 2 択（2026-09-01。SM-3.1.1-11）
export const WIN_ESTOP_RESUME_PREV = '元のモードに戻る'
export const WIN_ESTOP_TO_MENU = 'メインメニューへ'
export const WIN_ESTOP_TITLE = '非常停止中'
export const WIN_ESTOP_BODY = '非常停止ボタンが押されました。速度指令をゼロにしています。'
export const WIN_ESTOP_HINT = '解除は画面下端の解除バーから。解除すると元のモードの再開を選べます。'
// fault 起因の ESTOP（DRIVE_RUNAWAY 等）。「ボタンが押された」ではないので別文言（#1）。
export const WIN_ESTOP_SYSTEM_TITLE = '安全のため停止しました'
export const WIN_ESTOP_SYSTEM_BODY = '安全監視が異常を検知し、速度指令をゼロにしています。'
export const WIN_ESTOP_SYSTEM_HINT = '原因が解消したら「確認」を押すとメインメニューに戻ります。'
// WS-9O (2026-09-04): フォルト起因でも元のモードへ戻れるようになった。戻り先は
// 停止状態なので、走り出すにはもう一度「再生」を押す必要がある（VISION.md §2）。
export const WIN_ESTOP_SYSTEM_HINT_RESUMABLE =
  '原因は解消しました。元のモードに戻ると停止した状態から再開できます。'

// W-1 generalized to fault-caused PAUSE, not just ESTOP (DetailedDesign-wp1.md
// WP-UI-01 §11 "W-1 の一般化" -> WP-UI-02). The body text itself (cause and
// remedy) comes from i18n/faults.js, keyed by FaultStatus.fault_type.
export const WIN_FAULT_TITLE = '異常が発生しました'
export const WIN_FAULT_HINT = '解決すると、この画面が再開の選択肢に変わります。'
export const WIN_CARRY_TITLE = '手押しモード'
export const WIN_CARRY_BODY = '物理非常停止ボタンが押されています。人が機体を押して運んでいるとみなします。'
export const WIN_CARRY_HINT = '解除方法：機体後部の赤いボタンを右へひねってください。ロックが外れて飛び出します。'
export const WIN_CARRY_RELEASED = '解除されました'
export const WIN_CARRY_RESUME = '押下前のモードで再開する'
export const WIN_CARRY_DISMISS = '再開しない（待機に戻る）'

// C-2 (DetailedDesign-wp1.md WP-CARRY-01 §4.3, §7 / DetailedDesign-state.md
// :764): shown in W-2 regardless of whether the UI estop button has been
// pressed -- the drive is already cut for the whole time CARRY holds, not
// only after a rejected press. The event-specific reason
// (estop_disabled_in_carry, i18n/reasons.js) layers on top of this once
// SystemState.last_reject_reason actually reports the C-06r rejection.
export const WIN_CARRY_ESTOP_DISABLED = '駆動は既に切れています。'

// Operation card button labels (DetailedDesign-webui.md §4.1). `run`'s
// label varies per screen ("follow start" / "run" / "replay" / ...); this
// is only the generic default until a screen overrides it.
export const OP_LABELS = {
  stop: '停止',
  check: '確認',
  run: '走行',
  save: '保存',
  manual: '手動',
  finish: '終了',
}

// WS-9R (2026-09-04): 速度上限が 0 に落ちている理由。黙って止まるのをやめる。
export const STOP_REASON_LABELS = {
  fault: '異常を検知して停止しています。',
  presence: 'タブレットの在席が確認できないため停止しています。画面をこのアプリに戻すか、接続を確認してください。',
  obstacle: '前方に障害物があるため停止しています。',
  uncalibrated: '死角の校正が済んでいないため停止しています。',
}

export function stopReasonLabel(reason) {
  return STOP_REASON_LABELS[reason] ?? null
}
