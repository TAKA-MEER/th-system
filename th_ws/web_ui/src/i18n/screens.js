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
  // S-11 (手動操作) は driveTab kind='manual' の常設タブで使用 (WP-UI-03)。
  // 画面名の正式値は後続パケットで確定するが、テスト用に名前だけ用意する。
  S11: '手動操作',
  // P5 / demo-teach-replay: S-13 教示（手動）と S-14 教示再生。
  // 正式値は names.json / Spec-webui.md §3.7 が正。
  S13: '教示（手動）',
  S14: '教示再生',
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

// ---------------------------------------------------------------- 走行タブ / W-6 ----
// DetailedDesign-wp3.md WP-UI-03. parts/screens は日本語リテラル禁止 (c4) なので、
// スティック・速度プリセット・W-6 パネルの表示文字はすべてここに置く。
export const SPD_HIGH = '高速'
export const SPD_MID = '中速'
export const SPD_LOW = '低速'
export const SPD_SLIDER_ARIA = '速度'

// stickGeometry.js が返す label キー → 日本語。8方向セクターの動作表示。
export const STICK_LABELS = {
  forward: '前進',
  arc: '緩旋回',
  turn: '超信地旋回',
  rev_arc: '後退緩旋回',
  reverse: '後退',
}
export const STICK_ACTIVE = '操作中'
export const STICK_FWD = '前進'
export const STICK_BACK = '後退'
export const STICK_LEFT = '左'
export const STICK_RIGHT = '右'

// driveTab kind='follow' の状況 (差し込み口は WP-UI-04 で埋める)。空カーソルの aria 用。
export const FOLLOW_RADAR_SLOT = '対象選択'

// W-6 手動操作パネル (DetailedDesign-webui.md §6.3)。
export const W6_TITLE = '手動操作（触れている間だけ）'
export const W6_CLOSE = '閉じる'
export const W6_MODE = 'モードは「{mode}」のまま変わりません'

// 手動操作タブの「手動」ボタン (driveTab kind='manual' から W-6 を開く)。
export const DRIVE_MANUAL = '手動'
// テスト用ドライブ画面の「離れる」ボタン (U3-1 の unmount 確認用に
// DriveTab をマウントから外す。実画面の画面遷移に置き換わる)。
export const DRIVE_LEAVE = '離れる'

// ---------------------------------------------------------------- S-11 (手動走行) ----
// DetailedDesign-wp3.md WP-TRANSIT-01 §4.2 / mockup L620-656。
// parts/screens は日本語リテラル禁止 (c4) なので、S-11 の表示文字はすべてここに置く。
export const S11_TAB_DRIVE = '走行'
export const S11_OBSTACLE_TITLE = '障害物'
export const S11_OBSTACLE_WARN = (d) => `前方 ${d} m に障害物（警告）`
export const S11_OBSTACLE_STOP = (d) => `前方 ${d} m に障害物（停止）`
export const S11_OBSTACLE_PASS = '障害物なし'
// 未受信のときの表示（"障害物なし" と出さない。安全側 DetailedDesign-wp3.md §6.2）
export const S11_OBSTACLE_UNKNOWN = '障害物情報なし'
export const S11_AUTO_BRAKE_LABEL = '自動ブレーキ'
export const S11_AUTO_BRAKE_OFF = 'OFF'
export const S11_AUTO_BRAKE_ON = 'ON'
export const S11_REAR_TITLE = '後退'
export const S11_REAR_NOTE = '後方は LiDAR の死角。後退速度を制限中'
export const S11_MANUAL_TITLE = '手動操作'

// ---------------------------------------------------------------- S-13 (教示（手動）) ----
// P5 / demo-teach-replay. Spec-transit.md §0.5 / U-5: 教示（手動）＝手動走行画面＋
// 「教示」タブ 1 枚。走行部分は S-11 と完全に共通、教示タブに入るのは「経路の選択
// （新規／既存）」と「記録の状況（距離・時間・点数・開始時の向き）」だけ。
// 走行に関する見出し（障害物／後退／手動操作）は S-11 の定数を再利用する。
export const S13_TAB_TEACH = '教示'
export const S13_TEACH_TITLE = '教示'
export const S13_ROUTE_NAME_LABEL = '経路名'
export const S13_ROUTE_NAME_PLACEHOLDER = '経路名を入力'
export const S13_RECORD_START = '記録開始'
export const S13_RECORDING = '記録中'
export const S13_SAVED = '保存しました'
export const S13_RECORDED_LABEL = '記録距離'
export const S13_POINTS_LABEL = '点数'
export const S13_ELAPSED_LABEL = '経過時間'
export const S13_START_YAW_LABEL = '開始時の向き'
export const S13_SEC = (s) => `${s} 秒`
export const S13_M = (m) => `${m} m`
export const S13_DEG = (deg) => `${deg}°`
export const S13_REC_DIRECTION = '経路名を入れて「記録開始」を押すと、操作での走行を経路として記録します。'

// ---------------------------------------------------------------- S-14 (教示再生) ----
// P5 / demo-teach-replay. Spec-webui.md §3.7 の簡略版。
export const S14_TAB_REPLAY = '再生'
export const S14_SELECT_TITLE = '経路を選択'
export const S14_EMPTY = '教示済みの経路がありません'
export const S14_FWD = '順再生'
export const S14_REV = '逆再生'
export const S14_PROCEED = 'この経路で進む'
export const S14_POSE_TITLE = '初期姿勢'
export const S14_POSE_LOCALIZE = '初期姿勢を推定しています…'
export const S14_POSE_READY = '推定できました。地図と位置を確認して『再生』を押してください'
export const S14_POSE_RUN = '再生中'
export const S14_POSE_PAUSE = '一時停止（記録の終端に達しました。終了を押してください）'
export const S14_LENGTH = '距離'
export const S14_POINTS = '点数'

// ---------------------------------------------------------------- 経路プレビュー（WS-3） ----
// src/screens/RoutePreview.jsx の表示文字列。
export const ROUTE_PREVIEW_EMPTY = '経路がありません'
