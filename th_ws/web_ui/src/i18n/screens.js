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
  // WP-UI-06: S-20 試験準備（PREP）。Spec-webui.md §3.10。
  S20: '試験準備',
  // WP-UI-07: S-21 試験（PANEL_NAV / SUMMON / AT_PANEL / HOME_NAV）。
  S21: '試験',
  // WS-9X: S-50 設定（Spec-webui.md §3.15）。S-01 の「保守・設定」から開く
  // IDLE のサブ画面。
  S50: '設定',
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
// WS-9X: 「設定」ボタン（S-50 を開く）をこのカードに同居させたので「保守・設定」。
export const GROUP_MAINT_TITLE = '保守・設定'

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
export const S13_SAVE_FAILED = '保存されていません'
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
export const S14_POSE_LOCALIZE_TIMEOUT =
  '初期姿勢を推定できません。機体を始点マークに置き直すか、教示からやり直してください'
export const S14_POSE_READY = '推定できました。地図と位置を確認して『再生』を押してください'
export const S14_POSE_RUN = '再生中'
export const S14_POSE_PAUSE = '一時停止（記録の終端に達しました。終了を押してください）'
// WS-9P (2026-09-04): PAUSE は終端到達だけでなくフォルト・ジョグ介入・停止ボタンでも
// 起きる。以前はすべて上の「終了を押してください」を出していたため、ESP32 が一瞬
// 途切れて止まっただけの操作者に「もう終わりだ」と誤って案内していた。
export const S14_POSE_PAUSE_RESUMABLE =
  '一時停止中。「再生」を押すと止まったところから続きを走ります。'
export const S14_LENGTH = '距離'
export const S14_POINTS = '点数'
// WS-9T: 再生速度の低速/中速/高速 切替（走行中も変更可）。
export const S14_SPEED_TITLE = '再生速度'

// ---------------------------------------------------------------- 経路プレビュー（WS-3） ----
// src/screens/RoutePreview.jsx の表示文字列。
export const ROUTE_PREVIEW_EMPTY = '経路がありません'

// ---------------------------------------------------------------- キーボード操作（WS-4） ----
// parts/JogConsole.jsx の操作キー凡例。押下中のキーは .kbdHint.on でハイライト。
export const KBD_HINT_KEYS = 'W/A/S/D または矢印キー'
export const KBD_HINT = 'で走行'

// ---------------------------------------------------------------- S-20 (試験準備) ----
// WP-UI-06。Spec-webui.md §3.10 / mockup 957〜1054 行。
// 日本語はパケットの常規則どおり i18n だけに置く（screens/S20Prep.jsx と
// parts/RadarSelect.jsx はこの定数だけを参照する）。
export const S20_TAB_MAP = '地図'
export const S20_TAB_TARGET = '対象選択'
export const S20_UNSAVED = '未保存'
export const S20_MAP_TITLE = '試験場内地図'
export const S20_MAP_ARIA = '試験場内地図とピン'
export const S20_MAP_ROBOT = '現在位置'
export const S20_MAP_NO_POSE = '現在位置を取得中…'
export const S20_SUBTAB_REGISTER = '登録'
export const S20_SUBTAB_PINS = 'ピン'
export const S20_REGISTER_TITLE = '登録'
export const S20_REG_HOME = '待機場所を登録'
export const S20_REG_PANEL = '配電盤を登録'
// brief-onsite-register-fix REG-2: 機体姿勢での登録（/onsite/register_pin,
// method=ROBOT_POSE）。2 点指示（人が立つ場所を指定）と違い、機体の現在位置・
// 向きでその場で登録する。
export const S20_REG_HOME_HERE = 'いまの姿勢で待機場所を登録'
export const S20_REG_PANEL_HERE = 'いまの姿勢で配電盤を登録'
export const S20_REG_HERE_NOTE =
  '「待機場所/配電盤を登録」は 2 点指示（人が立つ場所）、「いまの姿勢で登録」は機体の現在位置で登録します'
export const S20_REG_HERE_OK = (kind) =>
  `${kind === 'HOME' ? '待機場所' : '配電盤'}を機体のいまの姿勢で登録しました`
export const S20_WIZ_STEP = (n) => `Step ${n}/2`
// Step 1: 立ち位置。Step 2: 向きたい方向へ一歩進んだ場所（Spec-onsite §3.1 F-08）。
// 同じ文言だと「2 回目も同じ場所で押せばよい」と誤解される
// （実機 2026-09-09。two_point_too_close の連発の原因）。
export const S20_WIZ_MSG_STEP1 = '登録したい場所に立って押してください'
export const S20_WIZ_MSG_STEP2 = '機体に向いてほしい方向へ一歩進んで押してください'
export const S20_WIZ_REGISTER = 'この位置を登録'
export const S20_PINS_TITLE = 'ピン'
export const S20_PIN_EDIT = '編集'
export const S20_PIN_RENAME = '改名'
export const S20_PIN_DELETE = '削除'
export const S20_PIN_CANCEL = 'やめる'
export const S20_PIN_YAW = (deg) => `向き ${deg}°`
export const S20_RETURN_HOME = '1 ボタンで待機場所に戻す'
export const S20_RETURN_HINT = '待機場所のピン "HOME" を登録すると使えます'
// brief-onsite-ux UX-1: S-20 の手順バー（screens/onsiteSteps.js の id に対応）と
// 「次にやること」ボタン（parts/StepBar.jsx / S20Prep.jsx の next-action）。
export const S20_STEP_MAP = '地図を作る'
export const S20_STEP_TARGET = '対象を選ぶ'
export const S20_STEP_HOME = '待機場所を登録'
export const S20_STEP_PANEL = '配電盤を登録'
export const S20_STEP_SAVE = '保存'
export const S20_NEXT_SELECT_TARGET = '対象を選ぶ'
export const S20_NEXT_REG_HOME = '待機場所を登録'
export const S20_NEXT_REG_PANEL = '配電盤を登録'
export const S20_NEXT_SAVE = '保存'
// brief-onsite-ux2 F-6: 「地図作成開始」を押すまで地図を出さない表示ゲート。
export const S20_MAP_GATE_MSG = '地図作成を開始してください'
export const S20_MAP_GATE_BUTTON = '地図作成開始'
export const S20_NEXT_START_MAPPING = '地図作成開始'

// 対象選択レーダー（parts/RadarSelect.jsx。S-20 / S-21 で共用。Spec-webui.md §9.1）。
export const RADAR_ARIA = '対象選択レーダー'
export const RADAR_LOST = '対象を見失っています'
// 実機で確認（2026-09-08）: multiple_sensor_person_tracking は require_explicit_target_selection
// が既定 true なので、対象を見失うと自動では再取得しない（安全設計。誤って別人を追わない）。
// 一方で脚検出（候補）自体は見失っている間も届き続けるので、タップで選び直せるようにする必要が
// ある。以前は isLost の間、候補を隠して「見失っています」だけを出していたため、候補が複数いる
// （＝自動再選択が働かない）と操作者に選び直す手段が無いまま復帰不能になっていた。
export const RADAR_LOST_RESELECT = '対象を見失いました。候補をタップして選び直してください'
export const RADAR_EMPTY = '試験員が検出されていません'
export const RADAR_SELECTED = '対象'
export const RADAR_CONFIDENCE = (c) => `確信度 ${c}`
// brief-onsite-fix F-2: レーダー機体マークの「前」ラベル。
export const RADAR_HEADING = '前'

// ---------------------------------------------------------------- S-21 試験（WP-UI-07） ----
// screens/S21Test.jsx の表示文字列。brief-UI-S21 / mockup 1055〜1153 行。
// 2 点指示ウィザードと「地図」「対象選択」タブは S-20 の定数をそのまま流用する。
export const S21_SELECT_HINT = '行き先を選んでください'
export const S21_SUBTAB_DEST = '行き先'
export const S21_SUBTAB_SUMMON = '呼び寄せ'
export const S21_SUBTAB_ATPANEL = '配電盤前'
export const S21_PREP_TITLE = '準備 ／ (a) ピンを選ぶ'
export const S21_NOW_AT_HOME = 'いま待機場所にいる'
// brief-onsite-fix E: 保存した会場地図（slot:VENUE）を当日に開き直すボタン。
export const S21_OPEN_VENUE_MAP = '保存した会場地図を開く'
export const S21_OPEN_VENUE_DONE = '会場地図を開きました'
// 失敗時に message が空で返ることがある（空の黄色い箱だけが出るのを防ぐ）。
export const S21_OPEN_VENUE_FAIL = '会場地図を開けませんでした'
// brief-onsite-ux2 F-6: 「会場地図を開く」まで地図タブに地図を出さない表示ゲート。
export const S21_MAP_GATE_MSG = '保存した会場地図を開いてください'
export const S21_HOME_UNDECLARED = '未宣言'
export const S21_HOME_DECLARED = '宣言済み'
export const S21_HOME_DECLARE = '宣言する'
export const S21_HOME_FORCE_DECLARE = '強制的に宣言'
export const S21_HOME_RETRY_LATER = 'あとで'
export const S21_PIN_MOVE = '移動'
export const S21_NEXT_DEST_TITLE = '次の行き先'
export const S21_DEST_HOME = '待機場所'
export const S21_DEST_NEXT_PANEL = '次の配電盤'
export const S21_DEST_SUMMON_HERE = 'その場で呼ぶ'
export const S21_PICK_PANEL_HINT = '移動したいピンを押してください'
// UX-7: ピンが 1 件も無いときの空表示。以前はカード見出し（S21_PREP_TITLE）を
// そのまま出していて、同じ文字列が 2 回並んでいた。
export const S21_PINS_EMPTY = '登録済みのピンがありません（前日の試験準備で登録します）'
export const S21_SUMMON_TITLE = '(b) その場で呼ぶ'
export const S21_SUMMON_START = '呼び寄せ（2 点指示）'
export const S21_WAIT_TITLE = '退避待ち — 退いてください'
export const S21_WAIT_DIST = (m) => `${m.toFixed(1)} m`
export const S21_WAIT_CANCEL = '中止'
export const S21_WAIT_CLEARING = '対象の場所が空き次第、自動で続行します'
export const S21_ATPANEL_TITLE = '配電盤前'
export const S21_WORKING = '作業中'
export const S21_WORK_ON = 'ON'
export const S21_WORK_OFF = 'OFF'
export const S21_MAP_UPDATE = '地図の更新'
export const S21_MAP_UPDATE_OFF = 'OFF'
export const S21_MAP_UPDATE_NOTE = '地図更新 OFF の間は「保存」を出しません'
// brief-onsite-ux UX-1: S-21 の手順バー（screens/onsiteSteps.js の id に対応）と
// 「次にやること」ボタン（parts/StepBar.jsx / S21Test.jsx の next-action）。
export const S21_STEP_OPEN = '会場地図を開く'
export const S21_STEP_HOME = '待機場所を宣言'
export const S21_STEP_DEST = '行き先を選ぶ'
export const S21_STEP_MOVE = '移動'
export const S21_STEP_WORK = '作業/呼び寄せ'
export const S21_NEXT_OPEN_VENUE = '会場地図を開く'
export const S21_NEXT_DECLARE_HOME = '待機場所を宣言'
export const S21_NEXT_PICK_DEST = '行き先を選ぶ'
export const S21_NEXT_STOP = '停止'
export const S21_NEXT_WORK = '作業中'
export const S21_NEXT_SUMMON = '呼び寄せ'
// brief-onsite-ux-fix UX-6: 画面ローカルの「押せない理由」バッジ。
// i18n/reasons.js の REJECT_REASONS は th_state（FSM）が返すキーの写しなので、
// 画面だけで判定できる理由はここに置く（onsiteSteps.js の onsiteReasons() が使う）。
export const BADGE_JOG_DENIED = 'このモードでは手動操作できません'
export const BADGE_NO_PANEL_PIN = '配電盤ピンが登録されていません'
export const BADGE_NO_HOME_PIN = '待機場所ピンが登録されていません'

// ---------------------------------------------------------------- S-50 設定（WS-9X） ----
// screens/S50Settings.jsx の表示文字列。Spec-webui.md §3.15 のタブ構成。
export const S01_SETTINGS = '設定'            // S-01「保守・設定」カードのボタン
// brief-UI-S21-entry: S-01「試験場」の「試験」ボタン（mockup 555 行の文言どおり。
// S-21 は FSM のモードではないので PREP のようなモード遷移ボタンではない）。
export const S01_ONSITE_TEST = '試験（当日）'
// brief-onsite-fix A: 画面の無いモードで S-01 が出ているとき、ui.finish で IDLE に戻す導線。
export const S01_FINISH_ESCAPE = '終了して待機に戻る'
export const S50_BACK = '戻る'
export const S50_TAB_GENERAL = '一般'
export const S50_TAB_DISPLAY = '表示'
export const S50_TAB_DEV = '開発モード'
// 変更可能なのは IDLE / MANUAL のときだけ（config_manager がサーバ側で再確認する）。
export const S50_GUARD = 'IDLE または MANUAL のときだけ変更できます（今は変更不可）'
export const S50_SAVE_YAML = 'YAML に保存'
export const S50_SAVING = '保存中…'
export const S50_SAVED = '保存しました'
export const S50_SAVE_FAILED = '保存に失敗しました'
export const S50_LOAD_FAILED = '現在値を取得できませんでした'
// 一般タブ: 3 セクション見出し
export const S50_SEC_FOLLOW = '軌跡追従（follow_planner_mapless）'
export const S50_SEC_LIDAR = 'LiDAR 死角マスク（lidar_filter）'
export const S50_SEC_SLAM = '再生の自己位置推定（slam_toolbox）'
export const S50_SLAM_NOTE =
  '変更は「YAML に保存」してから、次に「この経路で進む」を押したときに有効になります'
  + '（SLAM がその場では読み直さないため）。補正頻度を上げる＝間隔を小さく、'
  + '廊下でのずれに強くする＝探索窓を広げる。'
// 表示タブ: 文字サイズ
export const S50_FONT_TITLE = '文字サイズ'
export const S50_FONT_NORMAL = '標準'
export const S50_FONT_LARGE = '大'
export const S50_FONT_XLARGE = '特大'
// 開発モードタブ
export const S50_DEV_TITLE = '開発モード'
export const S50_DEV_ENABLE = '開発モードを有効にする'
export const S50_DEV_DISABLE = '開発モードを無効にする'
export const S50_DEV_NOTE =
  '安全系のバイパスなど開発向けの設定は今後ここに追加します。'
  + '現状はヘッダに「開発」表示が出るだけです。'
