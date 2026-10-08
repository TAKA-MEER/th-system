// i18n/calib.js — S-40 校正画面の文言（WP-MAINT-03）。日本語は i18n だけに置く
// （screens/S40Calib.jsx には直書きしない）。Spec-webui.md §3.14・
// DetailedDesign-maintenance.md §3。

export const S40_TITLE = '校正'

export const S40_ITEM_LABELS = {
  LINEAR: '直進校正（車輪半径）',
  ROTATION: '旋回校正（車輪間距離）',
  IMU: 'IMU 校正',
  BLIND: 'LiDAR 死角',
}
export const S40_UNIT = { LINEAR: 'm', ROTATION: '°' }

// ── 左カラム ──────────────────────────────────────────────
export const S40_TAB_ITEMS = '項目'
export const S40_TAB_HISTORY = '履歴'
export const S40_ABORT = '中断'
export const S40_FINISH = '終了'
export const S40_START = '開始'
export const S40_BACK_TO_OPCHECK = '始業点検へ戻る'
export const S40_LAST_CALIBRATED = '前回'
export const S40_NEVER_CALIBRATED = '未校正'
export const S40_ROTATION_FIRST = '先に直進校正を実施してください'
export const S40_PLACEHOLDER = '項目を選んで「開始」を押してください'
export const S40_COMMITTED = '補正値を確定しました。'

// ── ウィザード ────────────────────────────────────────────
export const S40_STEP_LABELS = ['案内', '実行・実測', '確認', '検証']
export function s40StepTitle(n) { return `Step ${n}/4` }

export function s40GuideText(item, detail) {
  if (item === 'LINEAR') {
    return `周囲 2 m 以上を空け、メジャーを用意してください。ロボットが ${detail?.commanded ?? '?'} m を自律で直進します。止めたいときは「中断」か非常停止を押してください。`
  }
  if (item === 'ROTATION') {
    return `周囲 2 m 以上を空け、分度器か壁の目印を用意してください。ロボットが ${detail?.commanded ?? '?'}° を自律でその場旋回します。止めたいときは「中断」か非常停止を押してください。`
  }
  if (item === 'BLIND') {
    return 'ロボットは動きません。次の画面でライブスキャンの上をなぞり、支柱などが写り込んでいる角度帯を選びます。選んだ帯は障害物として見なくなるので、本物の障害物の方向は選ばないでください。止めたいときは「中断」を押してください。'
  }
  return ''
}
export const S40_GUIDE_NEXT = '走り出す'
export const S40_GUIDE_NEXT_BLIND = 'スキャンを表示する'
export const S40_RUNNING = '実行中…'
export const S40_RUNNING_VERIFY = '補正を適用して検証走行中…'
export const S40_MEASURE_LABEL = {
  LINEAR: '実際に進んだ距離 (m)',
  ROTATION: '実際に回った角度 (°)',
}
export const S40_MEASURE_SUBMIT = '確認'
export const S40_VERIFY_LABEL = {
  LINEAR: '検証走行で実際に進んだ距離 (m)',
  ROTATION: '検証旋回で実際に回った角度 (°)',
}
export const S40_VERIFY_SUBMIT = '検証結果を送る'
export const S40_PREVIEW_TITLE = '補正のプレビュー'
export const S40_PREVIEW_BEFORE = '現在'
export const S40_PREVIEW_AFTER = '補正後'
export const S40_PREVIEW_INSANE = '入力値が不正か、補正が大きすぎます。測り直して入力してください（直進は ±10% まで。それ以上のずれは校正ではなく機械の異常です）。'
export const S40_PREVIEW_NEXT = '次へ（適用して検証走行）'
export const S40_SUBMIT_REJECTED = '受け付けられませんでした'
export const S40_RETRY_TITLE = '検証で許容範囲を超えたため、補正は適用前に戻しました。'
export const S40_RETRY_BUTTON = 'もう一度走る'
export const S40_RETRY_FAILED_RUN = '走行できませんでした。'
export const S40_START_REJECTED = '再走行を開始できませんでした'

// ── IMU ──────────────────────────────────────────────────
export const S40_IMU_GUIDE = '機体を手で持ち上げずに、床の上で 8 の字に動かしてください（機体は自走しません）。4 つとも「完全」になるまで続けます。'
export const S40_IMU_PARTS = { sys: 'システム', gyro: 'ジャイロ', accel: '加速度', mag: '地磁気' }
export const S40_IMU_DONE = 'IMU の校正が完了しました。'
export const S40_IMU_COMMIT = '確定する'

// ── 履歴 ─────────────────────────────────────────────────
export const S40_HISTORY_EMPTY = '履歴はありません'
export const S40_HISTORY_ROLLBACK = 'この値に戻す'
export const S40_HISTORY_ROLLBACK_ARMED = '本当に戻す？'
export function s40Generation(n) { return `${n} 世代前` }
export const S40_ROLLBACK_DONE = '戻しました'
export const S40_ROLLBACK_FAILED = '戻せませんでした（一覧の状態で、適用できる値のときだけ戻せます）'

// ── 警告・理由 ────────────────────────────────────────────
export const S40_WARNINGS = {
  linear_not_calibrated: '直進校正がまだ実施されていません。先に直進校正を実施してください（車輪半径と車輪間距離は互いに影響するため、直進 → 旋回の順です）。',
  linear_older_than_rotation: '直進校正の日時が旋回校正より古いです。先に直進校正をやり直してください。',
}

const REASONS = {
  tolerance_undefined: '許容範囲が未確定のため、検証は合格にできません（実装管理担当へ。calib_*_tolerance_* の値が必要です）',
  error_exceeds_tolerance: '誤差が許容範囲を超えました',
  invalid_measurement: '実測値が不正です',
  a10_violation: '補正が大きすぎるため適用しませんでした（A10。機械の異常の疑い）',
  apply_failed: '補正値を機体へ適用できませんでした',
  preview_not_sane: 'プレビューが確認できていません',
  no_odom: '自己位置（オドメトリ）が届いていないため走れません',
  odom_stale: '自己位置（オドメトリ）が途絶えたため止めました',
  run_timeout: '走行が時間内に終わらなかったため止めました',
  offset_exceeds_tolerance: '選んだ範囲の外に写り込みが残っています（許容 5° を超えました）。選び直してください',
  mask_not_effective: '選んだ範囲の点が /scan_filtered から消えていません（lidar_filter に届いていない疑い）',
  verify_timeout: 'スキャンが届かず検証できませんでした',
  blind_limit_violation: '幅の上限を超えるため適用しませんでした',
}

// calib_runner の detail.reason（"error_exceeds_tolerance:0.2>0.05" のように数値が付く
// ものがある）→ 日本語。知らない理由はそのまま返す（握りつぶさない）。
export function s40ReasonLabel(reason) {
  if (!reason) return ''
  const head = String(reason).split(':')[0]
  return REASONS[head] ?? reason
}


// ── BLIND（LiDAR 死角）──────────────────────────────────────
export const S40_BLIND_HINT = 'スキャン上をなぞって、写り込んでいる角度帯を選びます（上が前方）。灰色は登録済み、青が選択中、赤い点は選ぶと消える点です。'
export const S40_BLIND_NO_SCAN = 'スキャンが届いていません'
export const S40_BLIND_RANGES_TITLE = '選んだ範囲'
export const S40_BLIND_NONE = '（なし。確定するとマスクを全部外します）'
export const S40_BLIND_DELETE = '削除'
export const S40_BLIND_CLEAR = '全部外す'
export const S40_BLIND_RESET = '登録済みに戻す'
export const S40_BLIND_BUTTON_SUBMIT = '確認（プレビュー）'
export const S40_BLIND_RESUBMIT = '選び直して確認'
export const S40_BLIND_NEXT = '次へ（適用して検証）'
export const S40_BLIND_VERIFYING = '死角マスクを適用して検証中…（スキャンから写り込みを推定しています）'
export const S40_BLIND_CURRENT = '登録済み'
export function s40BlindTotal(total, limits) {
  return `総幅 ${Number(total.toFixed(1))}° / 上限 ${limits.max_total_deg}°（1 区間 ${limits.max_sector_deg}°・${limits.max_sectors} 区間まで）`
}
export function s40BlindPreview(masked, total) {
  return `消える点: ${masked} 点 ／ 総幅 ${Number(Number(total).toFixed(1))}°`
}
export const S40_BLIND_PROBLEMS = {
  too_many_sectors: '区間が多すぎます。',
  sector_too_wide: '広すぎる区間があります。広げすぎると本物の障害物が見えなくなります。',
  total_too_wide: '総幅が広すぎます。広げすぎると本物の障害物が見えなくなります。',
  zero_width: '幅のない区間があります。',
  invalid_format: '選択の形が不正です。',
  no_scan: 'スキャンが届いていないため確認できません。',
}
