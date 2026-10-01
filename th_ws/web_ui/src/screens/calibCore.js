// screens/calibCore.js — S-40 校正画面の純ロジック（React 非依存。ユニットテスト対象。WP-MAINT-03）。
//
// 画面は th_state（FSM）の state（LIST/S1〜S4）と calib_runner の /calib/status
// （result と detail）から「いま何を出すか」を導く。ここはその導出だけを持つ:
//   - どのステップか（Step n/4）と、右ペインに出す内容の種類（viewKind）
//   - 「次へ」を押せるか（S3 は /calib/status が PREVIEW_OK のときだけ。FSM の
//     ガード preview_sane も同じ条件で拒否するが、画面でも押せなくする）
//   - 旋回の警告（先に直進校正を）、IMU の進捗バー、履歴の行
// 状態の正本は機体側（FSM と calib_runner）。画面は表示しているだけ。

// 校正の項目。BLIND（LiDAR 死角）は走らず、ライブスキャン上で角度帯をなぞって選ぶ（§3.3）。
export const CALIB_ITEM_ORDER = ['LINEAR', 'ROTATION', 'IMU', 'BLIND']
export const CALIB_STARTABLE = ['LINEAR', 'ROTATION', 'IMU', 'BLIND']

// 4 ステップのウィザード（直進・旋回・死角）。IMU は進捗バーのみ（Spec-checks.md §3.3）。
export const WIZARD_ITEMS = ['LINEAR', 'ROTATION', 'BLIND']

export function isWizardItem(item) {
  return WIZARD_ITEMS.includes(item)
}

// /calib/status の detail（JSON 文字列。テストではオブジェクトも可）→ オブジェクト。
export function parseCalibDetail(detail) {
  if (detail && typeof detail === 'object') return detail
  if (typeof detail !== 'string' || !detail) return {}
  try {
    const parsed = JSON.parse(detail)
    return parsed && typeof parsed === 'object' ? parsed : {}
  } catch {
    return {}
  }
}

// FSM の state 名 → ステップ番号（S1〜S4 → 1〜4。LIST・その他は 0）。
export function stepNumber(fsmState) {
  const m = /^S([1-4])$/.exec(fsmState ?? '')
  return m ? Number(m[1]) : 0
}

// 実行中の項目。calib_runner が status.item に載せる値を正とし、押した直後
// （trigger が受理されて最初の status が来るまで）だけ画面で押した項目で補う。
export function activeItem(status, selectedItem, fsmState) {
  if (!fsmState || fsmState === 'LIST') return null
  return status?.item || selectedItem || null
}

// 右ペインに出す内容の種類。
//   list         一覧（LIST）
//   guide        Step 1 案内
//   running      Step 2 走行中（IMU は imu_progress）
//   retry        Step 2 に戻った（検証 NG・走行失敗）。補正は適用前に戻してある
//   measure      Step 3 実測値の入力（まだ確認していない・sane でなかった）
//   preview_ok   Step 3 プレビューが sane。次へ進める
//   blind_select  BLIND の Step 2（ライブスキャンで角度帯を選ぶ）。検証 NG で戻ったときも
//   blind_preview BLIND の Step 3 でプレビューが sane。選び直しもできる
//   verifying    Step 4 適用・検証走行中
//   verify_input Step 4 検証走行が終わった。実測値の入力
//   imu_progress IMU の進捗バー（S2）
//   imu_done     IMU が全て完全になった（S3）。確定を待つ
export function viewKind({ fsmState, item, status }) {
  const step = stepNumber(fsmState)
  if (step === 0) return 'list'
  const result = status && (!status.item || status.item === item) ? status.result : ''
  if (item === 'IMU') {
    if (step === 1) return 'guide'
    if (step === 2) return 'imu_progress'
    return 'imu_done'
  }
  if (item === 'BLIND') {
    if (step === 1) return 'guide'
    if (step === 2) return 'blind_select'
    if (step === 3) return result === 'PREVIEW_OK' ? 'blind_preview' : 'blind_select'
    return 'verifying'
  }
  if (step === 1) return 'guide'
  if (step === 2) return result === 'RETRY_WAIT' || result === 'NG' ? 'retry' : 'running'
  if (step === 3) return result === 'PREVIEW_OK' ? 'preview_ok' : 'measure'
  return result === 'WAIT_VERIFY' ? 'verify_input' : 'verifying'
}

// 「次へ」を押せるか。S1 は常に（走り出す）、S3 は PREVIEW_OK のときだけ。
// IMU の S3 は完了（PREVIEW_OK）を待って「確定」を押せる。S2・S4 は自動で進むので押せない。
export function canProceed({ fsmState, item, status }) {
  const step = stepNumber(fsmState)
  if (step === 1) return true
  if (step === 3) {
    return !!status && status.item === item && status.result === 'PREVIEW_OK'
  }
  return false
}

// 測定値として送ってよい入力か（正の有限数）。
export function parseMeasured(text) {
  if (typeof text !== 'string' || text.trim() === '') return null
  const n = Number(text)
  return Number.isFinite(n) && n > 0 ? n : null
}

// 旋回の警告キー（calib_runner が detail.warnings に載せる）。表示用の文言は i18n。
export function rotationWarnings(detail) {
  return Array.isArray(detail?.warnings) ? detail.warnings : []
}

// IMU の進捗（sys/gyro/accel/mag 各 0〜3）。全部 3 で 100%。
export const IMU_PARTS = ['sys', 'gyro', 'accel', 'mag']

export function imuProgress(detail) {
  const imu = detail?.imu ?? {}
  const parts = IMU_PARTS.map((key) => {
    const raw = Number(imu[key])
    const value = Number.isFinite(raw) ? Math.min(3, Math.max(0, Math.round(raw))) : 0
    return { key, value }
  })
  const percent = Math.round((parts.reduce((s, p) => s + p.value, 0) / 12) * 100)
  return { parts, percent, complete: parts.every((p) => p.value === 3) }
}

// 走行の進捗（0〜100 の整数）。
export function runPercent(detail) {
  const p = Number(detail?.progress)
  if (!Number.isFinite(p)) return 0
  return Math.round(Math.min(1, Math.max(0, p)) * 100)
}

// 補正値の表示用文字列。
export function formatValues(values) {
  if (!values || typeof values !== 'object') return ''
  return Object.entries(values)
    .map(([k, v]) => {
      if (k === 'blind_angle_ranges') return blindRangesText(v)
      return `${k}=${typeof v === 'number' ? Number(v.toFixed(6)) : v}`
    })
    .join(', ')
}

// 履歴の行（新しい順。generation 1 が 1 つ前）。
export function historyRows(detail, item) {
  const rows = detail?.history?.[item]
  if (!Array.isArray(rows)) return []
  return rows.map((r) => ({
    generation: Number(r.generation),
    text: formatValues(r.values),
    calibratedAt: r.calibrated_at ?? '',
  }))
}

// 項目の最終校正日（無ければ空）。
export function lastCalibrated(detail, item) {
  return detail?.last_calibrated?.[item] ?? ''
}

// 検証 NG・走行失敗の理由（detail.reason）→ 表示用キー。"error_exceeds_tolerance:0.2>0.05" の
// ように数値が付くものは接頭辞でまとめる。
export function reasonKey(reason) {
  if (!reason || typeof reason !== 'string') return ''
  const head = reason.split(':')[0]
  return head
}

// /calib/status の preview_before / preview_after（JSON 文字列）→ 表示用の「k=v, …」。
// 壊れていたら空文字（画面を落とさない）。
export function previewText(json) {
  if (!json || typeof json !== 'string') return ''
  try {
    const parsed = JSON.parse(json)
    return formatValues(parsed)
  } catch {
    return ''
  }
}


// ── BLIND（LiDAR 死角マスク）の純ロジック ─────────────────────
// 角度は度・laser_link 基準・反時計回り正（前方 0°・左 +90°）。区間 [a0, a1] は a0 から
// 反時計回りに a1 まで（a1 < a0 なら ±180° をまたぐ）。機体側（calib_runner の blind_core.py・
// obstacle_limiter）と同じ規約。**検査の正本は機体側**で、ここは「押す前に分かる」ための写し。

export const BLIND_LIMITS_DEFAULT = { max_sector_deg: 30, max_total_deg: 90, max_sectors: 8 }

export function normalizeDeg(a) {
  let r = a % 360
  if (r > 180) r -= 360
  else if (r <= -180) r += 360
  return r
}

export function sectorWidthDeg(a0, a1) {
  const w = (a1 - a0) % 360
  return w < 0 ? w + 360 : w
}

// 平坦配列 [a0,a1,a2,a3,...] ⇄ [[a0,a1],...]
export function rangesFromFlat(flat) {
  if (!Array.isArray(flat)) return []
  const out = []
  for (let i = 0; i + 1 < flat.length; i += 2) out.push([Number(flat[i]), Number(flat[i + 1])])
  return out
}

export function blindRangesText(flat) {
  const r = rangesFromFlat(flat)
  if (r.length === 0) return '（なし）'
  return r.map(([a, b]) => `${Number(a.toFixed(1))}〜${Number(b.toFixed(1))}°`).join(' / ')
}

// 画面の点 (x, y)（右が +x・下が +y）→ 機体基準の角度（前方 = 画面の上 = 0°、左 = 画面の左 = +90°）。
export function angleFromPoint(x, y, cx, cy) {
  const forward = cy - y
  const left = cx - x
  return normalizeDeg((Math.atan2(left, forward) * 180) / Math.PI)
}

// ドラッグの始点・終点の角度 → 区間。近い側の弧を取る（時計回りになぞっても同じ区間になる）。
export function rangeFromDrag(startDeg, endDeg) {
  const d = normalizeDeg(endDeg - startDeg)
  const a0 = d >= 0 ? startDeg : endDeg
  const a1 = d >= 0 ? endDeg : startDeg
  return [Number(normalizeDeg(a0).toFixed(1)), Number(normalizeDeg(a1).toFixed(1))]
}

export function inRange(angleDeg, [a0, a1]) {
  return sectorWidthDeg(a0, angleDeg) <= sectorWidthDeg(a0, a1) + 1e-9
}

export function blindTotalDeg(ranges) {
  return ranges.reduce((s, [a, b]) => s + sectorWidthDeg(a, b), 0)
}

// 上限の違反（無ければ null）。キーは i18n の S40_BLIND_PROBLEMS。
export function blindProblem(ranges, limits = BLIND_LIMITS_DEFAULT) {
  if (ranges.length > limits.max_sectors) return 'too_many_sectors'
  for (const [a, b] of ranges) {
    const w = sectorWidthDeg(a, b)
    if (!Number.isFinite(w) || w <= 1e-9) return 'zero_width'
    if (w > limits.max_sector_deg + 1e-9) return 'sector_too_wide'
  }
  if (blindTotalDeg(ranges) > limits.max_total_deg + 1e-9) return 'total_too_wide'
  return null
}

// 機体が出す reason（"sector_too_wide:35.0>30.0" 等）→ キー。
export function blindReasonKey(reason) {
  return typeof reason === 'string' ? reason.split(':')[0] : ''
}

// /calib/status.detail.blind.limits が来ていればそれ、無ければ既定。
export function blindLimits(detail) {
  const l = detail?.blind?.limits
  if (!l) return BLIND_LIMITS_DEFAULT
  return {
    max_sector_deg: Number(l.max_sector_deg) || BLIND_LIMITS_DEFAULT.max_sector_deg,
    max_total_deg: Number(l.max_total_deg) || BLIND_LIMITS_DEFAULT.max_total_deg,
    max_sectors: Number(l.max_sectors) || BLIND_LIMITS_DEFAULT.max_sectors,
  }
}

// スキャンの点を、選択の内側（マスクで消える）と外側に分ける（プレビューの描画用）。
// 点は {x, y, angleDeg}。ranges が空なら全部が外側。
export function scanPointsByMask(scan, ranges) {
  if (!scan || !Array.isArray(scan.ranges)) return { kept: [], masked: [] }
  const { angle_min = 0, angle_increment = 0, range_min = 0, range_max = Infinity } = scan
  const kept = []
  const masked = []
  for (let i = 0; i < scan.ranges.length; i += 1) {
    const r = scan.ranges[i]
    if (typeof r !== 'number' || !Number.isFinite(r) || r < range_min || r > range_max) continue
    const ang = angle_min + i * angle_increment
    const deg = normalizeDeg((ang * 180) / Math.PI)
    const pt = { x: r * Math.cos(ang), y: r * Math.sin(ang), angleDeg: deg }
    if (ranges.some((rg) => inRange(deg, rg))) masked.push(pt)
    else kept.push(pt)
  }
  return { kept, masked }
}
