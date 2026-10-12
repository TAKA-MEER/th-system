// ros/faultHistoryState.js — /system/fault_history (th_system_msgs/FaultHistory)
// の読み取りと表示用の純関数（Spec-safety.md §3.5.5、Spec-webui.md §1・§3.15・§4.1）。
// ROS も React も import しない（node --test で直接試験する）。
//
// 記録は新しい順・最大 50 件。継続中の継続時間は publish されない
// （duration_s = -1.0）ので、画面で started_at と現在時刻から求める。
import {
  faultShortLabel, FAULT_OUTCOME_LABELS, FAULT_SEVERITY_LABELS,
} from '../i18n/faults.js'

export const FAULT_HISTORY_MAX = 50
// 「ほか N 回」を数える窓（直前 60 秒以内の同種別）。
export const FAULT_REPEAT_WINDOW_S = 60

// rosbridge の購読設定。機体の発行側は RELIABLE＋TRANSIENT_LOCAL・depth 1 で、
// 最新の 1 通（全履歴）しか来ない。後から開いた画面にもその 1 通が届くことが
// 肝（rosbridge は発行側の QoS に合わせて購読する）。queue_length を 1 より
// 大きくしたり throttle を付けたりして最新の 1 通を取りこぼさない。
export function subscribeFaultHistory(ROSLIB, ros, topicName, messageType, onMessage) {
  const topic = new ROSLIB.Topic({
    ros, name: topicName, messageType, queue_length: 1,
  })
  topic.subscribe(onMessage)
  return topic
}

// メッセージ -> 記録の配列（新しい順・最大 50）。読めなければ null。
export function parseFaultHistory(msg) {
  if (!msg || !Array.isArray(msg.records)) return null
  const out = []
  for (const r of msg.records) {
    if (!r || typeof r.fault_type !== 'string') continue
    out.push({
      fault_type: r.fault_type,
      started_at: Number(r.started_at) || 0,
      duration_s: Number.isFinite(Number(r.duration_s)) ? Number(r.duration_s) : -1,
      ongoing: !!r.ongoing,
      mode: r.mode ?? '',
      state: r.state ?? '',
      severity: r.severity ?? '',
      outcome: r.outcome ?? '',
    })
  }
  // 並びは機体が新しい順で出すが、画面側でも保証する（安定ソート）。
  out.sort((a, b) => b.started_at - a.started_at)
  return out.slice(0, FAULT_HISTORY_MAX)
}

const pad2 = (n) => String(n).padStart(2, '0')

// epoch 秒 -> 「時:分:秒」（端末のローカル時刻）。
export function formatClock(epochSec) {
  if (!Number.isFinite(epochSec) || epochSec <= 0) return '--:--:--'
  const d = new Date(epochSec * 1000)
  return `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`
}

// 継続時間（秒）-> 「0.1 秒」「12 秒」「3 分 5 秒」。
// 10 秒未満は 0.1 秒単位。0 より大きい値を「0 秒」にしない（最小 0.1 秒）。
export function formatDuration(sec) {
  if (!Number.isFinite(sec) || sec < 0) return ''
  if (sec < 10) {
    const tenths = Math.max(1, Math.round(sec * 10))
    if (tenths >= 100) return '10 秒'
    return `${Math.floor(tenths / 10)}.${tenths % 10} 秒`
  }
  const total = Math.round(sec)
  if (total < 60) return `${total} 秒`
  const m = Math.floor(total / 60)
  const s = total % 60
  if (m >= 60) {
    const h = Math.floor(m / 60)
    return `${h} 時間 ${m % 60} 分`
  }
  return s === 0 ? `${m} 分` : `${m} 分 ${s} 秒`
}

// 継続中なら経過（現在時刻 - started_at）、解消済みなら記録された値。
export function recordDurationSec(rec, nowEpochSec) {
  if (rec.ongoing) return Math.max(0, nowEpochSec - rec.started_at)
  return rec.duration_s
}

export function outcomeLabel(outcome) {
  return FAULT_OUTCOME_LABELS[outcome] ?? '—'
}
export function severityLabel(severity) {
  return FAULT_SEVERITY_LABELS[severity] ?? '—'
}

// 最新の 1 件（ヘッダの「直近の異常」）。無ければ null。
export function latestRecord(records) {
  return records && records.length > 0 ? records[0] : null
}

// 直近に解消した記録（最新の ongoing=false の 1 件）。解消時刻
// （started_at + duration_s）が最も新しいもの。無ければ null。
export function latestResolved(records) {
  if (!records) return null
  let best = null
  let bestEnd = -Infinity
  for (const r of records) {
    if (r.ongoing) continue
    const end = r.started_at + Math.max(0, r.duration_s)
    if (end > bestEnd) { best = r; bestEnd = end }
  }
  return best
}

// rec の直前 FAULT_REPEAT_WINDOW_S 秒以内に始まった同種別の他の記録の件数。
export function repeatCount(records, rec) {
  if (!records || !rec) return 0
  return records.filter((r) => r !== rec && r.fault_type === rec.fault_type
    && r.started_at <= rec.started_at
    && rec.started_at - r.started_at <= FAULT_REPEAT_WINDOW_S).length
}

// 「解消した異常：LiDAR 途絶（0.1 秒・12:34:56）」＋「ほか N 回」。
export function resolvedSummary(records, rec = latestResolved(records)) {
  if (!rec) return null
  const others = repeatCount(records, rec)
  return {
    label: faultShortLabel(rec.fault_type),
    duration: formatDuration(rec.duration_s),
    clock: formatClock(rec.started_at),
    others,
    faultType: rec.fault_type,
  }
}

// ヘッダ：「直近の異常：LiDAR 途絶 12:34:56」。
export function recentFaultText(rec) {
  if (!rec) return null
  return `直近の異常：${faultShortLabel(rec.fault_type)} ${formatClock(rec.started_at)}`
}
