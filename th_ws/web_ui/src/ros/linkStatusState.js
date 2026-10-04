// ros/linkStatusState.js — /system/link_status (std_msgs/String に JSON) の純粋部。
// React を import しないので node --test で直接試験できる（autoCheckState.js と同じ流儀）。
//
// connectivity_checker が出す JSON の形 (names.md §6.2):
//   {"items": {"esp32_feedback": {"ok", "age_ms", "ignored", "excluded"},
//              "esp32_loopback": {...},
//              "lidar": {..., "points", "expected_points"},
//              "nodes": {"ok", "missing", "ignored", "excluded"}},
//    "timeout_ms": int, "estop_hw": {"seen", "pressed"}}
//
// `ok` は本当の受信で決まる（開発モードの link で外していても偽のまま）。
// 外していることは `ignored` で別に示す（Spec-webui.md §3.1 の機器別の行）。

export const LINK_ITEMS = ['esp32_feedback', 'esp32_loopback', 'lidar', 'nodes']

// 受信がこれだけ途絶えたら画面の値を信じない（/system/dev_mode の 3 秒と同じ考え方）。
export const LINK_STATUS_STALE_MS = 3000

function num(v) {
  return typeof v === 'number' && Number.isFinite(v) ? v : null
}

function cleanItem(src) {
  return {
    ok: src?.ok === true,
    ageMs: num(src?.age_ms),
    reportedAgeMs: num(src?.reported_age_ms),
    ignored: src?.ignored === true,
    excluded: src?.excluded === true,
    points: num(src?.points),
    expectedPoints: num(src?.expected_points),
    missing: Array.isArray(src?.missing) ? src.missing.filter((s) => typeof s === 'string') : [],
  }
}

export function parseLinkStatus(raw) {
  let obj = raw
  if (typeof raw === 'string') {
    try {
      obj = JSON.parse(raw)
    } catch {
      return null
    }
  }
  if (!obj || typeof obj !== 'object' || Array.isArray(obj)) return null
  if (!obj.items || typeof obj.items !== 'object') return null
  const items = {}
  for (const key of LINK_ITEMS) items[key] = cleanItem(obj.items[key])
  return {
    items,
    timeoutMs: num(obj.timeout_ms),
    estopHw: { seen: obj.estop_hw?.seen === true, pressed: obj.estop_hw?.pressed === true },
  }
}

// 画面の文言は i18n/screens.js（S00_LINK_TEXT）から渡す。既定値は試験用。
export const DEFAULT_LINK_TEXT = {
  checking: '確認中', ok: '✓ 疎通', ng: '× 未接続', excluded: '対象外',
  excludedDetail: 'シミュレーションでは判定しない',
  nodesOk: '必須ノードが揃っている', nodesMissing: '不足: ', unknown: '不明',
  never: '一度も受信していない', lastRx: '最後の受信 ', points: '点数 ',
  ignored: '（開発モードで無視中）', msAgo: ' ms 前', secAgo: ' 秒前',
  loopNoReport: 'ESP32 が折り返しを報告していない（ファームの書き込みが必要）',
  loopNotAlive: 'ESP32 に速度指令が届いていない',
}

function fmtAge(ms, t) {
  return ms < 1000 ? `${Math.round(ms)}${t.msAgo}` : `${(ms / 1000).toFixed(1)}${t.secAgo}`
}

// 1 行ぶんの表示。tone は既存の tone-ok / tone-warn / tone-ng のどれか。
// status が null（未受信・途絶）なら「確認中」（緑にしない）。
export function linkRowView(status, key, t = DEFAULT_LINK_TEXT) {
  const it = status?.items?.[key]
  if (!it) return { tone: 'tone-warn', label: t.checking, detail: '' }
  if (it.excluded) return { tone: '', label: t.excluded, detail: t.excludedDetail }
  let detail
  if (key === 'nodes') {
    detail = it.ok ? t.nodesOk : `${t.nodesMissing}${it.missing.join(', ') || t.unknown}`
  } else if (key === 'esp32_loopback' && !it.ok && status.items.esp32_feedback?.ok
             && it.reportedAgeMs == null) {
    // フィードバックは来るのに報告が一度も無い＝報告しない古いファーム（Spec-ops.md §2.2）
    detail = t.loopNoReport
  } else if (key === 'esp32_loopback' && !it.ok && it.reportedAgeMs != null) {
    detail = t.loopNotAlive
  } else if (it.ageMs == null) {
    detail = t.never
  } else {
    detail = `${t.lastRx}${fmtAge(it.ageMs, t)}`
    if (key === 'lidar' && it.points != null && it.expectedPoints != null
        && it.points !== it.expectedPoints) {
      detail += `・${t.points}${it.points}／${it.expectedPoints}`
    }
  }
  if (it.ignored) detail += t.ignored
  return it.ok
    ? { tone: 'tone-ok', label: t.ok, detail }
    : { tone: 'tone-ng', label: t.ng, detail }
}

// 繋がっていない項目のキー（S-01 の要約用）。sim で除外した項目は含めない。
// 未受信（null）は空を返す（「確認中」は呼び出し側が別に扱う）。
export function linkDownKeys(status) {
  if (!status) return []
  return LINK_ITEMS.filter((k) => status.items[k] && !status.items[k].ok && !status.items[k].excluded)
}
