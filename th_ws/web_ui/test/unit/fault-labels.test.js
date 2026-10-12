// SG-C1 (1b-12): safety_monitor が出しうるフォルト種別のすべてに日本語の文言がある。
// 種別は C++ の定義から機械的に取り出す（手で並べると、足したのに表に載せ忘れた
// 種別を検出できない）。取り出し元:
//   - safety_monitor.cpp の updateFaultState("X", ...) / checkTimeout("X", ...)
//   - safety_monitor_core.cpp の classify_severity の kCritical 集合
//   - FaultStatus.msg のコメントに列挙された種別
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { FAULT_LABELS, FAULT_UNKNOWN_LABEL, faultLabel } from '../../src/i18n/faults.js'

const SRC = (rel) => readFileSync(fileURLToPath(new URL(`../../../src/${rel}`, import.meta.url)), 'utf8')

function extractFaultTypes() {
  const found = new Set()
  const mon = SRC('th_safety/src/safety_monitor.cpp')
  for (const m of mon.matchAll(/(?:updateFaultState|checkTimeout)\(\s*"([A-Z0-9_]+)"/g)) found.add(m[1])
  const core = SRC('th_safety/src/safety_monitor_core.cpp')
  const crit = core.match(/kCritical\s*=\s*\{([^}]*)\}/)
  assert.ok(crit, 'kCritical が見つからない（抽出元が変わった）')
  for (const m of crit[1].matchAll(/"([A-Z0-9_]+)"/g)) found.add(m[1])
  const msg = SRC('th_system_msgs/msg/FaultStatus.msg')
  const line = msg.split('\n').filter((l) => l.includes('fault_type') || /^\s*#\s*"/.test(l)).join('\n')
  for (const m of line.matchAll(/"([A-Z0-9_]+)"/g)) found.add(m[1])
  found.delete('NONE')
  return [...found].sort()
}

test('抽出が空でない（抽出元の形が変わったら気づく）', () => {
  const types = extractFaultTypes()
  assert.ok(types.length >= 10, `抽出できた種別が少なすぎる: ${types.join(',')}`)
  for (const t of ['LIDAR_LOST', 'DRIVE_RUNAWAY', 'LOCALIZATION_LOST', 'UI_DISCONNECTED']) {
    assert.ok(types.includes(t), `${t} が抽出されていない`)
  }
})

test('safety_monitor が出しうるフォルト種別のすべてに日本語があり「種別不明」にならない', () => {
  for (const t of extractFaultTypes()) {
    const label = faultLabel(t)
    assert.ok(label && label !== FAULT_UNKNOWN_LABEL, `${t} に文言が無い`)
    assert.ok(/[぀-ヿ一-鿿]/.test(label), `${t} の文言が日本語でない: ${label}`)
    // 「原因と対処を 1 行で添える」(Spec-webui.md §8): 句点で原因と対処が分かれている
    assert.ok(label.includes('。'), `${t} に対処が添えられていない: ${label}`)
  }
})

test('表に載っている種別は実在する（古い種別が残らない）', () => {
  const real = new Set(extractFaultTypes())
  for (const t of Object.keys(FAULT_LABELS)) {
    if (t === 'NONE') continue
    assert.ok(real.has(t), `${t} は safety_monitor が出さない種別`)
  }
})

test('古い文言が残っていない（ESP32 は無線ではない／人物追跡はカメラではない）', () => {
  const all = Object.values(FAULT_LABELS).join('\n')
  assert.ok(!all.includes('Wi-Fi'))
  assert.ok(!all.includes('カメラ'))
})

test('未知の種別だけが「種別不明」になる', () => {
  assert.equal(faultLabel('NO_SUCH_FAULT'), FAULT_UNKNOWN_LABEL)
  assert.equal(faultLabel(''), null)
})

// ── フォルト履歴の短い名前（Spec-webui.md §1・§3.15・§4.1。2026-10-12）──
import { FAULT_SHORT_LABELS, FAULT_OUTCOME_LABELS, FAULT_SEVERITY_LABELS, faultShortLabel } from '../../src/i18n/faults.js'

test('safety_monitor が出しうるフォルト種別のすべてに短い名前がある', () => {
  for (const t of extractFaultTypes()) {
    const label = FAULT_SHORT_LABELS[t]
    assert.ok(label, `${t} に短い名前が無い`)
    assert.ok(/[぀-ヿ一-鿿]/.test(label), `${t} の短い名前が日本語でない: ${label}`)
    assert.ok(label.length <= 12, `${t} の短い名前が長すぎる（ヘッダに載せる）: ${label}`)
    assert.equal(faultShortLabel(t), label)
  }
})

test('短い名前の表に実在しない種別が残っていない', () => {
  const real = new Set(extractFaultTypes())
  for (const t of Object.keys(FAULT_SHORT_LABELS)) assert.ok(real.has(t), `${t} は出ない種別`)
})

test('FaultRecord.msg の outcome／severity の値すべてに表記がある', () => {
  const msg = SRC('th_system_msgs/msg/FaultRecord.msg')
  for (const field of ['severity', 'outcome']) {
    const line = msg.split('\n').find((l) => l.includes(`string ${field}`))
    const values = [...line.matchAll(/"([A-Z_]+)"/g)].map((m) => m[1])
    assert.ok(values.length >= 2, `${field} の値が抽出できない`)
    const table = field === 'severity' ? FAULT_SEVERITY_LABELS : FAULT_OUTCOME_LABELS
    for (const v of values) assert.ok(table[v], `${field}=${v} の表記が無い`)
  }
  assert.equal(FAULT_OUTCOME_LABELS.PAUSE, '一時停止')
  assert.equal(FAULT_OUTCOME_LABELS.ESTOP, '非常停止')
  assert.equal(FAULT_OUTCOME_LABELS.NONE, '停止なし')
  assert.equal(FAULT_SEVERITY_LABELS.RECOVERABLE, '回復')
  assert.equal(FAULT_SEVERITY_LABELS.CRITICAL, '重大')
})
