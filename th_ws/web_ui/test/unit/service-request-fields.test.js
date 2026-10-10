// fix-setflag-requester: rosbridge は .srv に無いフィールドを拒否する。
// 2026-10-10 実機で useSetFlag が `requester` を付けて送り、S-20/S-21 の人検出・
// S-11/S-13 の自動ブレーキ・S-14 の地図更新が全部黙って失敗した。
// `window.__thSetFlagCalls` の記録だけ見る試験は余計なフィールドを検出できない
// ので、実際に送るペイロードのキー集合を .srv 定義と突き合わせる。
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync, readdirSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import path from 'node:path'
import { buildSetFlagRequest } from '../../src/ros/setFlagPayload.js'

const here = path.dirname(fileURLToPath(import.meta.url))
const rosDir = path.join(here, '../../src/ros')
const screensDir = path.join(here, '../../src/screens')
const thSrvDir = path.join(here, '../../../src/th_system_msgs/srv')
const extSrvDir = path.join(here, '../../../src/multiple_sensor_person_tracking/srv')
const read = (dir, f) => readFileSync(path.join(dir, f), 'utf8')

// .srv の `---` より前（request 側）のフィールド名一覧。コメント・空行を除く。
// `string[] names`・`rcl_interfaces/Parameter[] parameters` は名前が 2 語目。
function srvRequestFields(text) {
  const out = []
  for (const line of text.split('\n')) {
    if (line.trim() === '---') break
    const body = line.split('#')[0].trim()
    if (!body) continue
    const parts = body.split(/\s+/)
    if (parts.length < 2) throw new Error(`cannot parse srv line: ${line}`)
    out.push(parts[1])
  }
  return out
}

const thSrv = (name) => srvRequestFields(read(thSrvDir, `${name}.srv`))
// ROS 2 標準・外部パッケージの request フィールド（WebUI が送る分だけ）。
const KNOWN = {
  'std_srvs/Trigger': [],
  'rcl_interfaces/GetParameters': ['names'],
  'rcl_interfaces/SetParameters': ['parameters'],
  'multiple_sensor_person_tracking/SelectTarget':
    srvRequestFields(read(extSrvDir, 'SelectTarget.srv')),
}
function srvFields(type) {
  const fields = type.startsWith('th_system_msgs/')
    ? thSrv(type.split('/')[1])
    : KNOWN[type]
  assert.ok(fields, `unknown service type ${type} (add to KNOWN or th_system_msgs)`)
  return fields
}

// `{ a, b: f(x, {c: 1}) }` → ['a', 'b']。文字列を除去してから深さ 0 の区切りで
// 割り、`key:`・shorthand の `key` を拾う（ネストの中は無視）。
function topKeys(literal) {
  const s = literal
    .replace(/'(?:[^'\\]|\\.)*'/g, "''")
    .replace(/"(?:[^"\\]|\\.)*"/g, '""')
    .replace(/`(?:[^`\\]|\\.)*`/g, '``')
  const keys = []
  let depth = 0
  let seg = ''
  const flush = () => {
    const m = seg.match(/^\s*([A-Za-z_$][\w$]*)\s*(?::|$)/)
    if (m) keys.push(m[1])
    seg = ''
  }
  for (const c of s) {
    if (c === '{' || c === '[' || c === '(') depth++
    if (c === '}' || c === ']' || c === ')') depth--
    if (c === ',' && depth === 0) flush()
    else seg += c
  }
  flush()
  return keys.filter((k) => k !== '')
}

// `new ROSLIB.ServiceRequest(` の引数が `{...}` なら中身、変数なら null。
function serviceRequests(src) {
  const out = []
  const re = /new ROSLIB\.ServiceRequest\(/g
  let m
  while ((m = re.exec(src))) {
    let i = m.index + m[0].length
    while (src[i] === ' ' || src[i] === '\n') i++
    if (src[i] !== '{') { out.push({ index: m.index, literal: null }); continue }
    let depth = 0
    let instr = null
    let j = i
    for (; j < src.length; j++) {
      const c = src[j]
      if (instr) { if (c === instr) instr = null; continue }
      if (c === "'" || c === '"' || c === '`') instr = c
      else if (c === '{') depth++
      else if (c === '}') { depth--; if (depth === 0) break }
    }
    out.push({ index: m.index, literal: src.slice(i + 1, j) })
  }
  return out
}

// topics.js の `NAME: 'pkg/Type'` を拾う（SRV_TYPES だけでなく定数全般）。
function topicsMap() {
  const src = read(rosDir, 'topics.js')
  return Object.fromEntries(
    [...src.matchAll(/(\w+):\s*'((?:th_system_msgs|std_srvs|rcl_interfaces|multiple_sensor_person_tracking)\/\w+)'/g)]
      .map((m) => [m[1], m[2]]),
  )
}

// ServiceRequest より前にある直近の serviceType（文字列・SRV_TYPES.*・同ファイルの
// 文字列定数）を型名に解決する。
function typeBefore(src, topics, index) {
  const head = src.slice(0, index)
  const found = []
  for (const m of head.matchAll(/serviceType:\s*'([^']+)'/g)) found.push([m.index, m[1]])
  for (const m of head.matchAll(/serviceType:\s*SRV_TYPES\.(\w+)/g)) {
    found.push([m.index, topics[m[1]]])
  }
  for (const m of head.matchAll(/serviceType:\s*([A-Z_][\w]*)/g)) {
    if (m[1] === 'SRV_TYPES') continue
    const c = src.match(new RegExp(`(?:const|let|var)\\s+${m[1]}\\s*=\\s*'([^']+)'`))
    if (c) found.push([m.index, c[1]])
  }
  found.sort((a, b) => a[0] - b[0])
  return found.length ? found[found.length - 1][1] : undefined
}

// ── T1: 今回の不具合（SetFlag に requester） ──────────────────────────────

test('SetFlag payload keys exactly match SetFlag.srv request fields', () => {
  const srv = thSrv('SetFlag').sort()
  assert.deepEqual(srv, ['flag', 'value'])
  for (const [flag, value] of [
    ['tracker_enabled', true], ['tracker_enabled', false],
    ['auto_brake', true], ['map_update', false],
  ]) {
    assert.deepEqual(Object.keys(buildSetFlagRequest(flag, value)).sort(), srv)
  }
})

test('useSetFlag production path sends buildSetFlagRequest (requester only in TEST_MODE record)', () => {
  const src = read(rosDir, 'useSetFlag.js')
  assert.match(src, /new ROSLIB\.ServiceRequest\(buildSetFlagRequest\(flag, value\)\)/)
  const prod = src.slice(src.indexOf('return new Promise'))
  assert.ok(
    !/ServiceRequest\(\{[^}]*requester/.test(prod),
    'requester must not be in the production ServiceRequest literal',
  )
})

// ── T2: ros/ フック内の静的 ServiceRequest リテラル全般 ────────────────────

test('every static ServiceRequest literal fits its .srv request fields', () => {
  const topics = topicsMap()
  const files = [
    'useTrigger.js', 'useStdTrigger.js', 'useRosbridge.js', 'useTunableParams.js',
    'devModeState.js', 'useOpcheckAnswer.js', 'useParamsOverrides.js', 'useSetFlag.js',
  ]
  let checked = 0
  for (const f of files) {
    const src = read(rosDir, f)
    for (const { index, literal } of serviceRequests(src)) {
      if (literal === null) continue // 変数 passthrough（T3 で縛る）
      const type = typeBefore(src, topics, index)
      assert.ok(type, `${f}: cannot resolve serviceType for a ServiceRequest`)
      const extra = topKeys(literal).filter((k) => !srvFields(type).includes(k))
      assert.deepEqual(extra, [], `${f} (${type}): payload keys not in srv: ${extra}`)
      checked++
    }
  }
  // useTrigger 1 + useStdTrigger 1 + useRosbridge 11 + useTunableParams 3 +
  // devModeState 1 + useOpcheckAnswer 1 + useParamsOverrides 2 = 19
  // （useSetFlag の本番は buildSetFlagRequest 呼び出しなのでここでは数えない）
  assert.ok(checked >= 19, `expected >=19 static payloads, got ${checked}`)
})

// ── T3: request 変数をそのまま送るラッパー（呼び出し側のリテラルを縛る） ──

test('onsite/calib wrapper request literals fit their .srv request fields', () => {
  const topics = topicsMap()
  const onsite = read(rosDir, 'useOnsiteService.js')
  const calib = read(rosDir, 'useCalibService.js')
  const cases = []
  // フック内で組み立てる既定リテラル: callService(service, SRV_TYPES.x, {...})
  for (const m of onsite.matchAll(
    /callService\(\s*[^,]+,\s*(SRV_TYPES\.\w+),\s*\{/g,
  )) {
    let i = m.index + m[0].length - 1
    let depth = 0
    let instr = null
    let j = i
    for (; j < onsite.length; j++) {
      const c = onsite[j]
      if (instr) { if (c === instr) instr = null; continue }
      if (c === "'" || c === '"' || c === '`') instr = c
      else if (c === '{') depth++
      else if (c === '}') { depth--; if (depth === 0) break }
    }
    cases.push({ where: 'useOnsiteService.js', type: topics[m[1].split('.')[1]], literal: onsite.slice(i + 1, j) })
  }
  // calib の 3 ラッパー: call('name', SERVICES.., SRV_TYPES.., {...})
  for (const m of calib.matchAll(
    /call\(\s*'\w+',\s*SERVICES\.\w+,\s*(SRV_TYPES\.\w+),\s*\{([^}]*)\}/g,
  )) {
    cases.push({ where: 'useCalibService.js', type: topics[m[1].split('.')[1]], literal: m[2] })
  }
  // 呼び出し側が渡すリテラル: twoPoint/editPin/declareHome/selectPin({...})
  const wrapperType = {
    twoPoint: 'th_system_msgs/TwoPointPress',
    editPin: 'th_system_msgs/EditPin',
    declareHome: 'th_system_msgs/DeclareHome',
    selectPin: 'th_system_msgs/GoToPanel',
  }
  for (const f of readdirSync(screensDir).filter((f) => f.endsWith('.jsx'))) {
    const src = read(screensDir, f)
    for (const m of src.matchAll(/\b(twoPoint|editPin|declareHome|selectPin)\(\s*\{/g)) {
      let i = m.index + m[0].length - 1
      let depth = 0
      let j = i
      for (; j < src.length; j++) {
        if (src[j] === '{') depth++
        else if (src[j] === '}') { depth--; if (depth === 0) break }
      }
      cases.push({ where: f, type: wrapperType[m[1]], literal: src.slice(i + 1, j) })
    }
  }
  assert.ok(cases.length >= 10, `expected >=10 wrapper payloads, got ${cases.length}`)
  for (const { where, type, literal } of cases) {
    const extra = topKeys(literal).filter((k) => !srvFields(type).includes(k))
    assert.deepEqual(extra, [], `${where} (${type}): payload keys not in srv: ${extra}`)
  }
})

test('no screen builds ServiceRequest directly (all go through ros/ hooks)', () => {
  for (const f of readdirSync(screensDir).filter((f) => f.endsWith('.jsx'))) {
    assert.ok(
      !read(screensDir, f).includes('ServiceRequest'),
      `${f} must not build ServiceRequest directly`,
    )
  }
  const parts = path.join(here, '../../src/parts')
  for (const f of readdirSync(parts).filter((f) => f.endsWith('.jsx'))) {
    assert.ok(
      !read(parts, f).includes('ServiceRequest'),
      `parts/${f} must not build ServiceRequest directly`,
    )
  }
})
