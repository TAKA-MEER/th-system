// S-00 の機器別の行（/system/link_status）の純粋部 ros/linkStatusState.js の試験。
import { test } from 'node:test'
import assert from 'node:assert/strict'
import { parseLinkStatus, linkRowView, LINK_ITEMS } from '../../src/ros/linkStatusState.js'

const SAMPLE = {
  items: {
    esp32_feedback: { ok: true, age_ms: 120, ignored: false, excluded: false },
    esp32_loopback: { ok: false, age_ms: null, ignored: false, excluded: false },
    lidar: { ok: false, age_ms: 250, points: 359, expected_points: 720, ignored: false, excluded: false },
    nodes: { ok: false, missing: ['state_manager', 'obstacle_limiter'], ignored: false, excluded: false },
  },
  timeout_ms: 3000,
  estop_hw: { seen: true, pressed: false },
}

test('parse: JSON 文字列から 4 項目を取り出す', () => {
  const s = parseLinkStatus(JSON.stringify(SAMPLE))
  assert.deepEqual(Object.keys(s.items), LINK_ITEMS)
  assert.equal(s.items.esp32_feedback.ok, true)
  assert.equal(s.items.lidar.points, 359)
  assert.deepEqual(s.items.nodes.missing, ['state_manager', 'obstacle_limiter'])
})

test('parse: 壊れた JSON・items 無しは null（緑にしない）', () => {
  assert.equal(parseLinkStatus('{'), null)
  assert.equal(parseLinkStatus('{}'), null)
  assert.equal(parseLinkStatus(null), null)
})

test('parse: ok が true 以外は偽（文字列 "true" 等で緑にしない）', () => {
  const s = parseLinkStatus({ items: { esp32_feedback: { ok: 'true' } } })
  assert.equal(s.items.esp32_feedback.ok, false)
  assert.equal(s.items.lidar.ok, false)   // 欠けている項目も偽
})

test('行: 項目ごとに独立した状態と詳細を出す', () => {
  const s = parseLinkStatus(SAMPLE)
  const fb = linkRowView(s, 'esp32_feedback')
  assert.equal(fb.tone, 'tone-ok')
  assert.match(fb.detail, /120 ms 前/)
  const lb = linkRowView(s, 'esp32_loopback')
  assert.equal(lb.tone, 'tone-ng')
  assert.match(lb.detail, /一度も受信していない/)
  const li = linkRowView(s, 'lidar')
  assert.equal(li.tone, 'tone-ng')
  assert.match(li.detail, /359／720/)
  const no = linkRowView(s, 'nodes')
  assert.match(no.detail, /state_manager, obstacle_limiter/)
})

test('行: 未受信（null）は確認中で、緑にしない', () => {
  for (const key of LINK_ITEMS) {
    const v = linkRowView(null, key)
    assert.equal(v.tone, 'tone-warn')
  }
})

test('行: 開発モードで外していても本当の状態を出し、無視中と添える', () => {
  const s = parseLinkStatus({
    items: { lidar: { ok: false, age_ms: null, ignored: true, excluded: false } },
  })
  const v = linkRowView(s, 'lidar')
  assert.equal(v.tone, 'tone-ng')
  assert.match(v.detail, /開発モードで無視中/)
})

test('行: sim で除外した項目は対象外', () => {
  const s = parseLinkStatus({ items: { esp32_feedback: { ok: false, excluded: true } } })
  assert.equal(linkRowView(s, 'esp32_feedback').label, '対象外')
})

test('S-01 要約: 繋がっていない項目だけを返す（除外・未受信は含めない）', async () => {
  const { linkDownKeys } = await import('../../src/ros/linkStatusState.js')
  assert.deepEqual(linkDownKeys(null), [])
  const s = parseLinkStatus(SAMPLE)
  assert.deepEqual(linkDownKeys(s), ['esp32_loopback', 'lidar', 'nodes'])
  const sim = parseLinkStatus({ items: { esp32_feedback: { ok: false, excluded: true },
    esp32_loopback: { ok: true }, lidar: { ok: true }, nodes: { ok: true } } })
  assert.deepEqual(linkDownKeys(sim), [])
})
