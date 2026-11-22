// S-14 の「再生」を押せるかの判定（A-23 ②）と、地図の読み込み中の判定（A-22）の unit。
// screens/s14Localize.js の純関数を試す（Spec-webui.md §3.7）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { replayRunGate, mapLoadingVisible } from '../../src/screens/s14Localize.js'
import {
  S14_QUALITY_FAILED,
  S14_QUALITY_OFF_ROUTE,
  S14_QUALITY_SEARCHING,
} from '../../src/i18n/screens.js'

test('LOCALIZE は確度にかかわらず「再生」を押せず、理由を返す', () => {
  const cases = [
    ['searching', S14_QUALITY_SEARCHING],
    ['failed', S14_QUALITY_FAILED],
    ['off_route', S14_QUALITY_OFF_ROUTE],
    // 位置は決まったが reload/TF 待ちの間、未受信、画像の無い経路の読み直し中。
    ['high', S14_QUALITY_SEARCHING],
    ['low_margin', S14_QUALITY_SEARCHING],
    ['unknown', S14_QUALITY_SEARCHING],
    ['', S14_QUALITY_SEARCHING],
    [undefined, S14_QUALITY_SEARCHING],
  ]
  for (const [quality, reason] of cases) {
    const gate = replayRunGate('LOCALIZE', quality)
    assert.equal(gate.enabled, false, `quality=${String(quality)}`)
    assert.equal(gate.reason, reason, `quality=${String(quality)}`)
  }
})

test('READY は押せる（画像の無い経路＝unknown も始点決め打ちで READY に進む）', () => {
  for (const quality of ['high', 'low_margin', 'unknown', '', undefined]) {
    const gate = replayRunGate('READY', quality)
    assert.equal(gate.enabled, true, `quality=${String(quality)}`)
    assert.equal(gate.reason, null)
  }
})

test('PAUSE の「再生」（一時停止の再開）は押せる', () => {
  assert.deepEqual(replayRunGate('PAUSE', 'high'), { enabled: true, reason: null })
})

test('RUN の「再生」も止めない', () => {
  assert.deepEqual(replayRunGate('RUN', undefined), { enabled: true, reason: null })
})

test('地図の読み込み中は LOCALIZE の間だけ出す', () => {
  // 探索＋読み直し中は、地図が届いていても出す。
  assert.equal(mapLoadingVisible('LOCALIZE', 'searching', true), true)
  // 地図のトピックがまだ届いていない間も出す。
  assert.equal(mapLoadingVisible('LOCALIZE', '', false), true)
  assert.equal(mapLoadingVisible('LOCALIZE', undefined, false), true)
  // 地図が届いていて探索もしていなければ出さない。
  assert.equal(mapLoadingVisible('LOCALIZE', '', true), false)
  // 失敗・経路から離れている（読み込みは終わった／失敗した）は出さない。
  assert.equal(mapLoadingVisible('LOCALIZE', 'failed', false), false)
  assert.equal(mapLoadingVisible('LOCALIZE', 'off_route', false), false)
  // LOCALIZE を抜けたら出さない。
  assert.equal(mapLoadingVisible('READY', 'searching', false), false)
  assert.equal(mapLoadingVisible('RUN', 'searching', false), false)
})
