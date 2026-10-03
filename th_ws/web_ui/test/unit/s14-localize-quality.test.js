// S-14 初期姿勢の確度表示（W-01 P3 / DetailedDesign-transit-localize-options.md §4）。
// localize_quality → 表示の写像（screens/s14Localize.js）の unit。数値は出さない。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  localizeQualityView,
  localizeQualityText,
} from '../../src/screens/s14Localize.js'
import {
  S14_QUALITY_TITLE,
  S14_QUALITY_HIGH,
  S14_QUALITY_LOW,
  S14_QUALITY_LOW_HINT,
  S14_QUALITY_FAILED,
  S14_QUALITY_FAILED_HINT,
  S14_QUALITY_SEARCHING,
  S14_GLOBAL_BUTTON,
} from '../../src/i18n/screens.js'

test('high は確度「高い」を出す', () => {
  assert.deepEqual(localizeQualityView('high'), { visible: true, level: 'high' })
  assert.equal(localizeQualityText('high'), `${S14_QUALITY_TITLE}：${S14_QUALITY_HIGH}`)
})

test('low_margin は「低い」＋取り違え警告（数値は出さない）', () => {
  assert.deepEqual(localizeQualityView('low_margin'), { visible: true, level: 'low' })
  const text = localizeQualityText('low')
  assert.ok(text.includes(S14_QUALITY_LOW))
  assert.ok(text.includes(S14_QUALITY_LOW_HINT))
  assert.ok(!text.includes(S14_QUALITY_HIGH))
  assert.match(text, /確度：低い/)
})

test('failed は「見つかりません」＋選び直しの案内', () => {
  assert.deepEqual(localizeQualityView('failed'), { visible: true, level: 'failed' })
  const text = localizeQualityText('failed')
  assert.ok(text.includes(S14_QUALITY_FAILED))
  assert.ok(text.includes(S14_QUALITY_FAILED_HINT))
  assert.ok(text.includes('経路を選び直し'))
})

test('searching は「探しています」', () => {
  assert.deepEqual(localizeQualityView('searching'), { visible: true, level: 'searching' })
  assert.ok(localizeQualityText('searching').includes(S14_QUALITY_SEARCHING))
})

test('unknown・空・未受信（旧経路）は確度欄を出さない', () => {
  for (const q of ['unknown', '', undefined, null]) {
    const view = localizeQualityView(q)
    assert.equal(view.visible, false, `quality=${String(q)}`)
    assert.equal(view.level, null, `quality=${String(q)}`)
  }
  assert.equal(localizeQualityText(null), null)
})

test('文言定数が空でない（ボタンを含む）', () => {
  for (const s of [
    S14_QUALITY_TITLE, S14_QUALITY_HIGH, S14_QUALITY_LOW, S14_QUALITY_LOW_HINT,
    S14_QUALITY_FAILED, S14_QUALITY_FAILED_HINT, S14_QUALITY_SEARCHING,
    S14_GLOBAL_BUTTON,
  ]) {
    assert.equal(typeof s, 'string')
    assert.ok(s.length > 0)
  }
  assert.equal(S14_GLOBAL_BUTTON, '完全グローバルで探す')
})
