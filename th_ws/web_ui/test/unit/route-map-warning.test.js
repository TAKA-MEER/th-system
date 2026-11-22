// A-21: 地図の画像（pgm/yaml）が無いことの表示判定（screens/routeMapWarning.js）。
// 保存の警告（S-13）と、'unknown' での案内・「完全グローバルで探す」の無効化（S-14）。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  mapSaveWarningVisible,
  noMapRoute,
} from '../../src/screens/routeMapWarning.js'
import { S13_MAP_SAVE_WARNING, S14_NO_MAP_GUIDE } from '../../src/i18n/screens.js'

test('保存できて画像も揃っていれば警告を出さない', () => {
  assert.equal(mapSaveWarningVisible({ saved: true, map_save_failed: false }), false)
  assert.equal(mapSaveWarningVisible({ saved: true }), false)
})

test('保存できたが画像が揃わなければ警告を出す', () => {
  assert.equal(mapSaveWarningVisible({ saved: true, map_save_failed: true }), true)
})

test('保存自体が失敗していれば警告ではなく「保存されていません」に任せる', () => {
  assert.equal(mapSaveWarningVisible({ saved: false, map_save_failed: true }), false)
  assert.equal(mapSaveWarningVisible(undefined), false)
})

test('localize_quality が unknown のときだけ地図の画像が無い経路とみなす', () => {
  assert.equal(noMapRoute({ localize_quality: 'unknown' }), true)
  for (const q of ['high', 'low_margin', 'failed', 'searching', '', undefined, null]) {
    assert.equal(noMapRoute({ localize_quality: q }), false, `quality=${String(q)}`)
  }
  assert.equal(noMapRoute(undefined), false)
})

test('文言定数が空でない', () => {
  assert.equal(typeof S13_MAP_SAVE_WARNING, 'string')
  assert.ok(S13_MAP_SAVE_WARNING.includes('地図の画像を保存できませんでした'))
  assert.ok(S13_MAP_SAVE_WARNING.includes('始点に置けば再生はできます'))
  assert.equal(typeof S14_NO_MAP_GUIDE, 'string')
  assert.ok(S14_NO_MAP_GUIDE.includes('始点の印に置いて再生してください'))
  assert.ok(S14_NO_MAP_GUIDE.includes('完全グローバル'))
})
