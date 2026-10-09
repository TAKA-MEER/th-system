// ros/presetOrder.js — S-50 速度プリセット3段階の順序（SG-C14）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { presetOrderError, PRESET_ORDER_NAMES } from '../../src/ros/presetOrder.js'
import { S50_PRESET_ORDER_REASONS } from '../../src/i18n/screens.js'

const ok3 = { speed_preset_low: 0.27, speed_preset_mid: 0.55, speed_preset_high: 1.0 }

test('順序どおり（等しいのも可）なら空', () => {
  assert.equal(presetOrderError(ok3), '')
  assert.equal(presetOrderError(
    { speed_preset_low: 0.5, speed_preset_mid: 0.5, speed_preset_high: 0.5 }), '')
})

test('違反のキーと文言（低速＞中速・中速＞高速・天井・下限）', () => {
  assert.equal(presetOrderError({ ...ok3, speed_preset_low: 0.6 }), 'low_gt_mid')
  assert.equal(presetOrderError({ ...ok3, speed_preset_mid: 1.0, speed_preset_high: 0.9 }), 'mid_gt_high')
  assert.equal(presetOrderError({ ...ok3, speed_preset_high: 1.2 }), 'over_ceiling')
  assert.equal(presetOrderError({ ...ok3, speed_preset_low: 0.01 }), 'below_min')
  // すべてのキーに文言がある（保存不可の理由表示が空にならない）。
  for (const key of ['below_min', 'over_ceiling', 'low_gt_mid', 'mid_gt_high']) {
    assert.ok(S50_PRESET_ORDER_REASONS[key], key)
  }
})

test('揃っていない間は判定しない（読み込み中・型違い）', () => {
  assert.equal(presetOrderError({}), '')
  assert.equal(presetOrderError({ ...ok3, speed_preset_mid: undefined }), '')
  assert.equal(presetOrderError(null), '')
  assert.deepEqual(PRESET_ORDER_NAMES,
    ['speed_preset_low', 'speed_preset_mid', 'speed_preset_high'])
})
