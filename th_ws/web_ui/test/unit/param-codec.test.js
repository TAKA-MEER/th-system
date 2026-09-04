// WS-9X: ros/paramCodec.js は useRosbridge.js から切り出した rcl_interfaces
// ParameterValue の JS 変換。S-50 設定画面（useTunableParams.js）と観客ビュー
// （useRosbridge.js）の両方が使うので、型往復の回帰を固定しておく。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  PARAM_TYPE, decodeParamValue, encodeParamValue,
} from '../../src/ros/paramCodec.js'

test('decodeParamValue: 型ごとに対応フィールドを返す', () => {
  assert.equal(decodeParamValue({ type: PARAM_TYPE.BOOL, bool_value: true }), true)
  assert.equal(decodeParamValue({ type: PARAM_TYPE.INTEGER, integer_value: 42 }), 42)
  assert.equal(decodeParamValue({ type: PARAM_TYPE.DOUBLE, double_value: 0.5 }), 0.5)
  assert.equal(decodeParamValue({ type: PARAM_TYPE.STRING, string_value: 'x' }), 'x')
  assert.deepEqual(
    decodeParamValue({ type: PARAM_TYPE.DOUBLE_ARRAY, double_array_value: [1, 2] }), [1, 2])
})

test('decodeParamValue: 未知の型は null', () => {
  assert.equal(decodeParamValue({ type: 999 }), null)
})

test('encodeParamValue: スカラは double / integer を選ぶ', () => {
  assert.deepEqual(encodeParamValue(0.5), { type: PARAM_TYPE.DOUBLE, double_value: 0.5 })
  assert.deepEqual(
    encodeParamValue(3, { isInt: true }), { type: PARAM_TYPE.INTEGER, integer_value: 3 })
})

test('encodeParamValue: 配列は double_array / integer_array', () => {
  assert.deepEqual(
    encodeParamValue([1, 2], { isArray: true }),
    { type: PARAM_TYPE.DOUBLE_ARRAY, double_array_value: [1, 2] })
  assert.deepEqual(
    encodeParamValue([1, 2], { isArray: true, isInt: true }),
    { type: PARAM_TYPE.INTEGER_ARRAY, integer_array_value: [1, 2] })
})

test('round-trip: encode -> decode でスカラ値が保たれる', () => {
  for (const v of [0, 0.25, -1.5, 3]) {
    assert.equal(decodeParamValue(encodeParamValue(v)), v)
  }
})
