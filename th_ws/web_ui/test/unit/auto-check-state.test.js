// 起動時の自動点検（/opcheck/auto_status）の純粋部 ros/autoCheckState.js の試験。
// React を import しない純粋モジュールなので node --test で直接回る。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  AUTO_CHECK_ITEMS, AUTO_CHECK_OVERALLS,
  parseAutoCheck, autoCheckWarns,
} from '../../src/ros/autoCheckState.js'

const FULL = {
  overall: 'NG',
  suppressed: false,
  items: {
    ESTOP: { result: 'OK', reason: '' },
    IMU: { result: 'WARN', reason: 'needs_calibration' },
    LIDAR: { result: 'NG', reason: 'no_data' },
  },
}

test('parseAutoCheck: JSON 文字列（実物の形）を読む', () => {
  assert.deepEqual(parseAutoCheck(JSON.stringify(FULL)), FULL)
})

test('parseAutoCheck: オブジェクトも受ける（TEST_MODE の種）', () => {
  assert.deepEqual(parseAutoCheck(FULL), FULL)
})

test('parseAutoCheck: 壊れた入力は null', () => {
  assert.equal(parseAutoCheck('not-json'), null)
  assert.equal(parseAutoCheck(null), null)
  assert.equal(parseAutoCheck('{}')?.overall, 'CHECKING')
})

test('parseAutoCheck: 不足項目は CHECKING で埋める', () => {
  const parsed = parseAutoCheck('{"overall":"OK","suppressed":false,"items":{}}')
  assert.equal(parsed.overall, 'OK')
  assert.deepEqual(parsed.items.ESTOP, { result: 'CHECKING', reason: '' })
})

test('autoCheckWarns: NG/WARN のときだけ真。suppressed では偽', () => {
  assert.equal(autoCheckWarns(parseAutoCheck({ ...FULL, overall: 'NG' })), true)
  assert.equal(autoCheckWarns(parseAutoCheck({ ...FULL, overall: 'WARN' })), true)
  assert.equal(autoCheckWarns(parseAutoCheck({ ...FULL, overall: 'OK' })), false)
  assert.equal(autoCheckWarns(parseAutoCheck({ ...FULL, overall: 'CHECKING' })), false)
  assert.equal(
    autoCheckWarns(parseAutoCheck({ ...FULL, overall: 'NG', suppressed: true })), false)
  assert.equal(autoCheckWarns(null), false)
})

test('定数: 項目は ESTOP/IMU/LIDAR（MOTOR は自動では行わない）', () => {
  assert.deepEqual(AUTO_CHECK_ITEMS, ['ESTOP', 'IMU', 'LIDAR'])
  assert.deepEqual(AUTO_CHECK_OVERALLS, ['OK', 'WARN', 'NG', 'CHECKING'])
})
