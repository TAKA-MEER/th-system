// WP-UI-08: s30OverallLabel() drives S-30's 総合 pill. Pure function so it's
// tested without React (i18n/screens.js).
import test from 'node:test'
import assert from 'node:assert/strict'
import { s30OverallLabel } from '../../src/i18n/screens.js'

test('4項目とも未確認なら「未確認の項目があります」', () => {
  assert.equal(s30OverallLabel({}), '総合: 未確認の項目があります')
})

test('4項目とも OK なら「すべて正常」', () => {
  const results = {
    ESTOP: { result: 'OK' }, MOTOR: { result: 'OK' }, IMU: { result: 'OK' }, LIDAR: { result: 'OK' },
  }
  assert.equal(s30OverallLabel(results), '総合: すべて正常')
})

test('NG が1件あれば件数付きで「要修理」（WARN より優先）', () => {
  const results = {
    ESTOP: { result: 'NG' }, MOTOR: { result: 'WARN' }, IMU: { result: 'OK' }, LIDAR: { result: 'OK' },
  }
  assert.equal(s30OverallLabel(results), '総合: 要修理 1 件')
})

test('NG が複数件なら件数を数える', () => {
  const results = { ESTOP: { result: 'NG' }, MOTOR: { result: 'NG' } }
  assert.equal(s30OverallLabel(results), '総合: 要修理 2 件')
})

test('NG が無く WARN があれば「要確認」', () => {
  const results = {
    ESTOP: { result: 'OK' }, MOTOR: { result: 'OK' }, IMU: { result: 'WARN' }, LIDAR: { result: 'OK' },
  }
  assert.equal(s30OverallLabel(results), '総合: 要確認 1 件')
})

test('一部だけ OK・残りが未確認なら「未確認の項目があります」（すべて揃うまで正常と言わない）', () => {
  const results = { ESTOP: { result: 'OK' }, MOTOR: { result: 'OK' } }
  assert.equal(s30OverallLabel(results), '総合: 未確認の項目があります')
})
