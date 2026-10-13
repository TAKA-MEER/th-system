// CheckStatus.basis_json（判定の根拠）の解釈と表記。機体側は
// th_maintenance/check_core.py の basis_item / basis_to_json が作る形。
import test from 'node:test'
import assert from 'node:assert/strict'
import { parseBasis } from '../../src/ros/checkBasis.js'
import {
  S30_BASIS_LABELS, s30BasisValueText, s30BasisThresholdText,
} from '../../src/i18n/screens.js'

const LIDAR = JSON.stringify([
  { key: 'coverage_gap_deg', value: 7.5, threshold: 20.0, unit: 'deg', cmp: 'lt', ok: true },
  { key: 'scan_period_s', value: 0.1, threshold: 2.0, unit: 's', cmp: 'le', ok: true },
  { key: 'blind_offset_deg', value: null, threshold: 5.0, unit: 'deg', cmp: 'le', ok: true },
])

test('parseBasis: 機体の形の JSON を読む（null は測れず）', () => {
  const b = parseBasis(LIDAR)
  assert.equal(b.length, 3)
  assert.deepEqual(b[0], { key: 'coverage_gap_deg', value: 7.5, threshold: 20, unit: 'deg', cmp: 'lt', ok: true })
  assert.equal(b[2].value, null)
})

test('parseBasis: 空文字・undefined・壊れた JSON・配列でないものは根拠なし(古い機体でも壊れない)', () => {
  for (const v of ['', undefined, null, 'not json', '{"a":1}', '42', 7]) {
    assert.deepEqual(parseBasis(v), [])
  }
})

test('parseBasis: key の無い要素は捨てる／ok:false だけが不合格', () => {
  const b = parseBasis(JSON.stringify([
    { value: 1 },
    { key: 'a', value: 1, threshold: 2, unit: 's', cmp: 'le', ok: false },
    { key: 'b', value: 1, threshold: 2, unit: 's', cmp: 'le' },
  ]))
  assert.deepEqual(b.map((x) => [x.key, x.ok]), [['a', false], ['b', true]])
})

test('表記: 例どおり「無効帯 7.5°（基準 20° 未満）／周期 0.1 秒（2 秒以内）／ズレ 5° 以内」', () => {
  const [gap, period, blind] = parseBasis(LIDAR)
  assert.equal(s30BasisValueText(gap), '7.5°')
  assert.equal(s30BasisThresholdText(gap), '20° 未満')
  assert.equal(s30BasisValueText(period), '0.1 秒')
  assert.equal(s30BasisThresholdText(period), '2 秒以内')
  assert.equal(s30BasisValueText(blind), '測れず')
  assert.equal(s30BasisThresholdText(blind), '5° 以内')
})

test('表記: IMU（rad/s は 3 桁固定・校正は整数で 3 以上）', () => {
  const [bias, calib] = parseBasis(JSON.stringify([
    { key: 'imu_bias', value: 0.0004, threshold: 0.2, unit: 'rad/s', cmp: 'le', ok: true },
    { key: 'calib_mag', value: 0, threshold: 3, unit: '', cmp: 'ge', ok: false },
  ]))
  assert.equal(s30BasisValueText(bias), '0.000 rad/s')
  assert.equal(s30BasisValueText(calib), '0')
  assert.equal(s30BasisThresholdText(calib), '3 以上')
})

test('機体が出すキーはすべて日本語ラベルを持つ', () => {
  for (const k of ['coverage_gap_deg', 'scan_period_s', 'blind_offset_deg', 'scan_alive',
    'imu_alive', 'imu_bias', 'imu_max_wz', 'calib_sys', 'calib_gyro', 'calib_accel', 'calib_mag']) {
    assert.ok(S30_BASIS_LABELS[k], k)
  }
})
