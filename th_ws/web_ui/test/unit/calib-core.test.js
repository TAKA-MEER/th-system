// screens/calibCore.js — S-40 校正画面の純ロジック（WP-MAINT-03）。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  parseCalibDetail, stepNumber, activeItem, viewKind, canProceed, parseMeasured,
  rotationWarnings, imuProgress, runPercent, formatValues, historyRows, lastCalibrated,
  previewText, isWizardItem, CALIB_STARTABLE,
} from '../../src/screens/calibCore.js'
import { s40ReasonLabel, S40_WARNINGS } from '../../src/i18n/calib.js'

test('stepNumber: S1〜S4 だけが 1〜4、LIST・その他は 0', () => {
  assert.deepEqual(['S1', 'S2', 'S3', 'S4'].map(stepNumber), [1, 2, 3, 4])
  for (const s of ['LIST', 'S5', 'S0', '', null, undefined]) assert.equal(stepNumber(s), 0)
})

test('parseCalibDetail: JSON 文字列・オブジェクト・壊れた入力', () => {
  assert.deepEqual(parseCalibDetail('{"a":1}'), { a: 1 })
  assert.deepEqual(parseCalibDetail({ a: 1 }), { a: 1 })
  for (const bad of ['', '{broken', null, undefined, 3, '[1]x']) assert.deepEqual(parseCalibDetail(bad), {})
})

test('canProceed: S3 は PREVIEW_OK（実行中の項目と一致）のときだけ', () => {
  const st = (result, item = 'LINEAR') => ({ item, result })
  assert.equal(canProceed({ fsmState: 'S1', item: 'LINEAR', status: null }), true)
  assert.equal(canProceed({ fsmState: 'S3', item: 'LINEAR', status: st('PREVIEW_OK') }), true)
  assert.equal(canProceed({ fsmState: 'S3', item: 'LINEAR', status: st('PREVIEW_INSANE') }), false)
  assert.equal(canProceed({ fsmState: 'S3', item: 'LINEAR', status: st('WAIT_MEASURED') }), false)
  assert.equal(canProceed({ fsmState: 'S3', item: 'LINEAR', status: st('PREVIEW_OK', 'ROTATION') }), false)
  assert.equal(canProceed({ fsmState: 'S3', item: 'LINEAR', status: null }), false)
  for (const s of ['LIST', 'S2', 'S4']) {
    assert.equal(canProceed({ fsmState: s, item: 'LINEAR', status: st('PREVIEW_OK') }), false, s)
  }
})

test('viewKind: FSM の state と status.result から右ペインの種類を導く', () => {
  const v = (fsmState, result, item = 'LINEAR') =>
    viewKind({ fsmState, item, status: { item, result } })
  assert.equal(v('LIST', ''), 'list')
  assert.equal(v('S1', 'GUIDE'), 'guide')
  assert.equal(v('S2', 'RUNNING'), 'running')
  assert.equal(v('S2', 'RETRY_WAIT'), 'retry')   // 検証 NG・走行失敗で S2 に戻った
  assert.equal(v('S3', 'WAIT_MEASURED'), 'measure')
  assert.equal(v('S3', 'PREVIEW_INSANE'), 'measure')
  assert.equal(v('S3', 'PREVIEW_OK'), 'preview_ok')
  assert.equal(v('S4', 'VERIFY_RUNNING'), 'verifying')
  assert.equal(v('S4', 'APPLYING'), 'verifying')
  assert.equal(v('S4', 'WAIT_VERIFY'), 'verify_input')
  assert.equal(v('S1', 'GUIDE', 'IMU'), 'guide')
  assert.equal(v('S2', 'RUNNING', 'IMU'), 'imu_progress')
  assert.equal(v('S3', 'PREVIEW_OK', 'IMU'), 'imu_done')
})

test('activeItem: status.item を優先し、押した直後だけ画面の選択で補う。LIST では無し', () => {
  assert.equal(activeItem({ item: 'ROTATION' }, 'LINEAR', 'S2'), 'ROTATION')
  assert.equal(activeItem({ item: '' }, 'LINEAR', 'S1'), 'LINEAR')
  assert.equal(activeItem(null, null, 'S1'), null)
  assert.equal(activeItem({ item: 'LINEAR' }, 'LINEAR', 'LIST'), null)
})

test('parseMeasured: 正の有限数だけ', () => {
  assert.equal(parseMeasured('0.97'), 0.97)
  assert.equal(parseMeasured(' 12 '), 12)
  for (const bad of ['', '  ', '0', '-1', 'abc', 'NaN', 'Infinity', null, undefined]) {
    assert.equal(parseMeasured(bad), null, String(bad))
  }
})

test('旋回の警告は detail.warnings をそのまま渡し、文言が全部ある', () => {
  assert.deepEqual(rotationWarnings({ warnings: ['linear_not_calibrated'] }), ['linear_not_calibrated'])
  assert.deepEqual(rotationWarnings({}), [])
  for (const key of ['linear_not_calibrated', 'linear_older_than_rotation']) {
    assert.ok(S40_WARNINGS[key], key)
  }
})

test('IMU 進捗: 各 0〜3、全部 3 で 100% ・完了', () => {
  const p = imuProgress({ imu: { sys: 3, gyro: 3, accel: 3, mag: 3 } })
  assert.equal(p.percent, 100)
  assert.equal(p.complete, true)
  const q = imuProgress({ imu: { sys: 3, gyro: 0, accel: 3, mag: 0 } })
  assert.equal(q.percent, 50)
  assert.equal(q.complete, false)
  assert.equal(imuProgress({}).percent, 0)
  assert.equal(imuProgress({ imu: { sys: 9, gyro: -1, accel: 'x' } }).parts[0].value, 3)
})

test('runPercent は 0〜100 に丸める', () => {
  assert.equal(runPercent({ progress: 0.456 }), 46)
  assert.equal(runPercent({ progress: 5 }), 100)
  assert.equal(runPercent({ progress: -1 }), 0)
  assert.equal(runPercent({}), 0)
})

test('履歴・最終校正日・補正値の表示', () => {
  const detail = {
    last_calibrated: { LINEAR: '2026-10-01T10:00:00', ROTATION: '' },
    history: { LINEAR: [{ generation: 1, values: { wheel_radius_scale: 0.98123456789 }, calibrated_at: 'a' }] },
  }
  assert.equal(lastCalibrated(detail, 'LINEAR'), '2026-10-01T10:00:00')
  assert.equal(lastCalibrated(detail, 'IMU'), '')
  assert.deepEqual(historyRows(detail, 'LINEAR'), [
    { generation: 1, text: 'wheel_radius_scale=0.981235', calibratedAt: 'a' }])
  assert.deepEqual(historyRows(detail, 'ROTATION'), [])
  assert.equal(formatValues(null), '')
  assert.equal(previewText('{"wheel_base":0.4}'), 'wheel_base=0.4')
  assert.equal(previewText('{broken'), '')
  assert.equal(previewText(''), '')
})

test('項目の分類: 直進・旋回はウィザード、IMU は違う、BLIND は開始できない', () => {
  assert.equal(isWizardItem('LINEAR'), true)
  assert.equal(isWizardItem('ROTATION'), true)
  assert.equal(isWizardItem('IMU'), false)
  assert.deepEqual(CALIB_STARTABLE, ['LINEAR', 'ROTATION', 'IMU'])
})

test('理由の文言: 数値付きは接頭辞で引き、知らない理由は握りつぶさない', () => {
  assert.match(s40ReasonLabel('error_exceeds_tolerance:0.2>0.05'), /許容範囲/)
  assert.match(s40ReasonLabel('tolerance_undefined'), /未確定/)
  assert.equal(s40ReasonLabel('something_new'), 'something_new')
  assert.equal(s40ReasonLabel(''), '')
})
