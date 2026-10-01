// screens/calibCore.js — S-40 校正画面の純ロジック（WP-MAINT-03）。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  angleFromPoint, rangeFromDrag, blindProblem, rangesFromFlat, blindRangesText, blindLimits,
  blindReasonKey, scanPointsByMask, BLIND_LIMITS_DEFAULT,
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

test('項目の分類: 直進・旋回・死角はウィザード、IMU は違う。4 項目すべて開始できる', () => {
  assert.equal(isWizardItem('LINEAR'), true)
  assert.equal(isWizardItem('ROTATION'), true)
  assert.equal(isWizardItem('BLIND'), true)
  assert.equal(isWizardItem('IMU'), false)
  assert.deepEqual(CALIB_STARTABLE, ['LINEAR', 'ROTATION', 'IMU', 'BLIND'])
})

test('BLIND の viewKind: S2 は選択、S3 は PREVIEW_OK のときだけプレビュー、S4 は検証中', () => {
  const k = (fsmState, result) => viewKind({ fsmState, item: 'BLIND', status: { item: 'BLIND', result } })
  assert.equal(k('S1', 'GUIDE'), 'guide')
  assert.equal(k('S2', 'RUNNING'), 'blind_select')
  assert.equal(k('S2', 'PREVIEW_INSANE'), 'blind_select')
  assert.equal(k('S2', 'RETRY_WAIT'), 'blind_select')        // 検証 NG で戻っても再選択できる
  assert.equal(k('S3', 'PREVIEW_OK'), 'blind_preview')
  assert.equal(k('S3', 'PREVIEW_INSANE'), 'blind_select')
  assert.equal(k('S4', 'VERIFY_RUNNING'), 'verifying')
  assert.equal(canProceed({ fsmState: 'S3', item: 'BLIND', status: { item: 'BLIND', result: 'PREVIEW_OK' } }), true)
  assert.equal(canProceed({ fsmState: 'S3', item: 'BLIND', status: { item: 'BLIND', result: 'PREVIEW_INSANE' } }), false)
})

test('BLIND: ドラッグの角度（前方 0°・左 +90°）と、近い側の弧への正規化', () => {
  // 画面中心 (100,100)。上 = 前方 0°、左 = +90°、右 = -90°、下 = 180°
  assert.equal(Math.round(angleFromPoint(100, 50, 100, 100)), 0)
  assert.equal(Math.round(angleFromPoint(50, 100, 100, 100)), 90)
  assert.equal(Math.round(angleFromPoint(150, 100, 100, 100)), -90)
  assert.equal(Math.round(Math.abs(angleFromPoint(100, 150, 100, 100))), 180)
  assert.deepEqual(rangeFromDrag(10, 40), [10, 40])
  assert.deepEqual(rangeFromDrag(40, 10), [10, 40])             // 逆向きになぞっても同じ
  assert.deepEqual(rangeFromDrag(170, -170), [170, -170])       // ±180° をまたぐ
  assert.deepEqual(rangeFromDrag(-170, 170), [170, -170])
})

test('BLIND: 幅の上限（1 区間 30°・総幅 90°・8 区間）。機体側と同じ判定', () => {
  assert.equal(blindProblem([]), null)
  assert.equal(blindProblem([[0, 30]]), null)
  assert.equal(blindProblem([[0, 31]]), 'sector_too_wide')
  assert.equal(blindProblem([[5, 5]]), 'zero_width')
  assert.equal(blindProblem([[0, 23], [60, 83], [120, 143], [-60, -37]]), 'total_too_wide')
  const nine = Array.from({ length: 9 }, (_, i) => [i * 40 - 170, i * 40 - 165])
  assert.equal(blindProblem(nine), 'too_many_sectors')
  assert.equal(blindProblem([[170, -170]]), null)                // 20°
  assert.equal(blindProblem([[170, -130]]), 'sector_too_wide')   // 60°
  // 出荷時のマスク（4 区間・約 68°）は上限内
  assert.equal(blindProblem(rangesFromFlat([-132.8, -117.5, -59.5, -38.9, 45.9, 61.5, 130.3, 143.5])), null)
})

test('BLIND: 平坦配列と表示、機体が返す上限を優先する', () => {
  assert.deepEqual(rangesFromFlat([1, 2, 3, 4, 5]), [[1, 2], [3, 4]])
  assert.equal(blindRangesText([]), '（なし）')
  assert.equal(blindRangesText([-12, 12]), '-12〜12°')
  assert.deepEqual(blindLimits({ blind: { limits: { max_sector_deg: 20, max_total_deg: 50, max_sectors: 4 } } }),
    { max_sector_deg: 20, max_total_deg: 50, max_sectors: 4 })
  assert.deepEqual(blindLimits({}), BLIND_LIMITS_DEFAULT)
  assert.equal(blindReasonKey('sector_too_wide:35.0>30.0'), 'sector_too_wide')
  assert.equal(previewText('{"blind_angle_ranges":[-12,12],"masked_points":5}').startsWith('-12〜12°'), true)
})

test('BLIND: スキャンの点を、選択の内側（消える）と外側に分ける', () => {
  const scan = {
    angle_min: -Math.PI, angle_increment: Math.PI / 180, range_min: 0.05, range_max: 12,
    ranges: Array.from({ length: 360 }, () => 2.0),
  }
  const { kept, masked } = scanPointsByMask(scan, [[-10, 10]])
  assert.equal(masked.length, 21)
  assert.equal(kept.length, 339)
  assert.equal(scanPointsByMask(scan, []).masked.length, 0)
  assert.equal(scanPointsByMask(null, [[0, 1]]).kept.length, 0)
})

test('理由の文言: 数値付きは接頭辞で引き、知らない理由は握りつぶさない', () => {
  assert.match(s40ReasonLabel('error_exceeds_tolerance:0.2>0.05'), /許容範囲/)
  assert.match(s40ReasonLabel('tolerance_undefined'), /未確定/)
  assert.equal(s40ReasonLabel('something_new'), 'something_new')
  assert.equal(s40ReasonLabel(''), '')
})
