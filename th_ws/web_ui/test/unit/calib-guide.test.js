// screens/calibGuide.js — S-30 → S-40 の誘導元の受け渡し（計画書 §6 #11）。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  OPCHECK_TO_CALIB, setCalibGuide, peekCalibGuide, takeCalibGuide, clearCalibGuide,
} from '../../src/screens/calibGuide.js'
import { s40CalibGuideText } from '../../src/i18n/screens.js'

test('対応: LIDAR→BLIND、IMU→IMU（Spec-checks.md §4）', () => {
  assert.deepEqual(OPCHECK_TO_CALIB, { LIDAR: 'BLIND', IMU: 'IMU' })
})

test('set→take で往復し、2 回目の take は null（1 回で消える）', () => {
  clearCalibGuide()
  assert.deepEqual(setCalibGuide({ from: 'LIDAR', result: 'NG' }),
    { from: 'LIDAR', result: 'NG', target: 'BLIND' })
  assert.deepEqual(takeCalibGuide(), { from: 'LIDAR', result: 'NG', target: 'BLIND' })
  assert.equal(takeCalibGuide(), null)
})

test('peek は消さない', () => {
  clearCalibGuide()
  setCalibGuide({ from: 'IMU', result: 'WARN' })
  assert.deepEqual(peekCalibGuide(), { from: 'IMU', result: 'WARN', target: 'IMU' })
  assert.deepEqual(peekCalibGuide(), { from: 'IMU', result: 'WARN', target: 'IMU' })
  clearCalibGuide()
})

test('対応表に無い項目（ESTOP・MOTOR）は置かずに捨てる', () => {
  clearCalibGuide()
  assert.equal(setCalibGuide({ from: 'ESTOP', result: 'NG' }), null)
  assert.equal(setCalibGuide({ from: 'MOTOR', result: 'NG' }), null)
  assert.equal(peekCalibGuide(), null)
})

test('対象外の結果（OK 等）は置かない', () => {
  clearCalibGuide()
  assert.equal(setCalibGuide({ from: 'LIDAR', result: 'OK' }), null)
  assert.equal(setCalibGuide({ from: 'LIDAR', result: 'UNKNOWN' }), null)
  assert.equal(setCalibGuide({ from: null, result: 'NG' }), null)
  assert.equal(peekCalibGuide(), null)
})

test('clear は残りを消す（enter_mode 拒否時）', () => {
  setCalibGuide({ from: 'LIDAR', result: 'NG' })
  clearCalibGuide()
  assert.equal(takeCalibGuide(), null)
})

test('案内文: NG と WARN で言い換え、自動で始めない（「開始」を押して）', () => {
  const lidarNg = s40CalibGuideText({ from: 'LIDAR', result: 'NG', target: 'BLIND' })
  assert.match(lidarNg, /始業点検/)
  assert.match(lidarNg, /LiDAR/)
  assert.match(lidarNg, /NGでした/)
  assert.match(lidarNg, /開始/)
  assert.match(lidarNg, /校正してください/)
  // IMU の WARN（IMU の NG は故障診断へ行き、ここには来ない）
  const imuWarn = s40CalibGuideText({ from: 'IMU', result: 'WARN', target: 'IMU' })
  assert.match(imuWarn, /要確認でした/)
  assert.match(imuWarn, /開始/)
})
