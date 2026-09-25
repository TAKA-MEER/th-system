// WP-UI-08: checkReasonLabel() maps th_maintenance/check_core.py's per-item
// reason keys to Japanese (i18n/checks.js). Same key means different things
// depending on the item (e.g. "no_data"/"no_samples"), so the lookup must
// be item-scoped.
import test from 'node:test'
import assert from 'node:assert/strict'
import { checkReasonLabel } from '../../src/i18n/checks.js'

test('空/未指定は空文字を返す', () => {
  assert.equal(checkReasonLabel('ESTOP', ''), '')
  assert.equal(checkReasonLabel('ESTOP', undefined), '')
})

test('ESTOP の既知キーは日本語になる', () => {
  assert.equal(checkReasonLabel('ESTOP', 'no_press'), '押下を検出できませんでした')
})

test('MOTOR の既知キーは日本語になる（左右で文言が違う）', () => {
  assert.notEqual(checkReasonLabel('MOTOR', 'sign_mismatch_L'), checkReasonLabel('MOTOR', 'sign_mismatch_R'))
})

test('同じキーでも項目が違えば違う訳になる（no_data: ESTOP vs IMU vs LIDAR）', () => {
  const estop = checkReasonLabel('ESTOP', 'no_data')
  const imu = checkReasonLabel('IMU', 'no_data')
  const lidar = checkReasonLabel('LIDAR', 'no_data')
  assert.notEqual(estop, imu)
  assert.notEqual(imu, lidar)
})

test('answer_mismatch は opcheck_runner.py 由来（check_core.py には無い）が ESTOP に定義されている', () => {
  assert.equal(checkReasonLabel('ESTOP', 'answer_mismatch'), '実物と画面表示が一致しないと回答されました')
})

test('未知のキーはそのまま返す（訳が無くても情報を消さない）', () => {
  assert.equal(checkReasonLabel('ESTOP', 'totally_unknown_key'), 'totally_unknown_key')
  assert.equal(checkReasonLabel('UNKNOWN_ITEM', 'no_data'), 'no_data')
})
