// brief-a13（SG-A13）: motorDirs.js の進捗・理由パース。
import test from 'node:test'
import assert from 'node:assert/strict'
import { MOTOR_DIRS, parseMotorDone, stripMotorDir } from '../../src/ros/motorDirs.js'

test('方向は FORWARD/BACK/LEFT/RIGHT の 4 つ', () => {
  assert.deepEqual(MOTOR_DIRS, ['FORWARD', 'BACK', 'LEFT', 'RIGHT'])
})

test('parseMotorDone: done= の CSV から済みだけ返す', () => {
  assert.deepEqual(parseMotorDone('done=FORWARD,BACK hold=NONE L 0.05 / R 0.05'), ['FORWARD', 'BACK'])
  assert.deepEqual(parseMotorDone('done= hold=NONE'), [])
  assert.deepEqual(parseMotorDone('done=FORWARD,UNKNOWN-DIR,RIGHT x'), ['FORWARD', 'RIGHT'])
})

test('parseMotorDone: done= が無ければ空', () => {
  assert.deepEqual(parseMotorDone('hold=FORWARD L 0.05'), [])
  assert.deepEqual(parseMotorDone(''), [])
  assert.deepEqual(parseMotorDone(undefined), [])
})

test('stripMotorDir: DIR:reason を割る', () => {
  assert.deepEqual(stripMotorDir('LEFT:sign_mismatch_L'), { dir: 'LEFT', reason: 'sign_mismatch_L' })
  assert.deepEqual(stripMotorDir('BACK:no_follow_R'), { dir: 'BACK', reason: 'no_follow_R' })
})

test('stripMotorDir: プレフィクス無しは dir=null', () => {
  assert.deepEqual(stripMotorDir('sign_mismatch_L'), { dir: null, reason: 'sign_mismatch_L' })
  assert.deepEqual(stripMotorDir(''), { dir: null, reason: '' })
})
