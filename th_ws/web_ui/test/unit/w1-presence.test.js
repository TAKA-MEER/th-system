// 1b-1 SG-A12: isW1Active() の presence_lost 条件の単体テスト。
import test from 'node:test'
import assert from 'node:assert/strict'
import { isW1Active } from '../../src/shell/limits.js'

test('presence_lost の PAUSE はフォルト無しでも W-1 を出す', () => {
  assert.equal(isW1Active('FOLLOW', 'PAUSE', false, 'presence_lost'), true)
  assert.equal(isW1Active('PREP', 'PAUSE', false, 'presence_lost'), true)
})

test('PAUSE でなければ理由があっても出さない・理由が無ければ従来どおり', () => {
  assert.equal(isW1Active('FOLLOW', 'RUN', false, 'presence_lost'), false)
  assert.equal(isW1Active('FOLLOW', 'PAUSE', false, ''), false)
  assert.equal(isW1Active('FOLLOW', 'PAUSE', true, ''), true)
  assert.equal(isW1Active('ESTOP', 'NONE', false, ''), true)
})

test('第4引数なしでも従来どおり動く（既存呼び出しの互換）', () => {
  assert.equal(isW1Active('FOLLOW', 'PAUSE', false), false)
  assert.equal(isW1Active('FOLLOW', 'PAUSE', true), true)
})
