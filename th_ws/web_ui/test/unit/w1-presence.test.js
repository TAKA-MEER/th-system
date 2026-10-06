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

// 1b-1: PREP/PAUSE は「帰還の一時停止」しか無く、出る手段が W-1 だけ。理由を問わず出す。
test('PREP/PAUSE は理由（jog・空・fault・presence_lost）を問わず W-1 を出す', () => {
  for (const reason of ['jog', '', 'fault', 'presence_lost']) {
    assert.equal(isW1Active('PREP', 'PAUSE', false, reason), true, `reason=${reason}`)
  }
})

test('PREP/PAUSE でもジョグ中は W-1 を出さない（手動操作パネルを覆わない）', () => {
  assert.equal(isW1Active('PREP', 'PAUSE', false, 'jog', true), false)
  assert.equal(isW1Active('PREP', 'PAUSE', false, '', true), false)
  assert.equal(isW1Active('PREP', 'PAUSE', false, 'jog', false), true)
})

test('PREP 以外のジョグ PAUSE と PREP の非 PAUSE は今までどおり出さない', () => {
  assert.equal(isW1Active('FOLLOW', 'PAUSE', false, 'jog'), false)
  assert.equal(isW1Active('PANEL_NAV', 'PAUSE', false, 'jog'), false)
  assert.equal(isW1Active('PREP', 'RETURN', false, ''), false)
  assert.equal(isW1Active('PREP', 'MAPPING', false, ''), false)
})
