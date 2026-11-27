// W0a 自動ブレーキのロックの縛り（3回で切れる：ロック2回＋「切」）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { nextBrakePress, nextBrakeSide, LOCKED, ARMED, UNLOCKED, LOCK_ARM_MS, relocksOn } from '../../src/shell/autoBrakeLock.js'

test('鍵の「入」: 1回目は黄（送信なし）、2回目でロックが外れる（送信なし）', () => {
  const first = nextBrakePress(LOCKED, true)
  assert.equal(first.lock, ARMED)
  assert.equal(first.request, null)
  const second = nextBrakePress(ARMED, true)
  assert.equal(second.lock, UNLOCKED)
  assert.equal(second.request, null)
})

test('ロックが外れたら「切」で OFF を送る、「入」で ON を送りロックがかかり直す', () => {
  const off = nextBrakeSide('off', true)
  assert.equal(off.request, false)
  assert.equal(off.lock, UNLOCKED)
  const on = nextBrakeSide('on', false)
  assert.equal(on.request, true)
  assert.equal(on.lock, LOCKED)
})

test('約3秒は固定値（仕様の「約3秒」）', () => {
  assert.equal(LOCK_ARM_MS, 3000)
})

test('「入」に戻す・状態が変わるとロックがかかり直す', () => {
  assert.equal(relocksOn(true, false), true)
  assert.equal(relocksOn(false, true), true)
  assert.equal(relocksOn(false, false), false)
})
