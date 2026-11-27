// W0a 自動ブレーキのロックの縛り。
import test from 'node:test'
import assert from 'node:assert/strict'
import { nextBrakePress, LOCKED, ARMED, UNLOCKED, LOCK_ARM_MS, relocksOn } from '../../src/shell/autoBrakeLock.js'

test('鍵の「入」: 1回目は黄（送信なし）、2回目で OFF 送信', () => {
  const first = nextBrakePress(LOCKED, true)
  assert.equal(first.lock, ARMED)
  assert.equal(first.request, null)
  const second = nextBrakePress(ARMED, true)
  assert.equal(second.lock, UNLOCKED)
  assert.equal(second.request, false)
})

test('OFF→ON は確認なしで送信し、ロックがかかり直す', () => {
  const r = nextBrakePress(UNLOCKED, false)
  assert.equal(r.request, true)
  assert.equal(r.lock, LOCKED)
})

test('約3秒は固定値（仕様の「約3秒」）', () => {
  assert.equal(LOCK_ARM_MS, 3000)
})

test('「入」に戻す・状態が変わる＋「入」とロックがかかり直す', () => {
  assert.equal(relocksOn(true, false), true)
  assert.equal(relocksOn(false, true), true)
  assert.equal(relocksOn(false, false), false)
})

test('未受信（null）では押しても送らない', () => {
  assert.equal(nextBrakePress(LOCKED, null).request, null)
})
