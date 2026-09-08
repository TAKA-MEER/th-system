// brief-onsite-fix A: needsFinishEscape() の真偽表テスト。
// 画面の無いモード（OPCHECK / CALIB / FOLLOW 等）で S-01 が表示されているとき、
// ui.finish で IDLE に戻す導線を出すべきかを判定する。
import test from 'node:test'
import assert from 'node:assert/strict'
import { needsFinishEscape } from '../../src/screens/mainMenuItems.js'

test('OPCHECK → true', () => {
  assert.equal(needsFinishEscape('OPCHECK'), true)
})

test('CALIB → true', () => {
  assert.equal(needsFinishEscape('CALIB'), true)
})

test('FOLLOW → true', () => {
  assert.equal(needsFinishEscape('FOLLOW'), true)
})

test('LINE → true', () => {
  assert.equal(needsFinishEscape('LINE'), true)
})

test('LEASH → true', () => {
  assert.equal(needsFinishEscape('LEASH'), true)
})

test('IDLE → false（IDLE にいるので escape 不要）', () => {
  assert.equal(needsFinishEscape('IDLE'), false)
})

test('INIT → false（起動中は escape 不要）', () => {
  assert.equal(needsFinishEscape('INIT'), false)
})

test('null → false（mode 未確定は escape 不要）', () => {
  assert.equal(needsFinishEscape(null), false)
})

test('undefined → false（mode 未確定は escape 不要）', () => {
  assert.equal(needsFinishEscape(undefined), false)
})
