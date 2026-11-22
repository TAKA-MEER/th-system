// modes/armConfirm.js — 腕の格納の確認（SM-3.1.1-18）を出すかの判定。
// ESTOP / CARRY は prev_mode で判定する（現在モードだけでは自律測定からの
// 非常停止・手押しを拾えない）。ESTOP / CARRY 以外は常に false。AUTO_MEASURE
// の回復フォルトで PAUSE に落ちた W-1「確認」(ui.resume_ack) は C-05 で
// resume_state（PAUSE）に留まり、IDLE へ抜けないため対象外。
import test from 'node:test'
import assert from 'node:assert/strict'
import { needsArmConfirmForExit } from '../../src/modes/armConfirm.js'

test('ESTOP で prev_mode が AUTO_MEASURE の「メインメニューへ」(ui.resume_no) は確認を出す', () => {
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.resume_no', currentMode: 'ESTOP', prevMode: 'AUTO_MEASURE' }),
    true)
})

test('ESTOP で prev_mode が MANUAL なら出さない（すぐ送る）', () => {
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.resume_no', currentMode: 'ESTOP', prevMode: 'MANUAL' }),
    false)
})

test('ESTOP の「確認」(ui.resume_ack) も prev_mode が AUTO_MEASURE のときだけ出す', () => {
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.resume_ack', currentMode: 'ESTOP', prevMode: 'AUTO_MEASURE' }),
    true)
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.resume_ack', currentMode: 'ESTOP', prevMode: 'MANUAL' }),
    false)
})

test('CARRY で prev_mode が AUTO_MEASURE の ui.finish は確認を出す', () => {
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.finish', currentMode: 'CARRY', prevMode: 'AUTO_MEASURE' }),
    true)
})

test('CARRY で prev_mode が MANUAL なら出さない', () => {
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.finish', currentMode: 'CARRY', prevMode: 'MANUAL' }),
    false)
})

// ESTOP/CARRY 以外は常に false。AUTO_MEASURE の回復フォルトで PAUSE に
// 落ちた W-1「確認」は IDLE へ抜けないので確認を挟まない（SM-3.1.1-18）。
test('ESTOP/CARRY 以外は常に false（AUTO_MEASURE の PAUSE の「確認」も含む）', () => {
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.resume_ack', currentMode: 'AUTO_MEASURE', prevMode: '' }),
    false)
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.finish', currentMode: 'AUTO_MEASURE', prevMode: '' }),
    false)
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.resume_no', currentMode: 'MANUAL', prevMode: '' }),
    false)
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.resume_ack', currentMode: 'PREP', prevMode: '' }),
    false)
})

// 元のモードへ戻るボタン（ui.resume_yes / ui.carry_resume）は IDLE へ抜けないので
// どのモードからでも確認を挟まない。
test('ui.resume_yes / ui.carry_resume は確認を挟まない', () => {
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.resume_yes', currentMode: 'ESTOP', prevMode: 'AUTO_MEASURE' }),
    false)
  assert.equal(
    needsArmConfirmForExit({ trigger: 'ui.carry_resume', currentMode: 'CARRY', prevMode: 'AUTO_MEASURE' }),
    false)
})
