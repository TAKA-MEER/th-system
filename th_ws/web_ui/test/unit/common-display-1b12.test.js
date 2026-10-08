// 1b-12 共通の表示: 解除バー（SG-C10）・ヘッダの色（SG-C3）・W-6 の自動ブレーキ表示（SG-C9）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { showEstopRelease, headerTone, stopReason } from '../../src/shell/limits.js'
import { autoBrakeLabelKey } from '../../src/shell/autoBrake.js'

test('SG-C10: 解除バーは UI 非常停止を保持しているときだけ（重大フォルトの ESTOP では出さない）', () => {
  // 自分が押した
  assert.equal(showEstopRelease(true, 'IDLE', false), true)
  // ESTOP で UI 非常停止が保持されている（再読込・別端末）
  assert.equal(showEstopRelease(false, 'ESTOP', true), true)
  // 重大フォルトで入った ESTOP（UI 非常停止は無い）: 押しても何も起きないので出さない
  assert.equal(showEstopRelease(false, 'ESTOP', false), false)
  // 通常時
  assert.equal(showEstopRelease(false, 'IDLE', false), false)
  assert.equal(showEstopRelease(false, 'CARRY', false), false)
  // ESTOP でなければ estop_ui だけでは出さない
  assert.equal(showEstopRelease(false, 'REPLAY', true), false)
})

test('SG-C3: ヘッダの色はモード・開発モードで変わり、非常停止が最優先', () => {
  assert.equal(headerTone('IDLE', false), 'normal')
  assert.equal(headerTone('REPLAY', false), 'normal')
  assert.equal(headerTone('ESTOP', false), 'estop')
  assert.equal(headerTone('CARRY', false), 'carry')
  assert.equal(headerTone('IDLE', true), 'dev')
  assert.equal(headerTone('ESTOP', true), 'estop')
  assert.equal(headerTone('CARRY', true), 'carry')
  // 通信が古いときは機体の状態を色で断言しない（開発モードの表示は残す）
  assert.equal(headerTone('ESTOP', false, true), 'normal')
  assert.equal(headerTone('IDLE', true, true), 'dev')
})

test('SG-C9: W-6 の自動ブレーキ表示は ON/OFF/不明を取り違えない', () => {
  assert.equal(autoBrakeLabelKey(true), 'on')
  assert.equal(autoBrakeLabelKey(false), 'off')
  assert.equal(autoBrakeLabelKey(null), 'unknown')
  assert.equal(autoBrakeLabelKey(undefined), 'unknown')
})

// SG-C10 後半「CARRY 中に UI 非常停止を押した後の停止理由を『障害物』と誤表示する」。
// 2026-10-08 の調査では stopReason() では再現しない（CARRY / ESTOP は run_state が null で
// 走行状態ゲートが null を返す）。押下後の状態（estop_ui が立つ・リミッタが STOP）でも
// 理由の帯を出さないことを固定する。
test('SG-C10: CARRY で UI 非常停止を押した後（estop_ui・リミッタ STOP）も停止理由を出さない', () => {
  const attrs = { CARRY: { run_state: null }, ESTOP: { run_state: null } }
  for (const state of ['NONE', 'RUN', 'PAUSE', null]) {
    const s = { mode: 'CARRY', state, estop_ui: true, estop_hw: true, zone: 'IN', speed_limit: 'stop', prev_mode: 'REPLAY' }
    assert.equal(stopReason(s, { action: 'STOP' }, { active: false }, attrs), null, `state=${state}`)
  }
})
