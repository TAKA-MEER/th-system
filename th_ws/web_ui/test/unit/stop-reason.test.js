// WS-9R (2026-09-04): stopReason() の単体テスト。
//
// 実機フィードバック「謎の一時停止が発生する。画面をスクロールしたりすると
// 復帰する」。原因は在席確認が speed_limit=stop に倒していたことだったが、
// フォルトでも一時停止でもないので画面に何も出ず、操作者に理由が分からなかった。
//
// 帯の描画は e2e/stop-reason-banner.spec.js が見るが、あちらは AppShell が
// W-1 の窓と二重に出さないよう抑制するので、stopReason 自身の分岐（非常停止・
// フォルト）までは届かない。ここで純関数として押さえる。
import test from 'node:test'
import assert from 'node:assert/strict'
import { stopReason } from '../../src/shell/limits.js'

const RUNNING = { mode: 'REPLAY', zone: 'OUT', speed_limit: 'v_max' }
const PASS = { action: 'PASS' }

test('走れているときは null', () => {
  assert.equal(stopReason(RUNNING, PASS, null), null)
})

test('状態が未受信なら null（憶測で理由を出さない）', () => {
  assert.equal(stopReason(null, PASS, null), null)
})

test('非常停止が最優先', () => {
  assert.equal(stopReason({ ...RUNNING, mode: 'ESTOP' }, PASS, null), 'estop')
  assert.equal(stopReason({ ...RUNNING, mode: 'CARRY' }, PASS, null), 'estop')
})

test('非常停止はフォルトや在席未確認より先に出す', () => {
  const state = { mode: 'ESTOP', zone: 'NA', speed_limit: 'stop' }
  assert.equal(stopReason(state, { action: 'STOP' }, { active: true }), 'estop')
})

test('フォルトは在席未確認より先に出す', () => {
  const state = { mode: 'REPLAY', zone: 'NA', speed_limit: 'stop' }
  assert.equal(stopReason(state, PASS, { active: true }), 'fault')
})

test('zone=NA かつ speed_limit=stop は在席未確認', () => {
  const state = { mode: 'REPLAY', zone: 'NA', speed_limit: 'stop' }
  assert.equal(stopReason(state, PASS, null), 'presence')
})

test('zone=NA でも speed_limit が stop でなければ在席未確認ではない', () => {
  // S-30 / S-40 は zone=NA だが走れる画面（v_check / v_calib）。
  const state = { mode: 'OPCHECK', zone: 'NA', speed_limit: 'v_check' }
  assert.equal(stopReason(state, PASS, null), null)
})

test('障害物での停止を拾う', () => {
  assert.equal(stopReason(RUNNING, { action: 'STOP' }, null), 'obstacle')
})

test('指令途絶での停止を拾う', () => {
  assert.equal(stopReason(RUNNING, { action: 'ZERO_STALE' }, null), 'stale')
})

test('リミッタ未受信なら null（憶測で理由を出さない）', () => {
  assert.equal(stopReason(RUNNING, null, null), null)
})
