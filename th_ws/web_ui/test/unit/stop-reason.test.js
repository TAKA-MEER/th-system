// WS-9R (2026-09-04): stopReason() の単体テスト。
//
// 実機フィードバック（1回目）「謎の一時停止が発生する。画面をスクロールしたり
// すると復帰する」。速度上限が 0 に落ちても画面に何も出ず理由が分からなかった。
//
// 実機フィードバック（2回目）「赤い帯、とくに『速度指令が届いていないため停止
// しています』が基本的に常に出ていて邪魔。一部表示を隠してしまっている」。
// 走らせていない間は速度指令が無いのが normal で、リミッタは常に ZERO_STALE を
// 返す。それを理由として出すと待機中ずっと帯が出たままになる。
//   → 機体が走るはずのとき（state == attributes[mode].run_state）だけ出す。
//   → ZERO_STALE は理由から外す。
import test from 'node:test'
import assert from 'node:assert/strict'
import { stopReason } from '../../src/shell/limits.js'

// attributes.yaml と同じ形（run_state だけ使う）。
const ATTRS = {
  IDLE: { run_state: null },
  REPLAY: { run_state: 'RUN' },
  TEACH_MANUAL: { run_state: 'REC' },
  ESTOP: { run_state: null },
  CARRY: { run_state: null },
}

const RUNNING = { mode: 'REPLAY', state: 'RUN', zone: 'OUT', speed_limit: 'v_max' }
const PASS = { action: 'PASS' }

test('走れているときは null', () => {
  assert.equal(stopReason(RUNNING, PASS, null, ATTRS), null)
})

test('状態が未受信なら null（憶測で理由を出さない）', () => {
  assert.equal(stopReason(null, PASS, null, ATTRS), null)
})

// ── 走るはずでない間は何も出さない（実機 2回目の指摘） ──────────────
test('待機中は理由を出さない', () => {
  const idle = { mode: 'IDLE', state: 'NONE', zone: 'NA', speed_limit: 'stop' }
  assert.equal(stopReason(idle, { action: 'ZERO_STALE' }, null, ATTRS), null)
})

test('経路選択中や一時停止中も出さない', () => {
  for (const st of ['ROUTE_SEL', 'LOCALIZE', 'READY', 'PAUSE']) {
    const s = { mode: 'REPLAY', state: st, zone: 'NA', speed_limit: 'stop' }
    assert.equal(stopReason(s, { action: 'ZERO_STALE' }, null, ATTRS), null,
      `state=${st} で帯を出してはいけない`)
  }
})

test('速度指令の途絶はもう理由にしない（待機中の通常状態なので）', () => {
  assert.equal(stopReason(RUNNING, { action: 'ZERO_STALE' }, null, ATTRS), null)
})

test('attributes が無ければ出さない（憶測しない）', () => {
  assert.equal(stopReason(RUNNING, { action: 'STOP' }, null, undefined), null)
})

test('教示中は REC が走行状態', () => {
  const rec = { mode: 'TEACH_MANUAL', state: 'REC', zone: 'OUT', speed_limit: 'v_max' }
  assert.equal(stopReason(rec, { action: 'STOP' }, null, ATTRS), 'obstacle')
  const paused = { ...rec, state: 'PAUSE' }
  assert.equal(stopReason(paused, { action: 'STOP' }, null, ATTRS), null)
})

// ── 走行中の理由（優先順位） ────────────────────────────────────
test('フォルトは在席未確認より先に出す', () => {
  const state = { mode: 'REPLAY', state: 'RUN', zone: 'NA', speed_limit: 'stop' }
  assert.equal(stopReason(state, PASS, { active: true }, ATTRS), 'fault')
})

test('zone=NA かつ speed_limit=stop は在席未確認', () => {
  const state = { mode: 'REPLAY', state: 'RUN', zone: 'NA', speed_limit: 'stop' }
  assert.equal(stopReason(state, PASS, null, ATTRS), 'presence')
})

test('zone=NA でも speed_limit が stop でなければ在席未確認ではない', () => {
  const state = { mode: 'REPLAY', state: 'RUN', zone: 'NA', speed_limit: 'v_check' }
  assert.equal(stopReason(state, PASS, null, ATTRS), null)
})

test('障害物での停止を拾う', () => {
  assert.equal(stopReason(RUNNING, { action: 'STOP' }, null, ATTRS), 'obstacle')
})

test('死角未校正での停止を拾う', () => {
  assert.equal(
    stopReason(RUNNING, { action: 'BLOCKED_UNCALIBRATED' }, null, ATTRS), 'uncalibrated')
})

test('リミッタ未受信なら null（憶測で理由を出さない）', () => {
  assert.equal(stopReason(RUNNING, null, null, ATTRS), null)
})

test('非常停止・手押しは走行状態が無いのでゲートで落ちる', () => {
  // ここを個別の分岐で書くと到達しないコードになる（run_state が null）。
  // ゲートが受け持っていることを固定しておく。
  for (const mode of ['ESTOP', 'CARRY']) {
    const s = { mode, state: 'NONE', zone: 'NA', speed_limit: 'stop' }
    assert.equal(stopReason(s, { action: 'STOP' }, { active: true }, ATTRS), null)
  }
})
