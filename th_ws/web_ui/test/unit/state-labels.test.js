// UIR-P3（Spec-webui.md §8・§8.1）: ヘッダの状態の札に内部の記号を出さない。
// 試験（当日）の状態（モード×状態）は語の表に在り、表に無いものは札を出さない。
import test from 'node:test'
import assert from 'node:assert/strict'
import { STATE_NAMES, stateLabel, stateChipTone, UNKNOWN_STATE_LABEL } from '../../src/i18n/states.js'

// 試験（当日）の状態。札の語はモックアップの札に合わせる。
const TRIAL_DAY = {
  IDLE_H: '待機中',
  IDLE_P: '待機中',
  NAV: '移動中',
  ALIGN: '配電盤前',
  BLOCKED: '経路待ち',
  PAUSE: '一時停止中',
  WORKING: '測定中',
  ADJUST: '旋回中',
  POINT: '位置指示中',
  WAIT_CLEAR: '退避待ち',
}

test('試験（当日の状態は語の表に在り、記号そのままでは無い', () => {
  for (const [state, word] of Object.entries(TRIAL_DAY)) {
    assert.equal(STATE_NAMES[state], word, `${state} の札の語`)
    assert.equal(stateLabel(state), word, `${state} の表示`)
    assert.notEqual(stateLabel(state), state, `${state} は記号を出さない`)
  }
})

test('表に無い状態は札を出さない（記号を出さない）', () => {
  assert.equal(stateLabel('NO_SUCH_STATE'), '')
  assert.equal(stateLabel('IDLE_X'), '')
})

test('状態が無いときは「不明」を出す', () => {
  assert.equal(stateLabel(''), UNKNOWN_STATE_LABEL)
  assert.equal(stateLabel(null), UNKNOWN_STATE_LABEL)
})

test('札の色は §3.3.1: 自分で走るときだけ緑、一時停止・待機は中立、経路待ち・退避待ちは注意', () => {
  for (const state of ['NAV', 'ALIGN', 'ADJUST']) {
    assert.equal(stateChipTone(state), 'go', `${state} は緑`)
  }
  for (const state of ['IDLE_H', 'IDLE_P', 'PAUSE', 'WORKING', 'POINT']) {
    assert.equal(stateChipTone(state), 'neutral', `${state} は中立`)
  }
  for (const state of ['BLOCKED', 'WAIT_CLEAR']) {
    assert.equal(stateChipTone(state), 'warn', `${state} は注意`)
  }
})
