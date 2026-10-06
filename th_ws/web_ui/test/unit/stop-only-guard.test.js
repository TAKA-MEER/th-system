// 1b-5: WebUI 側の「地図・設定値の操作ができるか」判定
// （modes/stopOnlyGuard.js）が、サーバ側の stop_only_allows
// （th_config_manager/th_config_manager/stop_only_guard.py）と同じ
// 許可条件を持つことを固定する。UI はこの判定で地図操作ボタンを事前
// 無効化する。安全の正本は server 側なので、ここが緩くてもサーバが
// 拒否する（このテストは UI の事前表示が SD-9 と矛盾しないことを縛る）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { mapOrParamOpAllowed, STOP_ONLY_PREP_STATES } from '../../src/modes/stopOnlyGuard.js'

const base = (over) => ({
  mode: 'IDLE', state: 'NONE', jog_active: false, ...over,
})

test('許可条件の集合が SD-9 どおり', () => {
  assert.deepEqual([...STOP_ONLY_PREP_STATES].sort(), ['EDIT', 'MAPPING', 'REGISTER', 'SAVED'])
})

test('IDLE は押せる', () => {
  assert.equal(mapOrParamOpAllowed(base(), false), true)
})

test('PREP の止まっている状態は押せる', () => {
  for (const state of ['MAPPING', 'REGISTER', 'EDIT', 'SAVED']) {
    assert.equal(mapOrParamOpAllowed(base({ mode: 'PREP', state }), false), true, `PREP/${state}`)
  }
})

test('PREP の RETURN・PAUSE、走行系、手動は押せない', () => {
  for (const [mode, state] of [
    ['PREP', 'RETURN'], ['PREP', 'PAUSE'],
    ['MANUAL', 'NONE'], ['REPLAY', 'RUN'], ['PANEL_NAV', 'NAV'],
    ['SUMMON', 'WAIT_CLEAR'], ['ESTOP', 'NONE'], ['INIT', 'CHECK'],
  ]) {
    assert.equal(mapOrParamOpAllowed(base({ mode, state }), false), false, `${mode}/${state}`)
  }
})

test('ジョグ中・未受信・stale は押せない', () => {
  assert.equal(mapOrParamOpAllowed(base({ jog_active: true }), false), false)
  assert.equal(mapOrParamOpAllowed(base(), true), false)
  assert.equal(mapOrParamOpAllowed(null, false), false)
  assert.equal(mapOrParamOpAllowed(undefined, false), false)
})
