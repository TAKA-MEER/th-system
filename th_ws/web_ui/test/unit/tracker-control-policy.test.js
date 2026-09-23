// brief-tracker-default-off §3.4: WebUI 側の「人検出を停止できるか」判定
// （modes/trackerControlPolicy.js）が、th_state 側の tracker_off_denied
// （src/th_state/th_state/tracker_policy.py、_TRACKER_OFF_DENIED）と同じ
// 拒否ポリシーを持つことを固定する。UI はこの判定で「人検出を停止」を事前
// 無効化して理由を出す。安全の正本は server 側なので、ここが緩くても
// サーバが拒否する（このテストは UI の事前表示が FSM と矛盾しないことを縛る）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { trackerStopAllowed } from '../../src/modes/trackerControlPolicy.js'

test('SUMMON はどの状態でも停止できない', () => {
  for (const state of ['POINT', 'WAIT_CLEAR', 'NONE', 'idle']) {
    assert.equal(trackerStopAllowed('SUMMON', state), false, `SUMMON/${state} は停止できるべきでない`)
  }
})

test('PREP/REGISTER は停止できない（2 点指示の登録中）', () => {
  assert.equal(trackerStopAllowed('PREP', 'REGISTER'), false)
  // PREP の他の状態（MAPPING 等）では停止できる。
  assert.equal(trackerStopAllowed('PREP', 'MAPPING'), true)
})

test('それ以外のモード・状態では停止できる', () => {
  for (const [mode, state] of [
    ['PREP', 'MAPPING'],
    ['IDLE', 'NONE'],
    ['MANUAL', 'PAUSE'],
    ['AT_HOME', 'NONE'],
    ['PANEL_NAV', 'NAV'],
    ['AT_PANEL', 'IDLE_P'],
    ['HOME_NAV', 'NAV'],
    ['ESTOP', 'NONE'],
  ]) {
    assert.equal(trackerStopAllowed(mode, state), true, `${mode}/${state} は停止できるべき`)
  }
})