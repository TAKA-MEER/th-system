// brief-onsite-ux UX-1: prepSteps() / testSteps() のユニットテスト。
// 各段の完了条件と currentIndex（「未完了の最初の段」。全部完了なら最後の段）を
// 検証する。ROS / React 不要の純関数なので node --test で直接走る。
import test from 'node:test'
import assert from 'node:assert/strict'
import { NAV_MODES, prepSteps, testSteps } from '../../src/screens/onsiteSteps.js'

test('NAV_MODES は S-21 の移動系モードを持ち、IDLE を含まない', () => {
  assert.deepEqual(
    [...NAV_MODES].sort(),
    ['AT_PANEL', 'HOME_NAV', 'PANEL_NAV', 'SUMMON'].sort(),
  )
  assert.equal(NAV_MODES.includes('IDLE'), false)
})

// ── prepSteps（S-20） ───────────────────────────────────────

test('prepSteps: 何も無し → 段1のみ完了、current は段2（対象を選ぶ）', () => {
  const { steps, currentIndex } = prepSteps({ pins: [], personTargets: { selected_index: -1 } })
  assert.deepEqual(steps.map((s) => s.done), [true, false, false, false, false])
  assert.equal(currentIndex, 1)
  assert.equal(steps[1].id, 'target')
  assert.equal(steps[1].labelKey, 'S20_STEP_TARGET')
})

test('prepSteps: 対象を選ぶと段2が完了、current は段3（待機場所）', () => {
  const { steps, currentIndex } = prepSteps({
    pins: [], personTargets: { selected_index: 0 },
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, false, false, false])
  assert.equal(currentIndex, 2)
})

test('prepSteps: 何も無し → 段1のみ完了、current は段2（対象を選ぶ）', () => {
  const { steps, currentIndex } = prepSteps({ pins: [], personTargets: { selected_index: -1 } })
  assert.deepEqual(steps.map((s) => s.done), [true, false, false, false, false])
  assert.equal(currentIndex, 1)
  assert.equal(steps[1].id, 'target')
  assert.equal(steps[1].labelKey, 'S20_STEP_TARGET')
})

test('prepSteps: 対象を選ぶと段2が完了、current は段3（待機場所）', () => {
  const { steps, currentIndex } = prepSteps({
    pins: [], personTargets: { selected_index: 0 },
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, false, false, false])
  assert.equal(currentIndex, 2)
})

test('prepSteps: HOME が在っても対象未選択なら current は段2のまま（順序を守る）', () => {
  const { steps, currentIndex } = prepSteps({
    pins: [{ id: 'h', kind: 'HOME' }],
    personTargets: { selected_index: -1 },
  })
  assert.deepEqual(steps.map((s) => s.done), [true, false, true, false, false])
  assert.equal(currentIndex, 1)
})

test('prepSteps: HOME 登録で段3完了、current は段4（配電盤）', () => {
  const { steps, currentIndex } = prepSteps({
    pins: [{ id: 'h', kind: 'HOME' }],
    personTargets: { selected_index: 0 },
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, true, false, false])
  assert.equal(currentIndex, 3)
})

test('prepSteps: PANEL も登録され unsaved が空なら全部完了、current は最後の段', () => {
  const { steps, currentIndex } = prepSteps({
    pins: [{ id: 'h', kind: 'HOME' }, { id: 'p', kind: 'PANEL' }],
    personTargets: { selected_index: 0 },
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, true, true, true])
  assert.equal(currentIndex, 4)
})

test('prepSteps: 全部そろっていても unsaved が残っていれば保存は未完了', () => {
  const { steps, currentIndex } = prepSteps({
    pins: [{ id: 'h', kind: 'HOME' }, { id: 'p', kind: 'PANEL' }],
    personTargets: { selected_index: 0 },
    unsaved: [{ type: 'x' }],
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, true, true, false])
  assert.equal(currentIndex, 4)
  assert.equal(steps[4].done, false)
})

test('prepSteps: unsaved undefined / 非配列は空扱い', () => {
  for (const unsaved of [undefined, null, [], '']) {
    const { steps } = prepSteps({
      pins: [{ id: 'h', kind: 'HOME' }, { id: 'p', kind: 'PANEL' }],
      personTargets: { selected_index: 0 },
      unsaved,
    })
    assert.equal(steps[4].done, true, `unsaved=${String(unsaved)} は空扱い`)
  }
})

// ── testSteps（S-21） ───────────────────────────────────────

test('testSteps: 何も無し → 段1のみ不明、current は段1（会場地図を開く）', () => {
  const { steps, currentIndex } = testSteps({
    homeDeclared: false, mapOpened: false, selectedPinId: null, mode: 'IDLE', stateName: 'NONE',
  })
  assert.deepEqual(steps.map((s) => s.done), [false, false, false, false, false])
  assert.equal(currentIndex, 0)
  assert.equal(steps[0].id, 'open')
  assert.equal(steps[0].labelKey, 'S21_STEP_OPEN')
})

test('testSteps: /map_session/open の成功（mapOpened）で段1完了', () => {
  const { steps, currentIndex } = testSteps({
    homeDeclared: false, mapOpened: true, selectedPinId: null, mode: 'IDLE', stateName: 'NONE',
  })
  assert.deepEqual(steps.map((s) => s.done), [true, false, false, false, false])
  assert.equal(currentIndex, 1)
})

test('testSteps: 待機場所の宣言で段2完了', () => {
  const { steps, currentIndex } = testSteps({
    homeDeclared: true, mapOpened: true, selectedPinId: null, mode: 'IDLE', stateName: 'NONE',
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, false, false, false])
  assert.equal(currentIndex, 2)
})

test('testSteps: ピン選択（selectedPinId）でも段3完了', () => {
  const { steps, currentIndex } = testSteps({
    homeDeclared: true, mapOpened: true, selectedPinId: 'p1', mode: 'IDLE', stateName: 'NONE',
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, true, false, false])
  assert.equal(currentIndex, 3)
})

test('testSteps: 移動系モードならピン未選択でも段3完了', () => {
  for (const mode of NAV_MODES) {
    const { steps } = testSteps({
      homeDeclared: true, mapOpened: true, selectedPinId: null, mode, stateName: 'NAV',
    })
    assert.equal(steps[2].done, true, `mode=${mode} で段3完了`)
  }
})

test('testSteps: AT_PANEL で段4完了、current は段5（作業/呼び寄せ）', () => {
  const { steps, currentIndex } = testSteps({
    homeDeclared: true, mapOpened: true, selectedPinId: null, mode: 'AT_PANEL', stateName: 'IDLE_P',
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, true, true, false])
  assert.equal(currentIndex, 4)
})

test('testSteps: WORKING なら全部完了（current は最後の段）', () => {
  const { steps, currentIndex } = testSteps({
    homeDeclared: true, mapOpened: true, selectedPinId: null, mode: 'AT_PANEL', stateName: 'WORKING',
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, true, true, true])
  assert.equal(currentIndex, 4)
})

test('testSteps: SUMMON 中は段5（作業/呼び寄せ）だけ完了、段4（移動）は未完了のまま', () => {
  // 呼び寄せは配電盤前への移動（段4）を経ないため、段4は AT_PANEL に届いて初めて
  // 完了扱い。現在段＝段4（移動＝「停止」操作）として描かれる。
  const { steps, currentIndex } = testSteps({
    homeDeclared: true, mapOpened: true, selectedPinId: null, mode: 'SUMMON', stateName: 'POINT',
  })
  assert.deepEqual(steps.map((s) => s.done), [true, true, true, false, true])
  assert.equal(currentIndex, 3)
})