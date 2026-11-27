// W0a 部品の純関数の縛り（stopRun・autonomous）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { stopRunSides } from '../../src/shell/stopRun.js'
import { isAutonomousRun, AUTONOMOUS_TABLE } from '../../src/shell/autonomous.js'

test('停止│走行: 押せる側だけ塗る（止める黒・走る緑）', () => {
  assert.deepEqual(stopRunSides({ stopEnabled: true, runEnabled: true }), { stop: 'paint-black', run: 'paint-green' })
  assert.deepEqual(stopRunSides({ stopEnabled: true, runEnabled: false }), { stop: 'paint-black', run: 'plain' })
  assert.deepEqual(stopRunSides({ stopEnabled: false, runEnabled: true }), { stop: 'plain', run: 'paint-green' })
  assert.deepEqual(stopRunSides({ stopEnabled: false, runEnabled: false }), { stop: 'plain', run: 'plain' })
})

test('停止│走行: 予約の間は押せない見た目（両方 plain）', () => {
  assert.deepEqual(stopRunSides({ stopEnabled: true, runEnabled: true, reserved: true }), { stop: 'plain', run: 'plain' })
})

for (const [mode, state] of [
  ['FOLLOW', 'RUN'], ['TEACH_FOLLOW', 'REC'], ['REPLAY', 'RUN'],
  ['PANEL_NAV', 'NAV'], ['PANEL_NAV', 'ALIGN'], ['SUMMON', 'NAV'], ['AT_HOME', 'NAV'],
  ['PREP', 'RETURN'], ['AUTO_MEASURE', 'ADJUST'], ['AUTO_MEASURE', 'RUN'],
]) {
  test(`自律枠: ${mode}/${state} は出す`, () => {
    assert.equal(isAutonomousRun(mode, state), true)
  })
}

for (const [mode, state] of [
  ['MANUAL', 'RUN'], ['TEACH_MANUAL', 'REC'], ['FOLLOW', 'PAUSE'], ['REPLAY', 'PAUSE'],
  ['PREP', 'MAPPING'], ['MANUAL', 'PAUSE'], [null, 'RUN'], ['FOLLOW', null], ['IDLE', 'NONE'],
]) {
  test(`自律枠: ${mode}/${state} は出さない（一時停止・手動・待機）`, () => {
    assert.equal(isAutonomousRun(mode, state), false)
  })
}

test('自律枠の表に無いモードは出さない', () => {
  for (const mode of Object.keys(AUTONOMOUS_TABLE)) {
    assert.ok(Array.isArray(AUTONOMOUS_TABLE[mode]))
  }
})
