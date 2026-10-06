// 1b-6 (SG-B2): /system/effect の振り分けと W-3 の自動クローズ条件を縛る。
// 変異: guide を捨てる／dest 判定を外す／自動クローズを外すと赤くなること。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  dispatchEffect, shouldGuideClose, resetEffectWarningsForTest,
} from '../../src/shell/effectDispatch.js'

function guideMsg(key, dest = 'WebUI') {
  return { name: 'guide', dest, args_json: JSON.stringify({ key }) }
}

test('guide で帯の状態が立つ', () => {
  resetEffectWarningsForTest()
  const decided = dispatchEffect(guideMsg('summon_clear_timeout'))
  assert.deepEqual(decided, { kind: 'guide', key: 'summon_clear_timeout' })
})

test('dest が WebUI でないものは無視 (restart_control_stack は connectivity_checker 宛)', () => {
  resetEffectWarningsForTest()
  assert.deepEqual(
    dispatchEffect({ name: 'guide', dest: 'connectivity_checker', args_json: '{}' }),
    { kind: 'ignore' },
  )
  assert.deepEqual(
    dispatchEffect({ name: 'guide', dest: 'unknown', args_json: '{}' }),
    { kind: 'ignore' },
  )
  // 有効なキーでも dest が WebUI でなければ見ない (dest 判定を外す変異で赤くなること)。
  assert.deepEqual(
    dispatchEffect(guideMsg('summon_clear_timeout', 'connectivity_checker')),
    { kind: 'ignore' },
  )
})

test('dest が WebUI+jog_gate のように + 結合でも WebUI を含めば見る', () => {
  resetEffectWarningsForTest()
  // disable_jog_ui 自体は別パケットで何もしないが、dest 判定は通る (ignore になる)。
  assert.deepEqual(
    dispatchEffect({ name: 'disable_jog_ui', dest: 'WebUI+jog_gate', args_json: '{}' }),
    { kind: 'ignore' },
  )
})

test('今回は実装しない effect は何もしない (open_window/close_window/show_resume/ask_save 系/offer 系)', () => {
  resetEffectWarningsForTest()
  const warnings = []
  const orig = console.warn
  console.warn = (m) => warnings.push(m)
  try {
    for (const name of [
      'open_window', 'close_window', 'show_resume',
      'ask_save', 'ask_save_if_unsaved', 'enable_main_menu',
      'offer_calib', 'offer_opcheck',
    ]) {
      assert.deepEqual(
        dispatchEffect({ name, dest: 'WebUI', args_json: '{}' }),
        { kind: 'ignore' },
        name,
      )
    }
  } finally {
    console.warn = orig
  }
  assert.equal(warnings.length, 0, '既知の名前で警告が出てはいけない')
})

test('未知の名前は警告して捨てる (警告は 1 回だけ)', () => {
  resetEffectWarningsForTest()
  const warnings = []
  const orig = console.warn
  console.warn = (m) => warnings.push(m)
  try {
    assert.deepEqual(
      dispatchEffect({ name: 'fly_to_moon', dest: 'WebUI', args_json: '{}' }),
      { kind: 'ignore' },
    )
    assert.deepEqual(
      dispatchEffect({ name: 'fly_to_moon', dest: 'WebUI', args_json: '{}' }),
      { kind: 'ignore' },
    )
  } finally {
    console.warn = orig
  }
  assert.equal(warnings.length, 1, `警告は 1 回だけのはず: ${JSON.stringify(warnings)}`)
})

test('キー無し・文言無しキーの guide は捨てる (空の帯を出さない)', () => {
  resetEffectWarningsForTest()
  const warnings = []
  const orig = console.warn
  console.warn = (m) => warnings.push(m)
  try {
    assert.deepEqual(dispatchEffect(guideMsg('wait_clear_timeout')), { kind: 'ignore' })
    assert.deepEqual(
      dispatchEffect({ name: 'guide', dest: 'WebUI', args_json: 'not-json' }),
      { kind: 'ignore' },
    )
    assert.deepEqual(
      dispatchEffect({ name: 'guide', dest: 'WebUI', args_json: '' }),
      { kind: 'ignore' },
    )
  } finally {
    console.warn = orig
  }
  assert.ok(warnings.length >= 1, '捨てた guide は警告すること')
})

test('自動クローズ: モードが変わったら閉じる', () => {
  assert.equal(shouldGuideClose({ key: 'route_end', modeAtOpen: 'REPLAY' }, { mode: 'IDLE', estopHw: false }), true)
  assert.equal(shouldGuideClose({ key: 'route_end', modeAtOpen: 'REPLAY' }, { mode: 'REPLAY', estopHw: false }), false)
})

test('自動クローズ: estop_held_at_boot は物理非常停止が離されたら閉じる', () => {
  const guide = { key: 'estop_held_at_boot', modeAtOpen: 'INIT' }
  assert.equal(shouldGuideClose(guide, { mode: 'INIT', estopHw: true }), false)
  assert.equal(shouldGuideClose(guide, { mode: 'INIT', estopHw: false }), true)
})
