// 1b-6 (SG-B2): /system/effect の振り分けと W-3 の自動クローズ条件を縛る。
// 変異: guide を捨てる／dest 判定を外す／自動クローズを外すと赤くなること。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  dispatchEffect, shouldGuideClose, shouldOpenSaveAsk, isUnsavedRecording,
  resetEffectWarningsForTest, stampToMs,
} from '../../src/shell/effectDispatch.js'

function guideMsg(key, dest = 'WebUI', stampSec = null) {
  const msg = { name: 'guide', dest, args_json: JSON.stringify({ key }) }
  if (stampSec != null) msg.header = { stamp: { sec: stampSec, nanosec: 0 }, frame_id: '' }
  return msg
}

test('guide で帯の状態が立つ (stamp 付き・無し)', () => {
  resetEffectWarningsForTest()
  assert.deepEqual(
    dispatchEffect(guideMsg('summon_clear_timeout')),
    { kind: 'guide', key: 'summon_clear_timeout', stampMs: null },
  )
  assert.deepEqual(
    dispatchEffect(guideMsg('summon_clear_timeout', 'WebUI', 100)),
    { kind: 'guide', key: 'summon_clear_timeout', stampMs: 100000 },
  )
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

test('今回は実装しない effect は何もしない (open_window/close_window/show_resume/enable_main_menu/offer 系)', () => {
  resetEffectWarningsForTest()
  const warnings = []
  const orig = console.warn
  console.warn = (m) => warnings.push(m)
  try {
    for (const name of [
      'open_window', 'close_window', 'show_resume',
      'enable_main_menu',
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

test('stamp の読み取り (sec/secs 両対応・0 と欠落は null)', () => {
  assert.equal(stampToMs({ sec: 100, nanosec: 5000000 }), 100005)
  assert.equal(stampToMs({ secs: 100, nsecs: 0 }), 100000)
  assert.equal(stampToMs({ sec: 0, nanosec: 0 }), null)
  assert.equal(stampToMs(null), null)
  assert.equal(stampToMs(undefined), null)
})

test('自動クローズ: guide より後に入ったモードに変わったら閉じる', () => {
  const guide = { key: 'home_arrived', modeAtOpen: 'HOME_NAV', stampMs: 100000, seenPressed: false }
  // 同じ遷移の effect→state (同時刻。審査の指摘 1) では閉じない
  assert.equal(shouldGuideClose(guide, { mode: 'AT_HOME', sinceMs: 100000, estopHw: false }), false)
  // 後の遷移で入ったモードでは閉じる
  assert.equal(shouldGuideClose(guide, { mode: 'IDLE', sinceMs: 200000, estopHw: false }), true)
  // 同じモードでは閉じない
  assert.equal(shouldGuideClose(guide, { mode: 'HOME_NAV', sinceMs: 200000, estopHw: false }), false)
})

test('自動クローズ: 同じ遷移で since が stamp+5ms でも閉じない・+1500ms のモード変化では閉じる', () => {
  const guide = { key: 'home_arrived', modeAtOpen: 'HOME_NAV', stampMs: 100000, seenPressed: false }
  assert.equal(shouldGuideClose(guide, { mode: 'AT_HOME', sinceMs: 100005, estopHw: false }), false)
  assert.equal(shouldGuideClose(guide, { mode: 'IDLE', sinceMs: 101500, estopHw: false }), true)
})

test('自動クローズ: stamp／since が比較不能ならモードでは閉じない', () => {
  const noStamp = { key: 'home_arrived', modeAtOpen: 'HOME_NAV', stampMs: null, seenPressed: false }
  assert.equal(shouldGuideClose(noStamp, { mode: 'IDLE', sinceMs: 200000, estopHw: false }), false)
  const stamped = { key: 'home_arrived', modeAtOpen: 'HOME_NAV', stampMs: 100000, seenPressed: false }
  assert.equal(shouldGuideClose(stamped, { mode: 'IDLE', sinceMs: null, estopHw: false }), false)
})

test('自動クローズ: estop_held_at_boot は押されているのを一度見てから離れたら閉じる', () => {
  // guide 到着時にまだ false (状態が後から届く。審査の指摘 2) では閉じない
  const unseen = { key: 'estop_held_at_boot', modeAtOpen: 'INIT', stampMs: 100000, seenPressed: false }
  assert.equal(shouldGuideClose(unseen, { mode: 'INIT', sinceMs: 50000, estopHw: false }), false)
  // 押されているのを見た後は出たまま
  const seen = { ...unseen, seenPressed: true }
  assert.equal(shouldGuideClose(seen, { mode: 'INIT', sinceMs: 50000, estopHw: true }), false)
  // 離されたら閉じる
  assert.equal(shouldGuideClose(seen, { mode: 'INIT', sinceMs: 50000, estopHw: false }), true)
})

// 1b-7 (SG-B3): ask_save／ask_save_if_unsaved の振り分けと W-4 を開く条件。
// 変異: ask を捨てる／route_id を落とす／未保存判定を外すと赤くなること。
function askMsg(name, routeId = null) {
  const args = routeId ? { route_id: routeId } : {}
  return { name, dest: 'WebUI', args_json: JSON.stringify(args) }
}

test('ask_save／ask_save_if_unsaved は route_id 付きで振り分けられる', () => {
  resetEffectWarningsForTest()
  assert.deepEqual(
    dispatchEffect(askMsg('ask_save', 'demo-route')),
    { kind: 'ask_save', routeId: 'demo-route', stampMs: null },
  )
  assert.deepEqual(
    dispatchEffect(askMsg('ask_save_if_unsaved')),
    { kind: 'ask_save_if_unsaved', routeId: null, stampMs: null },
  )
  // dest が WebUI でなければ見ない。
  assert.deepEqual(
    dispatchEffect({ name: 'ask_save', dest: 'route_recorder', args_json: '{}' }),
    { kind: 'ignore' },
  )
})

test('未保存の教示記録があるときだけ W-4 を開く', () => {
  const unsaved = { points: 12, saved: false }
  assert.equal(
    shouldOpenSaveAsk({ kind: 'ask_save', routeId: null }, unsaved), true)
  assert.equal(
    shouldOpenSaveAsk({ kind: 'ask_save_if_unsaved', routeId: null }, unsaved), true)
  // 保存済み・空・未到来では開かない（ゴミの経路を作らない・無いものを問わない）。
  assert.equal(
    shouldOpenSaveAsk({ kind: 'ask_save' }, { points: 12, saved: true }), false)
  assert.equal(
    shouldOpenSaveAsk({ kind: 'ask_save_if_unsaved' }, { points: 0, saved: false }), false)
  assert.equal(shouldOpenSaveAsk({ kind: 'ask_save_if_unsaved' }, null), false)
  assert.equal(shouldOpenSaveAsk({ kind: 'ask_save' }, null), false)
  // 再生側の status（current.id あり）は記録の未保存とみなさない。
  assert.equal(
    shouldOpenSaveAsk({ kind: 'ask_save_if_unsaved' },
      { points: 30, saved: false, current: { id: 'r1' } }), false)
  // ask 以外では開かない。
  assert.equal(shouldOpenSaveAsk({ kind: 'guide' }, unsaved), false)
  assert.equal(shouldOpenSaveAsk({ kind: 'ignore' }, unsaved), false)
  assert.equal(shouldOpenSaveAsk(null, unsaved), false)
})

test('isUnsavedRecording: saved=true・current.id あり・0 点は未保存ではない', () => {
  assert.equal(isUnsavedRecording({ points: 5, saved: false }), true)
  assert.equal(isUnsavedRecording({ points: 5, saved: true }), false)
  assert.equal(isUnsavedRecording({ points: 0, saved: false }), false)
  assert.equal(isUnsavedRecording(null), false)
  assert.equal(isUnsavedRecording(undefined), false)
})
