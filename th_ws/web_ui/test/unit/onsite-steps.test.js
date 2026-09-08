// brief-onsite-ux UX-1: prepSteps() / testSteps() のユニットテスト。
// 各段の完了条件と currentIndex（「未完了の最初の段」。全部完了なら最後の段）を
// 検証する。ROS / React 不要の純関数なので node --test で直接走る。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  NAV_MODES, jogAllowedForMode, onsiteReasons, prepSteps, testSteps,
} from '../../src/screens/onsiteSteps.js'
import { OP_BUTTON_KINDS } from '../../src/parts/opKinds.js'

test('NAV_MODES は S-21 の移動系モードを持ち、IDLE を含まない', () => {
  assert.deepEqual(
    [...NAV_MODES].sort(),
    ['AT_PANEL', 'HOME_NAV', 'PANEL_NAV', 'SUMMON'].sort(),
  )
  assert.equal(NAV_MODES.includes('IDLE'), false)
})

// ── UX-2-a: ボタンの形の種別 ────────────────────────────────
// 識別は色だけにしないためのクラス。特に「停止」は他と絶対に同じ形にしない
// （theme.css の .btn-stop は丸太枠）。OP_BUTTON_KINDS は node --test から
// 直接 import できる純モジュール（JSX なし）であることも同時に固定する。

test('UX-2-a: OP_BUTTON_KINDS は青塗り・停止・登録・手動・保存・終了を区別する', () => {
  const values = Object.values(OP_BUTTON_KINDS)
  assert.deepEqual(
    new Set(values).size,
    values.length,
    'どの kind も同じクラス名を共有しない',
  )
  assert.equal(new Set(values).has(undefined), false)
})

test('UX-2-a: 「停止」は他のどの kind とも異なるクラスを持つ', () => {
  const stop = OP_BUTTON_KINDS.stop
  assert.ok(stop && stop.startsWith('btn-'))
  for (const [key, value] of Object.entries(OP_BUTTON_KINDS)) {
    if (key !== 'stop') assert.notEqual(value, stop, `${key} が停止と同じクラス`)
  }
})

test('UX-2-a: 進む/停止/登録/手動/保存/終了の 6 kind が揃っている', () => {
  assert.deepEqual(
    Object.keys(OP_BUTTON_KINDS).sort(),
    ['advance', 'finish', 'manual', 'register', 'save', 'stop'].sort(),
  )
})

// ── UX-2-d: 手動操作の可否 ───────────────────────────────────
// attributes.yaml の jog 値からだけ判定する純関数。安全側の設定
// （jog_gate/guards.py）は触らないので、「該当モードでは押せない」を表示する。

const attributesLike = {
  IDLE: { jog: 'denied' },
  INIT: { jog: 'denied' },
  CARRY: { jog: 'denied' },
  ESTOP: { jog: 'denied' },
  CALIB: { jog: 'denied' },
  OPCHECK: { jog: 'denied' },
  PREP: { jog: 'allowed' },
  AT_PANEL: { jog: 'allowed' },
  PANEL_NAV: { jog: 'allowed' },
  SUMMON: { jog: 'allowed' },
  HOME_NAV: { jog: 'allowed' },
  MANUAL: { jog: 'is_drive' },
}

test('UX-2-d: jogAllowedForMode は attributes の jog 値のまま判定する', () => {
  for (const denied of ['IDLE', 'INIT', 'CARRY', 'ESTOP', 'CALIB', 'OPCHECK']) {
    assert.equal(jogAllowedForMode(denied, attributesLike), false, `${denied} は手動不可`)
  }
  for (const allowed of ['PREP', 'AT_PANEL', 'PANEL_NAV', 'SUMMON', 'HOME_NAV', 'MANUAL']) {
    assert.equal(jogAllowedForMode(allowed, attributesLike), true, `${allowed} は手動可`)
  }
})

test('UX-2-d: 未知モードや attributes 無しは「押せる側」に倒す（サーバが拒否を返す）', () => {
  assert.equal(jogAllowedForMode('BOGUS', attributesLike), true)
  assert.equal(jogAllowedForMode('IDLE', null), true)
  assert.equal(jogAllowedForMode(null, attributesLike), true)
  assert.equal(jogAllowedForMode(undefined, {}), true)
})

// pins に PANEL/HOME を両方持たせて destHome/destNextPanel を消し、manual の
// 判定だけを見る（両方のピンが無い場合の挙動は UX-6-b のテーブルで別に見る）。
const PINS_BOTH = [{ kind: 'PANEL' }, { kind: 'HOME' }]

test('UX-2-d: onsiteReasons は手動不可モードにだけ jog_denied を返す', () => {
  assert.deepEqual(onsiteReasons({ mode: 'IDLE', attributes: attributesLike, pins: PINS_BOTH }), { manual: 'jog_denied' })
  assert.deepEqual(onsiteReasons({ mode: 'PREP', attributes: attributesLike, pins: PINS_BOTH }), {})
  assert.deepEqual(onsiteReasons({ mode: 'AT_PANEL', attributes: attributesLike, pins: PINS_BOTH }), {})
  assert.deepEqual(onsiteReasons({ mode: null, attributes: attributesLike, pins: PINS_BOTH }), {})
  // pins/working 無しの既定呼び出し（S20Prep.jsx の実際の呼び方）。pins 既定は
  // [] なので destHome/destNextPanel も一緒に返るが、mode が無いので manual は無い。
  assert.deepEqual(onsiteReasons({}), { destHome: 'no_home_pin', destNextPanel: 'no_panel_pin' })
})

// ── UX-6-b: S-21「次の行き先」3 ボタンのバッジ真偽表 ────────────────────────
// working 中はピンの有無に関係なく working_in_progress（guards.py の
// _goto_allowed は ctx.flags.working を pin_kinds より先に見る）。working で
// なければ、PANEL ピンが無いと destNextPanel、HOME ピンが無いと destHome に
// no_panel_pin/no_home_pin（venue_nav_core.find_home_goal が HOME ピンを探す）。

test('UX-6-b: working 中は 3 ボタンとも working_in_progress（ピンの有無に関係なく）', () => {
  assert.deepEqual(
    onsiteReasons({ mode: 'AT_PANEL', attributes: attributesLike, pins: PINS_BOTH, working: true }),
    { destHome: 'working_in_progress', destNextPanel: 'working_in_progress', destSummonHere: 'working_in_progress' },
  )
  assert.deepEqual(
    onsiteReasons({ mode: 'AT_PANEL', attributes: attributesLike, pins: [], working: true }),
    { destHome: 'working_in_progress', destNextPanel: 'working_in_progress', destSummonHere: 'working_in_progress' },
  )
})

test('UX-6-b: working でなければ PANEL ピン無しで destNextPanel だけに no_panel_pin', () => {
  assert.deepEqual(
    onsiteReasons({ mode: 'AT_PANEL', attributes: attributesLike, pins: [{ kind: 'HOME' }], working: false }),
    { destNextPanel: 'no_panel_pin' },
  )
})

test('UX-6-b: working でなければ HOME ピン無しで destHome だけに no_home_pin', () => {
  assert.deepEqual(
    onsiteReasons({ mode: 'AT_PANEL', attributes: attributesLike, pins: [{ kind: 'PANEL' }], working: false }),
    { destHome: 'no_home_pin' },
  )
})

test('UX-6-b: PANEL/HOME どちらも無ければ両方のバッジが出る', () => {
  assert.deepEqual(
    onsiteReasons({ mode: 'AT_PANEL', attributes: attributesLike, pins: [], working: false }),
    { destHome: 'no_home_pin', destNextPanel: 'no_panel_pin' },
  )
})

test('UX-6-b: PANEL/HOME どちらも在れば working でなくてもバッジは無い', () => {
  assert.deepEqual(
    onsiteReasons({ mode: 'AT_PANEL', attributes: attributesLike, pins: PINS_BOTH, working: false }),
    {},
  )
})

test('UX-6-b: 手動は jog_denied を優先し、SUMMON/WAIT_CLEAR で wait_clear_active', () => {
  // jog_denied なモード（IDLE）では wait_clear_active の条件を満たしていても jog_denied 優先。
  assert.deepEqual(
    onsiteReasons({
      mode: 'IDLE', stateName: 'WAIT_CLEAR', attributes: attributesLike, pins: PINS_BOTH,
    }),
    { manual: 'jog_denied' },
  )
  // SUMMON は jog 許可モードなので、WAIT_CLEAR のときだけ wait_clear_active が出る。
  assert.deepEqual(
    onsiteReasons({
      mode: 'SUMMON', stateName: 'WAIT_CLEAR', attributes: attributesLike, pins: PINS_BOTH,
    }),
    { manual: 'wait_clear_active' },
  )
  assert.deepEqual(
    onsiteReasons({
      mode: 'SUMMON', stateName: 'POINT', attributes: attributesLike, pins: PINS_BOTH,
    }),
    {},
  )
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