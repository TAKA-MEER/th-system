// WS-9X: resolveScreen の settingsOpen 分岐。S-50 設定は FSM のモードではなく
// S-01 のサブ画面なので、「本来 S-01 を出す」ときだけ S-50 に差し替える。
// 動作系モード中は settingsOpen を無視する（走行中に設定画面がかぶらない）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { resolveScreen, MODE_TO_SCREEN } from '../../src/screens/screenRouting.js'

const base = { testScreen: undefined, passedConnect: true, mode: 'IDLE', settingsOpen: false }

test('settingsOpen かつ本来 S-01 なら S-50', () => {
  assert.equal(resolveScreen({ ...base, settingsOpen: true }), 'S50')
})

test('settingsOpen=false なら従来どおり S-01', () => {
  assert.equal(resolveScreen({ ...base }), 'S01')
})

test('動作系モード中は settingsOpen を無視して従来の画面', () => {
  for (const [mode, screen] of Object.entries(MODE_TO_SCREEN)) {
    assert.equal(
      resolveScreen({ ...base, mode, settingsOpen: true }), screen,
      `mode=${mode} は settingsOpen でも ${screen} のまま`)
  }
})

test('接続確認を通っていなければ settingsOpen でも S-00', () => {
  assert.equal(resolveScreen({ ...base, passedConnect: false, settingsOpen: true }), 'S00')
})

test('DRIVE_S11（e2e 合成画面）は settingsOpen より優先', () => {
  assert.equal(
    resolveScreen({ ...base, testScreen: 'DRIVE_S11', settingsOpen: true }), 'DRIVE_S11')
})

// WP-UI-06: PREP モードは S-20 試験準備。目盛りは S-01 のサブ画面である S-50 設定
// に置き換わらない（動作系モードの従来挙動。上のループに吸収済みだが、
// 失敗時に行が分かるよう明示もしておく）。
test('PREP なら S-20（settingsOpen でも）', () => {
  assert.equal(resolveScreen({ ...base, mode: 'PREP' }), 'S20')
  assert.equal(resolveScreen({ ...base, mode: 'PREP', settingsOpen: true }), 'S20')
})

// WP-UI-07: S-21 試験。PANEL_NAV / SUMMON / AT_PANEL / HOME_NAV の 4 モードが対象
// （brief-UI-S21 §2.1）。動作系モードなので settingsOpen でも置き換わらない。
test('PANEL_NAV / SUMMON / AT_PANEL / HOME_NAV なら S-21（settingsOpen でも）', () => {
  for (const mode of ['PANEL_NAV', 'SUMMON', 'AT_PANEL', 'HOME_NAV']) {
    assert.equal(resolveScreen({ ...base, mode }), 'S21', `mode=${mode} は S-21`)
    assert.equal(resolveScreen({ ...base, mode, settingsOpen: true }), 'S21',
      `mode=${mode} は settingsOpen でも S-21`)
  }
})

// brief-UI-S21 §2.1: IDLE は MODE_TO_SCREEN に入れない。S-21 をモードに関係なく
// 開く導線はテスト専用の TEST_SCREEN 経由（main.jsx 側）であり、screenRouting に
// IDLE 分岐を持たせるのは混線のもとになるので禁止。
test('IDLE は S-21 でなく S-01 のまま（screenRouting に IDLE 分岐を足さない）', () => {
  assert.equal(resolveScreen({ ...base, mode: 'IDLE' }), 'S01')
})

// brief-UI-S21-entry §4.1: onsiteTestOpen。S-50 と同じ「S-01 のサブ画面」方式で、
// 動作系モードが最優先。settingsOpen より onsiteTestOpen を先に見る。
test('onsiteTestOpen && IDLE なら S-21（S-01 のサブ画面として）', () => {
  assert.equal(resolveScreen({ ...base, mode: 'IDLE', onsiteTestOpen: true }), 'S21')
})

test('onsiteTestOpen でも PANEL_NAV なら S-21（モード優先でも結果は同じ）', () => {
  assert.equal(resolveScreen({ ...base, mode: 'PANEL_NAV', onsiteTestOpen: true }), 'S21')
})

test('onsiteTestOpen でも MANUAL なら S-11（動作系モードが優先）', () => {
  assert.equal(resolveScreen({ ...base, mode: 'MANUAL', onsiteTestOpen: true }), 'S11')
})

test('onsiteTestOpen と settingsOpen が両方立ったら S-21（onsiteTestOpen 優先）', () => {
  assert.equal(
    resolveScreen({ ...base, mode: 'IDLE', onsiteTestOpen: true, settingsOpen: true }),
    'S21')
  assert.equal(
    resolveScreen({ ...base, mode: 'IDLE', onsiteTestOpen: false, settingsOpen: true }),
    'S50')
})
