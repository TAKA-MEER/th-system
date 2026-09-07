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
