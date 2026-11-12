// A-15: W-6 と自律系の画面の自動ブレーキ切替の判断（shell/autoBrakeToggle.js）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { devAutoBrakeEffective, autoBrakeCardVisible, canToggleWithDev } from '../../src/shell/autoBrakeToggle.js'

test('開発モードの項目 auto_brake が effective に明示されているときだけ真', () => {
  assert.equal(devAutoBrakeEffective({ effective: { auto_brake: true } }), true)
  assert.equal(devAutoBrakeEffective({ effective: { auto_brake: false } }), false)
  // 選んだ（ignore）だけで effective に無い・未受信・形が違うものは偽（機体側と同じ）
  assert.equal(devAutoBrakeEffective({ ignore: { auto_brake: true }, effective: {} }), false)
  assert.equal(devAutoBrakeEffective(null), false)
  assert.equal(devAutoBrakeEffective(undefined), false)
  assert.equal(devAutoBrakeEffective({ effective: { auto_brake: 'true' } }), false)
})

test('自律系の画面の切替カードは項目を選んでいるときだけ', () => {
  assert.equal(autoBrakeCardVisible({ effective: { link: true } }), false)
  assert.equal(autoBrakeCardVisible({ effective: { auto_brake: true } }), true)
})

test('押せるか: 手動系・ジョグ中（canManual）か開発モード。古い通信・モード未受信は不可', () => {
  const st = { mode: 'PREP' }
  assert.equal(canToggleWithDev(true, st, false, false), true)
  assert.equal(canToggleWithDev(false, st, false, true), true)
  assert.equal(canToggleWithDev(false, st, false, false), false)
  assert.equal(canToggleWithDev(true, st, true, true), false)
  assert.equal(canToggleWithDev(true, { mode: null }, false, true), false)
  assert.equal(canToggleWithDev(true, null, false, true), false)
})
