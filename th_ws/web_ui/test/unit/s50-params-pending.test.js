// SG-B8: 効いている値と保存済み上書きの差分表示（ros/useParamsOverrides.js
// の pendingDiff）。差があるものだけを names 順に抜き出す。
import test from 'node:test'
import assert from 'node:assert/strict'
import { pendingDiff } from '../../src/ros/paramsPending.js'

const NAMES = ['speed_preset_low', 'speed_preset_mid', 'speed_preset_high']

test('差が無いときは空（再起動の必要なし）', () => {
  const eff = { speed_preset_low: 0.27, speed_preset_mid: 0.55, speed_preset_high: 1.0 }
  assert.deepEqual(pendingDiff(NAMES, eff, {}), [])
})

test('保存済みでまだ効いていない値だけを抜き出す', () => {
  const eff = { speed_preset_low: 0.27, speed_preset_mid: 0.55, speed_preset_high: 1.0 }
  const pending = { speed_preset_mid: 0.6 }
  assert.deepEqual(pendingDiff(NAMES, eff, pending), [
    { name: 'speed_preset_mid', effective: 0.55, pending: 0.6 },
  ])
})

test('pending が null でも落ちない', () => {
  const eff = { speed_preset_low: 0.27 }
  assert.deepEqual(pendingDiff(['speed_preset_low'], eff, null), [])
  assert.deepEqual(pendingDiff(['speed_preset_low'], eff, undefined), [])
})

test('前回起動時に適用済みの値は pending に入らない（サーバ側の約束）', () => {
  // /params/get が適用済みを pending から外して返すので、ここでは
  // pending に残ったものは全部「次回から」と表示してよい。
  const eff = { speed_preset_low: 0.3, speed_preset_mid: 0.55, speed_preset_high: 1.0 }
  const pending = { speed_preset_low: 0.35 }
  const diffs = pendingDiff(NAMES, eff, pending)
  assert.equal(diffs.length, 1)
  assert.equal(diffs[0].effective, 0.3)
  assert.equal(diffs[0].pending, 0.35)
})
