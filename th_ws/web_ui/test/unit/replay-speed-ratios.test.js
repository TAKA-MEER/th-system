// WS-9T: ReplaySpeedControl の 3 比率が「低 < 中 < 高」で distinct、かつ 0..1。
// 実速度への換算は replay_runner（route_replay_core.scale_replay_params）が持つので
// ここが検証するのは比率の順序と範囲だけ。
import test from 'node:test'
import assert from 'node:assert/strict'
import { REPLAY_SPEED_RATIOS } from '../../src/parts/replaySpeed.js'

test('replay speed ratios are distinct and increasing low < mid < high', () => {
  const { low, mid, high } = REPLAY_SPEED_RATIOS
  assert.ok(low < mid && mid < high, `expected low < mid < high, got ${low}/${mid}/${high}`)
  assert.equal(new Set([low, mid, high]).size, 3)
})

test('replay speed ratios stay within [0, 1]', () => {
  for (const r of Object.values(REPLAY_SPEED_RATIOS)) {
    assert.ok(r >= 0 && r <= 1, `ratio ${r} out of [0,1]`)
  }
})
