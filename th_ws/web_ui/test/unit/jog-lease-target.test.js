import test from 'node:test'
import assert from 'node:assert/strict'
import { jogLeaseTarget } from '../../src/ros/jogLeaseTarget.js'

test('release tail targets zero even if caller keeps passing a nonzero command', () => {
  assert.deepEqual(jogLeaseTarget(false, { vx: 0.8, wz: -0.6 }), { vx: 0, wz: 0 })
})

test('while operator is holding, use the caller command', () => {
  const cmd = { vx: 0.8, wz: -0.6 }
  assert.equal(jogLeaseTarget(true, cmd), cmd)
})
