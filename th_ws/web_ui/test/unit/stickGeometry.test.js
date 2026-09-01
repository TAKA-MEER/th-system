// DetailedDesign-wp3.md WP-UI-03 §4.2 / §7: stickToCmd()'s 8-direction sector
// logic (5 distinct motion labels), its deadzone, the magnitude normalization
// above the deadzone, and rampToward()'s rate-limit. This is the geometry
// tuned on the real robot; the value assertions here pin down that the
// relocated pure module still behaves exactly like the old App.jsx version
// (the design forbids changing stickToCmd()'s behavior).
import test from 'node:test'
import assert from 'node:assert/strict'
import { stickToCmd, rampToward } from '../../src/parts/stickGeometry.js'

// Forward (top) -> vn>0, no spin.
test('forward: straight up gives vn>=0 and wz=0', () => {
  const r = stickToCmd(0, 1, 1)
  assert.equal(r.label, 'forward')
  assert.equal(r.wn, 0)
  assert.ok(r.vn > 0)
})

// Turn (side) -> spin only.
test('right spin: wz<0 (turn right = negative), vn=0', () => {
  const r = stickToCmd(1, 0, 1)
  assert.equal(r.label, 'turn')
  assert.equal(r.vn, 0)
  assert.ok(r.wn < 0)
})

test('left spin: wz>0 mirror', () => {
  const r = stickToCmd(-1, 0, 1)
  assert.equal(r.label, 'turn')
  assert.ok(r.wn > 0)
})

// Arc (diagonal) -> both, spin half-scaled.
test('arc: diagonal combines vn and scaled spin', () => {
  const r = stickToCmd(1, 1, 1) // 45deg: right-forward
  assert.equal(r.label, 'arc')
  assert.ok(r.vn > 0)
  assert.ok(r.wn < 0)
  // vz/wn ratio must reflect the 0.5 arc scale: at a 45deg diagonal both
  // branches carry equal magnitude m, so |wn| = 0.5 * vn.
  assert.ok(Math.abs(Math.abs(r.wn / r.vn) - 0.5) < 1e-9)
})

// Reverse (bottom) -> vn<0.
test('reverse: straight down gives vn<0 and wz=0', () => {
  const r = stickToCmd(0, -1, 1)
  assert.equal(r.label, 'reverse')
  assert.ok(r.vn < 0)
  assert.equal(r.wn, 0)
})

// Backward-arc (bottom-right diagonal).
test('rev_arc: backward diagonal gives reverse with scaled spin', () => {
  const r = stickToCmd(1, -1, 1) // 135deg (right-rear)
  assert.equal(r.label, 'rev_arc')
  assert.ok(r.vn < 0)
  assert.ok(r.wn < 0)
})

// Deadzone: below 0.15 -> stop, independent of direction.
test('deadzone: inside the deadzone ratio returns a stop', () => {
  const r = stickToCmd(1, 0, 0.10)
  assert.deepEqual(r, { vn: 0, wn: 0, label: null })
})

// Magnitude: m normalizes (len - deadzone) / (1 - deadzone) to 0..1.
test('magnitude: at full throw m=1, just past deadzone m is near 0', () => {
  const full = stickToCmd(0, 1, 1)
  assert.ok(Math.abs(full.vn - 1) < 1e-9)
  const near = stickToCmd(0, 1, 0.16)
  assert.ok(near.vn > 0)
  assert.ok(near.vn < 0.2)
})

// rampToward clamps the step change.
test('rampToward: clamps within maxDelta and reaches target exactly', () => {
  assert.equal(rampToward(0, 10, 2), 2)
  assert.equal(rampToward(0, 10, 3), 3)
  assert.equal(rampToward(10, 0, 2), 8)
  assert.equal(rampToward(5, 6, 2), 6) // within delta -> target
  assert.equal(rampToward(8, 9, 10), 9)
})
