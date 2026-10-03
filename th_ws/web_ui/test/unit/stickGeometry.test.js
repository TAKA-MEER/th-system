// DetailedDesign-wp3.md WP-UI-03 §4.2 / §7: stickToCmd()'s 8-direction sector
// logic (5 distinct motion labels), its deadzone, the magnitude normalization
// above the deadzone, and rampToward()'s rate-limit. This is the geometry
// tuned on the real robot; the value assertions here pin down that the
// relocated pure module still behaves exactly like the old App.jsx version
// (the design forbids changing stickToCmd()'s behavior).
import test from 'node:test'
import assert from 'node:assert/strict'
import { stickToCmd, rampToward, scaleJogCmd } from '../../src/parts/stickGeometry.js'

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

// Arc (diagonal) -> both, spin arc-scaled.
// W-07 受け入れで JOG_ARC_ANG_SCALE は 0.5 → 0.275（= 0.5 × v_jog_max / w_jog_max）。
test('arc: diagonal combines vn and scaled spin', () => {
  const r = stickToCmd(1, 1, 1) // 45deg: right-forward
  assert.equal(r.label, 'arc')
  assert.ok(r.vn > 0)
  assert.ok(r.wn < 0)
  // vz/wn ratio must reflect the arc scale: at a 45deg diagonal both
  // branches carry equal magnitude m, so |wn| = 0.275 * vn.
  assert.ok(Math.abs(Math.abs(r.wn / r.vn) - 0.275) < 1e-9)
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

// WS-9T: scaleJogCmd leaves a pure in-place turn unscaled by the speed preset
// so a low forward preset doesn't also slow cornering; everything else keeps
// scaling by speedPct.
test('scaleJogCmd: pure turn ignores speedPct (wz = wn), vx=0', () => {
  const r = scaleJogCmd({ vn: 0, wn: -1 }, 0.15)
  assert.deepEqual(r, { vx: 0, wz: -1 })
  const half = scaleJogCmd({ vn: 0, wn: 0.4 }, 0.15)
  assert.deepEqual(half, { vx: 0, wz: 0.4 })
})

test('scaleJogCmd: forward/reverse/arc still scale both axes by speedPct', () => {
  assert.deepEqual(scaleJogCmd({ vn: 1, wn: 0 }, 0.3), { vx: 0.3, wz: 0 })
  assert.deepEqual(scaleJogCmd({ vn: -1, wn: 0 }, 0.5), { vx: -0.5, wz: 0 })
  // arc: vn and wn both non-zero -> not a pure turn -> both * speedPct
  const arc = scaleJogCmd({ vn: 1, wn: -0.5 }, 0.4)
  assert.ok(Math.abs(arc.vx - 0.4) < 1e-9)
  assert.ok(Math.abs(arc.wz - -0.2) < 1e-9)
})

test('scaleJogCmd: deadzone {0,0} stays {0,0}', () => {
  assert.deepEqual(scaleJogCmd({ vn: 0, wn: 0 }, 0.55), { vx: 0, wz: 0 })
})

// W-07 受け入れ: 緩旋回（arc / rev_arc）の実速度は変更前と一致すること。
// 変更前の実値（preset が m/s だった時代。JOG_ARC_ANG_SCALE=0.5。全倒し斜め）:
//   低速 0.15: vx 0.15 / wz ∓0.075 ／ 中速 0.30: vx 0.30 / wz ∓0.15 ／
//   高速 0.55: vx 0.55 / wz ∓0.275（曲率半径 2.0 m）
// 変更後は stickToCmd の wn が 0.275（= 0.5 × v_jog_max / w_jog_max）になり、
// jog_gate が ×(v_jog_max=0.55, w_jog_max=1.0) する。ここでは比率段の出力を
// 上限で戻して旧実値と比べ、±2% 以内を要求する（上限掛け自体は jog_gate 側の
// ScaledByLimits / test_ratio_scaling が縛る）。天井値（0.55 / 1.0）は
// registry.yaml の v_jog_max / w_jog_max と同じ。変えたらこのテストが赤になる
// のが正しい（実速度が変わるため）。
test('arc keeps pre-change physical speed within 2% (all presets)', () => {
  const V_JOG_MAX = 0.55
  const W_JOG_MAX = 1.0
  const arc = stickToCmd(1, 1, 1) // full right-forward diagonal
  assert.equal(arc.label, 'arc')
  assert.ok(Math.abs(arc.vn - 1) < 1e-9)
  assert.ok(Math.abs(arc.wn - -0.275) < 1e-9)
  const cases = [
    // [speedPct, oldVx, oldWz]
    [0.27, 0.15, -0.075],  // low
    [0.55, 0.30, -0.15],   // mid
    [1.0, 0.55, -0.275],   // high
  ]
  for (const [sp, oldVx, oldWz] of cases) {
    const s = scaleJogCmd(arc, sp)
    const vx = s.vx * V_JOG_MAX
    const wz = s.wz * W_JOG_MAX
    assert.ok(Math.abs(vx - oldVx) / oldVx < 0.02,
      `preset ${sp}: vx=${vx} vs pre-change ${oldVx}`)
    assert.ok(Math.abs(wz - oldWz) / Math.abs(oldWz) < 0.02,
      `preset ${sp}: wz=${wz} vs pre-change ${oldWz}`)
  }
})

test('rev_arc keeps pre-change ratio (same constant as arc)', () => {
  const r = stickToCmd(1, -1, 1) // 135deg (right-rear), full throw
  assert.equal(r.label, 'rev_arc')
  assert.ok(Math.abs(r.vn - -1) < 1e-9)
  assert.ok(Math.abs(r.wn - -0.275) < 1e-9)
})

// rampToward clamps the step change.
test('rampToward: clamps within maxDelta and reaches target exactly', () => {
  assert.equal(rampToward(0, 10, 2), 2)
  assert.equal(rampToward(0, 10, 3), 3)
  assert.equal(rampToward(10, 0, 2), 8)
  assert.equal(rampToward(5, 6, 2), 6) // within delta -> target
  assert.equal(rampToward(8, 9, 10), 9)
})
