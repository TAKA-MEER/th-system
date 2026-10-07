// 1b-15 SG-B10: 自動ブレーキの切替・確認・接近警告の判断（shell/autoBrake.js）。
// 受理は機体が決める。ここは「押せる見た目か」「確認を挟むか」「警告を出すか」だけ。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  canToggleAutoBrake, autoBrakeNeedsConfirm, showApproachWarning,
} from '../../src/shell/autoBrake.js'
import attributes from '../../src/generated/attributes.json' with { type: 'json' }

test('manual modes can toggle; autonomous modes cannot (from attributes.json)', () => {
  assert.equal(canToggleAutoBrake({ mode: 'MANUAL' }, attributes), true)
  assert.equal(canToggleAutoBrake({ mode: 'TEACH_MANUAL' }, attributes), true)
  assert.equal(canToggleAutoBrake({ mode: 'FOLLOW' }, attributes), false)
  assert.equal(canToggleAutoBrake({ mode: 'IDLE' }, attributes), false)
})

test('jog intervention makes an autonomous mode toggleable', () => {
  assert.equal(canToggleAutoBrake({ mode: 'FOLLOW', jog_active: true }, attributes), true)
})

test('stale link or unknown mode disables the toggle (fail-safe)', () => {
  assert.equal(canToggleAutoBrake({ mode: 'MANUAL' }, attributes, true), false)
  assert.equal(canToggleAutoBrake({ mode: null }, attributes), false)
  assert.equal(canToggleAutoBrake(null, attributes), false)
  assert.equal(canToggleAutoBrake({ mode: 'NOPE' }, attributes), false)
})

test('only ON -> OFF asks for confirmation', () => {
  assert.equal(autoBrakeNeedsConfirm(true, false), true)
  assert.equal(autoBrakeNeedsConfirm(false, true), false)
  assert.equal(autoBrakeNeedsConfirm(true, true), false)
  assert.equal(autoBrakeNeedsConfirm(false, false), false)
})

test('approach warning shows only when brake is OFF and the limiter says so', () => {
  assert.equal(showApproachWarning({ approach_warning: true }, false), true)
  assert.equal(showApproachWarning({ approach_warning: false }, false), false)
  assert.equal(showApproachWarning({ approach_warning: true }, true), false)
  assert.equal(showApproachWarning({ approach_warning: true }, undefined), false)
  assert.equal(showApproachWarning({ approach_warning: true }, null), false)
  assert.equal(showApproachWarning(null, false), false)
  assert.equal(showApproachWarning({}, false), false)
})
