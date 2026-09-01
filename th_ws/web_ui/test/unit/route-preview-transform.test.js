// WS-3 / demo-teach-replay: fitTransform (screens/routePreviewGeom.js) must
// actually fit a data bbox into the canvas with padding, aspect preserved, and
// centered — not be an identity/no-op. This catches the regression where the
// route-preview crop is silently dropped (mutation 2).
import test from 'node:test'
import assert from 'node:assert/strict'
import { fitTransform, ROUTE_PREVIEW_PAD } from '../../src/screens/routePreviewGeom.js'

test('returns null for an empty point set', () => {
  assert.equal(fitTransform([], 600, 380, 20), null)
  assert.equal(fitTransform(null, 600, 380, 20), null)
})

test('scale fills the canvas padded bbox (isotropic, not identity)', () => {
  // A square 10 m x 10 m route in a 600x380 canvas with 20px margin:
  // available draw area is 560 x 340, isotropic scale = min(560/10, 340/10)=34.
  const pts = [
    { x: 0, y: 0 }, { x: 10, y: 10 },
  ]
  const f = fitTransform(pts, 600, 380, 20)
  assert.ok(f, 'fitTransform returned null')
  assert.equal(f.scale, 34)
  assert.equal(f.minX, 0)
  assert.equal(f.minY, 0)
  // bbox must be centered: offX = (600 - 10*34)/2 = 130, offY = (380-10*34)/2 = 20
  assert.equal(f.offX, 130)
  assert.equal(f.offY, 20)
})

test('preserves aspect ratio: scale is the same for both axes (no stretching)', () => {
  // Very wide route (40m x 1m): the limiting axis is width.
  const pts = [{ x: 0, y: 0 }, { x: 40, y: 0 }, { x: 40, y: 1 }, { x: 0, y: 1 }]
  const f = fitTransform(pts, 640, 480, 20)
  assert.ok(f)
  // draw area 600 x 440 -> scale = min(600/40, 440/1) = 15
  assert.equal(f.scale, 15)
  // y extent 1m * 15 = 15px, centered: offY = (480 - 15)/2 = 232.5
  assert.ok(Math.abs(f.offY - 232.5) < 1e-6, `offY=${f.offY}`)
})

test('degenerate single point still renders (small default extent, centered)', () => {
  const f = fitTransform([{ x: 3, y: -2 }], 400, 300, 20)
  assert.ok(f)
  assert.ok(f.scale > 0)
  // small extents mean everything fits on one small box, roughly centered
  const cx = 3, cy = -2
  const px = (cx - f.minX) * f.scale + f.offX
  const pyCanvas = 300 - ((cy - f.minY) * f.scale + f.offY)
  assert.ok(Math.abs(px - 200) < 150, `point should render near canvas center, got x=${px}`)
  assert.ok(Math.abs(pyCanvas - 150) < 150, `point should render near canvas center, got y=${pyCanvas}`)
})