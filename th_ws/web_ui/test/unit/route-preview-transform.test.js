// WS-3 / demo-teach-replay: fitTransform (screens/routePreviewGeom.js) must
// actually fit a data bbox into the canvas with padding, aspect preserved, and
// centered — not be an identity/no-op. This catches the regression where the
// route-preview crop is silently dropped (mutation 2).
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  fitTransform, ROUTE_PREVIEW_PAD,
  centeredTransform, ROUTE_PREVIEW_HALF_SPAN_M,
} from '../../src/screens/routePreviewGeom.js'

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

// ---------------------------------------------------------------- WS-6.4: centeredTransform ----

const W = 600, H = 380

test('centeredTransform maps the center to the canvas centre', () => {
  const f = centeredTransform({ x: 0, y: 0 }, ROUTE_PREVIEW_HALF_SPAN_M, W, H)
  assert.ok(f)
  // scale short-side: min(600,380) / (2*7) = 380/14
  assert.equal(f.scale, Math.min(W, H) / (2 * ROUTE_PREVIEW_HALF_SPAN_M))
  assert.equal(f.minX, -7)
  assert.equal(f.minY, -7)
  // px/py math matches RoutePreview
  const px = (p) => (p.x - f.minX) * f.scale + f.offX
  const py = (p) => H - ((p.y - f.minY) * f.scale + f.offY)
  const cx = px({ x: 0, y: 0 })
  const cy = py({ x: 0, y: 0 })
  assert.ok(Math.abs(cx - W / 2) < 1e-6, `centre x should be W/2, got ${cx}`)
  assert.ok(Math.abs(cy - H / 2) < 1e-6, `centre y should be H/2, got ${cy}`)
})

test('centeredTransform fits ±halfSpan to the SHORT canvas edge', () => {
  const f = centeredTransform({ x: 0, y: 0 }, ROUTE_PREVIEW_HALF_SPAN_M, W, H)
  assert.ok(f)
  const px = (p) => (p.x - f.minX) * f.scale + f.offX
  const py = (p) => H - ((p.y - f.minY) * f.scale + f.offY)
  // short side (height): y = center ±7 lands exactly on the two edges.
  // Canvas y grows downward, so world y=-7 (below center) draws at the BOTTOM.
  assert.ok(Math.abs(py({ x: 0, y: -7 }) - H) < 1e-6, 'y=-7 should hit the bottom edge')
  assert.ok(Math.abs(py({ x: 0, y: 7 })) < 1e-6, 'y=+7 should hit the top edge')
  // long side is inset (the view is fit to the short side, aspect preserved)
  assert.ok(px({ x: -7 }) > 0 && px({ x: 7 }) < W, 'x=±7 should stay inside the canvas')
})

test('centeredTransform scale is invariant to the center (fixed zoom, no re-fit)', () => {
  const a = centeredTransform({ x: 0, y: 0 }, ROUTE_PREVIEW_HALF_SPAN_M, W, H)
  const b = centeredTransform({ x: 3.5, y: -2.7 }, ROUTE_PREVIEW_HALF_SPAN_M, W, H)
  assert.ok(a && b)
  assert.equal(a.scale, b.scale)
  // ...and neither depends on the route data at all (only center + halfSpan).
  assert.equal(
    centeredTransform({ x: 3.5, y: -2.7 }, ROUTE_PREVIEW_HALF_SPAN_M, W, H).scale,
    Math.min(W, H) / (2 * ROUTE_PREVIEW_HALF_SPAN_M))
})

test('centeredTransform returns null for an invalid center', () => {
  assert.equal(centeredTransform(null, ROUTE_PREVIEW_HALF_SPAN_M, W, H), null)
  assert.equal(centeredTransform({ x: NaN, y: 0 }, ROUTE_PREVIEW_HALF_SPAN_M, W, H), null)
  assert.equal(centeredTransform({ x: 0, y: Infinity }, ROUTE_PREVIEW_HALF_SPAN_M, W, H), null)
  assert.equal(centeredTransform({ x: 0, y: 0 }, 0, W, H), null)
})