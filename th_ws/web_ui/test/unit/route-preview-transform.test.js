// WS-3 / demo-teach-replay: fitTransform (screens/routePreviewGeom.js) must
// actually fit a data bbox into the canvas with padding, aspect preserved, and
// centered — not be an identity/no-op. This catches the regression where the
// route-preview crop is silently dropped (mutation 2).
import test from 'node:test'
import assert from 'node:assert/strict'
import { fitTransform, ROUTE_PREVIEW_PAD, centeredTransform, ROUTE_PREVIEW_HALF_SPAN_M, scanToPoints, SCAN_FALLBACK_MAX_M, mapDestRect, shortestAngleDelta, smoothPose } from '../../src/screens/routePreviewGeom.js'

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
  // scale short-side: min(600,380) / (2 * HALF_SPAN)
  assert.equal(f.scale, Math.min(W, H) / (2 * ROUTE_PREVIEW_HALF_SPAN_M))
  assert.equal(f.minX, -ROUTE_PREVIEW_HALF_SPAN_M)
  assert.equal(f.minY, -ROUTE_PREVIEW_HALF_SPAN_M)
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
  // short side (height): y = center ±halfSpan lands exactly on the two edges.
  // Canvas y grows downward, so world y=-halfSpan (below center) draws at BOTTOM.
  const hs = ROUTE_PREVIEW_HALF_SPAN_M
  assert.ok(Math.abs(py({ x: 0, y: -hs }) - H) < 1e-6, 'y=-halfSpan should hit the bottom edge')
  assert.ok(Math.abs(py({ x: 0, y: hs })) < 1e-6, 'y=+halfSpan should hit the top edge')
  // long side is inset (the view is fit to the short side, aspect preserved)
  assert.ok(px({ x: -hs }) > 0 && px({ x: hs }) < W, 'x=±halfSpan should stay inside the canvas')
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

// ----------------------------------------------------------------- WS-8A: scanToPoints ----

// 前・左・後・右（yaw 0, pose 原点）。
const SCAN4 = {
  angle_min: 0,
  angle_increment: Math.PI / 2,
  ranges: [1, 2, 3, 4],
  range_max: 16,
}
const ORIGIN = { x: 0, y: 0, yaw: 0 }

// coordinate compare with tolerance for trig floating-point noise
function assertPt(p, x, y) {
  assert.ok(Math.abs(p.x - x) < 1e-6, `x=${p.x} != ${x}`)
  assert.ok(Math.abs(p.y - y) < 1e-6, `y=${p.y} != ${y}`)
}

test('scanToPoints converts range/angle to world points (yaw 0, origin)', () => {
  const pts = scanToPoints(SCAN4, ORIGIN)
  assert.equal(pts.length, 4)
  // 前・左・後・右（cos/sin の浮動小数点誤差は収める）
  assertPt(pts[0], 1, 0)
  assertPt(pts[1], 0, 2)
  assertPt(pts[2], -3, 0)
  assertPt(pts[3], 0, -4)
})

test('scanToPoints clips to scanData.range_max', () => {
  const pts = scanToPoints({ ...SCAN4, range_max: 3 }, ORIGIN)
  // r=4 (> range_max) is dropped -> 3 points remain.
  assert.equal(pts.length, 3)
  assertPt(pts[0], 1, 0)
  assertPt(pts[1], 0, 2)
  assertPt(pts[2], -3, 0)
})

test('scanToPoints falls back to fallbackMaxRange when range_max is absent', () => {
  const { range_max, ...noMax } = SCAN4
  void range_max
  // fallback 16: r=4 kept, but r=20 would be dropped.
  assert.equal(scanToPoints(noMax, ORIGIN, 16).length, 4)
  assert.equal(scanToPoints({ ...noMax, ranges: [4, 20] }, ORIGIN, 16).length, 1)
  assert.equal(SCAN_FALLBACK_MAX_M, 16)
})

test('scanToPoints drops NaN / 0 / negative ranges', () => {
  const { range_max, ...noMax } = SCAN4
  void range_max
  const pts = scanToPoints({ ...noMax, ranges: [NaN, 0, -2, 1.5, Infinity] }, ORIGIN, 16)
  // index 3 -> angle 3·(π/2) = 3π/2 (straight down) => world (0, -1.5)
  assert.equal(pts.length, 1)
  assertPt(pts[0], 0, -1.5)
})

test('scanToPoints translates with a non-origin pose (baseToWorld)', () => {
  const pose = { x: 5, y: 3, yaw: Math.PI / 2 }
  // yaw=90°, forward == world +y: a single point straight ahead (localX=2, localY=0)
  // -> world = pose + rotate(2, 0) = (5, 3) + (0, 2) = (5, 5)
  const pts = scanToPoints({ angle_min: 0, angle_increment: 0, ranges: [2], range_max: 16 }, pose)
  assert.deepEqual(pts, [{ x: 5, y: 5 }])
})

test('scanToPoints returns [] when scan or pose is missing', () => {
  assert.deepEqual(scanToPoints(null, ORIGIN), [])
  assert.deepEqual(scanToPoints(SCAN4, null), [])
  assert.deepEqual(scanToPoints({ ...SCAN4, ranges: undefined }, ORIGIN), [])
})

// ----------------------------------------------------------------- WS-8B: mapDestRect ----

const H2 = 380
// 4×4 grid, res 0.5, origin at (-1,-1) -> world x/y ∈ [-1, 1]. Fit: pose centre
// (0,0), halfSpan 10, W=600, H=380 -> scale=19, offX=110, offY=0 (see above).
const MAP_INFO = {
  resolution: 0.5,
  width: 4,
  height: 4,
  origin: { position: { x: -1, y: -1 }, orientation: { z: 0, w: 1 } },
}
const MAP_FIT = centeredTransform({ x: 0, y: 0 }, ROUTE_PREVIEW_HALF_SPAN_M, W, H)

test('mapDestRect places the map world-rect through the centred fit', () => {
  assert.ok(MAP_FIT)
  // px(-1)=281, px(1)=319 -> dw=38. py flips: dy at y1(=1)=171, dh=38.
  assert.deepEqual(mapDestRect(MAP_INFO, MAP_FIT, H2), { dx: 281, dy: 171, dw: 38, dh: 38 })
})

test('mapDestRect flips vertically so the world TOP (y1) is the canvas top', () => {
  assert.ok(MAP_FIT)
  const r = mapDestRect(MAP_INFO, MAP_FIT, H2)
  assert.ok(r)
  // world y1 (=1) sits at dy, world y0 (=-1) at dy+dh — i.e. the map's +y
  // (north) edge is drawn at the smaller canvas y (flip). Swapping y1/y0 makes
  // dy=209 and dh negative, so these two hold independently of the exact rect.
  const py = (wy) => H2 - ((wy - MAP_FIT.minY) * MAP_FIT.scale + MAP_FIT.offY)
  assert.equal(r.dy, py(MAP_INFO.origin.position.y + MAP_INFO.height * MAP_INFO.resolution))
  assert.equal(r.dy + r.dh, py(MAP_INFO.origin.position.y))
})

test('mapDestRect returns null for missing/invalid info or fit', () => {
  assert.equal(mapDestRect(null, MAP_FIT, H2), null)
  assert.equal(mapDestRect(MAP_INFO, null, H2), null)
  assert.equal(mapDestRect({ ...MAP_INFO, resolution: 0 }, MAP_FIT, H2), null)
  assert.equal(mapDestRect({ ...MAP_INFO, width: -4 }, MAP_FIT, H2), null)
  assert.equal(mapDestRect(MAP_INFO, MAP_FIT, 0), null)
})
// ── pose 補間（2026-09-02「経路表示がガクガク動く」対策） ──────────────
// プレビューはロボット中心・固定倍率なので pose が画面全体の位置を決める。
// pose は 10Hz でしか来ないため、間のフレームを補間しないと地図・経路・点群が
// まとめて瞬間移動する。ここが守る性質:
//   1. yaw は必ず「短い方」に回る（179° → -179° を +2° と解釈する）
//   2. alpha=1 で目標に一致し、0 で動かない（境界で壊れない）
//   3. 繰り返し適用すると目標に収束する（発散・振動しない）
test('shortestAngleDelta: 境界をまたぐときは短い方を返す', () => {
  const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-9, `${a} != ${b}`)
  near(shortestAngleDelta(0, 0), 0)
  near(shortestAngleDelta(0, 1), 1)
  // 179° -> -179° は -358° ではなく +2°
  near(shortestAngleDelta(Math.PI - 0.01, -Math.PI + 0.01), 0.02)
  // 逆向きも同様
  near(shortestAngleDelta(-Math.PI + 0.01, Math.PI - 0.01), -0.02)
  // 返り値は必ず -π..π
  for (const [f, t] of [[0, 10], [0, -10], [5, -5], [-3, 3]]) {
    const d = shortestAngleDelta(f, t)
    assert.ok(d >= -Math.PI - 1e-9 && d <= Math.PI + 1e-9, `範囲外: ${d}`)
  }
})

test('smoothPose: alpha=1 で目標に一致し、alpha=0 で動かない', () => {
  const cur = { x: 0, y: 0, yaw: 0 }
  const tgt = { x: 2, y: -4, yaw: 1.2 }
  const at1 = smoothPose(cur, tgt, 1)
  assert.ok(Math.abs(at1.x - 2) < 1e-9 && Math.abs(at1.y + 4) < 1e-9)
  assert.ok(Math.abs(at1.yaw - 1.2) < 1e-9)
  const at0 = smoothPose(cur, tgt, 0)
  assert.ok(Math.abs(at0.x) < 1e-9 && Math.abs(at0.y) < 1e-9 && Math.abs(at0.yaw) < 1e-9)
})

test('smoothPose: current が無ければ目標をそのまま返す（初回受信）', () => {
  const tgt = { x: 1, y: 2, yaw: 0.5, frame: 'map' }
  const r = smoothPose(null, tgt, 0.25)
  assert.equal(r.x, 1); assert.equal(r.y, 2); assert.equal(r.yaw, 0.5)
  assert.equal(r.frame, 'map', 'frame 等の付随フィールドが落ちている')
})

test('smoothPose: target が無ければ current を保つ（pose 途絶で消さない）', () => {
  const cur = { x: 1, y: 2, yaw: 0.5 }
  assert.deepEqual(smoothPose(cur, null, 0.25), cur)
  assert.equal(smoothPose(null, null, 0.25), null)
})

test('smoothPose: 繰り返すと目標に収束する（発散しない）', () => {
  let p = { x: 0, y: 0, yaw: 0 }
  const tgt = { x: 5, y: -3, yaw: 2.0 }
  for (let i = 0; i < 200; i++) p = smoothPose(p, tgt, 0.25)
  assert.ok(Math.hypot(p.x - 5, p.y + 3) < 1e-6, `位置が収束しない: ${p.x},${p.y}`)
  assert.ok(Math.abs(p.yaw - 2.0) < 1e-6, `yaw が収束しない: ${p.yaw}`)
})

test('smoothPose: 角度の折返しを跨いでも大回りしない', () => {
  // 179.4° から -179.4° へ。素朴な線形補間だと中間が 0° 付近（真後ろ）を通る。
  let p = { x: 0, y: 0, yaw: Math.PI - 0.01 }
  const tgt = { x: 0, y: 0, yaw: -Math.PI + 0.01 }
  for (let i = 0; i < 60; i++) {
    p = smoothPose(p, tgt, 0.25)
    // 常に境界付近（|yaw| > 3.0）に留まること＝0 付近を通らない
    const wrapped = Math.atan2(Math.sin(p.yaw), Math.cos(p.yaw))
    assert.ok(Math.abs(wrapped) > 3.0, `大回りした: ${wrapped}`)
  }
})
