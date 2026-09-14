// brief-onsite-fix C / F-3: onsiteMapTransform() と yawToSvgDeg() のユニットテスト。
// 地図タブの変換は「地図あり（OccupancyGrid の info から等方スケール）／地図なし
// （中央寄せ 1m=24px フォールバック）」の両方で、ピン・ロボット・地図ラスタが同じ
// 変換（toPx）を通ることを保証する。回転は yawToSvgDeg() で符号を反転する。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  canvasToWorld, computeMapView, onsiteMapTransform, worldToCanvas, yawToSvgDeg,
} from '../../src/mapGeometry.js'

test('yawToSvgDeg: 0 は 0', () => {
  assert.equal(yawToSvgDeg(0), 0)
})

test('yawToSvgDeg: +90°（ROS 左）は -90（SVG では下でなく上...ではなく、符号反転で正しい向き）', () => {
  // SVG の rotate は画面上で時計回りが正、ROS yaw は反時計回りが正。符号を反転する。
  assert.equal(yawToSvgDeg(Math.PI / 2), -90)
})

test('yawToSvgDeg: -90° は +90', () => {
  assert.equal(yawToSvgDeg(-Math.PI / 2), 90)
})

test('yawToSvgDeg: π は -180', () => {
  assert.equal(yawToSvgDeg(Math.PI), -180)
})

// 地図なし: 中央寄せ 1m=24px。原点(0,0)はビュー中心、y は反転。
test('onsiteMapTransform: 地図なしなら中央寄せ 24px/m（0,0 が中心）', () => {
  const t = onsiteMapTransform(null, 340, 250)
  assert.equal(t.view, null)
  assert.deepEqual(t.toPx(0, 0), [170, 125])
  assert.deepEqual(t.toPx(1, 0), [170 + 24, 125])   // +x は右
  assert.deepEqual(t.toPx(0, 1), [170, 125 - 24])   // +y は上（SVG y 反転）
})

// 地図あり: info から等方スケール。res=0.2m/cell, 5x5 セル → 1m x 1m の地図。
function smallInfo() {
  return {
    resolution: 0.2,
    width: 5,
    height: 5,
    origin: {
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
  }
}

test('onsiteMapTransform: 地図ありなら computeMapView ベース（等方スケール）', () => {
  const info = smallInfo()
  const grid = { info, data: new Array(25).fill(0) }
  const t = onsiteMapTransform(grid, 340, 250)
  assert.ok(t.view, '地図ありなら view が非 null')
  // 1m x 1m の地図を 340x250 に等方フィット → scale = min(340/5, 250/5)/... 
  // computeMapView の scale = min(canvasW/width, canvasH/height) = min(340/5, 250/5)=50。
  assert.equal(t.view.scale, 50)
  // 地図の左下セル(セル0)が map 原点（0,0）。origin 回転なし。
  // worldToCanvas(0,0): gx=0, gy=0 -> px=offX, py=offY+drawH
  const drawW = 5 * 50, drawH = 5 * 50
  const offX = (340 - drawW) / 2, offY = (250 - drawH) / 2
  assert.deepEqual(t.toPx(0, 0), [offX, offY + drawH])
  assert.deepEqual(t.toPx(1, 0), [offX + 1 / 0.2 * 50, offY + drawH])
  assert.deepEqual(t.toPx(0, 1), [offX, offY + drawH - 1 / 0.2 * 50])
})

test('onsiteMapTransform: invalid info（data なし）は地図なし扱い', () => {
  assert.equal(onsiteMapTransform({ info: smallInfo(), data: null }, 340, 250).view, null)
  assert.equal(onsiteMapTransform(undefined, 340, 250).view, null)
})

// brief-MAPTAP-FRONTEND §1.3: canvasToWorld() は worldToCanvas() の逆変換。
// 回転なしの地図で round-trip する。
test('canvasToWorld: round-trip（回転なし）', () => {
  const info = smallInfo()
  const grid = { info, data: new Array(25).fill(0) }
  const { view } = onsiteMapTransform(grid, 340, 250)
  for (const [x, y] of [[0, 0], [0.3, 0.7], [1, 1], [-0.2, 0.5], [0.8, -0.1]]) {
    const [px, py] = worldToCanvas(x, y, info, view)
    const [rx, ry] = canvasToWorld(px, py, info, view)
    assert.ok(Math.abs(rx - x) < 1e-9, `x が戻らない: ${x} -> ${rx}`)
    assert.ok(Math.abs(ry - y) < 1e-9, `y が戻らない: ${y} -> ${ry}`)
  }
})

// 回転あり（yaw=90°）でも round-trip する。回転なしだけでは符号バグを見逃す。
test('canvasToWorld: round-trip（origin yaw=90°）', () => {
  const info = {
    ...smallInfo(),
    origin: {
      position: { x: 1, y: -2, z: 0 },
      orientation: { x: 0, y: 0, z: Math.sin(Math.PI / 4), w: Math.cos(Math.PI / 4) },
    },
  }
  const grid = { info, data: new Array(25).fill(0) }
  const { view } = onsiteMapTransform(grid, 340, 250)
  for (const [x, y] of [[1, -2], [1.3, -1.5], [0.5, -2.8], [2, -1]]) {
    const [px, py] = worldToCanvas(x, y, info, view)
    const [rx, ry] = canvasToWorld(px, py, info, view)
    assert.ok(Math.abs(rx - x) < 1e-9, `x が戻らない: ${x} -> ${rx}`)
    assert.ok(Math.abs(ry - y) < 1e-9, `y が戻らない: ${y} -> ${ry}`)
  }
})

test('onsiteMapTransform: zoom=2 で scale が2倍', () => {
  const info = smallInfo()
  const grid = { info, data: new Array(25).fill(0) }
  const base = onsiteMapTransform(grid, 340, 250, 24, 0)
  const zoomed = onsiteMapTransform(grid, 340, 250, 24, 0, { zoom: 2 })
  assert.equal(zoomed.view.scale, base.view.scale * 2)
})

test('onsiteMapTransform: panX/panY で toPx がずれる', () => {
  const info = smallInfo()
  const grid = { info, data: new Array(25).fill(0) }
  const base = onsiteMapTransform(grid, 340, 250, 24, 0)
  const panned = onsiteMapTransform(grid, 340, 250, 24, 0, { panX: 10, panY: 5 })
  const [bx, by] = base.toPx(0, 0)
  const [px, py] = panned.toPx(0, 0)
  assert.ok(Math.abs((px - bx) - 10) < 1e-9, `panX が +10 ずれていない: ${px - bx}`)
  assert.ok(Math.abs((py - by) - 5) < 1e-9, `panY が +5 ずれていない: ${py - by}`)
})

test('onsiteMapTransform: userView 省略時は従来どおり（zoom=1, pan=0）', () => {
  const info = smallInfo()
  const grid = { info, data: new Array(25).fill(0) }
  const t = onsiteMapTransform(grid, 340, 250, 24, 22)
  const ref = computeMapView(info, 340 - 44, 250 - 44, { panX: 22, panY: 22 })
  assert.equal(t.view.scale, ref.scale)
  assert.deepEqual(t.toPx(0, 0), [ref.offX, ref.offY + ref.drawH])
})
