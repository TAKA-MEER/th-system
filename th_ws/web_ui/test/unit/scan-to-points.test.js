// brief-MAP-SCAN: scanToPoints() のユニットテスト（Node の node:test）。
// RadarSelect の radar-to-svg.test.js と同じ流儀。cos/sin を使う極座標→直交座標の
// 変換なので、浮動小数点の比較はこれに合わせた。

import test from 'node:test'
import assert from 'node:assert/strict'
import { scanToPoints } from '../../src/parts/scanToPoints.js'

// angle_increment 既定 0 で「全点が前方（yaw=0, x=r）」のシンプルなスキャンを作る。
// 角度の分岐（increment が複数点で別の向き）は専用テストで改めて検証する。
function mkScan({ ranges, angle_min = 0, angle_increment = 0, range_min = 0, range_max = 10 }) {
  return {
    angle_min,
    angle_max: angle_min + angle_increment * (ranges.length - 1),
    angle_increment,
    range_min,
    range_max,
    ranges,
  }
}

test('scanToPoints: 正常な 1 点は正しい x/y になる（yaw=0 → x=r, y=0）', () => {
  const pts = scanToPoints(mkScan({ ranges: [1.5] }))
  assert.equal(pts.length, 1)
  assert.ok(Math.abs(pts[0].x - 1.5) < 1e-9, `x が 1.5 でない: ${pts[0].x}`)
  assert.ok(Math.abs(pts[0].y) < 1e-9, `y が 0 でない: ${pts[0].y}`)
})

test('scanToPoints: angle_increment で各点が別の角度になる（0°, 90°, 180°, 270°）', () => {
  const pts = scanToPoints(mkScan({
    angle_min: 0,
    angle_increment: Math.PI / 2,
    ranges: [1, 1, 1, 1],
  }))
  assert.equal(pts.length, 4)
  assert.ok(Math.abs(pts[0].x - 1) < 1e-9 && Math.abs(pts[0].y) < 1e-9, '0°は前方')
  assert.ok(Math.abs(pts[1].x) < 1e-9 && Math.abs(pts[1].y - 1) < 1e-9, '90°は左')
  assert.ok(Math.abs(pts[2].x + 1) < 1e-9 && Math.abs(pts[2].y) < 1e-9, '180°は後方')
  assert.ok(Math.abs(pts[3].x) < 1e-9 && Math.abs(pts[3].y + 1) < 1e-9, '270°は右')
})

test('scanToPoints: range_min 未満は除外される', () => {
  const pts = scanToPoints(mkScan({ range_min: 0.5, ranges: [0.2, 1, 0.4, 2] }))
  assert.equal(pts.length, 2)
  assert.ok(Math.abs(pts[0].x - 1) < 1e-9 && Math.abs(pts[0].y) < 1e-9)
  assert.ok(Math.abs(pts[1].x - 2) < 1e-9 && Math.abs(pts[1].y) < 1e-9)
})

test('scanToPoints: range_max 超過は除外される', () => {
  const pts = scanToPoints(mkScan({ range_max: 3, ranges: [3, 3.5, 4, 1] }))
  assert.equal(pts.length, 2)
  assert.ok(Math.abs(pts[0].x - 3) < 1e-9, 'range_max ちょうどは含む')
  assert.ok(Math.abs(pts[1].x - 1) < 1e-9)
})

test('scanToPoints: Infinity と NaN は「検出なし」として除外される', () => {
  const pts = scanToPoints(mkScan({ ranges: [Infinity, NaN, 1, -Infinity] }))
  assert.equal(pts.length, 1)
  assert.ok(Math.abs(pts[0].x - 1) < 1e-9)
})

test('scanToPoints: null や ranges のない scan は空配列を返す', () => {
  assert.deepEqual(scanToPoints(null), [])
  assert.deepEqual(scanToPoints({ angle_min: 0, ranges: undefined }), [])
})