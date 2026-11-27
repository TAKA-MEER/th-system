// Spec-webui.md §6.1 の配色定数（parts/mapColors.js）のユニットテスト。
// 値はモックアップ（docs/plan/spec/mockup/index.html の col-jis）の
// --jis-y（#ffd100）と合わせ、濃淡の順序（床＞壁＞点群）を縛る。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  MAP_FREE, MAP_OCCUPIED, MAP_SCAN, MAP_NEAR, MAP_GO, MAP_WARN,
  MAP_UNKNOWN, SLOWDOWN_DISTANCE_M, scanPointFill, isNearPoint, occupancyGray,
} from '../../src/parts/mapColors.js'

function grayOf(hex) {
  const n = parseInt(hex.slice(1), 16)
  return ((n >> 16) & 255)
}

test('濃淡の順序: 床（白）＞ 壁（中間の灰）＞ 点群（壁より濃い灰）', () => {
  assert.equal(MAP_FREE, '#ffffff')
  assert.ok(grayOf(MAP_FREE) > grayOf(MAP_OCCUPIED) && grayOf(MAP_OCCUPIED) > grayOf(MAP_SCAN),
    `床 ${MAP_FREE} ＞ 壁 ${MAP_OCCUPIED} ＞ 点群 ${MAP_SCAN}`)
})

test('進むの色は緑、注意の黄は JIS yellow（#ffd100）', () => {
  assert.equal(MAP_WARN, '#ffd100')
  assert.equal(MAP_NEAR, '#ffd100')
  // 緑は赤・青成分より緑成分が大きい
  const n = parseInt(MAP_GO.slice(1), 16)
  const g = (n >> 8) & 255, r = (n >> 16) & 255, b = n & 255
  assert.ok(g > r && g > b, `進む色は緑成分が最大: ${MAP_GO}`)
})

test('未探索は画面の地（theme.css --bg #0f1117）と同じ', () => {
  assert.equal(MAP_UNKNOWN, '#0f1117')
})

test('scanPointFill: 減速距離以下は黄、それより遠い点は濃い灰', () => {
  assert.equal(scanPointFill(0.3), MAP_NEAR)
  assert.equal(scanPointFill(SLOWDOWN_DISTANCE_M), MAP_NEAR)
  assert.equal(scanPointFill(SLOWDOWN_DISTANCE_M + 0.01), MAP_SCAN)
  assert.equal(scanPointFill(undefined), MAP_SCAN)
  assert.equal(isNearPoint(0.1), true)
  assert.equal(isNearPoint(10), false)
})

test('occupancyGray: 未知は null（透明）、0 は白、100 は中間の灰', () => {
  assert.equal(occupancyGray(-1), null)
  assert.equal(occupancyGray(255), null)
  assert.equal(occupancyGray(0), 255)
  assert.equal(occupancyGray(100), 128)
  const mid = occupancyGray(50)
  assert.ok(mid > 128 && mid < 255, `中間値は白と灰の間: ${mid}`)
})
