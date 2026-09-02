// WS-9F / demo-teach-replay: RoutePreview's map raster must render from both a
// plain Array (the e2e seed window.__thTestRouteMap) and a TypedArray (what
// /map arrives as under rosbridge compression:'cbor' — grid.data as Uint8Array,
// where unknown -1 shows up as 255). occupancyGridToPixels is a pure,
// canvas-free function so this runs with plain node --test.
import test from 'node:test'
import assert from 'node:assert/strict'
import { occupancyGridToPixels } from '../../src/screens/routePreviewGeom.js'

// 2x2 グリッド (row0 は map フレーム下端)。値: -1(未知/灰), 0(自由/白),
// 50(中間→灰 128 ※浮動小数で 255-127.5 が 128 になる), 100(占有/黒 0)。
//   row0: [-1,   0]
//   row1: [50, 100]
const W = 2
const H = 2
const ARRAY = [-1, 0, 50, 100]
const TYPED = Uint8Array.from([255, 0, 50, 100])   // 未知 -1 → 255

// 出力 ImageData の (row, col) ピクセルの RGBA。ImageData 行0 は上端なので、
// グリッド下端 row1 が出力行0 に来る（上下反転）。
function pixel(out, row, col) {
  const idx = (row * W + col) * 4
  return [out[idx], out[idx + 1], out[idx + 2], out[idx + 3]]
}

test('renders a plain Array to RGBA with row flip and unknown -> grey', () => {
  const out = occupancyGridToPixels(ARRAY, W, H)
  assert.ok(out instanceof Uint8ClampedArray)
  assert.deepEqual(pixel(out, 0, 0), [128, 128, 128, 255])   // grid row1 col0 = 50
  assert.deepEqual(pixel(out, 0, 1), [0, 0, 0, 255])         // grid row1 col1 = 100 (占有/黒)
  assert.deepEqual(pixel(out, 1, 0), [128, 128, 128, 255])   // grid row0 col0 = -1 (未知/灰)
  assert.deepEqual(pixel(out, 1, 1), [255, 255, 255, 255])   // grid row0 col1 = 0 (自由/白)
})

test('renders a TypedArray (Uint8Array) identically, 255 = unknown grey', () => {
  const typedOut = occupancyGridToPixels(TYPED, W, H)
  const arrayOut = occupancyGridToPixels(ARRAY, W, H)
  assert.ok(typedOut instanceof Uint8ClampedArray)
  assert.deepEqual(Array.from(typedOut), Array.from(arrayOut), (
    'Uint8Array（未知=255）と素の配列（未知=-1）が同じラスタになる'))
  // 明示的に未知セルが灰（128）であること
  assert.deepEqual(pixel(typedOut, 0, 0), [128, 128, 128, 255])   // 50 は中間灰
  assert.deepEqual(pixel(typedOut, 1, 0), [128, 128, 128, 255])   // 255 → 灰
})
