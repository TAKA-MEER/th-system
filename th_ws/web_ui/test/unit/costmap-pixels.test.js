// Spec-webui.md §6.1: コストマップは「入れない所」（Nav2 の値 99＝内接・
// 100＝致死）のセルだけを薄い灰で描く。膨張の帯（それ未満の値）は描かない。
import test from 'node:test'
import assert from 'node:assert/strict'
import { costmapToPixels, isCostmapForbidden } from '../../src/parts/costmapPixels.js'
import { MAP_COST } from '../../src/parts/mapColors.js'

function rgb(hex) {
  const n = parseInt(hex.slice(1), 16)
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255]
}

test('isCostmapForbidden は 99・100 だけ', () => {
  assert.equal(isCostmapForbidden(99), true)
  assert.equal(isCostmapForbidden(100), true)
  assert.equal(isCostmapForbidden(0), false)
  assert.equal(isCostmapForbidden(98), false)
  assert.equal(isCostmapForbidden(50), false)
  assert.equal(isCostmapForbidden(255), false)
})

test('costmapToPixels は 99・100 のセルだけ薄い灰、それ以外は透明', () => {
  const W = 4
  const H = 2
  // row0(下端): [0, 98, 99, 100]  row1(上端): [50, 99, 0, 255(未知)]
  const data = [0, 98, 99, 100, 50, 99, 0, 255]
  const out = costmapToPixels(data, W, H)
  const [r, g, b] = rgb(MAP_COST)
  const px = (row, col) => {
    const i = (row * W + col) * 4
    return [out[i], out[i + 1], out[i + 2], out[i + 3]]
  }
  // ImageData 行0（上端）= grid row1（上端）
  assert.deepEqual(px(0, 0), [0, 0, 0, 0]) // 50（膨張の帯）は描かない
  assert.deepEqual(px(0, 1), [r, g, b, 255]) // 99 は薄い灰
  assert.deepEqual(px(0, 2), [0, 0, 0, 0]) // 0 は描かない
  assert.deepEqual(px(0, 3), [0, 0, 0, 0]) // 未知は描かない
  // ImageData 行1（下端）= grid row0（下端）
  assert.deepEqual(px(1, 0), [0, 0, 0, 0])
  assert.deepEqual(px(1, 1), [0, 0, 0, 0]) // 98 は描かない
  assert.deepEqual(px(1, 2), [r, g, b, 255]) // 99
  assert.deepEqual(px(1, 3), [r, g, b, 255]) // 100
})

test('赤〜橙のヒートマップ（最大コスト 100 で赤）は出ない', () => {
  const out = costmapToPixels([100, 50, 0, 99], 2, 2)
  const rCh = out[0]
  assert.ok(rCh < 200, `赤チャンネルは薄い灰（<200）であるべき: ${rCh}`)
})
