// parts/costmapPixels.js — OccupancyGrid → RGBA ビットマップ変換（costmap 専用）。
// brief-MAP-COSTMAP: Nav2 のグローバル costmap（/global_costmap/costmap）は
// 0-100 の占有コストを持つ nav_msgs/OccupancyGrid。occupancyGridToPixels()
// （screens/routePreviewGeom.js）と同じ入力形 (data, width, height) →
// Uint8ClampedArray・同じ行反転規則（grid の行0=map フレーム下端、ImageData 行0=
// 上端）。
//
// Spec-webui.md §6.1（2026-11-27）: 「入れない所」（機体の中心が入ると壁に当たる
// 帯＝Nav2 の内接・致死のセル）だけを薄い灰で描く。避けたい帯（膨張の帯）は
// 描かない。以前の赤〜橙のヒートマップ（値を面で塗る方式）は廃止した。
//
// Nav2 のコスト値は 99＝internal max（内接）・100＝lethal（致死）。99/100 の
// セルだけが「入れない所」（機体の中心がそこへ入ると壁に当たる所）であり、
// それより低い値（膨張の帯）は通れるので描かない。白い床＝機体の中心が
// 通れる所、と読める。
import { MAP_COST } from './mapColors.js'

export const COSTMAP_INFLATED = 99 // Nav2 internal max（内接）
export const COSTMAP_LETHAL = 100 // Nav2 lethal（致死）

export function isCostmapForbidden(v) {
  return v === COSTMAP_INFLATED || v === COSTMAP_LETHAL
}

export function costmapToPixels(data, width, height) {
  const w = Math.floor(width)
  const h = Math.floor(height)
  const out = new Uint8ClampedArray(w * h * 4)
  const [r, g, b] = hexToRgb(MAP_COST)
  for (let row = 0; row < h; row++) {
    for (let col = 0; col < w; col++) {
      const srcIdx = row * w + col
      const destRow = h - 1 - row
      const destIdx = (destRow * w + col) * 4
      const v = data[srcIdx]
      if (isCostmapForbidden(v)) {
        out[destIdx] = r
        out[destIdx + 1] = g
        out[destIdx + 2] = b
        out[destIdx + 3] = 255
      }
      // それ以外は透明（下に描いた静的占有格子＝地図ラスタが透けて見える）
    }
  }
  return out
}

function hexToRgb(hex) {
  const n = parseInt(hex.slice(1), 16)
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255]
}
