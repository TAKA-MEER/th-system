// parts/costmapPixels.js — OccupancyGrid → RGBA ビットマップ変換（costmap 専用）。
// brief-MAP-COSTMAP: Nav2 のグローバル costmap（/global_costmap/costmap）は
// 0-100 の占有コストを持つ nav_msgs/OccupancyGrid。occupancyGridToPixels()
// （screens/routePreviewGeom.js）と同じ入力形 (data, width, height) →
// Uint8ClampedArray・同じ行反転規則（grid の行0=map フレーム下端、ImageData 行0=
// 上端）。
//
// 値が高いほど不透明な赤（赤→橙→黄のグラデーション）、0（自由）と -1（未知）は
// 透明に近くして、下に描いた静的占有格子（地図ラスタ）が透けて見えるようにする。
// 値の意味は決め打ちでよく、「高コストほど濃い赤」であることが操作者に伝われば良い。
// alpha チャンネルを利かせないと下の静的地図が見えなくなるので注意。
export function costmapToPixels(data, width, height) {
  const w = Math.floor(width)
  const h = Math.floor(height)
  const out = new Uint8ClampedArray(w * h * 4)
  for (let row = 0; row < h; row++) {
    for (let col = 0; col < w; col++) {
      const srcIdx = row * w + col
      const destRow = h - 1 - row
      const destIdx = (destRow * w + col) * 4
      const v = data[srcIdx]
      // 0 以下（自由・未知・欠損）は完全透明。costmap の値は 0-100 で、
      // rosbridge の cbor 化で -1（未知）が 255 に化けることがあるので v > 100 も
      // 未知扱いにする（occupancyGridToPixels と同じ考え方）。
      const alpha = v <= 0 || v > 100 ? 0 : Math.min(255, v * 2.2)
      out[destIdx] = 255
      out[destIdx + 1] = Math.max(0, 180 - v * 1.6)
      out[destIdx + 2] = 40
      out[destIdx + 3] = alpha
    }
  }
  return out
}