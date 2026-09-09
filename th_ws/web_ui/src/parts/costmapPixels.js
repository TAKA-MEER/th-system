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
//
// 2026-09-09 実機フィードバック「コストマップで地図が見えない」: 初版は
// v=100（最大コスト）で alpha≈220（86%）と、静的地図がほぼ隠れる濃さだった。
// 低コスト（軽い膨張）まで alpha が付いていたため、部屋全体が赤くにじんで
// 見えたのも一因。閾値を上げて低コスト域は透明に近くし、最大でも alpha を
// 140（55%）に抑えて下の地図が必ず透けるようにする。
export function costmapToPixels(data, width, height) {
  const w = Math.floor(width)
  const h = Math.floor(height)
  const out = new Uint8ClampedArray(w * h * 4)
  const ALPHA_CAP = 140
  const VISIBLE_FROM = 20   // これ以下（自由・軽い膨張）は描かない
  for (let row = 0; row < h; row++) {
    for (let col = 0; col < w; col++) {
      const srcIdx = row * w + col
      const destRow = h - 1 - row
      const destIdx = (destRow * w + col) * 4
      const v = data[srcIdx]
      // rosbridge の cbor 化で -1（未知）が 255 に化けることがあるので v > 100 も
      // 未知扱いにする（occupancyGridToPixels と同じ考え方）。
      const alpha = v <= VISIBLE_FROM || v > 100
        ? 0
        : Math.min(ALPHA_CAP, (v - VISIBLE_FROM) * (ALPHA_CAP / (100 - VISIBLE_FROM)))
      out[destIdx] = 255
      out[destIdx + 1] = Math.max(0, 180 - v * 1.6)
      out[destIdx + 2] = 40
      out[destIdx + 3] = alpha
    }
  }
  return out
}