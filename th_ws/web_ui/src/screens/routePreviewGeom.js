// screens/routePreviewGeom.js — pure geometry for the route-preview canvas
// (WS-3 / demo-teach-replay). Kept as a plain .js module (no JSX / no canvas)
// so test/unit/route-preview-transform.test.js can drive it with node --test.
import { baseToWorld } from '../mapGeometry.js'

export const ROUTE_PREVIEW_PAD = 20
// WS-6.4: robot-centred fixed-scale preview. The display spans ±halfSpan metres
// around the robot (RViz-default-like). Because the scale does not depend on the
// route data it never re-fits on every odom tick, so the view does not jump
// while the robot moves (feedback #4), and the /scan_filtered point cloud stays
// on-canvas. WS-8A widened this 7 -> 10m so walls of an open venue are more often
// on screen; points outside are naturally clipped by the canvas.
export const ROUTE_PREVIEW_HALF_SPAN_M = 10
// WS-8A: hard clip fallback when the sensor's scanData.range_max is absent.
// RPLIDAR S1 range_max is ~12m; 16m keeps all valid returns drawn.
export const SCAN_FALLBACK_MAX_M = 16

// ── pose の補間（2026-09-02「経路表示がガクガク動く」対策） ──────────────
//
// プレビューはロボット中心・固定倍率なので、pose が画面全体の位置を決める。
// pose は ROS 側から離散的にしか来ない（10Hz。以前は 2Hz だった）ため、受信の
// たびに地図・経路・点群がまとめて瞬間移動する＝ガクガク見える。特に旋回は
// 効きが大きく、w=0.5rad/s・10Hz でも 1 更新 2.9°、半径 10m の点群外周で
// 約 9px が一度に動く（2Hz だった頃は 14.3°・47px）。
// そこで描画側は毎フレーム目標 pose へ指数的に近づけた値を使う。

// -π..π に畳んだ角度差。yaw を補間するとき 179° → -179° を
// 「-358°回る」ではなく「+2°回る」と解釈させるために要る。
export function shortestAngleDelta(from, to) {
  let d = to - from
  while (d > Math.PI) d -= 2 * Math.PI
  while (d < -Math.PI) d += 2 * Math.PI
  return d
}

// current を target へ alpha（0..1）だけ近づけた pose を返す純関数。
// alpha=1 で即座に target（テスト・初回受信時はこれを使う）。
// current が無ければ target をそのまま返す（初期化）。
export function smoothPose(current, target, alpha) {
  if (!target) return current ?? null
  if (!current) return { ...target }
  const a = Math.max(0, Math.min(1, alpha))
  return {
    ...target,
    x: current.x + (target.x - current.x) * a,
    y: current.y + (target.y - current.y) * a,
    yaw: current.yaw + shortestAngleDelta(current.yaw, target.yaw) * a,
  }
}

// centeredTransform(center, halfSpanM, w, h) -> { scale, minX, minY, offX, offY } | null
// Put `center` ({x,y}, odom world) at the canvas centre and fit ±halfSpanM to
// the SHORT canvas side, preserving aspect ratio. No rotation, fixed scale —
// the returned shape matches fitTransform so RoutePreview's px/py math is
// unchanged. Returns null for an invalid center (null / NaN).
export function centeredTransform(center, halfSpanM, w, h) {
  if (!center) return null
  const { x, y } = center
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null
  if (!Number.isFinite(halfSpanM) || halfSpanM <= 0) return null
  const scale = Math.min(w, h) / (2 * halfSpanM)
  return {
    scale,
    minX: x - halfSpanM,
    minY: y - halfSpanM,
    offX: (w - 2 * halfSpanM * scale) / 2,
    offY: (h - 2 * halfSpanM * scale) / 2,
  }
}

// previewFromPath(msg, prev) -> next preview array, applying the WS-6.4 rule
// that an empty Path message is ignored (keeps the last non-empty preview) so a
// momentary zero-frame gap never blanks the view. Pure & react-free so
// test/unit can drive it with node --test (mutation 3: dropping the empty-ignore
// makes this return every value, and the "payload-kept on []" test goes red).
// Placed here with the other react-free route-preview helpers because importing
// useRoutePreview.js would drag the whole react hook module + extensionless
// sibling imports into node --test.
export function previewFromPath(msg, prev) {
  const pts = (msg?.poses ?? []).map((p) => ({
    x: p.pose.position.x, y: p.pose.position.y,
  }))
  return pts.length > 0 ? pts : prev
}

// scanToPoints(scan, pose, fallbackMaxRange) -> [{x, y}]  (odom world 座標)
// Convert a /scan_filtered LaserScan's finite returns to robot-centred world
// points (via baseToWorld against `pose`), clipping to the sensor's own
// range_max so open-venue walls past the old hard 7.5m clip are no longer
// dropped (WS-8A / #2). Pure & react/canvas-free so test/unit drives it.
//   - max range = scan.range_max if it is finite and > 0, else fallbackMaxRange
//   - r that is non-finite / <= 0 / > max is dropped
//   - scan or pose missing -> []
export function scanToPoints(scan, pose, fallbackMaxRange = 16) {
  if (!scan || !pose || !Array.isArray(scan.ranges)) return []
  const { angle_min, angle_increment, ranges, range_max } = scan
  const maxRange = Number.isFinite(range_max) && range_max > 0 ? range_max : fallbackMaxRange
  const pts = []
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i]
    if (!Number.isFinite(r) || r <= 0 || r > maxRange) continue
    const angle = angle_min + i * angle_increment
    const [wx, wy] = baseToWorld(r * Math.cos(angle), r * Math.sin(angle), pose)
    pts.push({ x: wx, y: wy })
  }
  return pts
}

// mapDestRect(mapInfo, fit, viewH) -> { dx, dy, dw, dh }  (canvas px) | null
// Where to blit the OccupancyGrid bitmap on the route-preview canvas, using the
// same robot-centred `fit` transform (centeredTransform / fitTransform shape:
// { scale, minX, minY, offX, offY }) as the route/scan/robot layers (WS-8B).
// `mapInfo` is nav_msgs/OccupancyGrid.info { resolution, width, height, origin };
// `viewH` is the canvas height for py()'s vertical flip. The map's world
// rectangle runs x:[origin.x, origin.x + width*res] and
// y:[origin.y, origin.y + height*res]. py() flips vertically (canvas y grows
// down), so the destination top is world y1. Returns null on missing/invalid
// info or fit. Pure & react/canvas-free so test/unit drives it.
export function mapDestRect(mapInfo, fit, viewH) {
  if (!mapInfo || !fit) return null
  const res = mapInfo.resolution
  const width = mapInfo.width
  const height = mapInfo.height
  const ox = mapInfo.origin?.position?.x
  const oy = mapInfo.origin?.position?.y
  if (!Number.isFinite(res) || res <= 0 || !Number.isInteger(width) ||
      !Number.isInteger(height) || width <= 0 || height <= 0 ||
      !Number.isFinite(ox) || !Number.isFinite(oy) || !Number.isFinite(viewH) || viewH <= 0) {
    return null
  }
  const { minX, minY, scale, offX, offY } = fit
  const px = (wx) => (wx - minX) * scale + offX
  const py = (wy) => viewH - ((wy - minY) * scale + offY)
  const x0 = ox
  const x1 = ox + width * res
  const y0 = oy
  const y1 = oy + height * res
  const dx = px(x0)
  const dw = px(x1) - px(x0)
  const dy = py(y1)
  const dh = py(y0) - py(y1)
  return { dx, dy, dw, dh }
}

// fitTransform(points, w, h, pad) -> { scale, minX, minY, offX, offY } | null
// Pure geometry: fit the bbox of `points` ([{x,y},...]) into a w x h canvas
// with `pad` margin, preserving aspect ratio and centering. Returns null for
// an empty point set (caller shows the "no route" placeholder). A degenerate
// single point is given a small default extent so it still renders centered.
export function fitTransform(points, w, h, pad = ROUTE_PREVIEW_PAD) {
  if (!points || points.length === 0) return null
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity
  for (const p of points) {
    if (p.x < minX) minX = p.x
    if (p.x > maxX) maxX = p.x
    if (p.y < minY) minY = p.y
    if (p.y > maxY) maxY = p.y
  }
  let aw = maxX - minX
  let ah = maxY - minY
  if (aw < 1e-6) aw = 2
  if (ah < 1e-6) ah = 2
  const iw = Math.max(1, w - 2 * pad)
  const ih = Math.max(1, h - 2 * pad)
  const scale = Math.min(iw / aw, ih / ah)
  const offX = (w - aw * scale) / 2
  const offY = (h - ah * scale) / 2
  return { scale, minX, minY, offX, offY }
}

// occupancyGridToPixels(data, width, height) -> Uint8ClampedArray (RGBA)
// Convert an OccupancyGrid data column to the RGBA bitmap RoutePreview blits on
// the route-preview canvas. Pure & canvas-free so test/unit drives it with node
// --test. Row 0 of the grid is the map-frame bottom, ImageData row 0 is the top,
// so rows are flipped here. Unknown is mid-grey 128, 0 (free) is white, 100
// (occupied) is black. Only uses index access (`data[i]`), so it works for both
// a plain Array and a TypedArray — with rosbridge `compression:'cbor'` (/map,
// WS-9F) the data may arrive as a Uint8Array where unknown (-1) is stored as
// 255, hence `v < 0 || v > 100` (instead of bare `v < 0`) is treated as unknown.
export function occupancyGridToPixels(data, width, height) {
  const w = Math.floor(width)
  const h = Math.floor(height)
  const out = new Uint8ClampedArray(w * h * 4)
  for (let row = 0; row < h; row++) {
    for (let col = 0; col < w; col++) {
      const srcIdx = row * w + col
      const destRow = h - 1 - row
      const destIdx = (destRow * w + col) * 4
      const v = data[srcIdx]
      let gray
      if (v < 0 || v > 100) gray = 128
      else gray = 255 - Math.round(v * 2.55)
      out[destIdx] = out[destIdx + 1] = out[destIdx + 2] = gray
      out[destIdx + 3] = 255
    }
  }
  return out
}