// screens/routePreviewGeom.js — pure geometry for the route-preview canvas
// (WS-3 / demo-teach-replay). Kept as a plain .js module (no JSX / no canvas)
// so test/unit/route-preview-transform.test.js can drive it with node --test.
export const ROUTE_PREVIEW_PAD = 20
// WS-6.4: robot-centred fixed-scale preview. The display spans ±halfSpan metres
// around the robot (RViz-default-like, ~7m). Because the scale does not depend
// on the route data it never re-fits on every odom tick, so the view does not
// jump while the robot moves (feedback #4), and the /scan_filtered point cloud
// (up to this radius) stays on-canvas (#5).
export const ROUTE_PREVIEW_HALF_SPAN_M = 7

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