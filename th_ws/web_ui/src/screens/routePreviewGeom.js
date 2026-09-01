// screens/routePreviewGeom.js — pure geometry for the route-preview canvas
// (WS-3 / demo-teach-replay). Kept as a plain .js module (no JSX / no canvas)
// so test/unit/route-preview-transform.test.js can drive it with node --test.
export const ROUTE_PREVIEW_PAD = 20

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