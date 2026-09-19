// screens/RoutePreview.jsx — lightweight canvas preview of the teaching/replay
// route (S-13 / S-14), WS-3 / demo-teach-replay.
//
// MapView (src/MapView.jsx) is OccupancyGrid-based and cannot render the
// odom-frame route preview, so this is a dedicated, lightweight canvas:
//   layers: ⓪ /map background raster (when a map is available, WS-8B)
//           ① preview polyline  ② /scan_filtered point cloud (red, converted
//           via baseToWorld against the robot pose)  ③ robot triangle marker
//           ④ the pure-pursuit target point (a prominent dot) when
//           targetIndex >= 0 and preview[targetIndex] exists.
//
// The drawing coords come from fitTransform(), a pure function exported for
// test/unit (no canvas dependency). Pan/zoom-handling kept out on purpose:
// the preview auto-fits its data bbox each render (WS-3 scope is "show the
// path", not interactive inspect).
//
// route_preview / route_status come in as props; /scan_filtered is subscribed
// here because the screen shouldn't have to wire a point cloud it doesn't
// otherwise use (useRosbridge.js subscribes it the same raw-name way). In
// TEST_MODE a seeded window.__thTestRouteScan can stand in for the wire.
import { useCallback, useEffect, useRef, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { ROUTE_PREVIEW_EMPTY } from '../i18n/screens.js'
import { fitTransform, ROUTE_PREVIEW_PAD, centeredTransform, ROUTE_PREVIEW_HALF_SPAN_M, scanToPoints, SCAN_FALLBACK_MAX_M, mapDestRect, smoothPose, occupancyGridToPixels } from './routePreviewGeom.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined
const SCAN_TOPIC = '/scan_filtered'
const SCAN_MSG = 'sensor_msgs/LaserScan'
const W = 600
const H = 380
// pose 補間の追従係数（0..1）。1 フレームで目標との差をこの割合だけ詰める。
// 大きいほど機敏だが階段が残り、小さいほど滑らかだが表示が遅れる。
// 60fps・pose 10Hz（= 6 フレームに 1 回更新）で、0.25 なら 1 更新分の差を
// おおむね次の更新までに詰めきる（0.75^6 ≒ 0.18 まで縮む）。
const POSE_SMOOTH_ALPHA = 0.25

export default function RoutePreview({ preview, pose, targetIndex, mapData }) {
  const { ros } = useSystemState()
  const canvasRef = useRef(null)
  const offscreenRef = useRef(null)
  const [scanData, setScanData] = useState(null)

  // Off-screen map raster (OccupancyGrid -> ImageData), rebuilt on map change.
  // Mirrors MapView.jsx: row 0 of the grid is the map-frame bottom, ImageData
  // row 0 is the top, so rows are flipped; unknown (-1) is mid-grey, 0 white,
  // 100 black.
  useEffect(() => {
    if (!mapData) { offscreenRef.current = null; return undefined }
    const { width, height } = mapData.info
    const data = mapData.data
    // `data` は素の配列（TEST_MODE のシード）か、rosbridge cbor で届く Uint8Array
    // （WS-9F）。どちらでも解釈できるように、配列/型付き配列を判定して許す。
    if (!(Array.isArray(data) || ArrayBuffer.isView(data)) || width <= 0 || height <= 0) {
      return undefined
    }
    if (!offscreenRef.current) offscreenRef.current = document.createElement('canvas')
    const off = offscreenRef.current
    off.width = width
    off.height = height
    const ctx = off.getContext('2d')
    const img = ctx.createImageData(width, height)
    img.data.set(occupancyGridToPixels(data, width, height))
    ctx.putImageData(img, 0, 0)
    return undefined
  }, [mapData])

  // Own /scan_filtered subscription (raw topic name, matching useRosbridge.js).
  useEffect(() => {
    if (TEST_MODE) {
      setScanData(window.__thTestRouteScan ?? null)
      window.__thSetTestRouteScan = (v) => setScanData(v)
      return () => { delete window.__thSetTestRouteScan }
    }
    if (!ros) { setScanData(null); return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) { setScanData(null); return undefined }
    const topic = new ROSLIB.Topic({
      ros, name: SCAN_TOPIC, messageType: SCAN_MSG, throttle_rate: 200,
    })
    topic.subscribe((msg) => setScanData(msg))
    return () => { topic.unsubscribe() }
  }, [ros])

  // WS-6.4: with a pose use the robot-centred fixed-scale transform (never
  // re-fits, so the view does not jump while the robot moves + the scan fits on
  // screen). Only without a pose (odom not yet received) fall back to the
  // data-bbox fit from WS-3.
  const points = []
  if (preview) points.push(...preview)
  if (pose) points.push(pose)
  // `fit` はここでは「描けるデータがあるか」の判定とオーバレイ表示にだけ使う。
  // 実際の描画は下の rAF ループが補間後の pose から毎フレーム作り直す。
  const fit = pose
    ? centeredTransform(pose, ROUTE_PREVIEW_HALF_SPAN_M, W, H)
    : fitTransform(points, W, H, ROUTE_PREVIEW_PAD)

  // TEST_MODE: reflect what was actually drawn so e2e can assert the passed
  // targetIndex really reaches the canvas (mutation 3). No production effect.
  const drawnTarget = fit && targetIndex != null && preview && preview[targetIndex] != null
    ? targetIndex : -1
  useEffect(() => {
    if (!TEST_MODE) return
    window.__thRoutePreviewDrawn = { targetIndex: drawnTarget, pointCount: preview?.length ?? 0 }
  }, [drawnTarget, preview])

  // rAF ループが「張り直さずに」最新の props を読めるようにする。以前は描画が
  // useEffect の依存（fit を含む）で走っており、fit は毎レンダー新オブジェクト
  // だったため、/system/state の 10Hz 再レンダーのたびに中身が変わっていなくても
  // フル再描画していた（地図 blit ＋ 数百 fillRect）。
  const latestRef = useRef(null)
  latestRef.current = { preview, scanData, targetIndex, mapData, pose }

  const drawFrame = useCallback((dpose) => {
    const canvas = canvasRef.current
    if (!canvas) return
    const { preview, scanData, targetIndex, mapData } = latestRef.current
    const pose = dpose
    const pts = []
    if (preview) pts.push(...preview)
    if (pose) pts.push(pose)
    const fit = pose
      ? centeredTransform(pose, ROUTE_PREVIEW_HALF_SPAN_M, W, H)
      : fitTransform(pts, W, H, ROUTE_PREVIEW_PAD)
    const ctx = canvas.getContext('2d')
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    if (TEST_MODE) window.__thRoutePreviewFit = { scale: fit ? fit.scale : null }
    if (TEST_MODE) window.__thRoutePreviewPose = pose ? { x: pose.x, y: pose.y } : null
    if (!fit) return

    const px = (p) => (p.x - fit.minX) * fit.scale + fit.offX
    const py = (p) => canvas.height - ((p.y - fit.minY) * fit.scale + fit.offY)

    // ⓪ /map background raster (deepest layer). Only when a map exists
    // (enable_route_slam:=true); map-less operation skips this entirely.
    // Blit the native-resolution offscreen bitmap through the same robot-centred
    // fit, so it lines up with the route/scan/robot layers. TEST_MODE flag for e2e.
    let mapDrawn = false
    if (mapData && offscreenRef.current) {
      const rect = mapDestRect(mapData.info, fit, canvas.height)
      if (rect) {
        ctx.imageSmoothingEnabled = false
        ctx.drawImage(offscreenRef.current, rect.dx, rect.dy, rect.dw, rect.dh)
        mapDrawn = true
      }
    }
    if (TEST_MODE) window.__thRoutePreviewMapDrawn = mapDrawn

    // ① preview polyline
    if (preview && preview.length > 1) {
      ctx.beginPath()
      preview.forEach((p, i) => {
        if (i === 0) ctx.moveTo(px(p), py(p))
        else ctx.lineTo(px(p), py(p))
      })
      ctx.strokeStyle = '#ffb74d'
      ctx.lineWidth = 2
      ctx.stroke()
    }

    // ② /scan_filtered point cloud (laser_link = base_link x/y/yaw). WS-8A:
    // the clip now honours the sensor's scanData.range_max (open-venue walls
    // past the old 7.5m hard clip are drawn), converted to world via the pure
    // scanToPoints. Points drawn are counted for the e2e, production unaffected.
    let scanDrawn = 0
    const scanPts = scanToPoints(scanData, pose, SCAN_FALLBACK_MAX_M)
    if (scanPts.length) {
      ctx.fillStyle = '#ef5350'
      for (const wp of scanPts) {
        const cx = px(wp), cy = py(wp)
        ctx.fillRect(cx - 1, cy - 1, 2, 2)
        scanDrawn++
      }
    }
    if (TEST_MODE) window.__thRoutePreviewScanDrawn = scanDrawn

    // ③ robot marker (MapView's triangle, no map-origin rotation: odom frame)
    if (pose) {
      const cx = px(pose), cy = py(pose)
      // WS-6.4: with the centred transform the robot fixes the canvas centre.
      ctx.save()
      ctx.translate(cx, cy)
      ctx.rotate(Math.PI / 2 - pose.yaw)
      ctx.beginPath()
      ctx.moveTo(0, -9); ctx.lineTo(-7, 7); ctx.lineTo(7, 7); ctx.closePath()
      ctx.fillStyle = '#90caf9'
      ctx.fill()
      ctx.restore()
    }

    // ④ pure-pursuit target point
    if (preview && targetIndex != null && preview[targetIndex] != null) {
      const t = preview[targetIndex]
      ctx.beginPath()
      ctx.arc(px(t), py(t), 5, 0, Math.PI * 2)
      ctx.fillStyle = '#26c6da'
      ctx.fill()
      ctx.strokeStyle = '#fff'
      ctx.lineWidth = 1.5
      ctx.stroke()
    }
  }, [])

  // ── 描画の駆動 ───────────────────────────────────────────────
  // 本番: rAF ループで毎フレーム、pose を目標へ指数的に近づけてから描く。
  // pose は 10Hz でしか来ないので、間のフレームを補間しないと画面全体
  // （地図・経路・点群）が秒 10 回まとめて瞬間移動する＝ガクガクになる。
  // ループは一度だけ張り、データは latestRef から読むので再レンダーで
  // 張り直さない。
  const smoothRef = useRef(null)
  const rafRef = useRef(0)
  useEffect(() => {
    if (TEST_MODE) return undefined
    const step = () => {
      smoothRef.current = smoothPose(
        smoothRef.current, latestRef.current.pose, POSE_SMOOTH_ALPHA)
      drawFrame(smoothRef.current)
      rafRef.current = requestAnimationFrame(step)
    }
    rafRef.current = requestAnimationFrame(step)
    return () => cancelAnimationFrame(rafRef.current)
  }, [drawFrame])

  // TEST_MODE: e2e は seed した直後に window.__thRoutePreview* を読むので、
  // 補間も rAF も挟まず同期で 1 回描く（決定的にする）。
  useEffect(() => {
    if (!TEST_MODE) return
    smoothRef.current = pose ? { ...pose } : null
    drawFrame(smoothRef.current)
  }, [drawFrame, pose, preview, scanData, targetIndex, mapData])

  // WS-6.4: the canvas is ALWAYS mounted (so a zero-frame gap never swaps the
  // DOM, further reducing #4 flicker). With no drawable data the placeholder
  // overlay shows on top; with data it hides behind the drawing.
  return (
    <div className="routePreview">
      <canvas
        ref={canvasRef}
        data-testid="route-preview"
        data-target-index={drawnTarget}
        width={W}
        height={H}
        className="route-preview"
      />
      {!fit && (
        <p className="routePreviewOverlay" data-testid="route-preview-empty">{ROUTE_PREVIEW_EMPTY}</p>
      )}
    </div>
  )
}