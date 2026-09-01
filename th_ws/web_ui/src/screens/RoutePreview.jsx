// screens/RoutePreview.jsx — lightweight canvas preview of the teaching/replay
// route (S-13 / S-14), WS-3 / demo-teach-replay.
//
// MapView (src/MapView.jsx) is OccupancyGrid-based and cannot render the
// odom-frame route preview, so this is a dedicated, map-less canvas:
//   layers: ① preview polyline  ② /scan_filtered point cloud (red, converted
//            via baseToWorld against the robot pose)  ③ robot triangle marker
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
import { useEffect, useRef, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { baseToWorld } from '../mapGeometry.js'
import { ROUTE_PREVIEW_EMPTY } from '../i18n/screens.js'
import { fitTransform, ROUTE_PREVIEW_PAD } from './routePreviewGeom.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined
const SCAN_TOPIC = '/scan_filtered'
const SCAN_MSG = 'sensor_msgs/LaserScan'
const SCAN_MAX = 8 // m; draw closer points only (point cloud is for nearby obstacles)

export default function RoutePreview({ preview, pose, targetIndex }) {
  const { ros } = useSystemState()
  const canvasRef = useRef(null)
  const [scanData, setScanData] = useState(null)

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

  const points = []
  if (preview) points.push(...preview)
  if (pose) points.push(pose)
  const fit = fitTransform(points, 600, 380, ROUTE_PREVIEW_PAD)

  // TEST_MODE: reflect what was actually drawn so e2e can assert the passed
  // targetIndex really reaches the canvas (mutation 3). No production effect.
  const drawnTarget = fit && targetIndex != null && preview && preview[targetIndex] != null
    ? targetIndex : -1
  useEffect(() => {
    if (!TEST_MODE) return
    window.__thRoutePreviewDrawn = { targetIndex: drawnTarget, pointCount: preview?.length ?? 0 }
  }, [drawnTarget, preview])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    if (!fit) return

    const px = (p) => (p.x - fit.minX) * fit.scale + fit.offX
    const py = (p) => canvas.height - ((p.y - fit.minY) * fit.scale + fit.offY)

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

    // ② /scan_filtered point cloud (laser_link = base_link x/y/yaw; reuse
    // MapView's conversion via baseToWorld against the odom pose).
    if (scanData && pose) {
      const { angle_min, angle_increment, ranges } = scanData
      ctx.fillStyle = '#ef5350'
      for (let i = 0; i < ranges.length; i++) {
        const r = ranges[i]
        if (!Number.isFinite(r) || r <= 0 || r > SCAN_MAX) continue
        const angle = angle_min + i * angle_increment
        const [wx, wy] = baseToWorld(r * Math.cos(angle), r * Math.sin(angle), pose)
        const [cx, cy] = [px({ x: wx, y: wy }), py({ x: wx, y: wy })]
        ctx.fillRect(cx - 1, cy - 1, 2, 2)
      }
    }

    // ③ robot marker (MapView's triangle, no map-origin rotation: odom frame)
    if (pose) {
      const cx = px(pose), cy = py(pose)
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
  }, [fit, preview, pose, scanData, targetIndex])

  if (!fit) {
    return <p className="note" data-testid="route-preview-empty">{ROUTE_PREVIEW_EMPTY}</p>
  }

  return (
    <div className="card">
      <canvas
        ref={canvasRef}
        data-testid="route-preview"
        data-target-index={drawnTarget}
        width={600}
        height={380}
        className="route-preview"
      />
    </div>
  )
}