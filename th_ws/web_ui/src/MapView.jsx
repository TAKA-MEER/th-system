// ============================================================
// MapView.jsx — SLAM 地図 + 自己位置 + 点群 + ルート表示
// (RViz の Map + LaserScan + Path + TF 相当を軽量に再現)
//
// 表示はドラッグで移動・ホイール/ピンチで拡大縮小できる。地図が広くなると
// 320px の枠に全体を収めた状態では細部が潰れて使い物にならないため。
// 「全体表示」ボタンでいつでも初期状態へ戻せる。
// ============================================================
import { useRef, useEffect, useState, useCallback } from 'react'

import { quatToYaw, worldToCanvas, computeMapView } from './mapGeometry.js'

const CANVAS_SIZE = 320   // 表示 canvas の一辺 (px)。CandidateRadar の 260px よりやや大きく
const MAX_SCAN_RANGE_M = 12.0   // LiDAR (RPLIDAR S1) の最大レンジ。異常値の足切りに使う
const ZOOM_MIN = 1
const ZOOM_MAX = 12
const ZOOM_STEP = 1.2

export default function MapView({ mapData, robotPose, scanData, pathData }) {
  const canvasRef = useRef(null)
  const offscreenRef = useRef(null)   // グリッド1セル=1pxの地図ビットマップ

  // ── パン・ズーム ────────────────────────────────────────
  const [view, setView] = useState({ zoom: 1, panX: 0, panY: 0 })
  const dragRef = useRef(null)        // {pointerId, startX, startY, panX, panY}

  const resetView = useCallback(() => setView({ zoom: 1, panX: 0, panY: 0 }), [])

  const onPointerDown = useCallback((e) => {
    // 描画は canvas 内で完結するのでポインタを捕捉しておく。枠外へ出ても
    // ドラッグが続き、離した位置で確実に終わる
    e.currentTarget.setPointerCapture(e.pointerId)
    dragRef.current = {
      pointerId: e.pointerId,
      startX: e.clientX, startY: e.clientY,
      panX: view.panX, panY: view.panY,
    }
  }, [view.panX, view.panY])

  const onPointerMove = useCallback((e) => {
    const d = dragRef.current
    if (!d || d.pointerId !== e.pointerId) return
    setView((v) => ({
      ...v,
      panX: d.panX + (e.clientX - d.startX),
      panY: d.panY + (e.clientY - d.startY),
    }))
  }, [])

  const onPointerUp = useCallback((e) => {
    if (dragRef.current?.pointerId === e.pointerId) dragRef.current = null
  }, [])

  const onWheel = useCallback((e) => {
    e.preventDefault()
    setView((v) => {
      const next = e.deltaY < 0 ? v.zoom * ZOOM_STEP : v.zoom / ZOOM_STEP
      const zoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, next))
      if (zoom === v.zoom) return v
      // canvas 中心を基準に拡大する。パンも同じ倍率で伸ばすと、
      // 中心に見えていたものが中心に留まる
      const k = zoom / v.zoom
      return { zoom, panX: v.panX * k, panY: v.panY * k }
    })
  }, [])

  // ── オフスクリーン: mapData が変わった時だけ地図ビットマップを再構築 ──
  useEffect(() => {
    if (!mapData) { offscreenRef.current = null; return }
    const { width, height } = mapData.info
    const data = mapData.data
    if (!offscreenRef.current) offscreenRef.current = document.createElement('canvas')
    const off = offscreenRef.current
    off.width = width
    off.height = height
    const ctx = off.getContext('2d')
    const img = ctx.createImageData(width, height)
    for (let row = 0; row < height; row++) {
      for (let col = 0; col < width; col++) {
        const srcIdx = row * width + col
        // OccupancyGrid の行0はmapフレーム下端、ImageDataの行0は上端 → 上下反転
        const destRow = height - 1 - row
        const destIdx = (destRow * width + col) * 4
        const v = data[srcIdx]
        const gray = v < 0 ? 128 : 255 - Math.round(v * 2.55)   // -1未知→灰, 0自由→白, 100占有→黒
        img.data[destIdx] = img.data[destIdx + 1] = img.data[destIdx + 2] = gray
        img.data[destIdx + 3] = 255
      }
    }
    ctx.putImageData(img, 0, 0)
  }, [mapData])

  // ── 表示 canvas: 地図 + ルート + 点群 + ロボットマーカーを重畳描画 ──
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas || !mapData || !offscreenRef.current) return
    const ctx = canvas.getContext('2d')
    const mv = computeMapView(mapData.info, canvas.width, canvas.height, view)

    ctx.imageSmoothingEnabled = false   // セル境界をくっきり
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    ctx.drawImage(offscreenRef.current, mv.offX, mv.offY, mv.drawW, mv.drawH)

    // ── ルート (Nav2 /plan、既にmapフレーム) ──
    if (pathData?.poses?.length > 1) {
      ctx.beginPath()
      pathData.poses.forEach((p, i) => {
        const [px, py] = worldToCanvas(
          p.pose.position.x, p.pose.position.y, mapData.info, mv)
        if (i === 0) ctx.moveTo(px, py)
        else ctx.lineTo(px, py)
      })
      ctx.strokeStyle = '#ffb74d'
      ctx.lineWidth = 2
      ctx.stroke()
    }

    // ── 点群 (/scan_filtered。laser_link は base_link と x/y/yaw が同一
    // (Zのみ車輪半径分違う、th_robot.urdf.xacro laser_joint) なので
    // robotPose をそのまま使ってよい) ──
    if (scanData && robotPose) {
      const { angle_min, angle_increment, ranges, range_max } = scanData
      const maxRange = Math.min(range_max, MAX_SCAN_RANGE_M)
      const cosR = Math.cos(robotPose.yaw), sinR = Math.sin(robotPose.yaw)
      ctx.fillStyle = '#ef5350'
      for (let i = 0; i < ranges.length; i++) {
        const r = ranges[i]
        if (!Number.isFinite(r) || r <= 0 || r > maxRange) continue
        const angle = angle_min + i * angle_increment
        const localX = r * Math.cos(angle)
        const localY = r * Math.sin(angle)
        const wx = robotPose.x + localX * cosR - localY * sinR
        const wy = robotPose.y + localX * sinR + localY * cosR
        const [px, py] = worldToCanvas(wx, wy, mapData.info, mv)
        ctx.fillRect(px - 1, py - 1, 2, 2)
      }
    }

    // ── ロボットマーカー ──
    if (robotPose) {
      const [px, py] = worldToCanvas(robotPose.x, robotPose.y, mapData.info, mv)
      const originYaw = quatToYaw(mapData.info.origin.orientation)
      const markerYaw = robotPose.yaw - originYaw

      ctx.save()
      ctx.translate(px, py)
      // 三角形はデフォルトで画面上向き(-Y)に描く。canvas上で画面上向きは
      // 地図上の world+Y方向 = yaw90°に相当するため、そのままだと基準が
      // 90°ズレる。ctx.rotate は時計回り正 (Y下向き画面座標) のため、
      // markerYaw=0(world+X, 画面右向き)で90°回す必要がある補正
      // (+Math.PI/2) を追加する。
      ctx.rotate(Math.PI / 2 - markerYaw)
      ctx.beginPath()
      ctx.moveTo(0, -9); ctx.lineTo(-7, 7); ctx.lineTo(7, 7); ctx.closePath()
      ctx.fillStyle = '#90caf9'
      ctx.fill()
      ctx.restore()
    }
  }, [mapData, robotPose, scanData, pathData, view])

  if (!mapData) {
    return (
      <p className="note map-placeholder">
        地図がまだありません。「地図作成開始」を押すと表示されます。
      </p>
    )
  }

  const moved = view.zoom !== 1 || view.panX !== 0 || view.panY !== 0

  return (
    <div className="map-view">
      <canvas
        ref={canvasRef}
        className="map-canvas"
        width={CANVAS_SIZE}
        height={CANVAS_SIZE}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
        onWheel={onWheel}
      />
      <div className="map-view-controls">
        <span className="note">ドラッグで移動 / ホイールで拡大縮小</span>
        <span className="map-zoom">×{view.zoom.toFixed(1)}</span>
        <button className="mode-btn secondary" disabled={!moved} onClick={resetView}>
          全体表示
        </button>
      </div>
    </div>
  )
}
