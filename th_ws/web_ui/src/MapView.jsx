// ============================================================
// MapView.jsx — SLAM 地図 + 自己位置 + 点群 + ルート表示
// (RViz の Map + LaserScan + Path + TF 相当を軽量に再現)
// ============================================================
import { useRef, useEffect } from 'react'

const CANVAS_SIZE = 320   // 表示 canvas の一辺 (px)。CandidateRadar の 260px よりやや大きく
const MAX_SCAN_RANGE_M = 12.0   // LiDAR (RPLIDAR S1) の最大レンジ。異常値の足切りに使う

function quatToYaw(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
}

// map座標系の (x, y) [m] を canvas ピクセル座標 [px, py] に変換する。
// OccupancyGrid の行0はmapフレーム下端・originはセル(0,0)の座標という前提
// (MapView 全体でここ一箇所にまとめ、ロボットマーカー・点群・ルートで共有する)
function worldToCanvas(x, y, mapInfo, canvasW, canvasH) {
  const { resolution, width, height, origin } = mapInfo
  const originYaw = quatToYaw(origin.orientation)
  const dx = x - origin.position.x
  const dy = y - origin.position.y
  const cos = Math.cos(-originYaw), sin = Math.sin(-originYaw)
  const gx = (dx * cos - dy * sin) / resolution   // グリッドセル座標 (列)
  const gy = (dx * sin + dy * cos) / resolution   // グリッドセル座標 (行, 下端基準)
  const scaleX = canvasW / width
  const scaleY = canvasH / height
  return [gx * scaleX, canvasH - gy * scaleY]     // 地図と同じ上下反転
}

export default function MapView({ mapData, robotPose, scanData, pathData }) {
  const canvasRef = useRef(null)
  const offscreenRef = useRef(null)   // グリッド1セル=1pxの地図ビットマップ

  // ── オフスクリーン: mapData が変わった時だけ地図ビットマップを再構築 ──
  useEffect(() => {
    if (!mapData) return
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
    ctx.imageSmoothingEnabled = false   // セル境界をくっきり
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    ctx.drawImage(offscreenRef.current, 0, 0, canvas.width, canvas.height)

    // ── ルート (Nav2 /plan、既にmapフレーム) ──
    if (pathData?.poses?.length > 1) {
      ctx.beginPath()
      pathData.poses.forEach((p, i) => {
        const [px, py] = worldToCanvas(
          p.pose.position.x, p.pose.position.y, mapData.info, canvas.width, canvas.height)
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
        const [px, py] = worldToCanvas(wx, wy, mapData.info, canvas.width, canvas.height)
        ctx.fillRect(px - 1, py - 1, 2, 2)
      }
    }

    // ── ロボットマーカー ──
    if (robotPose) {
      const [px, py] = worldToCanvas(
        robotPose.x, robotPose.y, mapData.info, canvas.width, canvas.height)
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
  }, [mapData, robotPose, scanData, pathData])

  if (!mapData) {
    return (
      <p className="note map-placeholder">
        地図がまだありません。「地図作成開始」を押すと表示されます。
      </p>
    )
  }

  return (
    <canvas
      ref={canvasRef}
      className="map-canvas"
      width={CANVAS_SIZE}
      height={CANVAS_SIZE}
    />
  )
}
