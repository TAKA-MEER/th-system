// ============================================================
// WorldCanvas.jsx — 観客向け「センサが見る世界」(VISION.md §6.3)
//
// 主役は点群そのものではなく、点群の上に浮かぶ**人の位置**。
// 搭載センサは 2D LiDAR (RPLIDAR S1) のみで 3D 点群は存在しないため、
// 点の密度で魅せることはできない。「候補の中から 1 人を選んで追っている」
// 構図が伝わることを最優先に描画順と明度を決めてある。
//
// 座標系は地図の有無で切り替える:
//   地図あり → map フレーム (地図が育っていく様子が見える)
//   地図なし → ロボット中心 (FOLLOWING_MAPLESS で地図作成前でも成立させる)
// ============================================================
import { useRef, useEffect } from 'react'

import { quatToYaw, worldToCanvas, baseToWorld } from '../mapGeometry.js'

const MAX_SCAN_RANGE_M = 12.0   // RPLIDAR S1 の最大レンジ。異常値の足切り
// 追跡中ターゲットと候補の同一判定距離 (m)。App.jsx の CandidateRadar と同値。
// 一致する候補は追跡マーカーに置き換える (二重描画で番号が潰れるのを防ぐ)
const TRACKED_MATCH_DIST = 0.35
const ROBOT_VIEW_RANGE_M = 6.0  // ロボット中心表示のときの表示半径
const SCAN_TRAIL = 4            // 点群の残光フレーム数
const TRAIL_MAX = 600           // 走行軌跡の保持点数 (上限付きリングバッファ)

export const COLOR = {
  scan:      '#ef5350',
  candidate: '#b0bec5',
  target:    '#4caf50',
  path:      '#ffb74d',
  robot:     '#90caf9',
  trail:     '#546e7a',
}

// 描画対象を1つの座標変換インタフェースに揃える。地図あり/なしで
// 分岐するのはこの関数を作るところだけにして、描画本体は共通にする
function makeProjector(mapData, robotPose, w, h) {
  if (mapData) {
    return {
      // map フレームの点 → canvas
      world: (x, y) => worldToCanvas(x, y, mapData.info, w, h),
      // base_link 相対の点 → canvas (robotPose 経由で map へ)
      base: (lx, ly) => {
        const [wx, wy] = baseToWorld(lx, ly, robotPose)
        return worldToCanvas(wx, wy, mapData.info, w, h)
      },
      // 地図の向きに合わせたロボットの見かけ角
      markerYaw: robotPose.yaw - quatToYaw(mapData.info.origin.orientation),
      mPerPx: mapData.info.resolution * (mapData.info.width / w),
    }
  }
  // ロボット中心。ロボットを画面中央に固定し、前方を上に向ける。
  // 世界の側が回るため base_link 相対の点はそのまま置ける
  const scale = Math.min(w, h) / 2 / ROBOT_VIEW_RANGE_M
  const cx = w / 2, cy = h / 2
  return {
    world: (x, y) => {
      if (!robotPose) return [cx, cy]
      const dx = x - robotPose.x, dy = y - robotPose.y
      const cos = Math.cos(-robotPose.yaw), sin = Math.sin(-robotPose.yaw)
      const lx = dx * cos - dy * sin, ly = dx * sin + dy * cos
      return [cx - ly * scale, cy - lx * scale]
    },
    base: (lx, ly) => [cx - ly * scale, cy - lx * scale],
    markerYaw: Math.PI / 2,   // 常に上向き
    mPerPx: 1 / scale,
  }
}

export default function WorldCanvas({
  mapData, robotPose, scanData, pathData, personStatus, candidates, layers,
}) {
  const canvasRef = useRef(null)
  const offscreenRef = useRef(null)   // グリッド1セル=1pxの地図ビットマップ
  const scanTrailRef = useRef([])     // 直近フレームの点群 (残光用)
  const trailRef = useRef([])         // 走行軌跡 (map座標)
  const rafRef = useRef(null)
  // 描画ループが最新値を読むための箱。ROS の受信ごとに再描画すると
  // ホスト機の CPU を食うため、描画は rAF 側で一定レートに間引く
  const dataRef = useRef({})
  dataRef.current = { mapData, robotPose, scanData, pathData, personStatus, candidates, layers }

  // ── 地図ビットマップは mapData が変わった時だけ再構築 ──
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
        const destRow = height - 1 - row   // OccupancyGrid は行0が下端
        const destIdx = (destRow * width + col) * 4
        const v = data[srcIdx]
        // 観客向けは暗背景に合わせて反転 (未知=暗い / 自由=やや明るい / 占有=白)
        let g
        if (v < 0)      g = 26
        else if (v < 50) g = 58
        else             g = 200
        img.data[destIdx] = img.data[destIdx + 1] = img.data[destIdx + 2] = g
        img.data[destIdx + 3] = 255
      }
    }
    ctx.putImageData(img, 0, 0)
  }, [mapData])

  // ── 点群の残光バッファ ──
  useEffect(() => {
    if (!scanData) return
    const buf = scanTrailRef.current
    buf.push(scanData)
    while (buf.length > SCAN_TRAIL) buf.shift()
  }, [scanData])

  // ── 走行軌跡 ──
  useEffect(() => {
    if (!robotPose) return
    const t = trailRef.current
    const last = t[t.length - 1]
    // 5cm 以上動いたときだけ点を足す (静止中に同じ点が溜まらないように)
    if (!last || Math.hypot(robotPose.x - last.x, robotPose.y - last.y) > 0.05) {
      t.push({ x: robotPose.x, y: robotPose.y })
      while (t.length > TRAIL_MAX) t.shift()
    }
  }, [robotPose])

  // ── 描画ループ (約10fps に制限) ──
  useEffect(() => {
    const FRAME_MS = 100
    let lastDraw = 0

    const draw = (now) => {
      rafRef.current = requestAnimationFrame(draw)
      if (now - lastDraw < FRAME_MS) return
      lastDraw = now

      const canvas = canvasRef.current
      if (!canvas) return
      const { mapData, robotPose, pathData, personStatus, candidates, layers } = dataRef.current

      // 表示解像度を CSS サイズに追従させる (プロジェクタ解像度でボケないように)
      const rect = canvas.getBoundingClientRect()
      const dpr = Math.min(window.devicePixelRatio || 1, 2)
      const w = Math.round(rect.width * dpr), h = Math.round(rect.height * dpr)
      if (w === 0 || h === 0) return
      if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h }

      const ctx = canvas.getContext('2d')
      ctx.fillStyle = '#0b0d12'
      ctx.fillRect(0, 0, w, h)

      // 地図フレームで描くには自己位置 (TF) が要る。地図が無い / 地図レイヤ OFF /
      // TF 未確立のいずれでもロボット中心にフォールバックする。地図作成前の
      // FOLLOWING_MAPLESS 運用でも画面が死なないことがここの狙い
      const useMap = !!(mapData && layers.map && offscreenRef.current && robotPose)
      const proj = makeProjector(useMap ? mapData : null, robotPose, w, h)

      // ── 地図 ──
      if (useMap) {
        ctx.imageSmoothingEnabled = false
        ctx.drawImage(offscreenRef.current, 0, 0, w, h)
      } else {
        drawRangeRings(ctx, w, h, proj, dpr)
      }

      const canProjectWorld = useMap || !!robotPose

      // ── 走行軌跡 ──
      if (layers.trail && canProjectWorld && trailRef.current.length > 1) {
        ctx.beginPath()
        trailRef.current.forEach((p, i) => {
          const [px, py] = proj.world(p.x, p.y)
          if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py)
        })
        ctx.strokeStyle = COLOR.trail
        ctx.lineWidth = 2 * dpr
        ctx.stroke()
      }

      // ── Nav2 経路 ──
      if (layers.path && canProjectWorld && pathData?.poses?.length > 1) {
        ctx.beginPath()
        pathData.poses.forEach((p, i) => {
          const [px, py] = proj.world(p.pose.position.x, p.pose.position.y)
          if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py)
        })
        ctx.strokeStyle = COLOR.path
        ctx.lineWidth = 3 * dpr
        ctx.stroke()
      }

      // ── 点群 (人マーカーより暗く。主役を食わせない) ──
      // 地図表示中は robotPose が要る。ロボット中心表示なら base 相対で直接置ける
      if (layers.scan && (useMap ? robotPose : true)) {
        const buf = scanTrailRef.current
        buf.forEach((scan, i) => {
          const age = buf.length - 1 - i           // 0 = 最新
          ctx.fillStyle = COLOR.scan
          ctx.globalAlpha = 0.85 * Math.pow(0.55, age)
          const size = (age === 0 ? 3 : 2) * dpr
          drawScan(ctx, scan, proj, size)
        })
        ctx.globalAlpha = 1
      }

      // ── ★ 脚検出候補 (base_link 相対) ──
      // 追跡中の対象と重なる候補は描かない。同じ場所に灰丸と緑リングが
      // 重なると番号が潰れ、「候補の中の1人を追っている」構図も伝わらない
      const tracked = personStatus && !personStatus.is_lost ? personStatus.position : null
      if (layers.candidates && candidates?.length) {
        candidates.forEach((p, i) => {
          if (layers.target && tracked &&
              Math.hypot(p.x - tracked.x, p.y - tracked.y) < TRACKED_MATCH_DIST) return
          const [px, py] = proj.base(p.x, p.y)
          ctx.beginPath()
          ctx.arc(px, py, 13 * dpr, 0, Math.PI * 2)
          ctx.fillStyle = 'rgba(38,50,56,0.85)'
          ctx.fill()
          ctx.strokeStyle = COLOR.candidate
          ctx.lineWidth = 2 * dpr
          ctx.stroke()
          ctx.fillStyle = '#eceff1'
          ctx.font = `bold ${13 * dpr}px sans-serif`
          ctx.textAlign = 'center'
          ctx.textBaseline = 'middle'
          ctx.fillText(String(i + 1), px, py)
        })
      }

      // ── ★ 追跡対象 (この画面の主役) ──
      if (layers.target && personStatus) {
        const { x, y } = personStatus.position
        const [px, py] = proj.base(x, y)
        const lost = personStatus.is_lost
        const dist = Math.hypot(x, y)
        const t = (now % 1500) / 1500
        const pulse = 1 + 0.18 * Math.sin(t * Math.PI * 2)

        ctx.globalAlpha = lost ? 0.35 : 1
        // 脈動リング
        ctx.beginPath()
        ctx.arc(px, py, 26 * dpr * pulse, 0, Math.PI * 2)
        ctx.strokeStyle = COLOR.target
        ctx.lineWidth = 3 * dpr
        ctx.stroke()
        // 芯
        ctx.beginPath()
        ctx.arc(px, py, 11 * dpr, 0, Math.PI * 2)
        ctx.fillStyle = COLOR.target
        ctx.fill()
        // 距離ラベル
        const label = lost ? '見失い中' : `${dist.toFixed(1)} m`
        ctx.font = `bold ${17 * dpr}px sans-serif`
        ctx.textAlign = 'center'
        ctx.textBaseline = 'bottom'
        const tw = ctx.measureText(label).width
        ctx.fillStyle = 'rgba(0,0,0,0.6)'
        ctx.fillRect(px - tw / 2 - 6 * dpr, py - 52 * dpr, tw + 12 * dpr, 24 * dpr)
        ctx.fillStyle = COLOR.target
        ctx.fillText(label, px, py - 32 * dpr)
        ctx.globalAlpha = 1
      }

      // ── ロボット ──
      if (useMap ? robotPose : true) {
        const [px, py] = useMap ? proj.world(robotPose.x, robotPose.y) : [w / 2, h / 2]
        ctx.save()
        ctx.translate(px, py)
        ctx.rotate(Math.PI / 2 - proj.markerYaw)
        ctx.beginPath()
        ctx.moveTo(0, -15 * dpr); ctx.lineTo(-12 * dpr, 11 * dpr); ctx.lineTo(12 * dpr, 11 * dpr)
        ctx.closePath()
        ctx.fillStyle = COLOR.robot
        ctx.fill()
        ctx.restore()
      }
    }

    rafRef.current = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(rafRef.current)
  }, [])

  return <canvas ref={canvasRef} className="aud-canvas" />
}

function drawScan(ctx, scan, proj, size) {
  const { angle_min, angle_increment, ranges, range_max } = scan
  const maxRange = Math.min(range_max, MAX_SCAN_RANGE_M)
  const half = size / 2
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i]
    if (!Number.isFinite(r) || r <= 0 || r > maxRange) continue
    const a = angle_min + i * angle_increment
    const [px, py] = proj.base(r * Math.cos(a), r * Math.sin(a))
    ctx.fillRect(px - half, py - half, size, size)
  }
}

// ロボット中心表示のときの距離目安 (地図が無いとスケール感が掴めないため)
function drawRangeRings(ctx, w, h, proj, dpr) {
  const cx = w / 2, cy = h / 2
  ctx.strokeStyle = '#1c2028'
  ctx.fillStyle = '#3a4048'
  ctx.lineWidth = 1 * dpr
  ctx.font = `${12 * dpr}px sans-serif`
  ctx.textAlign = 'left'
  ctx.textBaseline = 'middle'
  for (let r = 1; r <= ROBOT_VIEW_RANGE_M; r++) {
    const [px] = proj.base(0, -r)   // 右方向に r メートル
    const rad = px - cx
    ctx.beginPath()
    ctx.arc(cx, cy, rad, 0, Math.PI * 2)
    ctx.stroke()
    if (r % 2 === 0) ctx.fillText(`${r}m`, cx + rad + 4, cy)
  }
}
