// parts/SurroundView.jsx — S-11 機体のまわりの図（Spec-webui.md §3.5）。
//
// 機体を中心に LiDAR の点・機体の外形・減速が始まる距離の輪・死角の扇形・
// 進む向きのガイド線を描く。値はすべて機体から届いたもの（/scan_filtered、
// blind_angle_ranges、WheelFeedback+/odom のガイド、LimiterStatus の距離）。
// 死角の文は出さない（D69）。
import { scanToPoints } from './scanToPoints.js'
import { sectorWidthDeg } from '../screens/calibCore.js'
import { MAP_NEAR, MAP_SCAN, MAP_GO, MAP_WARN, SLOWDOWN_DISTANCE_M, ROBOT_HALF_M, ROBOT_SIZE_M } from './mapColors.js'
import { GUIDE_LENGTH_M } from './surroundGeom.js'

const SIZE = 400
const C = SIZE / 2
const SCALE = 70 // px / m
const BLIND_R_M = 2.2

function polarDeg(deg, rPx) {
  const a = (deg * Math.PI) / 180
  return [C - rPx * Math.sin(a), C - rPx * Math.cos(a)]
}

function wedge(rg) {
  const [a0] = rg
  const w = sectorWidthDeg(rg[0], rg[1])
  const R = BLIND_R_M * SCALE
  const [x0, y0] = polarDeg(a0, R)
  const [x1, y1] = polarDeg(a0 + w, R)
  return `M ${C} ${C} L ${x0} ${y0} A ${R} ${R} 0 ${w > 180 ? 1 : 0} 0 ${x1} ${y1} Z`
}

// 機体座標 (x=前方, y=左) -> SVG 座標（上が前方、左が左。ScanMaskView と同じ規約）
const toXY = (p) => [C - p.y * SCALE, C - p.x * SCALE]

function offsetPoints(points, halfW) {
  // 各点で進路の接線に垂直な方向へ halfW ずらした 2 本の線を作る。
  const left = []
  const right = []
  for (let i = 0; i < points.length; i += 1) {
    const prev = points[Math.max(0, i - 1)]
    const next = points[Math.min(points.length - 1, i + 1)]
    let tx = next.x - prev.x
    let ty = next.y - prev.y
    const len = Math.hypot(tx, ty) || 1
    tx /= len; ty /= len
    // 法線（左側）
    const nx = -ty
    const ny = tx
    left.push({ x: points[i].x + nx * halfW, y: points[i].y + ny * halfW })
    right.push({ x: points[i].x - nx * halfW, y: points[i].y - ny * halfW })
  }
  return [left, right]
}

function distAlong(points, target) {
  // 進路に沿って target m の所の点と接線を返す（横線用）。
  let acc = 0
  for (let i = 1; i < points.length; i += 1) {
    const seg = Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y)
    if (acc + seg >= target) {
      const f = seg === 0 ? 0 : (target - acc) / seg
      return {
        x: points[i - 1].x + (points[i].x - points[i - 1].x) * f,
        y: points[i - 1].y + (points[i].y - points[i - 1].y) * f,
        tx: points[i].x - points[i - 1].x,
        ty: points[i].y - points[i - 1].y,
      }
    }
    acc += seg
  }
  return null
}

const ptsAttr = (pts) => pts.map((p) => toXY(p).join(',')).join(' ')

export default function SurroundView({
  scan = null,
  blindRanges = [],
  guide = null,
  brakeActive = false,
  nearText = null,
  testId = 's11-surround',
}) {
  const points = scanToPoints(scan)
  const guideStroke = brakeActive ? MAP_WARN : MAP_GO
  const [gLeft, gRight] = guide?.kind === 'path' ? offsetPoints(guide.points, ROBOT_HALF_M) : [[], []]

  return (
    <svg viewBox={`0 0 ${SIZE} ${SIZE}`} data-testid={testId} className="surround-svg" role="img" aria-label="機体のまわりの図">
      <circle cx={C} cy={C} r={SLOWDOWN_DISTANCE_M * SCALE} fill="none" stroke="#9aa3ad" strokeWidth="1.5" strokeDasharray="6 4" data-testid="surround-slow-ring" />
      {blindRanges.map((rg, i) => (
        <path key={`b${i}`} d={wedge(rg)} fill="#9aa3ad" fillOpacity="0.25" data-testid="surround-blind-wedge" />
      ))}
      {points.map((p, i) => {
        const [x, y] = toXY(p)
        const d = Math.hypot(p.x, p.y)
        return <circle key={`p${i}`} cx={x} cy={y} r="2.5" fill={d <= SLOWDOWN_DISTANCE_M ? MAP_NEAR : MAP_SCAN} data-testid={d <= SLOWDOWN_DISTANCE_M ? 'surround-point-near' : undefined} />
      })}
      {guide?.kind === 'path' && (
        <g data-testid="surround-guide" stroke={guideStroke} strokeWidth="2" fill="none" opacity="0.75">
          <polyline points={ptsAttr(gLeft)} />
          <polyline points={ptsAttr(gRight)} />
          {[0.5, 1.0, GUIDE_LENGTH_M].map((t) => {
            const at = distAlong(guide.points, t)
            if (!at) return null
            const len = Math.hypot(at.tx, at.ty) || 1
            const nx = (-at.ty / len) * ROBOT_HALF_M
            const ny = (at.tx / len) * ROBOT_HALF_M
            const [x1, y1] = toXY({ x: at.x - nx, y: at.y - ny })
            const [x2, y2] = toXY({ x: at.x + nx, y: at.y + ny })
            return <line key={t} x1={x1} y1={y1} x2={x2} y2={y2} />
          })}
        </g>
      )}
      {guide?.kind === 'arc' && (
        <g data-testid="surround-guide-arc" stroke={guideStroke} strokeWidth="2.5" fill="none" opacity="0.8">
          <path d={`M ${C} ${C - guide.r * SCALE} A ${guide.r * SCALE} ${guide.r * SCALE} 0 0 ${guide.sweep} ${guide.sweep === 0 ? C - guide.r * SCALE : C + guide.r * SCALE} ${C}`} />
        </g>
      )}
      {/* 機体の外形（0.55 m 角）＋前向き */}
      <g data-testid="surround-robot">
        <rect
          x={C - (ROBOT_SIZE_M / 2) * SCALE}
          y={C - (ROBOT_SIZE_M / 2) * SCALE}
          width={ROBOT_SIZE_M * SCALE}
          height={ROBOT_SIZE_M * SCALE}
          fill="#e8f5e9" stroke="#2e7d32" strokeWidth="2.5"
        />
        <polygon points={`${C},${C - (ROBOT_HALF_M + 0.10) * SCALE} ${C - 7},${C - (ROBOT_HALF_M - 0.08) * SCALE} ${C + 7},${C - (ROBOT_HALF_M - 0.08) * SCALE}`} fill="#2e7d32" />
      </g>
      {nearText && (
        <text x={C + (ROBOT_HALF_M + 0.35) * SCALE} y={C - (ROBOT_HALF_M + 0.1) * SCALE} textAnchor="start" fontSize="17" fontWeight="700" fill="#8a6d00" data-testid="surround-near-label">
          {nearText}
        </text>
      )}
    </svg>
  )
}
