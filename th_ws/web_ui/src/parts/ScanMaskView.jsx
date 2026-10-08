// parts/ScanMaskView.jsx — S-30 LiDAR の読み取り専用ビュー（Spec-checks.md §2.4 #4-1・#4-3）。
// 生スキャンのライブ表示に、いま登録されている死角マスクを重ねる。BlindScanSelect と
// 同じ座標規約（上が前方 0°・左が +90°）。マスクは呼び出し側が機体の値（lidar_filter の
// blind_angle_ranges）から渡す。ここにも画面にも値を直書きしない。
import { scanPointsByMask, sectorWidthDeg } from '../screens/calibCore.js'

const SIZE = 400
const C = SIZE / 2
const SCALE = 80          // px / m（端まで 2.5 m）
const R = C - 6

function polar(deg, r) {
  const a = (deg * Math.PI) / 180
  return [C - r * Math.sin(a), C - r * Math.cos(a)]
}

function wedge([a0, a1]) {
  const w = sectorWidthDeg(a0, a1)
  const [x0, y0] = polar(a0, R)
  const [x1, y1] = polar(a0 + w, R)
  return `M ${C} ${C} L ${x0} ${y0} A ${R} ${R} 0 ${w > 180 ? 1 : 0} 0 ${x1} ${y1} Z`
}

export default function ScanMaskView({ scan, ranges }) {
  const { kept, masked } = scanPointsByMask(scan, ranges)
  const toXY = (p) => [C - p.y * SCALE, C - p.x * SCALE]
  return (
    <svg viewBox={`0 0 ${SIZE} ${SIZE}`} data-testid="s30-lidar-view"
      style={{ width: '100%', maxWidth: 420 }}>
      <circle cx={C} cy={C} r={R} fill="none" stroke="currentColor" strokeOpacity="0.25" />
      <circle cx={C} cy={C} r={R / 2} fill="none" stroke="currentColor" strokeOpacity="0.15" />
      {ranges.map((rg, i) => (
        <path key={`m-${i}`} d={wedge(rg)} fill="gray" fillOpacity="0.30"
          data-testid="s30-lidar-mask-wedge" />
      ))}
      {kept.map((p, i) => { const [x, y] = toXY(p); return <circle key={`k${i}`} cx={x} cy={y} r="1.8" fill="currentColor" /> })}
      {masked.map((p, i) => { const [x, y] = toXY(p); return <circle key={`x${i}`} cx={x} cy={y} r="2.4" fill="crimson" /> })}
      <polygon points={`${C},${C - 10} ${C - 6},${C + 6} ${C + 6},${C + 6}`} fill="currentColor" />
    </svg>
  )
}
