// parts/WheelSpeedView.jsx — left/right wheel 指令 vs 実測 time-series
// (Spec-webui §3.12: モーターのグラフ。モックアップ motorGraph と同じ中身).
//
// 色を使わず、細い線＝指令・太い線＝実測。左の車輪のグラフを左に置き、
// 位置で左右を示す。グラフの下に実測を大きく、指令を小さく出す。
// 中央の点線が 0。直近 15 秒。機体から届かない値は「—」を出す（値は作らない）。
//
// 一例の hook の真実は ros/useWheelSpeeds.js の形 ({ measured: [{t,left,right}],
// command: [{t,left,right}] })。
import { useMemo } from 'react'
import { WHEEL_SPEED_WAIT } from '../i18n/screens.js'

const Ws = 250
const Hs = 150
const HISTORY_SEC = 15
const MIN_SPAN_MPS = 0.15

function fmt(v) {
  return v == null ? '—' : v.toFixed(2)
}

function fmtSigned(v) {
  if (v == null) return '—'
  return `${v < 0 ? '−' : ''}${Math.abs(v).toFixed(2)}`
}

// 指令をなぞる（細い線）・実測を追う（太い線）。時刻は秒。
function linePoints(samples, side, now, X, Y) {
  return samples
    .map((s) => {
      const v = side === 'L' ? s.left : s.right
      if (typeof v !== 'number' || typeof s.t !== 'number') return null
      const dt = now - s.t
      if (dt < 0 || dt > HISTORY_SEC) return null
      return `${X(dt).toFixed(1)},${Y(v).toFixed(1)}`
    })
    .filter(Boolean)
    .join(' ')
}

export default function WheelSpeedView({ measured = [], command = [], nowSec = null }) {
  const now = nowSec ?? Date.now() / 1000

  const scaled = useMemo(() => {
    const all = [...measured, ...command].flatMap((s) => [s.left, s.right])
    const max = Math.max(MIN_SPAN_MPS, ...all.map((v) => (typeof v === 'number' ? Math.abs(v) : 0)))
    return Math.ceil(max * 1.15 * 10) / 10
  }, [measured, command])

  const X = (dt) => 4 + ((HISTORY_SEC - dt) / HISTORY_SEC) * (Ws - 8)
  const Y = (v) => Hs / 2 - (v / scaled) * (Hs / 2 - 6)

  const hasData = measured.length > 0 || command.length > 0
  const latestM = measured[measured.length - 1] ?? null
  const latestC = command[command.length - 1] ?? null

  const graph = (side) => (
    <div className="mg" data-testid={`wheel-speed-${side === 'L' ? 'left' : 'right'}`}>
      <svg
        viewBox={`0 0 ${Ws} ${Hs}`}
        className="wheel-speed-svg"
        role="img"
        aria-label={side === 'L' ? '左の車輪の指令と実測' : '右の車輪の指令と実測'}
        preserveAspectRatio="none"
      >
        <rect x="1" y="1" width={Ws - 2} height={Hs - 2} fill="var(--inset)" stroke="var(--bd)" strokeWidth="1.5" />
        <line x1="4" y1={Hs / 2} x2={Ws - 4} y2={Hs / 2} stroke="var(--mut)" strokeWidth="1" strokeDasharray="4 4" />
        {linePoints(command, side, now, X, Y) && (
          <polyline
            points={linePoints(command, side, now, X, Y)}
            fill="none" stroke="var(--tx)" strokeWidth="1.6" strokeOpacity="0.6"
            strokeLinejoin="round" data-testid={`wheel-speed-cmd-${side === 'L' ? 'left' : 'right'}`}
          />
        )}
        {linePoints(measured, side, now, X, Y) && (
          <polyline
            points={linePoints(measured, side, now, X, Y)}
            fill="none" stroke="var(--tx)" strokeWidth="3.2"
            strokeLinejoin="round" data-testid={`wheel-speed-meas-${side === 'L' ? 'left' : 'right'}`}
          />
        )}
      </svg>
      <b data-testid={`wheel-speed-meas-value-${side === 'L' ? 'left' : 'right'}`}>
        {fmtSigned(side === 'L' ? latestM?.left : latestM?.right)}
      </b>
      <small data-testid={`wheel-speed-cmd-value-${side === 'L' ? 'left' : 'right'}`}>
        指令 {fmtSigned(side === 'L' ? latestC?.left : latestC?.right)}
      </small>
    </div>
  )

  return (
    <div className="wheel-speed-wrap" data-testid="wheel-speed-view">
      {!hasData && <p className="note">{WHEEL_SPEED_WAIT}</p>}
      <div className="mgs">
        {graph('L')}
        <span className="mgc" aria-hidden="true">
          <svg viewBox="0 0 60 60" width="48" height="48">
            <rect x="6" y="6" width="48" height="48" rx="4" fill="var(--inset)" stroke="var(--tx)" strokeWidth="2" />
            <path d="M20 40L30 16L40 40Z" fill="var(--tx)" />
          </svg>
          <small>m/s</small>
        </span>
        {graph('R')}
      </div>
      {/* 常設の現在値（ホバー不要。タブレットのタッチで hover が効かないため）。 */}
      <div className="wsv-tooltip" data-testid="wheel-speed-current">
        <span>左 実測 {fmt(latestM?.left)} 指令 {fmt(latestC?.left)}</span>
        <span>右 実測 {fmt(latestM?.right)} 指令 {fmt(latestC?.right)}</span>
      </div>
    </div>
  )
}
