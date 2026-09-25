// parts/WheelSpeedView.jsx — left/right wheel 指令 vs 実測 time-series
// (DetailedDesign-webui.md §3: "WheelSpeedView.jsx -- そのまま -- S-30 の
// 項目2（指令 vs 実測）に流用"). Ported from the pre-refactor
// th_ws/web_ui/src/WheelSpeedView.jsx (commit aa5a5b5, App.jsx era) into
// parts/, fed by ros/useWheelSpeeds.js instead of the old useRosbridge.js.
//
// One addition over the original: a permanent "current values" line below
// the graph, not gated on hover. The original only showed numbers via
// onMouseMove, which doesn't fire reliably from touch on the field tablets
// this UI targets (Spec-webui.md §0 "PCとタブレットの両方"); hover stays as
// a bonus for desktop/mouse.
import { useMemo, useRef, useState } from 'react'
import {
  S30_MOTOR_CMD_L, S30_MOTOR_CMD_R, S30_MOTOR_MEAS_L, S30_MOTOR_MEAS_R,
} from '../i18n/screens.js'

const VIEW_W = 340
const VIEW_H = 170
const PAD_L = 34
const PAD_R = 10
const PAD_T = 10
const PAD_B = 20
const PLOT_W = VIEW_W - PAD_L - PAD_R
const PLOT_H = VIEW_H - PAD_T - PAD_B

const HISTORY_SEC = 15
const MIN_SPAN_MPS = 0.1

const COLOR_LEFT = '#64b5f6'
const COLOR_RIGHT = '#ffb74d'

function buildPath(samples, key, now, xOf, yOf) {
  if (samples.length === 0) return ''
  return samples
    .map((s, i) => `${i === 0 ? 'M' : 'L'} ${xOf(s.t - now).toFixed(1)} ${yOf(s[key]).toFixed(1)}`)
    .join(' ')
}

function nearestSample(samples, targetT) {
  if (samples.length === 0) return null
  let best = samples[0]
  let bestDiff = Math.abs(samples[0].t - targetT)
  for (const s of samples) {
    const diff = Math.abs(s.t - targetT)
    if (diff < bestDiff) { best = s; bestDiff = diff }
  }
  return best
}

function fmt(v) {
  return v == null ? '--' : v.toFixed(3)
}

export default function WheelSpeedView({ measured = [], command = [] }) {
  const svgRef = useRef(null)
  const [hoverX, setHoverX] = useState(null)

  const now = useMemo(() => Date.now() / 1000, [measured, command])

  const xOf = (dtFromNow) => PAD_L + PLOT_W * (1 + dtFromNow / HISTORY_SEC)
  const allSpeeds = [...measured, ...command].flatMap((s) => [s.left, s.right])
  const maxAbs = Math.max(MIN_SPAN_MPS, ...allSpeeds.map((v) => Math.abs(v)), 0)
  const yOf = (v) => PAD_T + PLOT_H * (1 - (v + maxAbs) / (2 * maxAbs))

  const hasData = measured.length > 0 || command.length > 0
  const latestMeasured = measured[measured.length - 1] ?? null
  const latestCommand = command[command.length - 1] ?? null

  const hoverT = hoverX == null ? null : now - HISTORY_SEC * (1 - (hoverX - PAD_L) / PLOT_W)
  const hoverMeasured = hoverT == null ? null : nearestSample(measured, hoverT)
  const hoverCommand = hoverT == null ? null : nearestSample(command, hoverT)
  const shownMeasured = hoverMeasured ?? latestMeasured
  const shownCommand = hoverCommand ?? latestCommand

  const handleMove = (e) => {
    const rect = svgRef.current.getBoundingClientRect()
    const px = ((e.clientX - rect.left) / rect.width) * VIEW_W
    if (px < PAD_L || px > VIEW_W - PAD_R) { setHoverX(null); return }
    setHoverX(px)
  }

  return (
    <div className="wheel-speed-wrap" data-testid="wheel-speed-view">
      {!hasData && <p className="note">ESP32 からの速度データ待ち…</p>}
      <svg
        ref={svgRef}
        viewBox={`0 0 ${VIEW_W} ${VIEW_H}`}
        className="wheel-speed-svg"
        onMouseMove={handleMove}
        onMouseLeave={() => setHoverX(null)}
      >
        <line x1={PAD_L} y1={yOf(0)} x2={VIEW_W - PAD_R} y2={yOf(0)} className="wsv-axis" />
        <line x1={PAD_L} y1={PAD_T} x2={PAD_L} y2={VIEW_H - PAD_B} className="wsv-axis" />
        <text x={PAD_L - 4} y={yOf(maxAbs) + 3} textAnchor="end" className="wsv-tick">
          {maxAbs.toFixed(2)}
        </text>
        <text x={PAD_L - 4} y={yOf(0) + 3} textAnchor="end" className="wsv-tick">0</text>
        <text x={PAD_L - 4} y={yOf(-maxAbs) + 3} textAnchor="end" className="wsv-tick">
          -{maxAbs.toFixed(2)}
        </text>
        <text x={VIEW_W - PAD_R} y={VIEW_H - 4} textAnchor="end" className="wsv-tick">今</text>
        <text x={PAD_L} y={VIEW_H - 4} textAnchor="start" className="wsv-tick">-{HISTORY_SEC}s</text>

        <path d={buildPath(command, 'left', now, xOf, yOf)} className="wsv-line wsv-cmd" stroke={COLOR_LEFT} />
        <path d={buildPath(command, 'right', now, xOf, yOf)} className="wsv-line wsv-cmd" stroke={COLOR_RIGHT} />
        <path d={buildPath(measured, 'left', now, xOf, yOf)} className="wsv-line wsv-meas" stroke={COLOR_LEFT} />
        <path d={buildPath(measured, 'right', now, xOf, yOf)} className="wsv-line wsv-meas" stroke={COLOR_RIGHT} />

        {hoverX != null && (
          <line x1={hoverX} y1={PAD_T} x2={hoverX} y2={VIEW_H - PAD_B} className="wsv-crosshair" />
        )}
      </svg>

      <div className="wsv-legend">
        <span className="wsv-legend-item"><i className="wsv-swatch wsv-swatch-dash" style={{ borderColor: COLOR_LEFT }} />{S30_MOTOR_CMD_L}</span>
        <span className="wsv-legend-item"><i className="wsv-swatch" style={{ background: COLOR_LEFT }} />{S30_MOTOR_MEAS_L}</span>
        <span className="wsv-legend-item"><i className="wsv-swatch wsv-swatch-dash" style={{ borderColor: COLOR_RIGHT }} />{S30_MOTOR_CMD_R}</span>
        <span className="wsv-legend-item"><i className="wsv-swatch" style={{ background: COLOR_RIGHT }} />{S30_MOTOR_MEAS_R}</span>
      </div>

      {/* 常設の現在値（ホバー不要。タブレットのタッチで hover が効かないため）。*/}
      <div className="wsv-tooltip" data-testid="wheel-speed-current">
        <div>{S30_MOTOR_CMD_L} {fmt(shownCommand?.left)} / {S30_MOTOR_MEAS_L} {fmt(shownMeasured?.left)} m/s</div>
        <div>{S30_MOTOR_CMD_R} {fmt(shownCommand?.right)} / {S30_MOTOR_MEAS_R} {fmt(shownMeasured?.right)} m/s</div>
      </div>
    </div>
  )
}
