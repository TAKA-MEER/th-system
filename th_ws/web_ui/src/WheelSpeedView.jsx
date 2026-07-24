// ============================================================
// WheelSpeedView.jsx — 左右輪 指令 vs 実測 速度グラフ
// (PID の追従遅れ・定常偏差・振動を目視で確認するためのカード)
// ============================================================
import { useMemo, useRef, useState } from 'react'

const VIEW_W = 340
const VIEW_H = 170
const PAD_L = 34   // y軸ラベル分
const PAD_R = 10
const PAD_T = 10
const PAD_B = 20   // x軸ラベル分
const PLOT_W = VIEW_W - PAD_L - PAD_R
const PLOT_H = VIEW_H - PAD_T - PAD_B

const HISTORY_SEC = 15
const MIN_SPAN_MPS = 0.1   // 静止時にグラフが潰れないための最小レンジ

// 左右輪の色 (指令=破線/実測=実線で系列を区別するため、色は「どちらの車輪か」だけを担う)
const COLOR_LEFT  = '#64b5f6'
const COLOR_RIGHT = '#ffb74d'

function buildPath(samples, key, now, xOf, yOf) {
  if (samples.length === 0) return ''
  return samples
    .map((s, i) => `${i === 0 ? 'M' : 'L'} ${xOf(s.t - now).toFixed(1)} ${yOf(s[key]).toFixed(1)}`)
    .join(' ')
}

// カーソル位置に最も近い時刻のサンプルを返す (ホバー用)
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

export default function WheelSpeedView({ wheelSpeedData }) {
  const { measured = [], command = [] } = wheelSpeedData ?? {}
  const svgRef = useRef(null)
  const [hoverX, setHoverX] = useState(null)   // カーソルの plot 内 x 座標 (px)、非ホバー時 null

  const now = useMemo(() => Date.now() / 1000, [measured, command])

  // ── スケール ──────────────────────────────────────────────
  const xOf = (dtFromNow) => PAD_L + PLOT_W * (1 + dtFromNow / HISTORY_SEC)
  const allSpeeds = [...measured, ...command].flatMap((s) => [s.left, s.right])
  const maxAbs = Math.max(MIN_SPAN_MPS, ...allSpeeds.map((v) => Math.abs(v)), 0)
  const yOf = (v) => PAD_T + PLOT_H * (1 - (v + maxAbs) / (2 * maxAbs))

  const hasData = measured.length > 0 || command.length > 0

  // ── ホバー: カーソル x → 時刻 → 最近傍サンプル ───────────────
  const hoverT = hoverX == null ? null : now - HISTORY_SEC * (1 - (hoverX - PAD_L) / PLOT_W)
  const hoverMeasured = hoverT == null ? null : nearestSample(measured, hoverT)
  const hoverCommand  = hoverT == null ? null : nearestSample(command, hoverT)

  const handleMove = (e) => {
    const rect = svgRef.current.getBoundingClientRect()
    const px = ((e.clientX - rect.left) / rect.width) * VIEW_W
    if (px < PAD_L || px > VIEW_W - PAD_R) { setHoverX(null); return }
    setHoverX(px)
  }

  return (
    <div className="wheel-speed-wrap">
      {!hasData && <p className="note">ESP32 からの速度データ待ち…</p>}
      <svg
        ref={svgRef}
        viewBox={`0 0 ${VIEW_W} ${VIEW_H}`}
        className="wheel-speed-svg"
        onMouseMove={handleMove}
        onMouseLeave={() => setHoverX(null)}
      >
        {/* ── グリッド (ゼロ線 + 上下端) ─────────────────── */}
        <line x1={PAD_L} y1={yOf(0)} x2={VIEW_W - PAD_R} y2={yOf(0)}
              className="wsv-axis" />
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

        {/* ── 指令 (破線・細) ─────────────────────────────── */}
        <path d={buildPath(command, 'left',  now, xOf, yOf)} className="wsv-line wsv-cmd" stroke={COLOR_LEFT} />
        <path d={buildPath(command, 'right', now, xOf, yOf)} className="wsv-line wsv-cmd" stroke={COLOR_RIGHT} />
        {/* ── 実測 (実線) ─────────────────────────────────── */}
        <path d={buildPath(measured, 'left',  now, xOf, yOf)} className="wsv-line wsv-meas" stroke={COLOR_LEFT} />
        <path d={buildPath(measured, 'right', now, xOf, yOf)} className="wsv-line wsv-meas" stroke={COLOR_RIGHT} />

        {/* ── ホバー: クロスヘア + 値 ──────────────────────── */}
        {hoverX != null && (
          <>
            <line x1={hoverX} y1={PAD_T} x2={hoverX} y2={VIEW_H - PAD_B} className="wsv-crosshair" />
          </>
        )}
      </svg>

      {/* ── 凡例 ──────────────────────────────────────────── */}
      <div className="wsv-legend">
        <span className="wsv-legend-item"><i className="wsv-swatch wsv-swatch-dash" style={{ borderColor: COLOR_LEFT }} />左 指令</span>
        <span className="wsv-legend-item"><i className="wsv-swatch" style={{ background: COLOR_LEFT }} />左 実測</span>
        <span className="wsv-legend-item"><i className="wsv-swatch wsv-swatch-dash" style={{ borderColor: COLOR_RIGHT }} />右 指令</span>
        <span className="wsv-legend-item"><i className="wsv-swatch" style={{ background: COLOR_RIGHT }} />右 実測</span>
      </div>

      {/* ── ホバー時の数値読み取り ────────────────────────── */}
      {hoverX != null && (hoverMeasured || hoverCommand) && (
        <div className="wsv-tooltip">
          <div>左: 指令 {hoverCommand ? hoverCommand.left.toFixed(3) : '--'} / 実測 {hoverMeasured ? hoverMeasured.left.toFixed(3) : '--'} m/s</div>
          <div>右: 指令 {hoverCommand ? hoverCommand.right.toFixed(3) : '--'} / 実測 {hoverMeasured ? hoverMeasured.right.toFixed(3) : '--'} m/s</div>
        </div>
      )}
    </div>
  )
}
