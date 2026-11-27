// parts/SpeedGauge.jsx — S-11 の計器（Spec-webui.md §3.5 D81・D82）。
//
// 大きな速さと回転／速さの帯（塗り＝実際・太い縦線＝指令・点線＝上限・
// 黄＝自動ブレーキで削った分）／直近 5 秒の線／左右の車輪の帯（間に機体の絵と
// ガイド線の縮小）。モーター出力とエンコーダは機体から届かないので「—」を出す
// （値を作らない）。S-13・W-6 には入れない。
import {
  S11_GAUGE_ACTUAL, S11_GAUGE_TARGET, S11_GAUGE_OUTPUT, S11_GAUGE_ENCODER,
  S11_GAUGE_LEFT, S11_GAUGE_RIGHT, S11_GAUGE_SPEED_UNIT, S11_GAUGE_ROT_UNIT,
  S11_GAUGE_PERCENT, S11_GAUGE_PER_SEC, S11_GAUGE_NO_VALUE,
} from '../i18n/screens.js'
import { latestSpeed, history5, limitOf, brakeCutOf, gapOf } from './surroundGeom.js'

const BAND_W = 320
const BAND_H = 64
const LINE_W = 320
const LINE_H = 72

function fmt(v, digits = 2) {
  return v == null ? S11_GAUGE_NO_VALUE : v.toFixed(digits)
}

// 帯の横方向の目安（正負の向き。上が前進ではない）。範囲は指令・実測・上限の最大。
function bandScale(values) {
  const max = Math.max(0.1, ...values.filter((v) => v != null).map((v) => Math.abs(v)))
  return max
}

export default function SpeedGauge({
  measured = [],
  command = [],
  twist = null,
  limiter = null,
  nowSec = null,
  testId = 's11-gauge',
}) {
  const now = nowSec ?? Date.now() / 1000
  const actual = latestSpeed(measured)
  const target = latestSpeed(command)
  const latestM = measured[measured.length - 1] ?? null
  const latestC = command[command.length - 1] ?? null
  const wz = twist?.wz ?? null
  const limit = limitOf(limiter)
  const cut = brakeCutOf(limiter)
  const gap = gapOf(actual, target)
  const gapWarn = gap != null && gap > 0.05

  const histA = history5(measured, now)
  const histC = history5(command, now)

  // 帯（横長: 左が後退、右が前進）
  const scale = bandScale([actual, target, limit])
  const xOf = (v) => (BAND_W / 2) * (1 + v / scale)
  const bandPts = (hist, thick) => hist.map((s) => `${xOf(s.v).toFixed(1)},${LINE_H / 2 - (s.v / scale) * (LINE_H / 2 - 4)}`).join(' ')

  const wheelBand = (side, key) => {
    const act = latestM?.[key] ?? null
    const tgt = latestC?.[key] ?? null
    const s = bandScale([act, tgt])
    const h = act == null ? 0 : Math.abs(act) / s
    return (
      <div className="gauge-wheel" data-testid={`gauge-wheel-${side}`}>
        <svg viewBox={`0 0 40 ${BAND_H}`} className="gauge-wheel-svg" aria-hidden="true">
          <rect x="8" y="4" width="24" height={BAND_H - 8} fill="none" stroke="#9aa3ad" strokeWidth="1.5" />
          {act != null && (
            <rect
              x="8"
              y={act >= 0 ? BAND_H / 2 - h * (BAND_H / 2 - 4) : BAND_H / 2}
              width="24"
              height={Math.max(2, h * (BAND_H / 2 - 4))}
              fill="#5b6675"
              data-testid={`gauge-wheel-${side}-actual`}
            />
          )}
          {tgt != null && <line x1="4" x2="36" y1={BAND_H / 2 - (tgt / s) * (BAND_H / 2 - 4)} y2={BAND_H / 2 - (tgt / s) * (BAND_H / 2 - 4)} stroke="#222" strokeWidth="2.5" data-testid={`gauge-wheel-${side}-target`} />}
        </svg>
        <span className="gauge-wheel-name">{side === 'left' ? S11_GAUGE_LEFT : S11_GAUGE_RIGHT}</span>
      </div>
    )
  }

  return (
    <div className={`gauge${gapWarn ? ' gap-warn' : ''}`} data-testid={testId}>
      <div className="gauge-top">
        <div className="gauge-big">
          <b data-testid="gauge-speed">{fmt(actual)}</b>
          <small>{S11_GAUGE_SPEED_UNIT}</small>
        </div>
        <div className="gauge-rot" data-testid="gauge-rot">
          <svg viewBox="0 0 40 40" className="gauge-rot-svg" aria-hidden="true">
            <path d="M 8 28 A 14 14 0 0 1 32 28" fill="none" stroke="#5b6675" strokeWidth="2.5" markerEnd="url(#gauge-arrow)" />
            <defs>
              <marker id="gauge-arrow" markerWidth="6" markerHeight="6" refX="3" refY="3" orient="auto">
                <path d="M0,0 L6,3 L0,6 Z" fill="#5b6675" />
              </marker>
            </defs>
          </svg>
          <b>{fmt(wz)}</b>
          <small>{S11_GAUGE_ROT_UNIT}</small>
        </div>
      </div>

      <svg viewBox={`0 0 ${BAND_W} ${BAND_H}`} className="gauge-band" data-testid="gauge-band" role="img" aria-label="速さの帯">
        <line x1={xOf(0)} y1="4" x2={xOf(0)} y2={BAND_H - 4} stroke="#c9ced6" strokeWidth="1.5" />
        {limit != null && <line x1={xOf(limit)} y1="4" x2={xOf(limit)} y2={BAND_H - 4} stroke="#5b6675" strokeWidth="2" strokeDasharray="4 3" data-testid="gauge-limit" />}
        {actual != null && limit != null && cut > 0 && (
          <rect x={xOf(Math.min(actual, limit))} y="10" width={Math.max(2, Math.abs(xOf(limit) - xOf(actual)))} height={BAND_H - 20} fill="#ffd100" data-testid="gauge-cut" />
        )}
        {actual != null && <rect x={xOf(0)} y={actual >= 0 ? BAND_H / 2 - (Math.abs(actual) / scale) * (BAND_H / 2 - 4) : BAND_H / 2} width={Math.abs(xOf(actual) - xOf(0))} height={Math.max(3, (Math.abs(actual) / scale) * (BAND_H / 2 - 4))} fill="#5b6675" data-testid="gauge-actual" />}
        {target != null && <line x1={xOf(target)} y1="6" x2={xOf(target)} y2={BAND_H - 6} stroke="#222" strokeWidth="4" data-testid="gauge-command" />}
      </svg>

      <svg viewBox={`0 0 ${LINE_W} ${LINE_H}`} className="gauge-line" data-testid="gauge-history" role="img" aria-label="直近5秒の速さ">
        <line x1="0" y1={LINE_H / 2} x2={LINE_W} y2={LINE_H / 2} stroke="#c9ced6" strokeWidth="1" />
        {histC.length > 1 && <polyline points={bandPts(histC)} fill="none" stroke="#9aa3ad" strokeWidth="1.5" data-testid="gauge-history-command" />}
        {histA.length > 1 && <polyline points={bandPts(histA)} fill="none" stroke="#333" strokeWidth="3" data-testid="gauge-history-actual" />}
      </svg>

      <div className="gauge-wheels">
        {wheelBand('left', 'left')}
        <div className="gauge-middle" aria-hidden="true">
          <svg viewBox="0 0 64 64" className="gauge-mini">
            <rect x="22" y="18" width="20" height="28" fill="#fff" stroke="#333" strokeWidth="2" />
            <line x1="27" y1="18" x2="27" y2="2" stroke="#4caf50" strokeWidth="2" opacity="0.7" />
            <line x1="37" y1="18" x2="37" y2="2" stroke="#4caf50" strokeWidth="2" opacity="0.7" />
          </svg>
        </div>
        {wheelBand('right', 'right')}
      </div>

      <table className="gauge-values">
        <tbody>
          <tr>
            <td>{S11_GAUGE_ACTUAL}</td>
            <td className="r" data-testid="gauge-actual-value">{fmt(actual)} {S11_GAUGE_SPEED_UNIT}</td>
          </tr>
          <tr>
            <td>{S11_GAUGE_TARGET}</td>
            <td className="r" data-testid="gauge-target-value">{fmt(target)} {S11_GAUGE_SPEED_UNIT}</td>
          </tr>
          <tr>
            <td>{S11_GAUGE_OUTPUT}</td>
            <td className="r" data-testid="gauge-output-value">{S11_GAUGE_NO_VALUE} {S11_GAUGE_PERCENT}</td>
          </tr>
          <tr>
            <td>{S11_GAUGE_ENCODER}</td>
            <td className="r" data-testid="gauge-encoder-value">{S11_GAUGE_NO_VALUE} {S11_GAUGE_PER_SEC}</td>
          </tr>
        </tbody>
      </table>
    </div>
  )
}
