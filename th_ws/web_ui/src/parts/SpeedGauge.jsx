// parts/SpeedGauge.jsx — S-11 の計器（Spec-webui.md §3.5 D81・D82、モックアップ）。
//
// 枠のあるパネル。上の段: いまの速さ（大きな数）と回転（弧の矢印＋数）。
// 次の段: 速さの帯（太く。塗り＝実際・太い縦線＝指令・点線＝上限・黄＝自動ブレーキで
// 削った分）と直近 5 秒の線（枠つき）。下の段: 左右の車輪の帯を左右に置き
// （位置で左右を示す）、間に機体の絵とガイド線の縮小。帯の下にそれぞれの実速度
// （大きく）・目標・出力 %・エンコーダ /秒を並べ、語（m/s・目標・出力・エンコーダ）は
// 帯の間の列に 1 つずつだけ。モーター出力とエンコーダは機体から届かないので
// 「—」を出す（値を作らない）。S-13・W-6 には入れない。
import {
  S11_GAUGE_ACTUAL, S11_GAUGE_TARGET, S11_GAUGE_OUTPUT, S11_GAUGE_ENCODER,
  S11_GAUGE_SPEED_UNIT, S11_GAUGE_ROT_UNIT,
  S11_GAUGE_NO_VALUE,
} from '../i18n/screens.js'
import { latestSpeed, history5, limitOf, brakeCutOf, gapOf } from './surroundGeom.js'

const BAND_W = 320
const BAND_H = 40
const LINE_W = 320
const LINE_H = 64

function fmt(v, digits = 2) {
  return v == null ? S11_GAUGE_NO_VALUE : v.toFixed(digits)
}

// 帯は大きさだけ（モックアップ: 0 を左端に、後退も長さで出す）。
function bandScale(values) {
  const max = Math.max(0.2, ...values.filter((v) => v != null).map((v) => Math.abs(v)))
  return Math.ceil(max * 1.15 * 5) / 5
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
  const inLin = typeof limiter?.in_linear === 'number' ? limiter.in_linear : null
  const outLin = typeof limiter?.out_linear === 'number' ? limiter.out_linear : null
  const gap = gapOf(actual, target)
  const gapWarn = gap != null && gap > 0.05

  const scale = bandScale([actual, target, limit, inLin, latestM?.left, latestM?.right, latestC?.left, latestC?.right])
  const histA = history5(measured, now)
  const histC = history5(command, now)

  const xOf = (v) => (Math.abs(v) / scale) * BAND_W
  const ticks = [0, scale / 3, (2 * scale) / 3, scale]

  const linePts = (hist) => hist.map((s) => {
    const x = LINE_W * (1 - Math.max(0, (now - s.t) / 5))
    const y = LINE_H / 2 - (s.v / scale) * (LINE_H / 2 - 4)
    return `${x.toFixed(1)},${y.toFixed(1)}`
  }).join(' ')

  const wheelBand = (side, key) => {
    const act = latestM?.[key] ?? null
    const tgt = latestC?.[key] ?? null
    const s = scale
    // 帯は -s..+s（上が前進）。塗りは実際の位置の印、横線は目標の位置（モックアップ）。
    const yAct = act == null ? null : 50 - (act / s) * 46
    const yTgt = tgt == null ? null : 50 - (tgt / s) * 46
    return (
      <div className="gauge-wheel" data-testid={`gauge-wheel-${side}`}>
        <svg viewBox={`0 0 40 100`} className="gauge-wheel-svg" aria-hidden="true" preserveAspectRatio="none">
          <rect x="6" y="2" width="28" height="96" fill="#eef1f5" stroke="#c9ced6" strokeWidth="1.5" />
          {yAct != null && (
            <rect
              x="6"
              y={Math.max(2, Math.min(82, yAct - 9))}
              width="28"
              height="18"
              fill="#333a48"
              data-testid={`gauge-wheel-${side}-actual`}
            />
          )}
          {yTgt != null && <line x1="2" x2="38" y1={yTgt} y2={yTgt} stroke="#111" strokeWidth="3.5" data-testid={`gauge-wheel-${side}-target`} />}
        </svg>
      </div>
    )
  }

  const wheelValues = (side, key) => {
    const act = latestM?.[key] ?? null
    const tgt = latestC?.[key] ?? null
    return (
      <div className="gauge-wheelvals" data-testid={`gauge-wheelvals-${side}`}>
        <b className="gw gw-actual" data-testid={`gauge-${side}-actual-value`}>{fmt(act)}</b>
        <span className="gw gw-target" data-testid={`gauge-${side}-target-value`}>{fmt(tgt)}</span>
        <b className="gw gw-output" data-testid={`gauge-${side}-output-value`}>{S11_GAUGE_NO_VALUE}%</b>
        <b className="gw gw-encoder" data-testid={`gauge-${side}-encoder-value`}>{S11_GAUGE_NO_VALUE}</b>
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
            <path d="M 20 6 A 14 14 0 1 1 7 13" fill="none" stroke="#333" strokeWidth="3" />
            <path d="M 4 4 L 12 3 L 10 13 Z" fill="#333" />
          </svg>
          <b>{fmt(wz)}</b>
          <small>{S11_GAUGE_ROT_UNIT}</small>
        </div>
      </div>

      <div className="gauge-bandwrap">
        <svg viewBox={`0 0 ${BAND_W} ${BAND_H}`} className="gauge-band" data-testid="gauge-band" role="img" aria-label="速さの帯">
          <rect x="0" y="2" width={BAND_W} height={BAND_H - 4} fill="#eef1f5" stroke="#c9ced6" strokeWidth="1.5" />
          {cut > 0 && inLin != null && outLin != null && (
            <rect x={xOf(outLin)} y="4" width={Math.max(2, xOf(inLin) - xOf(outLin))} height={BAND_H - 8} fill="#ffd100" data-testid="gauge-cut" />
          )}
          {actual != null && <rect x="0" y={BAND_H / 2 - 14} width={xOf(actual)} height="28" fill="#333a48" data-testid="gauge-actual" />}
          {(inLin ?? target) != null && <line x1={xOf(inLin ?? target)} y1="2" x2={xOf(inLin ?? target)} y2={BAND_H - 2} stroke="#111" strokeWidth="4" data-testid="gauge-command" />}
          {limit != null && <line x1={xOf(limit)} y1="2" x2={xOf(limit)} y2={BAND_H - 2} stroke="#5b6675" strokeWidth="2" strokeDasharray="5 4" data-testid="gauge-limit" />}
        </svg>
        <div className="gauge-ticks" aria-hidden="true">
          {ticks.map((t) => <span key={t}>{t.toFixed(1)}</span>)}
        </div>
      </div>

      <svg viewBox={`0 0 ${LINE_W} ${LINE_H}`} className="gauge-line" data-testid="gauge-history" role="img" aria-label="直近5秒の速さ">
        <rect x="0.5" y="0.5" width={LINE_W - 1} height={LINE_H - 1} fill="#fff" stroke="#c9ced6" strokeWidth="1.5" />
        <line x1="0" y1={LINE_H / 2} x2={LINE_W} y2={LINE_H / 2} stroke="#c9ced6" strokeWidth="1" />
        {histC.length > 1 && <polyline points={linePts(histC)} fill="none" stroke="#9aa3ad" strokeWidth="1.5" strokeDasharray="4 3" data-testid="gauge-history-command" />}
        {histA.length > 1 && <polyline points={linePts(histA)} fill="none" stroke="#111" strokeWidth="2.5" data-testid="gauge-history-actual" />}
      </svg>

      <div className="gauge-wheels" data-testid="gauge-wheels">
        <div className="gauge-wheelcol">
          {wheelBand('left', 'left')}
          {wheelValues('left', 'left')}
        </div>
        <div className="gauge-middle" aria-hidden="true">
          <svg viewBox="0 0 72 110" className="gauge-mini">
            <rect x="22" y="38" width="28" height="36" fill="#e8f5e9" stroke="#2e7d32" strokeWidth="2" />
            <polygon points="36,42 31,51 41,51" fill="#2e7d32" />
            {actual != null && Math.abs(actual) > 0.01 && (
              <g stroke={cut > 0 ? '#d9a800' : '#4caf50'} strokeWidth="2" opacity="0.85" fill="none">
                {actual >= 0 ? (
                  <>
                    <line x1="28" y1="38" x2="28" y2="6" />
                    <line x1="44" y1="38" x2="44" y2="6" />
                    <line x1="28" y1="20" x2="44" y2="20" strokeWidth="1.5" />
                  </>
                ) : (
                  <>
                    <line x1="28" y1="74" x2="28" y2="104" />
                    <line x1="44" y1="74" x2="44" y2="104" />
                    <line x1="28" y1="90" x2="44" y2="90" strokeWidth="1.5" />
                  </>
                )}
              </g>
            )}
          </svg>
          <div className="gauge-labels">
            <span className="gw gw-actual" data-testid="gauge-label-unit">{S11_GAUGE_SPEED_UNIT}</span>
            <span className="gw gw-target" data-testid="gauge-label-target">{S11_GAUGE_TARGET}</span>
            <span className="gw gw-output" data-testid="gauge-label-output">{S11_GAUGE_OUTPUT}</span>
            <span className="gw gw-encoder" data-testid="gauge-label-encoder">{S11_GAUGE_ENCODER}</span>
          </div>
        </div>
        <div className="gauge-wheelcol">
          {wheelBand('right', 'right')}
          {wheelValues('right', 'right')}
        </div>
      </div>

    </div>
  )
}
