// parts/RadarSelect.jsx — 対象選択レーダー（Spec-webui.md §9.1「対象選択レーダー
// （全画面で同一部品）」、mockup 2055〜2122 行）。S-20 / S-21 で共用する。
//
// base_link 相対の候補座標（m、前方 +x / 左 +y）を機体中心の SVG に描き、
// selectedIndex を強調する。タップ（Enter/Space も可）で onSelect(index) を呼ぶ。
// is_lost なら「対象を見失っています」、候補 0 件なら「試験員が検出されていません」。
// 日本語は i18n/screens.js の RADAR_* 定数に置く（parts/ は c4）。onSelect を
// 空関数にしない（e2e/s20-prep.spec.js が「レーダーのタップ → ui.select_target」
// として実在のハンドラであることを検証する）。
import {
  RADAR_ARIA, RADAR_CONFIDENCE, RADAR_EMPTY, RADAR_HEADING, RADAR_LOST, RADAR_SELECTED,
} from '../i18n/screens.js'
import { RADAR_PX_PER_M, RADAR_CX, RADAR_CY, radarToSvg } from './radarGeometry.js'

const VW = 240
const VH = 240
const CX = RADAR_CX
const CY = RADAR_CY

export default function RadarSelect({
  candidates = [],
  selectedIndex = -1,
  isLost = false,
  onSelect,
  confidence,
}) {
  return (
    <div className="card">
      <svg
        className="radar"
        viewBox={`0 0 ${VW} ${VH}`}
        role="group"
        aria-label={RADAR_ARIA}
        data-testid="radar"
      >
        <circle className="rv-ring" cx={CX} cy={CY} r={RADAR_PX_PER_M * 2} />
        <circle className="rv-ring" cx={CX} cy={CY} r={RADAR_PX_PER_M} />
        {/* 前方（機体 +x / 画面の上）の軸線を強調。F-2。 */}
        <line className="rv-axis rv-axis-fwd" x1={CX} y1={CY - RADAR_PX_PER_M * 2} x2={CX} y2={CY} />
        <line className="rv-axis" x1={CX - RADAR_PX_PER_M * 2} y1={CY} x2={CX + RADAR_PX_PER_M * 2} y2={CY} />
        {/* 機体マーク（中心の丸＋上向き三角）。候補より下のレイヤに描いて隠れないようにする。 */}
        <g data-testid="radar-heading">
          <polygon
            points={`${CX},${CY - 20} ${CX - 10},${CY + 6} ${CX + 10},${CY + 6}`}
            fill="none"
            stroke="#90caf9"
            strokeWidth="2"
            strokeLinejoin="round"
          />
          <circle cx={CX} cy={CY} r={6} fill="#90caf9" />
          <text x={CX} y={CY - 26} textAnchor="middle" fontSize="11" fill="#90caf9">
            {RADAR_HEADING}
          </text>
        </g>
        {isLost ? (
          <text x={CX} y={CY} textAnchor="middle" className="rv-msg" data-testid="radar-lost">
            {RADAR_LOST}
          </text>
        ) : candidates.length === 0 ? (
          <text x={CX} y={CY} textAnchor="middle" className="rv-msg" data-testid="radar-empty">
            {RADAR_EMPTY}
          </text>
        ) : (
          candidates.map((c, i) => {
            const [sx, sy] = radarToSvg(c?.x ?? 0, c?.y ?? 0)
            const sel = i === selectedIndex
            return (
              <g
                key={i}
                className={`rv-cand${sel ? ' sel' : ''}`}
                transform={`translate(${sx} ${sy})`}
                role="button"
                tabIndex={0}
                aria-label={`${RADAR_SELECTED} ${i + 1}`}
                data-testid={`radar-cand-${i}`}
                onClick={() => onSelect?.(i)}
                onKeyDown={(e) => {
                  if (e.key === 'Enter' || e.key === ' ') {
                    e.preventDefault()
                    onSelect?.(i)
                  }
                }}
              >
                <circle r={10} />
                {sel && <text y={-18} textAnchor="middle" className="rv-sel">{RADAR_SELECTED}</text>}
                {/* 先頭の候補だけ確信度を添える（全部出すと重なるため）。 */}
                {i === 0 && candidates.length > 0 && typeof confidence === 'number' && (
                  <text y={26} textAnchor="middle" className="rv-cnf">
                    {RADAR_CONFIDENCE(Math.round(confidence * 100))}
                  </text>
                )}
              </g>
            )
          })
        )}
      </svg>
    </div>
  )
}