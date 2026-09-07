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
  RADAR_ARIA, RADAR_CONFIDENCE, RADAR_EMPTY, RADAR_LOST, RADAR_SELECTED,
} from '../i18n/screens.js'

const VW = 240
const VH = 240
const CX = VW / 2
const CY = VH / 2
// 1 m を 60 px。レーダー半径 2 m を描く（可視範囲の目安）。
const PX_PER_M = 60

// base_link: +x 前方・+y 左をそのまま絵の x / 逆 y に写す（画面は下方向正）。
function toSvg(x, y) {
  return [CX + x * PX_PER_M, CY - y * PX_PER_M]
}

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
        <circle className="rv-ring" cx={CX} cy={CY} r={PX_PER_M * 2} />
        <circle className="rv-ring" cx={CX} cy={CY} r={PX_PER_M} />
        <line className="rv-axis" x1={CX} y1={CY - PX_PER_M * 2} x2={CX} y2={CY + PX_PER_M * 2} />
        <line className="rv-axis" x1={CX - PX_PER_M * 2} y1={CY} x2={CX + PX_PER_M * 2} y2={CY} />
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
            const [sx, sy] = toSvg(c?.x ?? 0, c?.y ?? 0)
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