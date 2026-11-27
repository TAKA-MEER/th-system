// parts/MapTools.jsx — 地図の端の道具（Spec-webui.md §6.1・2026-11-27 D78・D79・D95）。
//
// 右下に置く。右にレーダー・スティックがある画面はその左、下に帯がある画面は
// その上へ逃がせるように、置き場所は className / style で親から渡せる形にする
// （既定は右下）。回転と「入れない所」はアイコンだけのボタン（語は aria-label
// と title）。「凡例」は語のボタン（§3.3.7・§6.1）。
//   - 「入れない所」＝コストマップの入切。入っている間は反転。
//   - 「凡例」＝押すと上へ開く 2 列の凡例。× かもう一度押すと閉じる。
//   - 「左へ回す」「右へ回す」＝地図の向きを 15° ずつ回す（試験準備だけ）。
// 地図にはカーソルのヘルプを出さないので、この部品にも data-help を付けない。
import { useState } from 'react'
import { MAP_FREE, MAP_OCCUPIED, MAP_SCAN, MAP_NEAR, MAP_COST, MAP_GO, MAP_WARN, MAP_PIN, MAP_PIN_EDGE } from './mapColors.js'

export const MAP_TOOLS_COSTMAP_LABEL = '入れない所'
export const MAP_TOOLS_LEGEND_LABEL = '凡例'
export const MAP_TOOLS_ROTATE_LEFT_LABEL = '左へ回す'
export const MAP_TOOLS_ROTATE_RIGHT_LABEL = '右へ回す'
export const MAP_TOOLS_LEGEND_CLOSE_LABEL = '凡例を閉じる'

function IconNoEntry() {
  return (
    <svg viewBox="0 0 24 24" width="18" height="18" aria-hidden="true">
      <circle cx="12" cy="12" r="10" fill="none" stroke="currentColor" strokeWidth="2.4" />
      <line x1="5" y1="5" x2="19" y2="19" stroke="currentColor" strokeWidth="2.4" />
    </svg>
  )
}
function IconRotateLeft() {
  return (
    <svg viewBox="0 0 24 24" width="18" height="18" aria-hidden="true">
      <path d="M6 4 L3 7 L6 10" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
      <path d="M3 7 A9 9 0 1 1 6 17" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
    </svg>
  )
}
function IconRotateRight() {
  return (
    <svg viewBox="0 0 24 24" width="18" height="18" aria-hidden="true">
      <path d="M18 4 L21 7 L18 10" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
      <path d="M21 7 A9 9 0 1 0 18 17" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
    </svg>
  )
}
function IconClose() {
  return (
    <svg viewBox="0 0 24 24" width="16" height="16" aria-hidden="true">
      <line x1="5" y1="5" x2="19" y2="19" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round" />
      <line x1="19" y1="5" x2="5" y2="19" stroke="currentColor" strokeWidth="2.2" strokeLinecap="round" />
    </svg>
  )
}

// 地図の見方（凡例。Spec-webui.md §6.1 の 11 項目）。
const LEGEND_ITEMS = [
  { label: '通れる床', style: { background: MAP_FREE } },
  { label: '入れない所', style: { background: MAP_COST } },
  { label: '壁', style: { background: MAP_OCCUPIED } },
  { label: 'まだ見ていない所', style: { background: 'var(--bg)' } }, // 未探索は画面の地と同じ（§6.1）
  { label: 'いま見えている物', style: { background: MAP_SCAN } },
  { label: '近い物', style: { background: MAP_NEAR } },
  { label: '経路を塞いでいる所', ring: MAP_WARN },
  { label: '経路', line: MAP_GO },
  { label: '機体', robot: true },
  { label: '待機場所', house: true },
  { label: '配電盤', panel: true },
]

function LegendSwatch({ item }) {
  const base = { width: 22, height: 14, borderRadius: 2, border: '1px solid #555', flex: '0 0 auto' }
  if (item.style) return <span style={{ ...base, ...item.style }} />
  if (item.ring) {
    return (
      <span style={{ ...base, border: `2px dashed ${MAP_WARN}`, background: 'transparent' }} />
    )
  }
  if (item.line) {
    return (
      <span style={{ ...base, height: 0, border: 'none', alignSelf: 'center' }}>
        <span style={{ display: 'block', height: 3, background: item.line }} />
      </span>
    )
  }
  if (item.robot) {
    return (
      <svg viewBox="0 0 22 14" width="22" height="14" aria-hidden="true">
        <rect x="3" y="1" width="12" height="12" fill="none" stroke={MAP_GO} strokeWidth="1.6" />
        <path d="M18 7 L13 3.5 L13 10.5 Z" fill={MAP_GO} />
      </svg>
    )
  }
  if (item.house) {
    return (
      <svg viewBox="0 0 22 14" width="22" height="14" aria-hidden="true">
        <path d="M20 7 L13 0.5 L3 0.5 L3 13.5 L20 13.5 Z" fill={MAP_PIN} stroke={MAP_PIN_EDGE} strokeWidth="1.4" />
      </svg>
    )
  }
  // 配電盤（四角に向きの三角）
  return (
    <svg viewBox="0 0 22 14" width="22" height="14" aria-hidden="true">
      <rect x="1" y="1" width="12" height="12" fill={MAP_PIN} stroke={MAP_PIN_EDGE} strokeWidth="1.4" />
      <path d="M13 3 L20 7 L13 11 Z" fill={MAP_PIN} stroke={MAP_PIN_EDGE} strokeWidth="1.4" />
    </svg>
  )
}

export default function MapTools({
  testId,
  costmapOn,
  onCostmapToggle,
  onRotateLeft = null, // null の間は回転ボタンを出さない（試験準備だけ）
  onRotateRight = null,
  className = '',
  style = undefined,
}) {
  const [legendOpen, setLegendOpen] = useState(false)
  const showRotate = onRotateLeft && onRotateRight
  return (
    <div className={`mapTools ${className}`} style={style} data-testid={`${testId}-map-tools`}>
      {legendOpen ? (
        <div className="mapLegend" data-testid={`${testId}-map-legend`} role="dialog" aria-label={MAP_TOOLS_LEGEND_LABEL}>
          <button
            type="button"
            className="mapLegendClose"
            aria-label={MAP_TOOLS_LEGEND_CLOSE_LABEL}
            title={MAP_TOOLS_LEGEND_CLOSE_LABEL}
            data-testid={`${testId}-map-legend-close`}
            onClick={() => setLegendOpen(false)}
          >
            <IconClose />
          </button>
          <ul>
            {LEGEND_ITEMS.map((item) => (
              <li key={item.label}>
                <LegendSwatch item={item} />
                <span>{item.label}</span>
              </li>
            ))}
          </ul>
        </div>
      ) : null}
      {showRotate ? (
        <>
          <button
            type="button"
            className="mapToolBtn"
            data-testid={`${testId}-map-rotate-left`}
            aria-label={MAP_TOOLS_ROTATE_LEFT_LABEL}
            title={MAP_TOOLS_ROTATE_LEFT_LABEL}
            onClick={onRotateLeft}
          >
            <IconRotateLeft />
          </button>
          <button
            type="button"
            className="mapToolBtn"
            data-testid={`${testId}-map-rotate-right`}
            aria-label={MAP_TOOLS_ROTATE_RIGHT_LABEL}
            title={MAP_TOOLS_ROTATE_RIGHT_LABEL}
            onClick={onRotateRight}
          >
            <IconRotateRight />
          </button>
        </>
      ) : null}
      <button
        type="button"
        className={`mapToolBtn${costmapOn ? ' on' : ''}`}
        data-testid={`${testId}-map-costmap-toggle`}
        aria-label={MAP_TOOLS_COSTMAP_LABEL}
        title={MAP_TOOLS_COSTMAP_LABEL}
        aria-pressed={costmapOn}
        onClick={onCostmapToggle}
      >
        <IconNoEntry />
      </button>
      <button
        type="button"
        className={`mapToolBtn${legendOpen ? ' on' : ''}`}
        data-testid={`${testId}-map-legend-toggle`}
        aria-label={MAP_TOOLS_LEGEND_LABEL}
        title={MAP_TOOLS_LEGEND_LABEL}
        aria-expanded={legendOpen}
        onClick={() => setLegendOpen((v) => !v)}
      >
        凡例 {/* §3.3.7・§6.1: 「凡例」は語のボタン。アイコンだけは回転と「入れない所」だけ */}
      </button>
    </div>
  )
}
