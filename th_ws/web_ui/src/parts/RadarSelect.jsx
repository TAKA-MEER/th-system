// parts/RadarSelect.jsx — 対象選択レーダー（Spec-webui.md §9.1「対象選択レーダー
// （全画面で同一部品）」、mockup 2055〜2122 行）。S-20 / S-21 で共用する。
//
// base_link 相対の候補座標（m、前方 +x / 左 +y）を機体中心の SVG に描き、
// selectedIndex を強調する。タップ（Enter/Space も可）で onSelect(index) を呼ぶ。
// 候補 0 件なら「試験員が検出されていません」。
//
// is_lost（対象を見失った）の扱い（2026-09-08 実機で確定・修正）:
// multiple_sensor_person_tracking は require_explicit_target_selection が既定 true
// で、見失うと自動では再取得しない（安全設計）。候補（脚検出）自体は見失っている間も
// 届き続けるので、**候補が 1 件以上あるときは isLost でも候補を隠さずタップ可能に
// する**（RADAR_LOST_RESELECT を添えるだけ）。候補が 0 件で isLost のときだけ
// 「見失っています」の全面メッセージにする（本当にタップする対象が無い）。
// 以前は isLost の間は無条件で候補を隠していたため、候補が複数いて自動再選択
// （bridge の auto_select、候補 1 件限定）が働かない状況だと、操作者が選び直す
// 手段が無いまま復帰不能になっていた。
// 日本語は i18n/screens.js の RADAR_* 定数に置く（parts/ は c4）。onSelect を
// 空関数にしない（e2e/s20-prep.spec.js が「レーダーのタップ → ui.select_target」
// として実在のハンドラであることを検証する）。
import {
  RADAR_ARIA, RADAR_CONFIDENCE, RADAR_EMPTY, RADAR_HEADING, RADAR_LOST,
  RADAR_LOST_RESELECT, RADAR_SELECTED,
} from '../i18n/screens.js'
import { RADAR_PX_PER_M, RADAR_CX, RADAR_CY, radarToSvg } from './radarGeometry.js'

// 論理 viewBox は 240x240（radarGeometry はこの座標系が前提。変更禁止）。
// 表示サイズは CSS（.radar の width:100% / max-height）が決める
// （brief-onsite-ux UX-2-c: 固定 px で描かない）。
const RADAR_VB = 240
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
        viewBox={`0 0 ${RADAR_VB} ${RADAR_VB}`}
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
        {isLost && candidates.length === 0 ? (
          <text x={CX} y={CY} textAnchor="middle" className="rv-msg" data-testid="radar-lost">
            {RADAR_LOST}
          </text>
        ) : candidates.length === 0 ? (
          <text x={CX} y={CY} textAnchor="middle" className="rv-msg" data-testid="radar-empty">
            {RADAR_EMPTY}
          </text>
        ) : (
          <>
            {isLost && (
              <text
                x={CX} y={16} textAnchor="middle" className="rv-msg-lost"
                data-testid="radar-lost-reselect"
              >
                {RADAR_LOST_RESELECT}
              </text>
            )}
            {candidates.map((c, i) => {
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
            })}
          </>
        )}
      </svg>
    </div>
  )
}