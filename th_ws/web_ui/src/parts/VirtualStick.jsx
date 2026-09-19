// parts/VirtualStick.jsx — the 8-direction virtual stick, extracted from
// App.jsx (L149-237) and re-styled to the mockup's `#tplStick` look
// (docs/plan/spec/mockup/index.html L491-514, the visual source of truth).
//
// Responsibility (DetailedDesign-wp3.md WP-UI-03 §4.2): pointer events only.
// It maps pointer position -> a normalized { vn, wn, label } via
// parts/stickGeometry.js and calls `onChange(cmd)` while the pointer is
// down, `onRelease()` when it goes up / cancels / unmounts. It never
// publishes anything -- that is useJogLease.js's job (§4.2 "送出を 1 箇所に
// 閉じる"), so W-6 and the perpetual stick cannot double-send.
//
// U3-1 (dangerous if dropped): if this component unmounts mid-drag (tab
// switch), pointerup never arrives and the last jog command would keep
// flowing. The cleanup below calls onRelease unconditionally when dragged
// (the same safety the original App.jsx had at L163-168).
//
// U3-4: the target screen id / callbacks must not be captured in the event
// handlers' closures; onRelease/onChange are kept on refs and read at call
// time, so a re-render while dragging still calls the newest handler.
import { useEffect, useRef, useState } from 'react'
import { stickToCmd } from './stickGeometry.js'
import { STICK_ACTIVE, STICK_FWD, STICK_BACK, STICK_LEFT, STICK_RIGHT } from '../i18n/screens.js'

const SIZE = 200
const C = SIZE / 2
const KNOB_R = 30
const TRAVEL = C - KNOB_R - 4 // ノブ中心の可動半径 (SVG 論理座標)

export default function VirtualStick({ disabled = false, onChange, onRelease }) {
  const svgRef = useRef(null)
  const pointerIdRef = useRef(null)
  const [stick, setStick] = useState({ x: 0, y: 0, active: false })

  // U3-1 / U3-4: keep the latest handlers on refs so the unmount cleanup and
  // the pointer handlers always call the current one, not a captured value.
  const onChangeRef = useRef(onChange)
  const onReleaseRef = useRef(onRelease)
  onChangeRef.current = onChange
  onReleaseRef.current = onRelease

  useEffect(() => () => {
    if (pointerIdRef.current !== null) onReleaseRef.current()
  }, [])

  const update = (e) => {
    const rect = svgRef.current.getBoundingClientRect()
    // 基準は要素の幅・高さではなく **短辺**。svg は viewBox が 1:1 で
    // preserveAspectRatio が既定（meet）なので、要素が正方形でないときは
    // 描画された円が短辺いっぱいに収まり、余った側はレターボックスになる。
    // 幅と高さを別々に基準にすると、**画面に見えている円と当たり判定がずれる**
    // （横長の箱では、円の縁まで倒しても倒し量が deadzone に届かず反応しない）。
    // 既存 App.jsx から移してきたときの計算がこれで、走行タブでは
    // `.stick` が横に伸びるため実際に無反応になった（2026-09-01 実測: 箱が
    // 1129x230 で、円の縁まで倒しても倒し量 0.08 < deadzone 0.15）。
    const span = Math.min(rect.width, rect.height) / 2
    let dx = (e.clientX - (rect.left + rect.width / 2)) / span
    let dy = ((rect.top + rect.height / 2) - e.clientY) / span
    const len = Math.hypot(dx, dy)
    if (len > 1) { dx /= len; dy /= len }
    const cmd = stickToCmd(dx, dy, Math.min(1, len))
    setStick({ x: dx * TRAVEL, y: -dy * TRAVEL, active: cmd.label !== null })
    onChangeRef.current(cmd)
  }

  const handleDown = (e) => {
    e.preventDefault()
    // Capture so pointermove/up keep reaching the stick after leaving the svg.
    // Wrapped for e2e: a real browser gesture always has an active pointer and
    // capture succeeds; a synthetic test PointerEvent may not (InvalidPointerId),
    // and we must not let that abort the drag.
    try { e.currentTarget.setPointerCapture(e.pointerId) } catch { /* synthetic event */ }
    pointerIdRef.current = e.pointerId
    update(e)
  }
  const handleMove = (e) => {
    if (pointerIdRef.current !== e.pointerId) return
    update(e)
  }
  const handleUp = (e) => {
    if (pointerIdRef.current !== e.pointerId) return
    pointerIdRef.current = null
    setStick({ x: 0, y: 0, active: false })
    onReleaseRef.current()
  }

  return (
    <div className={`stick ${stick.active ? 'act' : ''} ${disabled ? 'off' : ''}`}>
      <div className="jogBadge">{STICK_ACTIVE}</div>
      <svg
        ref={svgRef}
        viewBox={`0 0 ${SIZE} ${SIZE}`}
        aria-label="仮想スティック"
        onPointerDown={disabled ? undefined : handleDown}
        onPointerMove={disabled ? undefined : handleMove}
        onPointerUp={disabled ? undefined : handleUp}
        onPointerCancel={disabled ? undefined : handleUp}
        onContextMenu={(e) => e.preventDefault()}
      >
        <circle cx={C} cy={C} r={C - 8} className="stick-plate" />
        <g className="stick-guide">
          <path d={`M${C} 8 V60 M${C} 140 V${SIZE - 8} M8 ${C} H60 M140 ${C} H${SIZE - 8}`} />
          <path d={`M35 35 L70 70 M165 35 L130 70 M35 165 L70 130 M165 165 L130 130`} />
        </g>
        <circle cx={C} cy={C} r={C * 0.15} className="stick-dead" />
        <text x={C} y="30" textAnchor="middle" className="stick-hint">{STICK_FWD}</text>
        <text x={C} y={SIZE - 18} textAnchor="middle" className="stick-hint">{STICK_BACK}</text>
        <text x="28" y={C + 4} textAnchor="middle" className="stick-hint">{STICK_LEFT}</text>
        <text x={SIZE - 28} y={C + 4} textAnchor="middle" className="stick-hint">{STICK_RIGHT}</text>
        <circle
          cx={C + stick.x} cy={C + stick.y} r={KNOB_R}
          className={`stick-knob ${stick.active ? 'active' : ''}`}
        />
      </svg>
    </div>
  )
}
