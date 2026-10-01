// parts/BlindScanSelect.jsx — S-40 校正「LiDAR 死角」の角度帯の選択（ライブスキャン上をなぞる）。
//
// 上が前方 0°・左が +90°（laser_link 基準・反時計回り正。機体側と同じ規約）。
//   - 灰色の扇 … いま登録されている死角（detail.current.blind_angle_ranges）
//   - 青い扇   … 選択中（ドラッグで足す。ここから削除・全部外す・登録済みに戻すができる）
//   - 赤い点   … 選択の内側の点（マスクすると障害物として見なくなる＝消える点）
// 幅の上限（1 区間・総幅・区間数）を超える選択は「確認」を押せない。**検査の正本は機体側**で、
// 画面は押す前に分かるようにしているだけ（機体も同じ検査をして、超えたら適用しない）。
//
// ドラッグの送信先は親（S40Calib）の onSubmit。ここは状態を持たず、選択（draft）だけを持つ。
import { useRef, useState } from 'react'
import {
  angleFromPoint, rangeFromDrag, blindProblem, blindTotalDeg, sectorWidthDeg, scanPointsByMask,
  rangesFromFlat,
} from '../screens/calibCore.js'
import * as T from '../i18n/calib.js'

const SIZE = 400
const C = SIZE / 2
const SCALE = 80          // px / m（端まで 2.5 m）
const R = C - 6

function polar(deg, r) {
  const a = (deg * Math.PI) / 180
  return [C - r * Math.sin(a), C - r * Math.cos(a)]
}

function wedge([a0, a1]) {
  const w = sectorWidthDeg(a0, a1)
  const [x0, y0] = polar(a0, R)
  const [x1, y1] = polar(a0 + w, R)
  return `M ${C} ${C} L ${x0} ${y0} A ${R} ${R} 0 ${w > 180 ? 1 : 0} 0 ${x1} ${y1} Z`
}

export default function BlindScanSelect({
  scan, registeredFlat, limits, disabled, problemKey, submitLabel, onSubmit,
}) {
  const registered = rangesFromFlat(registeredFlat)
  const [draft, setDraft] = useState(null)
  const [drag, setDrag] = useState(null)        // { start, end }
  const svgRef = useRef(null)
  const ranges = draft ?? registered

  const problem = blindProblem(ranges, limits)
  const total = blindTotalDeg(ranges)
  const { kept, masked } = scanPointsByMask(scan, ranges)
  const toXY = (p) => [C - p.y * SCALE, C - p.x * SCALE]

  function pointerAngle(e) {
    const rect = svgRef.current.getBoundingClientRect()
    const x = ((e.clientX - rect.left) / rect.width) * SIZE
    const y = ((e.clientY - rect.top) / rect.height) * SIZE
    return angleFromPoint(x, y, C, C)
  }
  function onDown(e) {
    if (disabled) return
    e.currentTarget.setPointerCapture?.(e.pointerId)
    const a = pointerAngle(e)
    setDrag({ start: a, end: a })
  }
  function onMove(e) {
    if (!drag) return
    setDrag({ ...drag, end: pointerAngle(e) })
  }
  function onUp(e) {
    if (!drag) return
    const end = pointerAngle(e)
    const added = rangeFromDrag(drag.start, end)
    setDrag(null)
    if (sectorWidthDeg(added[0], added[1]) < 1) return      // クリックだけ・ほぼ動かさない
    setDraft([...ranges, added])
  }

  const dragRange = drag ? rangeFromDrag(drag.start, drag.end) : null

  return (
    <div className="s40-blind" data-testid="s40-blind">
      <p className="note">{T.S40_BLIND_HINT}</p>
      <svg ref={svgRef} viewBox={`0 0 ${SIZE} ${SIZE}`} className="s40-blind-svg"
        data-testid="s40-blind-canvas" style={{ touchAction: 'none', width: '100%', maxWidth: 420 }}
        onPointerDown={onDown} onPointerMove={onMove} onPointerUp={onUp}
        onPointerCancel={() => setDrag(null)}>
        <circle cx={C} cy={C} r={R} fill="none" stroke="currentColor" strokeOpacity="0.25" />
        <circle cx={C} cy={C} r={R / 2} fill="none" stroke="currentColor" strokeOpacity="0.15" />
        {registered.map((rg, i) => (
          <path key={`reg-${i}`} d={wedge(rg)} fill="gray" fillOpacity="0.25" />
        ))}
        {ranges.map((rg, i) => (
          <path key={`sel-${i}`} d={wedge(rg)} fill="dodgerblue" fillOpacity="0.30"
            data-testid="s40-blind-wedge" />
        ))}
        {dragRange && <path d={wedge(dragRange)} fill="dodgerblue" fillOpacity="0.15"
          stroke="dodgerblue" strokeDasharray="4 3" />}
        {kept.map((p, i) => { const [x, y] = toXY(p); return <circle key={`k${i}`} cx={x} cy={y} r="1.8" fill="currentColor" /> })}
        {masked.map((p, i) => { const [x, y] = toXY(p); return <circle key={`m${i}`} cx={x} cy={y} r="2.4" fill="crimson" /> })}
        <polygon points={`${C},${C - 10} ${C - 6},${C + 6} ${C + 6},${C + 6}`} fill="currentColor" />
      </svg>
      {!scan && <p className="s40-warn" data-testid="s40-blind-no-scan">{T.S40_BLIND_NO_SCAN}</p>}
      <h4>{T.S40_BLIND_RANGES_TITLE}</h4>
      {ranges.length === 0 && <p className="note" data-testid="s40-blind-none">{T.S40_BLIND_NONE}</p>}
      <ul className="s40-blind-list">
        {ranges.map(([a, b], i) => (
          <li key={`${a}-${b}-${i}`} data-testid={`s40-blind-range-${i}`}>
            {Number(a.toFixed(1))}〜{Number(b.toFixed(1))}°（{Number(sectorWidthDeg(a, b).toFixed(1))}°）
            <button type="button" className="btn sm" disabled={disabled}
              data-testid={`s40-blind-delete-${i}`}
              onClick={() => setDraft(ranges.filter((_, j) => j !== i))}>
              {T.S40_BLIND_DELETE}
            </button>
          </li>
        ))}
      </ul>
      <p className="note" data-testid="s40-blind-total">{T.s40BlindTotal(total, limits)}</p>
      {problem && (
        <p className="s40-warn" data-testid="s40-blind-problem">{T.S40_BLIND_PROBLEMS[problem]}</p>
      )}
      {!problem && problemKey && (
        <p className="s40-warn" data-testid="s40-insane">{T.S40_BLIND_PROBLEMS[problemKey] ?? problemKey}</p>
      )}
      <div className="row mt">
        <button type="button" className="btn sm" disabled={disabled || ranges.length === 0}
          data-testid="s40-blind-clear" onClick={() => setDraft([])}>
          {T.S40_BLIND_CLEAR}
        </button>
        <button type="button" className="btn sm" disabled={disabled || draft === null}
          data-testid="s40-blind-reset" onClick={() => setDraft(null)}>
          {T.S40_BLIND_RESET}
        </button>
        <button type="button" className="btn grow" data-testid="s40-blind-submit"
          disabled={disabled || !!problem || !scan}
          onClick={() => onSubmit(ranges)}>
          {submitLabel}
        </button>
      </div>
    </div>
  )
}
