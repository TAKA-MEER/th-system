// parts/ReasonButton.jsx — 押せないボタンの理由（Spec-webui §3.3.3）。
//
// 押せない間は塗りを外し枠と語を残す。押すと、そのボタンのすぐ上
// （入らなければ下）に三角付きで「押せません　〈理由〉」を出す。
// ヘッダには重ねない（上に入らなければ下）。
import { useRef, useState } from 'react'

export default function ReasonButton({
  className = '',
  reason = null,
  reasonTestId,
  disabled = false,
  onClick,
  children,
  ...rest
}) {
  const [showReason, setShowReason] = useState(false)
  const [below, setBelow] = useState(false)
  const btnRef = useRef(null)
  const show = !!reason && showReason

  // 吹き出しの置き場: ボタンのすぐ上。ヘッダに重なるなら下。
  function fitsAbove() {
    const el = btnRef.current
    if (!el) return true
    const rect = el.getBoundingClientRect()
    const hdr = document.querySelector('#hdr')
    const hdrBottom = hdr ? hdr.getBoundingClientRect().bottom : 0
    // 吹き出しは約40px + 余白6px。ヘッダの下端に食い込めば上に置けない。
    return rect.top - hdrBottom >= 48
  }

  function handleClick(e) {
    if (disabled) {
      e.preventDefault()
      setBelow(!fitsAbove())
      setShowReason(true)
      return
    }
    onClick?.(e)
  }

  const bubble = (pos) => (
    <span className={`reason-bubble${pos === 'below' ? ' below' : ''}`} data-testid={reasonTestId}>
      押せません　{reason}
    </span>
  )

  return (
    <span className="reason-wrap">
      {show && !below && bubble('above')}
      <button
        type="button"
        ref={btnRef}
        className={`btn ${className}${disabled ? ' dis' : ''}`}
        aria-disabled={disabled || undefined}
        onClick={handleClick}
        {...rest}
      >
        {children}
      </button>
      {show && below && bubble('below')}
    </span>
  )
}
