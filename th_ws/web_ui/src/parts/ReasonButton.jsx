// parts/ReasonButton.jsx — 押せないボタンの理由（Spec-webui §3.3.3）。
//
// 押せない間は塗りを外し枠と語を残す。押すと、そのボタンのすぐ上
// （入らなければ下）に三角付きで「押せません　〈理由〉」を出す。
// ヘッダには重ねない。使う側は disabled の代わりに押せる見た目で置き、
// disabled=true の間は理由吹き出しが出る形として使うか、
// 呼び出し側が理由文字列を渡せば常時出す（旧来の badge 相当）。
// e2e/testid の互換のため reasonTestId で testid の受け渡しができる。
import { useState } from 'react'

export default function ReasonButton({
  className = '',
  reason = null,
  reasonTestId,
  disabled = false,
  alwaysShowReason = false,
  onClick,
  children,
  ...rest
}) {
  const [showReason, setShowReason] = useState(false)
  const show = !!reason && (alwaysShowReason || showReason)

  function handleClick(e) {
    if (disabled) {
      e.preventDefault()
      setShowReason(true)
      return
    }
    onClick?.(e)
  }

  return (
    <span className="reason-wrap">
      {show && (
        <span className="reason-bubble above" data-testid={reasonTestId}>
          押せません　{reason}
        </span>
      )}
      <button
        type="button"
        className={`btn ${className}${disabled ? ' dis' : ''}`}
        aria-disabled={disabled || undefined}
        onClick={handleClick}
        {...rest}
      >
        {children}
      </button>
    </span>
  )
}
