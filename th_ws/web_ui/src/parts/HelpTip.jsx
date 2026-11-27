// parts/HelpTip.jsx — ヘルプ表示（Spec-webui §3.3.6）。
//
// マウスは重ねて約0.7秒、タップは長押し約0.6秒。止める操作・押している間だけ動く操作は
// 対象外（helpKey を渡さない）。ヘッダと「停止する」は、ヘルプが出ていても1回で効く
// （この部品はクリックを止めない。閉じるだけのクリックは呼び出し側＝枠の外で処理する）。
// 出す・出さないは端末ごと（parts/helpPrefs.js。読めないときは「出す」）。
// 位置はボタンの下：ヘッダには絶対に重ねない。
import { useEffect, useRef, useState } from 'react'
import { helpText } from '../i18n/help.js'
import { readHelpEnabled } from './helpPrefs.js'

const HOVER_MS = 700
const LONG_PRESS_MS = 600

export default function HelpTip({ helpKey, children }) {
  const [show, setShow] = useState(false)
  const [sticky, setSticky] = useState(false)
  const timerRef = useRef(null)
  const text = helpKey ? helpText(helpKey) : null
  const [title, ...rest] = (text ?? '').split('\n')
  const body = rest.join('\n')

  function clearTimer() {
    clearTimeout(timerRef.current)
    timerRef.current = null
  }

  useEffect(() => () => clearTimer(), [])

  if (!helpKey || !text || !readHelpEnabled()) {
    return <span className="help-wrap">{children}</span>
  }

  return (
    <span
      className="help-wrap"
      data-help-key={helpKey}
      onMouseEnter={() => {
        clearTimer()
        timerRef.current = setTimeout(() => setShow(true), HOVER_MS)
      }}
      onMouseLeave={() => {
        clearTimer()
        if (!sticky) setShow(false)
      }}
      onTouchStart={() => {
        clearTimer()
        timerRef.current = setTimeout(() => { setShow(true); setSticky(true) }, LONG_PRESS_MS)
      }}
      onTouchEnd={clearTimer}
    >
      {children}
      {show && (
        <span className="help-pop" role="tooltip" data-testid={`help-${helpKey}`}>
          <span className="help-title">{title}</span>
          {body && <span className="help-body">{body}</span>}
          <button
            type="button"
            className="help-close"
            aria-label="ヘルプを閉じる"
            onClick={() => { setShow(false); setSticky(false) }}
          >
            ×
          </button>
        </span>
      )}
    </span>
  )
}
