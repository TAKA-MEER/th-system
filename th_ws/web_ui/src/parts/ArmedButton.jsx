// parts/ArmedButton.jsx — two-stage "arm, then confirm" button.
// Extracted from App.jsx's map-discard button (handleDiscardClick, ~L262-286
// pre-WP-UI-01). Same behavior: first press arms (label swaps, 10s
// auto-disarm); a second press while armed fires onConfirm. Used for
// irreversible actions where a single accidental tap must not be enough.
import { useCallback, useEffect, useRef, useState } from 'react'

const DEFAULT_ARM_MS = 10000

export default function ArmedButton({
  idleLabel,
  armedLabel,
  onConfirm,
  disabled = false,
  armMs = DEFAULT_ARM_MS,
  className = '',
}) {
  const [armed, setArmed] = useState(false)
  const timerRef = useRef(null)

  const disarm = useCallback(() => {
    clearTimeout(timerRef.current)
    timerRef.current = null
    setArmed(false)
  }, [])

  const handleClick = useCallback(() => {
    if (armed) {
      disarm()
      onConfirm()
      return
    }
    setArmed(true)
    timerRef.current = setTimeout(disarm, armMs)
  }, [armed, disarm, onConfirm, armMs])

  // A newly-disabled button (e.g. mode changed underneath it) must not stay
  // armed silently.
  useEffect(() => {
    if (disabled && armed) disarm()
  }, [disabled, armed, disarm])

  useEffect(() => () => clearTimeout(timerRef.current), [])

  return (
    <button
      type="button"
      className={`btn ${armed ? 'danger' : ''} ${className}`}
      disabled={disabled}
      onClick={handleClick}
    >
      {armed ? armedLabel : idleLabel}
    </button>
  )
}
