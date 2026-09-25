// parts/HoldButton.jsx — the "共通部品: ホールドボタン" from
// Spec-webui.md §6 (press-and-hold; release stops it). First user: S-30's
// MOTOR item (4 direction buttons, WP-UI-08 / DetailedDesign-maintenance.md
// §2.3), where release() feeding ros/useMotorHold.js is a safety path, not
// just UI polish.
//
// Deliberately dumb: this component only turns pointer events into
// onPress(direction) / onRelease() calls. It does not publish anything
// itself (same split as parts/VirtualStick.jsx + ros/useJogLease.js) so a
// caller can layer its own safety gating (useMotorHold's `gate`) without
// this component needing to know about it.
//
// Pointer capture (same as VirtualStick.jsx): once pressed, the button
// keeps receiving pointerup/pointercancel even if the pointer physically
// leaves it before release, so "dragged off the button" still stops the
// hold correctly. try/catch guards a synthetic Playwright PointerEvent that
// may not have an active pointer to capture (InvalidPointerId) -- the drag
// must not abort because of that.
//
// U3-4: onPress/onRelease are read from refs at call time, so a re-render
// mid-hold (e.g. the gate flips) always calls the latest callback, and the
// unmount cleanup calls onRelease unconditionally if still held -- mirrors
// VirtualStick.jsx's own unmount safety (U3-1).
import { useEffect, useRef } from 'react'

export default function HoldButton({
  direction, label, active = false, disabled = false, onPress, onRelease, testId,
}) {
  const pointerIdRef = useRef(null)
  const onPressRef = useRef(onPress)
  const onReleaseRef = useRef(onRelease)
  onPressRef.current = onPress
  onReleaseRef.current = onRelease

  useEffect(() => () => {
    if (pointerIdRef.current !== null) {
      pointerIdRef.current = null
      onReleaseRef.current?.()
    }
  }, [])

  const handleDown = (e) => {
    e.preventDefault()
    try { e.currentTarget.setPointerCapture(e.pointerId) } catch { /* synthetic event */ }
    pointerIdRef.current = e.pointerId
    onPressRef.current?.(direction)
  }
  const handleUp = (e) => {
    if (pointerIdRef.current !== e.pointerId) return
    pointerIdRef.current = null
    onReleaseRef.current?.()
  }

  return (
    <button
      type="button"
      className={`btn hold-btn ${active ? 'on' : ''}`}
      disabled={disabled}
      data-testid={testId}
      onPointerDown={disabled ? undefined : handleDown}
      onPointerUp={handleUp}
      onPointerCancel={handleUp}
      onContextMenu={(e) => e.preventDefault()}
    >
      {label}
    </button>
  )
}
