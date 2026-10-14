// parts/useKeyboardJog.js — WASD / arrow-key manual drive for the drive tab
// (WS-4 / demo-teach-replay). Keyboard is a *second proportional source* for the
// same JogConsole (parts/JogConsole.jsx) state machine: it produces a normalized
// {vn, wn} command (vn forward-positive, wn left-positive, matching
// stickGeometry.js) and a `held` flag, exactly like the virtual stick does, and
// JogConsole merges it (stick wins while touched, else keyboard). Nothing is
// published here — the only publisher remains useJogLease.
//
// enabled: should the drive tab accept keyboard right now (i.e. !stale). While
// disabled the hook ignores drive keys entirely and drops any in-progress hold,
// so a stale display can never keep a key command alive.
//
// Rules (DetailedDesign-wp3.md WP-UI-03 §9 note on U3-1 / focus):
//   - Key mapping: w/ArrowUp -> vn=+1, s/ArrowDown -> vn=-1, a/ArrowLeft -> wn=+1
//     (left turn), d/ArrowRight -> wn=-1 (right turn).
//   - Simultaneous presses add and clamp to [-1, 1] per axis.
//   - held = at least one drive key is down; all released -> held=false, cmd={0,0}.
//   - Never steals keys from text entry: when document.activeElement is a
//     TEXTAREA/SELECT, contentEditable, or an INPUT whose type takes
//     characters, the event is ignored. range/checkbox/radio/button etc.
//     never steal (R-52: the speed slider keeps focus after a tap).
//   - preventDefault() only for the arrow keys, to stop page scroll.
//
// `held` and `cmd` live in ONE state object so a keydown or keyup is a single
// render — otherwise JogConsole would briefly see held=true with cmd={0,0}
// (or vice-versa) and useJogLease would emit an extra zero
// (e2e/keyboard-jog.spec.js "release ... a single zero").
import { useEffect, useRef, useState } from 'react'

// e.key values, normalised: letters are compared lower-case, arrows keep their
// exact `ArrowXxx` spelling (they are not affected by case).
const VN = { w: 1, arrowup: 1, s: -1, arrowdown: -1 }
const WN = { a: 1, arrowleft: 1, d: -1, arrowright: -1 } // 左旋回 = wn 正
const ARROW = new Set(['arrowup', 'arrowdown', 'arrowleft', 'arrowright'])
const DRIVE_KEYS = new Set([...Object.keys(VN), ...Object.keys(WN)])

const clamp = (v) => Math.max(-1, Math.min(1, v))

// Only text-entry controls steal keys. INPUTs without a character type
// (range/checkbox/radio/button/...) keep keyboard jog working while focused.
// el.type is "text" when the attribute is missing or unknown, so a bare
// <input> still counts as typing.
const TYPING_INPUT_TYPES = new Set([
  'text', 'number', 'search', 'password', 'email', 'tel', 'url',
])

const isTypingTarget = (el) => {
  if (el == null) return false
  if (el.isContentEditable) return true
  if (el.tagName === 'TEXTAREA' || el.tagName === 'SELECT') return true
  if (el.tagName === 'INPUT') return TYPING_INPUT_TYPES.has((el.type || 'text').toLowerCase())
  return false
}

const STOP = { held: false, vn: 0, wn: 0 }

function fromKeys(keys) {
  let vn = 0
  let wn = 0
  keys.forEach((k) => {
    vn += VN[k] || 0
    wn += WN[k] || 0
  })
  return { held: keys.size > 0, vn: clamp(vn), wn: clamp(wn) }
}

export function useKeyboardJog(enabled) {
  const [s, setS] = useState(STOP)
  const keys = useRef(new Set())
  const enabledRef = useRef(enabled)
  enabledRef.current = enabled

  useEffect(() => {
    const apply = () => setS(fromKeys(keys.current))

    const onKeyDown = (e) => {
      const k = e.key.toLowerCase()
      if (ARROW.has(k)) e.preventDefault()
      if (!DRIVE_KEYS.has(k)) return
      if (isTypingTarget(document.activeElement) || !enabledRef.current) return
      if (e.repeat || keys.current.has(k)) return
      keys.current.add(k)
      apply()
    }
    const onKeyUp = (e) => {
      const k = e.key.toLowerCase()
      if (!keys.current.has(k)) return
      keys.current.delete(k)
      apply()
    }
    const onBlur = () => {
      if (keys.current.size) {
        keys.current.clear()
        apply()
      }
    }
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    window.addEventListener('blur', onBlur)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
      window.removeEventListener('blur', onBlur)
      keys.current.clear()
    }
  }, [])

  // Drop an in-progress hold as soon as the drive tab is no longer enabled.
  useEffect(() => {
    if (!enabled && keys.current.size) {
      keys.current.clear()
      setS(STOP)
    }
  }, [enabled])

  return { held: s.held, cmd: { vn: s.vn, wn: s.wn } }
}
