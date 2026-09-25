// ros/useOpcheckStatus.js — subscribes to /opcheck/status (CheckStatus) for
// S-30 (WP-UI-08 / WP-MAINT-01).
//
// opcheck_runner.py publishes this topic ~10Hz while an item is running
// (_monitor_tick), and it is volatile (no TRANSIENT_LOCAL) -- a screen that
// mounts after the last message arrived (e.g. S-31 after S-30 transitioned
// away) sees nothing until the next publish. Worse: opcheck_runner's
// _monitor_tick always ends with `_publish_status(result="UNKNOWN", ...)`
// even the tick right after a final OK/WARN/NG was published (its item-close
// effects -- record_result / abort_check -- aren't wired for the NG paths,
// T-OPC-02/03/06 in transitions.yaml carry no such effect), so a final
// verdict can be overwritten by "UNKNOWN" 100ms later even while `_item`
// stays set. This is a node/FSM-side gap, out of this packet's scope
// (reported instead of patched -- see the completion report).
//
// To stay correct despite that, this hook LATCHES the last non-UNKNOWN
// result per item in a module-level object (`latched`), which survives the
// S-30 <-> S-31 screen swap within the same page load (React unmounts the
// screen component, but this module is not re-imported). Every hook
// instance seeds its own React `results` state from that latch on mount, so
// a freshly-mounted screen shows the last known verdicts immediately, then
// keeps them updated live.
//
// TEST_MODE mirrors ros/useLimiterStatus.js: window.__thTestOpcheckStatus
// seeds the initial value, window.__thSetTestOpcheckStatus mutates it after
// mount (both go through the same applyStatus() so the latch updates too).
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

// item -> { result, detail, next_screen } for the last non-UNKNOWN message
// seen. Module-level on purpose (see file header) -- not React state.
const latched = {}

function applyLatch(msg) {
  if (!msg || !msg.item || !msg.result || msg.result === 'UNKNOWN') return
  latched[msg.item] = {
    result: msg.result,
    detail: msg.detail ?? '',
    next_screen: msg.next_screen ?? '',
  }
}

export function useOpcheckStatus(ros) {
  const topicRef = useRef(null)
  const [status, setStatus] = useState(TEST_MODE ? (window.__thTestOpcheckStatus ?? null) : null)
  const [results, setResults] = useState(() => ({ ...latched }))

  // Seed the latch from a TEST_MODE initial value too, so a spec that opens
  // straight into S-31 with a pre-seeded status still sees it.
  useEffect(() => {
    if (TEST_MODE && window.__thTestOpcheckStatus) {
      applyLatch(window.__thTestOpcheckStatus)
      setResults({ ...latched })
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  const applyStatus = (msg) => {
    setStatus(msg)
    applyLatch(msg)
    setResults({ ...latched })
  }

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestOpcheckStatus = (v) => applyStatus(v)
    return () => { delete window.__thSetTestOpcheckStatus }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros, name: TOPICS.OPCHECK_STATUS, messageType: MSG_TYPES.CHECK_STATUS,
    })
    topicRef.current.subscribe((msg) => applyStatus(msg))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ros])

  return { status, results }
}
