// ros/useSystemState.js — subscribes to /system/state (+ /system/params_status)
// and distributes it through React context (DetailedDesign-webui.md §4.2).
//
// Owns its own rosbridge connection, independent of ros/useRosbridge.js
// (which screens use for their own, heavier topic set). The shell must keep
// working even before any screen mounts, so it does not depend on that hook.
//
// Test hook (no production behavior change): if `window.__thTestState` is
// defined before this module runs (e.g. via Playwright's addInitScript), no
// WebSocket connection is opened at all. Instead the provider serves a local
// state object seeded from that value, and installs `window.__thSetTestState`
// so a test can mutate it afterward (e.g. to open W-1 by setting
// `{ mode: 'ESTOP' }`). This lets e2e specs run with zero network requests
// (U-3) and no rosbridge backend. `window.__thSetTestFault` does the same
// for /safety/fault (WP-UI-02: W-1 generalized to fault-caused PAUSE, not
// just ESTOP -- DetailedDesign-wp1.md WP-UI-01 §11).
// No JSX here on purpose: this file must stay a plain .js module per
// DetailedDesign-webui.md §1's file layout, and Vite's default build
// pipeline only runs the JSX transform on .jsx (dev-server transforms are
// more lenient, which hid this until `npm run build` was actually run).
import { createContext, createElement, useContext, useEffect, useMemo, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

// /system/state publishes at 10Hz "+ on change" (DetailedDesign-wp1.md WP-UI-01
// §3.1). This is a display-only fail-safe threshold, not a safety boundary —
// real fault-to-motor-stop enforcement lives server-side (safety_monitor).
// Five missed periods is a reasonable "the link is gone" signal.
export const STATE_STALE_MS = 500

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const TEST_STATE_DEFAULTS = {
  mode: 'IDLE',
  state: 'NONE',
  prev_mode: '',
  prev_state: '',
  zone: 'NA',
  jog_active: false,
  estop_ui: false,
  estop_hw: false,
  tracker_enabled: true,
  auto_brake: true,
  working: false,
  map_update: false,
  unsaved: [],
  since: { sec: 0, nanosec: 0 },
  last_event: '',
  last_reject_reason: '',
}

// FaultStatus.msg defaults (DetailedDesign-names.md §5.1): no fault.
const TEST_FAULT_DEFAULTS = { active: false, fault_type: 'NONE', description: '', severity: 'RECOVERABLE' }

const SystemStateContext = createContext(null)

export function SystemStateProvider({ url = `ws://${window.location.hostname}:9090`, children }) {
  const rosRef = useRef(null)
  const [connected, setConnected] = useState(TEST_MODE)
  const [state, setState] = useState(TEST_MODE ? { ...TEST_STATE_DEFAULTS, ...window.__thTestState } : null)
  const [paramsStatus, setParamsStatus] = useState(null)
  const [fault, setFault] = useState(
    TEST_MODE ? { ...TEST_FAULT_DEFAULTS, ...window.__thTestFault } : TEST_FAULT_DEFAULTS)
  const [stale, setStale] = useState(!TEST_MODE)
  const staleTimerRef = useRef(null)

  // ── Test mode: no socket, just a local state object a test can mutate ──
  useEffect(() => {
    if (!TEST_MODE) return
    window.__thSetTestState = (patch) => {
      setState((prev) => ({ ...(prev ?? TEST_STATE_DEFAULTS), ...patch }))
    }
    window.__thSetTestFault = (patch) => {
      setFault((prev) => ({ ...(prev ?? TEST_FAULT_DEFAULTS), ...patch }))
    }
    return () => { delete window.__thSetTestState; delete window.__thSetTestFault }
  }, [])

  // ── Real mode: rosbridge connection ─────────────────────────────────
  useEffect(() => {
    if (TEST_MODE) return
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) { console.error('roslibjs is not loaded'); return }

    const ros = new ROSLIB.Ros({ url })
    rosRef.current = ros

    const armStaleTimer = () => {
      clearTimeout(staleTimerRef.current)
      staleTimerRef.current = setTimeout(() => setStale(true), STATE_STALE_MS)
    }

    ros.on('connection', () => setConnected(true))
    ros.on('error', () => { setConnected(false); setStale(true) })
    ros.on('close', () => { setConnected(false); setStale(true) })

    const subState = new ROSLIB.Topic({
      ros, name: TOPICS.SYSTEM_STATE, messageType: MSG_TYPES.SYSTEM_STATE,
    })
    subState.subscribe((msg) => { setState(msg); setStale(false); armStaleTimer() })

    const subParams = new ROSLIB.Topic({
      ros, name: TOPICS.PARAMS_STATUS, messageType: MSG_TYPES.PARAMS_STATUS,
    })
    subParams.subscribe((msg) => setParamsStatus(msg))

    const subFault = new ROSLIB.Topic({
      ros, name: TOPICS.SAFETY_FAULT, messageType: MSG_TYPES.FAULT_STATUS,
    })
    subFault.subscribe((msg) => setFault(msg))

    return () => {
      clearTimeout(staleTimerRef.current)
      subState.unsubscribe()
      subParams.unsubscribe()
      subFault.unsubscribe()
      ros.close()
    }
  }, [url])

  const value = useMemo(() => ({
    ros: TEST_MODE ? null : rosRef.current,
    connected,
    state,
    paramsStatus,
    fault,
    // Fail-safe default (§6.2): no state yet, or the link is stale/down ->
    // callers must treat mode/state as unknown, not as "whatever we saw last".
    stale: stale || !connected,
  }), [connected, state, paramsStatus, fault, stale])

  return createElement(SystemStateContext.Provider, { value }, children)
}

export function useSystemState() {
  const ctx = useContext(SystemStateContext)
  if (!ctx) throw new Error('useSystemState() must be used inside <SystemStateProvider>')
  return ctx
}
