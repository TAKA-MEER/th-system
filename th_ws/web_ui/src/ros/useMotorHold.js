// ros/useMotorHold.js — the ONLY place S-30's MOTOR item publishes
// /opcheck/motor_hold (std_msgs/String, WP-UI-08 / WP-MAINT-01).
//
// opcheck_runner.py runs this as a deadman: while a direction keeps arriving
// it drives /cmd_vel_behavior at v_check; if the topic goes quiet for
// opcheck_deadman_timeout_s (registry default 0.5s, PARS in
// opcheck_runner.py) it treats that as "released" and zeroes the command.
// This hook re-sends the held direction every 100ms (5x inside the 0.5s
// budget) so a single dropped rosbridge frame can't be read as a release,
// and sends "NONE" exactly once the moment the hold ends for any reason.
//
// Modeled directly on ros/useJogLease.js (same shape: one Topic, one
// interval, a single release path used by every trigger). Differences from
// useJogLease: this is a discrete direction string, not a ramped Twist, and
// there is a second stop condition useJogLease doesn't need -- `gate`
// (caller-supplied bool). opcheck_runner._sync_command does NOT check which
// item is running (see completion report "B"): as long as mode==OPCHECK it
// will drive /cmd_vel_behavior off of whatever /opcheck/motor_hold last
// said, even in LIST state. So the UI-side gate (RUNNING_CHECK + item==
// 'MOTOR' + not stale + not estopped) is the only thing stopping a hold
// begun on the MOTOR screen from continuing to drive the base after the
// item ends server-side; it must be re-evaluated continuously, not just at
// press time.
//
// Stops (sends "NONE" once, idempotent) on: pointerup/pointercancel/
// lostpointercapture (caller's responsibility to call release()),
// visibilitychange -> hidden, window blur, unmount, and gate turning false.
//
// TEST_MODE: no rosbridge under Playwright. Every publish (including the
// terminal "NONE") is recorded on window.__thMotorHoldPublishes as
// { data, t } (t = performance.now(), for e2e to check spacing/counts).
import { useEffect, useRef } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

// names.json's endpoint dictionary doesn't have this topic yet (it postdates
// this packet's node-side merge; opcheck_runner.py's own header comment says
// the name/type/timeout are to be written into DetailedDesign-names.md by
// the implementation lead from the completion report). Kept as a local
// constant, same carve-out as ros/useOdomPose.js's ODOM_TOPIC, so
// test/unit/topics-in-dictionary.test.js doesn't have to know about it.
const MOTOR_HOLD_TOPIC = '/opcheck/motor_hold'
const MOTOR_HOLD_MSG = 'std_msgs/String'

// 5x within opcheck_deadman_timeout_s's 0.5s default (PARS.opcheck_deadman_timeout_s
// in opcheck_runner.py) -- a comfortable margin without spamming rosbridge.
const REPEAT_MS = 100

export function useMotorHold(ros, gate) {
  const topicRef = useRef(null)
  const timerRef = useRef(null)
  const dirRef = useRef('NONE') // currently-held direction, 'NONE' when idle
  const gateRef = useRef(gate)
  gateRef.current = gate

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros, name: MOTOR_HOLD_TOPIC, messageType: MOTOR_HOLD_MSG,
    })
    return () => { topicRef.current = null }
  }, [ros])

  const send = (data) => {
    if (TEST_MODE) {
      window.__thMotorHoldPublishes = window.__thMotorHoldPublishes ?? []
      window.__thMotorHoldPublishes.push({ data, t: performance.now() })
      return
    }
    if (topicRef.current && window.ROSLIB) {
      topicRef.current.publish(new window.ROSLIB.Message({ data }))
    }
  }

  const stopTimer = () => {
    if (timerRef.current) clearInterval(timerRef.current)
    timerRef.current = null
  }

  // Idempotent: only actually sends "NONE" if a direction was held.
  const doRelease = () => {
    if (dirRef.current === 'NONE') { stopTimer(); return }
    dirRef.current = 'NONE'
    stopTimer()
    send('NONE')
  }

  const press = (direction) => {
    if (!gateRef.current) return // safety gate closed -- refuse the hold entirely
    if (dirRef.current === direction) return // already holding this one
    dirRef.current = direction
    send(direction)
    stopTimer()
    timerRef.current = setInterval(() => {
      if (!gateRef.current) { doRelease(); return }
      send(dirRef.current)
    }, REPEAT_MS)
  }

  const release = () => doRelease()

  // Gate turning false while held (item finished, mode left OPCHECK, link
  // went stale, estop engaged) must stop the hold even with no pointer
  // event -- the interval tick above catches most of this, but react
  // immediately on the transition too instead of waiting up to REPEAT_MS.
  useEffect(() => {
    if (!gate) doRelease()
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [gate])

  // Tab hidden / window blur: treat exactly like a release (same reasoning
  // as useJogLease.js). Not skipped in TEST_MODE -- e2e must be able to
  // exercise this path (a prior gap in useJogLease.js left it untested).
  useEffect(() => {
    const onVisibility = () => {
      if (typeof document !== 'undefined' && document.visibilityState !== 'visible') doRelease()
    }
    const onBlur = () => doRelease()
    if (typeof document !== 'undefined') document.addEventListener('visibilitychange', onVisibility)
    if (typeof window !== 'undefined') window.addEventListener('blur', onBlur)
    return () => {
      if (typeof document !== 'undefined') document.removeEventListener('visibilitychange', onVisibility)
      if (typeof window !== 'undefined') window.removeEventListener('blur', onBlur)
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // Unmount mid-hold (screen switches away, e.g. RUNNING_CHECK -> LIST
  // moved the item on): send the terminal NONE regardless of what caused
  // the unmount.
  useEffect(() => () => doRelease(), [])

  return { press, release }
}
