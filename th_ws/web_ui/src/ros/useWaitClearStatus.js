// ros/useWaitClearStatus.js — subscribes to /onsite/wait_clear
// (th_system_msgs/WaitClearStatus) for the S-21 呼び寄せ tab (WP-UI-07).
// wait_clear_gate publishes it only while SUMMON/WAIT_CLEAR; the screen uses
// distance_m / remaining_sec / verdict ("OK" | "WAITING" | "NOT_CLEAR") to
// render the 退避待ち box.
//
// Mirrors ros/usePersonTargets.js: owns its own rosbridge Topic and hands back
// the whole observed message (defaults until a message arrives). In TEST_MODE
// reads window.__thTestWaitClear (seed) + installs
// window.__thSetTestWaitClear (mutate) for e2e.
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const DEFAULT_WAIT = {
  distance_m: 0,
  remaining_sec: 0,
  satisfied: false,
  verdict: 'WAITING',
}

export function useWaitClearStatus(ros) {
  const topicRef = useRef(null)
  const [wait, setWait] = useState(
    TEST_MODE ? ({ ...DEFAULT_WAIT, ...(window.__thTestWaitClear ?? {}) }) : DEFAULT_WAIT)

  // TEST_MODE: let e2e mutate the seeded value after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestWaitClear = (v) => setWait({ ...DEFAULT_WAIT, ...(v ?? {}) })
    return () => { delete window.__thSetTestWaitClear }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.WAIT_CLEAR,
      messageType: MSG_TYPES.WAIT_CLEAR,
      queue_length: 1,   // 最新のみ
    })
    topicRef.current.subscribe((msg) => setWait({ ...DEFAULT_WAIT, ...(msg ?? {}) }))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return wait
}