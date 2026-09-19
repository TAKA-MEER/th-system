// ros/useLimiterStatus.js — subscribes to /safety/limiter_status for the
// S-11 obstacle-warning display (DetailedDesign-wp3.md WP-TRANSIT-01 §3.1).
//
// The publisher (obstacle_limiter) is 20 Hz best_effort, so the subscription
// must set the same QoS: best_effort + depth 1. A lossy best_effort stream is
// exactly why the S-11 obstacle card renders "不明" when nothing has arrived
// yet and must not claim "障害物なし" (fail-safe, §6.2) — the hook only ever
// hands back the latest received message (or null).
//
// Like /system/state (ros/useSystemState.js) this owns its own rosbridge
// topic; it does not go through useRosbridge.js. TEST_MODE mirrors the other
// ros/ hooks so e2e can seed window.__thTestLimiterStatus.
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useLimiterStatus(ros) {
  const topicRef = useRef(null)
  const [status, setStatus] = useState(
    TEST_MODE ? (window.__thTestLimiterStatus ?? null) : null)

  // TEST_MODE: let e2e mutate the value after mount (same pattern as
  // useSystemState.js's window.__thSetTestState).
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestLimiterStatus = (v) => setStatus(v)
    return () => { delete window.__thSetTestLimiterStatus }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders. QoS: the publisher
  // is best_effort at 20 Hz; roslibjs exposes local-queue depth only (no
  // native policy switch), so depth 1 matches the publisher's intent and
  // keeps a burst from stalling the UI.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.LIMITER_STATUS,
      messageType: MSG_TYPES.LIMITER_STATUS,
      queue_length: 1,
    })
    topicRef.current.subscribe((msg) => setStatus(msg))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return status
}
