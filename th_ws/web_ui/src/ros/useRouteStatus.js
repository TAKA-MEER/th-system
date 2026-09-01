// ros/useRouteStatus.js — subscribes to /route/status (th_system_msgs/RouteStatus)
// for the S-13 recording-status display (P5 / demo-teach-replay).
//
// route_recorder publishes the in-progress teaching status (state / recorded_m /
// elapsed_sec / points); this hook hands back the latest RouteStatus message
// (or null until one arrives). Mirrors ros/useLimiterStatus.js: owns its own
// rosbridge Topic, does not go through useRosbridge.js, and in TEST_MODE reads
// window.__thTestRouteStatus (seed) + exposes window.__thSetTestRouteStatus
// (mutate) for e2e.
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useRouteStatus(ros) {
  const topicRef = useRef(null)
  const [status, setStatus] = useState(
    TEST_MODE ? (window.__thTestRouteStatus ?? null) : null)

  // TEST_MODE: let e2e mutate the seeded status after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestRouteStatus = (v) => setStatus(v)
    return () => { delete window.__thSetTestRouteStatus }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.ROUTE_STATUS,
      messageType: MSG_TYPES.ROUTE_STATUS,
      subscribeOptions: { queueSize: 1, throttle_rate: 0, latching: true },
    })
    topicRef.current.subscribe((msg) => setStatus(msg))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return status
}
