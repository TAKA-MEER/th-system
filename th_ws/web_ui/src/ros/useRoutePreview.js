// ros/useRoutePreview.js — subscribes to /route/preview (nav_msgs/Path) for the
// S-13/S-14 route-preview polyline (WS-3 / demo-teach-replay).
//
// route_recorder publishes the in-progress preview, replay_runner the playback
// preview (odom frame). The hook flattens Path.poses to [{x,y}, ...] (initial
// []). It owns its own rosbridge Topic like ros/useLimiterStatus.js, and in
// TEST_MODE reads window.__thTestRoutePreview (seed, as [{x,y},...]) + installs
// window.__thSetTestRoutePreview (mutate) for e2e.
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useRoutePreview(ros) {
  const topicRef = useRef(null)
  const [preview, setPreview] = useState(
    TEST_MODE ? (window.__thTestRoutePreview ?? []) : [])

  // TEST_MODE: let e2e mutate the seeded preview after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestRoutePreview = (v) => setPreview(v)
    return () => { delete window.__thSetTestRoutePreview }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.ROUTE_PREVIEW,
      messageType: MSG_TYPES.PATH,
      subscribeOptions: { queueSize: 2, throttle_rate: 0, latching: false },
    })
    topicRef.current.subscribe((msg) => {
      const pts = (msg?.poses ?? []).map((p) => ({
        x: p.pose.position.x, y: p.pose.position.y,
      }))
      setPreview(pts)
    })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return preview
}