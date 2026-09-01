// ros/useRouteCatalog.js — subscribes to /route/catalog (th_system_msgs/RouteList)
// for the S-13/S-14 taught-route list (P5 / demo-teach-replay).
//
// /route/catalog is published by route_recorder (while teaching) and
// replay_runner (while replaying); both are 20 Hz-adjacent best_effort
// publishers but the hook only ever hands back the latest RouteList (or the
// routes array it carries). Mirrors ros/useLimiterStatus.js: owns its own
// rosbridge Topic, does not go through useRosbridge.js, and in TEST_MODE
// reads window.__thTestRouteCatalog (seed) + exposes
// window.__thSetTestRouteCatalog (mutate) for e2e.
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useRouteCatalog(ros) {
  const topicRef = useRef(null)
  // The hook's public value is the routes array (RouteList.routes).
  const [routes, setRoutes] = useState(
    TEST_MODE ? (window.__thTestRouteCatalog ?? []) : [])

  // TEST_MODE: let e2e mutate the seeded catalog after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestRouteCatalog = (v) => setRoutes(v)
    return () => { delete window.__thSetTestRouteCatalog }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.ROUTE_CATALOG,
      messageType: MSG_TYPES.ROUTE_LIST,
      subscribeOptions: { queueSize: 1, throttle_rate: 0, latching: true },
    })
    topicRef.current.subscribe((msg) => setRoutes(msg?.routes ?? []))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return routes
}
