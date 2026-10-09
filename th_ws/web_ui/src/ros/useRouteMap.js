// ros/useRouteMap.js — subscribes to /route/map_view (nav_msgs/OccupancyGrid) for
// the S-13/S-14 route-preview background map layer (WS-8B / demo-teach-replay).
//
// /route/map_view is published by map_downsampler (th_planning), which takes the
// raw slam_toolbox /map (resolution 0.05m/cell) and downsamples it to a
// display-only copy (factor 4 → 0.20m/cell) so a school-loop-sized map does not
// eat the 2.4GHz wireless link (WS-9G). It is a transient_local topic, so the
// latest single map arrives to clients that connect late. The hook hands back
// the latest OccupancyGrid as { info, data } (null until one is received), so
// the route preview falls back to the map-less view when no map exists
// (enable_route_slam:=false). Mirrors ros/useOdomPose.js / the transient_local
// useRouteCatalog pattern: owns its own rosbridge Topic, and in TEST_MODE reads
// window.__thTestRouteMap (seed) for e2e.
import { useEffect, useRef, useState } from 'react'
import {
  routeMapTopicConfig, MAP_THROTTLE_MS,
  ESTIMATOR_RESTARTING_TOPIC, restartClearsMap,
} from './routeMapTopicConfig.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useRouteMap(ros, { clearOnRestart = false } = {}) {
  const topicRef = useRef(null)
  const restartRef = useRef(null)
  const [mapData, setMapData] = useState(
    TEST_MODE ? (window.__thTestRouteMap ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestRouteMap = (v) => setMapData(v)
    window.__thTestRouteMapRestart = (v) => {
      if (clearOnRestart && restartClearsMap(v)) setMapData(null)
    }
    return () => {
      delete window.__thSetTestRouteMap
      delete window.__thTestRouteMapRestart
    }
  }, [clearOnRestart])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic(routeMapTopicConfig(ros))
    topicRef.current.subscribe((grid) => {
      if (!grid?.info || !grid.data) return
      setMapData(grid)
    })
    if (clearOnRestart) {
      restartRef.current = new ROSLIB.Topic({
        ros, name: ESTIMATOR_RESTARTING_TOPIC, messageType: 'std_msgs/Bool',
        queue_length: 1,
      })
      restartRef.current.subscribe((msg) => {
        if (restartClearsMap(msg)) setMapData(null)
      })
    }
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
      restartRef.current?.unsubscribe()
      restartRef.current = null
    }
  }, [ros, clearOnRestart])

  return mapData
}