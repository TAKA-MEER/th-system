// ros/useRouteMap.js — subscribes to /map (nav_msgs/OccupancyGrid) for the
// S-13/S-14 route-preview background map layer (WS-8B / demo-teach-replay).
//
// /map is published by slam_toolbox when the route is taught/replayed with
// enable_route_slam:=true, in the map frame, as a transient_local topic. The
// hook hands back the latest OccupancyGrid as { info, data } (null until one is
// received), so the route preview can fall back to the map-less view when no
// map exists (enable_route_slam:=false). Mirrors ros/useOdomPose.js / the
// transient_local useRouteCatalog pattern: owns its own rosbridge Topic, and in
// TEST_MODE reads window.__thTestRouteMap (seed) for e2e.
import { useEffect, useRef, useState } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const ROUTE_MAP_TOPIC = '/map'
const ROUTE_MAP_MSG = 'nav_msgs/OccupancyGrid'

// slam_toolbox 側の map_update_interval は 5.0s（slam_params.yaml）。地図 1 枚は
// 数十万セルの JSON で、受信のたびに RoutePreview が width×height の二重ループを
// 回して ImageData を作る＝メインスレッドが固まる。publish 周期より短く間引いても
// 意味が無いので、受信側もこの程度に抑えて取りこぼしの再送だけ受ける。
const MAP_THROTTLE_MS = 2000

export function useRouteMap(ros) {
  const topicRef = useRef(null)
  const [mapData, setMapData] = useState(
    TEST_MODE ? (window.__thTestRouteMap ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestRouteMap = (v) => setMapData(v)
    return () => { delete window.__thSetTestRouteMap }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: ROUTE_MAP_TOPIC,
      messageType: ROUTE_MAP_MSG,
      queue_length: 1,
      // /map は数十万セル。5秒ごとの巨大 JSON でメインスレッドが固まるので間引く。
      throttle_rate: MAP_THROTTLE_MS,
    })
    topicRef.current.subscribe((grid) => {
      if (!grid?.info || !grid.data) return
      setMapData(grid)
    })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return mapData
}