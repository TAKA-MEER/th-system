// ros/useOnsiteMapView.js — subscribes to /onsite/map_view (nav_msgs/OccupancyGrid)
// for the S-20/S-21 試験場内地図タブ（brief-onsite-ux2 F-2）。
//
// /route/map_view（map_downsampler, factor=4=0.20m/セル）は校舎 1 周級の教示・
// 再生用で、これは絶対に変えない（S-13/S-14 が使う）。試験場内は狭いので、
// bringup.launch.py が 2 個目の map_downsampler インスタンス（name=
// onsite_map_downsampler, factor=1=間引かない）を onsite_enabled のときだけ
// 立てて /onsite/map_view へ配信する。publish 周期（既定 2s）は
// /route/map_view と同じ既定値のまま。
//
// /onsite/map_view は names.json の endpoints にまだ無いため、ros/topics.js の
// TOPICS には足さない（足すと test/unit/topics-in-dictionary.test.js の辞書
// ゲートに引っかかる）。ros/useOdomPose.js / ros/routeMapTopicConfig.js と同じ
// く、トピック名をこのファイルの中だけで持つローカル定数にする。
//
// フックの形は ros/useRouteMap.js と完全に同じ（latest OccupancyGrid を
// { info, data } で返す。null なら未受信）。TEST_MODE のシードは
// window.__thTestOnsiteMap（mutate 用に window.__thSetTestOnsiteMap も足す）。
import { useEffect, useRef, useState } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const ONSITE_MAP_VIEW_TOPIC = '/onsite/map_view'
const ONSITE_MAP_VIEW_MSG = 'nav_msgs/OccupancyGrid'
// ros/routeMapTopicConfig.js の MAP_THROTTLE_MS と同じ考え方（publish 周期より
// 短く間引いても意味が無い。取りこぼしの再送だけ受ける）。
const ONSITE_MAP_THROTTLE_MS = 2000

export function useOnsiteMapView(ros) {
  const topicRef = useRef(null)
  const [mapData, setMapData] = useState(
    TEST_MODE ? (window.__thTestOnsiteMap ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestOnsiteMap = (v) => setMapData(v)
    return () => { delete window.__thSetTestOnsiteMap }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: ONSITE_MAP_VIEW_TOPIC,
      messageType: ONSITE_MAP_VIEW_MSG,
      queue_length: 1,
      throttle_rate: ONSITE_MAP_THROTTLE_MS,
      compression: 'cbor',
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
