// ros/useCostmap.js — subscribes to /global_costmap/costmap
// (nav_msgs/OccupancyGrid, Nav2 のグローバル costmap) for the S-20/S-21 地図タブ
// （brief-MAP-COSTMAP）。
//
// このトピックは names.json の endpoints に無いため、ros/topics.js の TOPICS には
// 足さない（足すと test/unit/topics-in-dictionary.test.js の辞書ゲートに引っかかる）。
// トピック名は ros/useOnsiteMapView.js と同じくローカル定数にする。
//
// costmap は Nav2 が高頻度で更新するが、表示目的がメインなので throttle_rate
// 2000ms で間引く（useOnsiteMapView の ONSITE_MAP_THROTTLE_MS と同じ考え方）。
// フックの形も useOnsiteMapView と完全に同じ: latest OccupancyGrid を
// { info, data } で返す。null なら未受信。TEST_MODE のシードは
// window.__thTestCostmap（mutate 用に window.__thSetTestCostmap も足す）。
//
// 「入れない所」の入切（parts/costmapVisibility.js の th.costmapVisible）は
// ここでは見ない。購読は常時して、描くか描かないかは地図の部品（OnsiteMap）
// が持つ。入切を購読で止めると、地図上の「入れない所」ボタンで入れたあと
// 再購読するまで地図が戻らないため。
import { useEffect, useRef, useState } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const COSTMAP_TOPIC = '/global_costmap/costmap'
const COSTMAP_MSG = 'nav_msgs/OccupancyGrid'
const COSTMAP_THROTTLE_MS = 2000

export function useOnsiteCostmap(ros) {
  const topicRef = useRef(null)
  const [costmapData, setCostmapData] = useState(
    TEST_MODE ? (window.__thTestCostmap ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestCostmap = (v) => setCostmapData(v)
    return () => { delete window.__thSetTestCostmap }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: COSTMAP_TOPIC,
      messageType: COSTMAP_MSG,
      queue_length: 1,
      throttle_rate: COSTMAP_THROTTLE_MS,
      compression: 'cbor',
    })
    topicRef.current.subscribe((grid) => {
      if (!grid?.info || !grid.data) return
      setCostmapData(grid)
    })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return costmapData
}
