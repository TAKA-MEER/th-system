// ros/useScan.js — subscribes to /scan_filtered (sensor_msgs/LaserScan)
// for the S-20/S-21 地図タブの LiDAR 点群オーバーレイ（brief-MAP-SCAN）。
//
// /scan_filtered は lidar_filter が死角除去した 10Hz の生スキャン。base_link/laser_link
// 相対（laser_link は base_link と x/y/yaw 同一、Z のみ車輪半径分違う）。点群への
// 変換（scanToPoints）は呼び出し側（画面）で行う。
//
// TEST_MODE: window.__thTestScan（seed）＋ window.__thSetTestScan（mutate）—
// ros/usePersonStatus.js と同じ形（MAP-1）。シードは addInitScript で
// window.__thTestState より先に置く。
//
// /scan_filtered は辞書（names.json）には載っているが、topics.js の TOPICS には
// 追加しない（useOnsiteMapView.js の先頭コメント参照。test/unit/
// topics-in-dictionary.test.js の辞書ゲートを通すため、割り込み系はローカル定数で
// 持つ）。トピック名はこのファイルの中だけで使う。
import { useEffect, useRef, useState } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const SCAN_TOPIC = '/scan_filtered'
const SCAN_MSG = 'sensor_msgs/LaserScan'
// 10Hz は描画には多すぎるので 0.5 秒に 1 回程度に間引く（useOnsiteMapView.js の
// ONSITE_MAP_THROTTLE_MS と同じ考え方）。
const SCAN_THROTTLE_MS = 500

export function useScan(ros) {
  const topicRef = useRef(null)
  const [scan, setScan] = useState(
    TEST_MODE ? (window.__thTestScan ?? null) : null,
  )

  // TEST_MODE: let e2e mutate the seeded scan after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestScan = (v) => setScan(v)
    return () => { delete window.__thSetTestScan }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: SCAN_TOPIC,
      messageType: SCAN_MSG,
      queue_length: 1,
      throttle_rate: SCAN_THROTTLE_MS,
    })
    topicRef.current.subscribe((msg) => setScan(msg ?? null))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return scan
}