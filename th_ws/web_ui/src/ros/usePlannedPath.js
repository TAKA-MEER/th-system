// ros/usePlannedPath.js — subscribes to /plan (nav_msgs/Path, planner_server が
// ComputePathToPose を実行するたびに publish するグローバル経路) for the
// S-20/S-21 地図タブ（brief-MAP-COSTMAP）。新規に publish する必要は無く、
// 既存のトピックを購読するだけ。
//
// このトピックも names.json の endpoints に無いため ros/topics.js の TOPICS には
// 足さない（useOnsiteMapView.js / useCostmap.js と同じ取り扱い）。
//
// /plan は replan 時だけ publish される低頻度トピックなので throttle は掛けない。
// 戻り値は poses を [ { x, y }, ... ] に整形した配列（map フレームなので
// 座標変換は不要、pose.position.x/y をそのまま使う）。空/未受信なら [] を返す
// （null ではなく空配列 — 呼び出し側は path.length > 0 だけ見ればよい）。
// TEST_MODE のシードは window.__thTestPlannedPath（配列を直接入れる。
// 例 [{x:0,y:0},{x:0.5,y:0.1}...]） / window.__thSetTestPlannedPath。
import { useEffect, useRef, useState } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const PLANNED_PATH_TOPIC = '/plan'
const PLANNED_PATH_MSG = 'nav_msgs/Path'

export function usePlannedPath(ros) {
  const topicRef = useRef(null)
  const [path, setPath] = useState(
    TEST_MODE ? (window.__thTestPlannedPath ?? []) : [])

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestPlannedPath = (v) => setPath(v)
    return () => { delete window.__thSetTestPlannedPath }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: PLANNED_PATH_TOPIC,
      messageType: PLANNED_PATH_MSG,
      queue_length: 1,
      compression: 'cbor',
    })
    topicRef.current.subscribe((msg) => {
      const poses = (msg?.poses ?? []).map((p) => ({
        x: p.pose.position.x,
        y: p.pose.position.y,
      }))
      setPath(poses)
    })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return path
}