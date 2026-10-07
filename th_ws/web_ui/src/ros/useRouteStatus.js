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

// TEST_MODE で同じトピックの購読者が複数いても全員に届ける（1b-7: シェルが
// W-4 の未保存判定（useSaveConfirm）で購読し、S-13 等の画面も購読する。
// useSystemEffect.js と同じ流儀）。
const _testSinks = new Set()

export function useRouteStatus(ros) {
  const topicRef = useRef(null)
  const [status, setStatus] = useState(
    TEST_MODE ? (window.__thTestRouteStatus ?? null) : null)

  // TEST_MODE: let e2e mutate the seeded status after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    const sink = (v) => setStatus(v)
    _testSinks.add(sink)
    window.__thSetTestRouteStatus = (v) => { _testSinks.forEach((fn) => fn(v)) }
    return () => { _testSinks.delete(sink) }
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
      queue_length: 1,   // 最新のみ保持（詰まった後の古いメッセージのバースト配信を防ぐ）
    })
    topicRef.current.subscribe((msg) => setStatus(msg))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return status
}
