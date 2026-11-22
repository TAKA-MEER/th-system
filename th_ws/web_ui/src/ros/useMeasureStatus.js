// ros/useMeasureStatus.js — 測定機能（fk-system）の状態（/measure/status）を購読する。
//
// DetailedDesign-measure.md §3.3 の MeasureStatus。S-01 の「自律測定」の入口
// （link_ok が true のときだけ許可の W-4 を出す）と S-60 の状態表示が使う。
//
// TEST_MODE は useAutoCheckStatus.js と同じ流儀: window.__thTestMeasureStatus を
// 種にし、window.__thSetTestMeasureStatus(v) で表示を更新する。
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const testSetters = new Set()

export function useMeasureStatus(ros) {
  const topicRef = useRef(null)
  const [measure, setMeasure] = useState(
    TEST_MODE ? (window.__thTestMeasureStatus ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    testSetters.add(setMeasure)
    window.__thSetTestMeasureStatus = (v) => {
      testSetters.forEach((fn) => fn(v))
    }
    return () => {
      testSetters.delete(setMeasure)
      if (testSetters.size === 0) delete window.__thSetTestMeasureStatus
    }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.MEASURE_STATUS,
      messageType: MSG_TYPES.MEASURE_STATUS,
      queue_length: 1,
    })
    topicRef.current.subscribe((msg) => { setMeasure(msg ?? null) })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return { measure }
}
