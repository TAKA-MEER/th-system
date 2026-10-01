// ros/useAutoCheckStatus.js — /opcheck/auto_status (std_msgs/String に JSON)
// を購読するフック (S-00 / S-01 の始業点検の自動判定表示用)。
//
// opcheck_auto.py が transient_local・1Hz で出すので、後から開いた画面でも
// 最新が届く。parse は ros/autoCheckState.js の純粋関数。
//
// TEST_MODE は ros/useDevMode.js と同じ流儀: window.__thTestAutoCheck を
// 種にし、window.__thSetTestAutoCheck で表示を更新する。
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'
import { parseAutoCheck } from './autoCheckState.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

// TEST_MODE の同報先（useDevMode.js と同じ形）。本番では使わない。
const testSetters = new Set()

export function useAutoCheckStatus(ros) {
  const topicRef = useRef(null)
  const [auto, setAuto] = useState(
    TEST_MODE ? parseAutoCheck(window.__thTestAutoCheck ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    testSetters.add(setAuto)
    window.__thSetTestAutoCheck = (v) => {
      const next = parseAutoCheck(v)
      testSetters.forEach((fn) => fn(next))
    }
    return () => {
      testSetters.delete(setAuto)
      if (testSetters.size === 0) delete window.__thSetTestAutoCheck
    }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.AUTO_CHECK,
      messageType: MSG_TYPES.STRING,
      queue_length: 1,
    })
    topicRef.current.subscribe((msg) => {
      const next = parseAutoCheck(msg?.data ?? null)
      if (next) setAuto(next)
    })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return { auto }
}
