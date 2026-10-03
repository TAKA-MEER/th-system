// ros/useLinkStatus.js — /system/link_status を購読するフック（S-00 の機器別の行）。
//
// connectivity_checker が transient_local・1Hz で出す。LINK_STATUS_STALE_MS 以上
// 届かなければ null を返す（古い ✓ を出し続けない）。parse は linkStatusState.js。
//
// TEST_MODE は useAutoCheckStatus.js と同じ流儀: window.__thTestLinkStatus を種にし、
// window.__thSetTestLinkStatus で表示を更新する（TEST_MODE では途絶判定をしない）。
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'
import { parseLinkStatus, LINK_STATUS_STALE_MS } from './linkStatusState.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const testSetters = new Set()

export function useLinkStatus(ros) {
  const topicRef = useRef(null)
  const [link, setLink] = useState(
    TEST_MODE ? parseLinkStatus(window.__thTestLinkStatus ?? null) : null)
  const [receivedAt, setReceivedAt] = useState(0)
  const [now, setNow] = useState(() => Date.now())

  useEffect(() => {
    if (!TEST_MODE) return undefined
    testSetters.add(setLink)
    window.__thSetTestLinkStatus = (v) => {
      const next = parseLinkStatus(v)
      testSetters.forEach((fn) => fn(next))
    }
    return () => {
      testSetters.delete(setLink)
      if (testSetters.size === 0) delete window.__thSetTestLinkStatus
    }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.LINK_STATUS,
      messageType: MSG_TYPES.STRING,
      queue_length: 1,
    })
    topicRef.current.subscribe((msg) => {
      const next = parseLinkStatus(msg?.data ?? null)
      if (next) { setLink(next); setReceivedAt(Date.now()) }
    })
    const timer = setInterval(() => setNow(Date.now()), 1000)
    return () => {
      clearInterval(timer)
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  if (TEST_MODE) return { link }
  const fresh = link && now - receivedAt <= LINK_STATUS_STALE_MS
  return { link: fresh ? link : null }
}
