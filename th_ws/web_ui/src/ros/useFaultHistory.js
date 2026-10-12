// ros/useFaultHistory.js — /system/fault_history (th_system_msgs/FaultHistory)
// を購読して、記録の配列（新しい順・最大 50 件）を返す。無い間は空配列。
//
// 機体の発行側は RELIABLE＋TRANSIENT_LOCAL・depth 1（fault_recorder）。最新の 1 通に
// 全履歴が入っているので、後から開いた画面（S-50 のタブなど）にも、購読した
// 時点でその 1 通が届く。購読の組み立ては faultHistoryState.subscribeFaultHistory
// （unit 試験で縛る）。AppShell と S-50 の両方が使うので、TEST_MODE では
// 生きている全インスタンスへ同報し、後から開いたインスタンスには最後の値を渡す。
//
// TEST_MODE は useDevMode.js と同じ流儀: window.__thTestFaultHistory（メッセージ
// {records:[...]}）を種にし、window.__thSetTestFaultHistory(msg) で「/system/fault_history
// の 1 通が届いた」を再現する。
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'
import { parseFaultHistory, subscribeFaultHistory } from './faultHistoryState.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const testSetters = new Set()
let testLatest = null

export function useFaultHistory(ros) {
  const topicRef = useRef(null)
  const [records, setRecords] = useState(() => {
    if (!TEST_MODE) return []
    return parseFaultHistory(testLatest ?? window.__thTestFaultHistory ?? null) ?? []
  })

  useEffect(() => {
    if (!TEST_MODE) return undefined
    testSetters.add(setRecords)
    window.__thSetTestFaultHistory = (msg) => {
      testLatest = msg
      const next = parseFaultHistory(msg) ?? []
      testSetters.forEach((fn) => fn(next))
    }
    return () => {
      testSetters.delete(setRecords)
      if (testSetters.size === 0) { delete window.__thSetTestFaultHistory; testLatest = null }
    }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = subscribeFaultHistory(
      ROSLIB, ros, TOPICS.FAULT_HISTORY, MSG_TYPES.FAULT_HISTORY,
      (msg) => { const next = parseFaultHistory(msg); if (next) setRecords(next) })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return records
}
