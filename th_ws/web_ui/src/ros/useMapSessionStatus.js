// ros/useMapSessionStatus.js — /map_session/status を購読して最新の 1 通
// （{slot, mode, dirty, instance_id}）を返す。まだ届いていなければ null。
//
// 機体の発行側は RELIABLE＋TRANSIENT_LOCAL・depth 1（slam_control）。購読の組み立ては
// mapSessionState.subscribeMapSession（unit 試験で縛る）。画面ごとに作られては消える
// （手押し復帰・再読み込み）ので、TEST_MODE では最後の値をモジュールに残し、後から
// 開いたインスタンスにも渡す（TRANSIENT_LOCAL の再現）。
//
// TEST_MODE は useFaultHistory.js と同じ流儀: window.__thTestMapSession を種にし、
// window.__thSetTestMapSession(msg) で「/map_session/status の 1 通が届いた」を再現する。
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'
import { parseMapSession, subscribeMapSession } from './mapSessionState.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const testSetters = new Set()
let testLatest = null

export function useMapSessionStatus(ros) {
  const topicRef = useRef(null)
  const [status, setStatus] = useState(() => {
    if (!TEST_MODE) return null
    return parseMapSession(testLatest ?? window.__thTestMapSession ?? null)
  })

  useEffect(() => {
    if (!TEST_MODE) return undefined
    testSetters.add(setStatus)
    window.__thSetTestMapSession = (msg) => {
      testLatest = msg
      const next = parseMapSession(msg)
      testSetters.forEach((fn) => fn(next))
    }
    return () => {
      // 最後の値は残す（画面が作り直されても、購読し直した側へ最新の 1 通が届く
      // TRANSIENT_LOCAL の再現）。
      testSetters.delete(setStatus)
    }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = subscribeMapSession(
      ROSLIB, ros, TOPICS.MAP_SESSION_STATUS, MSG_TYPES.MAP_SESSION_STATUS,
      (msg) => { const next = parseMapSession(msg); if (next) setStatus(next) })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return status
}
