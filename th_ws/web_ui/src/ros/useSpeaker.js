// ros/useSpeaker.js — 機体スピーカー（Spec-safety §11）の WebUI 接続。
//
// 読みは /speaker/cue と /speaker/status の購読（どちらも std_msgs/String の
// JSON・transient_local。後から開いても最新が届く）。
// 書きは /speaker_announcer/set_parameters への直接呼び出し
// (rcl_interfaces/SetParameters。`ros2 param set` と同じ標準サービス)。
// /config_manager/set_tunable_params 経由ではない — speaker は実行中に変える値で、
// config_manager は停止中しか受け付けない（useDevMode.js と同じ理由）。
//
// TEST_MODE (window.__thTestState 定義時) は useDevMode.js と同じ流儀:
// 購読も送信もせず、window.__thTestSpeaker を種にし、
// window.__thSetTestSpeaker で表示を更新、window.__thSpeakerParamCalls に送信を記録。
import { useEffect, useRef, useState } from 'react'
import { TOPICS } from './topics'
import {
  SPEAKER_NODE, SPEAKER_PARAM_ENABLED, SPEAKER_PARAM_VOLUME, SPEAKER_PARAM_TEST,
  parseSpeakerCue, parseSpeakerStatus, setSpeakerParamRequest, subscribeSpeaker,
} from './speakerState.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

// TEST_MODE の同報先（useDevMode.js と同じ狙い）。本番では使わない。
const testSetters = new Set()

const NO_BACKEND = () => Promise.reject(new Error('rosbridge not connected'))

export function useSpeaker(ros) {
  const unsubRef = useRef(null)
  const [cue, setCue] = useState(
    TEST_MODE ? parseSpeakerCue(window.__thTestSpeaker?.cue ?? null) : null)
  const [status, setStatus] = useState(
    TEST_MODE ? parseSpeakerStatus(window.__thTestSpeaker?.status ?? null) : null)

  // TEST_MODE: e2e が表示を更新する。生きている全インスタンスへ同報する。
  useEffect(() => {
    if (!TEST_MODE) return undefined
    const setter = (v) => {
      setCue(parseSpeakerCue(v?.cue ?? null))
      setStatus(parseSpeakerStatus(v?.status ?? null))
    }
    testSetters.add(setter)
    window.__thSetTestSpeaker = (v) => testSetters.forEach((fn) => fn(v))
    return () => {
      testSetters.delete(setter)
      if (testSetters.size === 0) delete window.__thSetTestSpeaker
    }
  }, [])

  // 本番: /speaker/cue と /speaker/status を購読する。
  useEffect(() => {
    if (TEST_MODE || !ros) { unsubRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    unsubRef.current = subscribeSpeaker(
      ROSLIB, ros, TOPICS.SPEAKER_CUE, TOPICS.SPEAKER_STATUS,
      (raw) => { const next = parseSpeakerCue(raw); if (next) setCue(next) },
      (raw) => { const next = parseSpeakerStatus(raw); if (next) setStatus(next) },
    )
    return () => {
      unsubRef.current?.()
      unsubRef.current = null
    }
  }, [ros])

  const setSpeakerParam = (name, value, node = SPEAKER_NODE) => {
    const isBool = name !== SPEAKER_PARAM_VOLUME
    if (TEST_MODE) {
      window.__thSpeakerParamCalls = window.__thSpeakerParamCalls ?? []
      window.__thSpeakerParamCalls.push({ node, name, value })
      // 見た目の即時反映（本番は次の 2Hz 配信で返ってくる）。
      if (name === SPEAKER_PARAM_ENABLED) {
        setCue((prev) => (prev ? { ...prev, enabled: !!value } : prev))
      } else if (name === SPEAKER_PARAM_VOLUME) {
        setCue((prev) => (prev ? { ...prev, volume: value } : prev))
      }
      return Promise.resolve({ successful: true })
    }
    if (!ros) return NO_BACKEND()
    return setSpeakerParamRequest({ ROSLIB: window.ROSLIB, ros, node, name, value, isBool })
  }

  const setEnabled = (value) => setSpeakerParam(SPEAKER_PARAM_ENABLED, !!value)
  const setVolume = (value) => setSpeakerParam(SPEAKER_PARAM_VOLUME, value)
  const testChime = () => setSpeakerParam(SPEAKER_PARAM_TEST, true)

  return { cue, status, setEnabled, setVolume, testChime }
}
