// ============================================================
// useVoice.js — voiceQueue の React 束ね
//
// キュー規則そのものは voiceQueue.js が持つ。ここがやるのは
//   ・2つのトグル state と localStorage 永続化
//   ・スナップショットの購読
//   ・自動再生ポリシーの解除と、その成否の可視化
// の3つだけ。
// ============================================================

import { useState, useEffect, useCallback } from 'react'

import { voiceQueue } from '../voice/voiceQueue.js'
import { getAudioState, unlockAudio } from '../voice/audioPlayer.js'
import { LAYER } from '../voice/announcements.js'

// 既定値は VISION.md §7.3 に従い、安全通知 ON・デモ実況 OFF。
// 実運用の既定がこれで、発表時のみ両方 ON にする。
const SAFETY_KEY = 'th_voice_safety'
const DEMO_KEY   = 'th_voice_demo'

const loadFlag = (key, dflt) => {
  const v = localStorage.getItem(key)
  return v === null ? dflt : v === '1'
}

export function useVoice() {
  const [safetyOn, setSafetyOn] = useState(() => loadFlag(SAFETY_KEY, true))
  const [demoOn,   setDemoOn]   = useState(() => loadFlag(DEMO_KEY, false))
  const [snapshot, setSnapshot] = useState(voiceQueue.getSnapshot)
  const [audioState, setAudioState] = useState(getAudioState)

  // ── キューへの反映と永続化 ──────────────────────────────
  useEffect(() => {
    voiceQueue.setLayerEnabled(LAYER.SAFETY, safetyOn)
    localStorage.setItem(SAFETY_KEY, safetyOn ? '1' : '0')
  }, [safetyOn])

  useEffect(() => {
    voiceQueue.setLayerEnabled(LAYER.DEMO, demoOn)
    localStorage.setItem(DEMO_KEY, demoOn ? '1' : '0')
  }, [demoOn])

  useEffect(() => voiceQueue.subscribe(setSnapshot), [])

  // ── 自動再生ポリシーの解除 ──────────────────────────────
  // 既定が「安全通知 ON」なので、初回ロード時点で必ず
  // 「有効なレイヤがあるがユーザー操作をまだ得ていない」状態になる。
  // トグル操作を待たず、画面のどこを最初に触っても解除されるようにする。
  useEffect(() => {
    if (audioState === 'running') return
    const onFirstTouch = () => { unlockAudio().then(() => setAudioState(getAudioState())) }
    window.addEventListener('pointerdown', onFirstTouch, { once: true, capture: true })
    return () => window.removeEventListener('pointerdown', onFirstTouch, { capture: true })
  }, [audioState])

  // 解除されるまでの間だけ状態を見張る。running になったら止まる
  useEffect(() => {
    if (audioState === 'running' || audioState === 'unavailable') return
    const id = setInterval(() => setAudioState(getAudioState()), 1000)
    return () => clearInterval(id)
  }, [audioState])

  const toggleLayer = useCallback((layer) => {
    // 解除はユーザー操作のハンドラ内で呼ぶ必要がある
    unlockAudio().then(() => setAudioState(getAudioState()))
    if (layer === LAYER.SAFETY) setSafetyOn((v) => !v)
    else                        setDemoOn((v) => !v)
  }, [])

  const announce     = useCallback((id) => voiceQueue.announce(id), [])
  const setCondition = useCallback((id, active) => voiceQueue.setCondition(id, active), [])
  const stopAll      = useCallback((opts) => voiceQueue.stopAll(opts), [])

  return {
    safetyOn, demoOn, toggleLayer,
    audioState,
    audioReady: audioState === 'running',
    snapshot,
    announce, setCondition, stopAll,
    isConditionActive: voiceQueue.isConditionActive,
  }
}
