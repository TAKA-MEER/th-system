// ============================================================
// useVoice.js — voiceQueue の React 束ね
//
// キュー規則そのものは voiceQueue.js が持つ。ここがやるのは
//   ・2つのレイヤ トグル state + 話者選択 state と localStorage 永続化
//   ・スナップショットの購読
//   ・自動再生ポリシーの解除と、その成否の可視化
// の3つだけ。
// ============================================================

import { useState, useEffect, useCallback } from 'react'

import { voiceQueue } from '../voice/voiceQueue.js'
import { getAudioState, unlockAudio, setSpeaker as setAudioSpeaker } from '../voice/audioPlayer.js'
import { LAYER } from '../voice/announcements.js'

// 既定値は VISION.md §7.3 に従い、安全通知 ON・デモ実況 OFF。
// 実運用の既定がこれで、発表時のみ両方 ON にする。
const SAFETY_KEY = 'th_voice_safety'
const DEMO_KEY   = 'th_voice_demo'

// 話者。既定は nemo (通常運用)。zundamon は展示専用の例外採用
// (docs/voice-credits.md「展示専用の例外: ずんだもん」参照)。端末ごとに永続化する
const SPEAKER_KEY = 'th_voice_speaker'

const loadFlag = (key, dflt) => {
  const v = localStorage.getItem(key)
  return v === null ? dflt : v === '1'
}

export function useVoice() {
  const [safetyOn, setSafetyOn] = useState(() => loadFlag(SAFETY_KEY, true))
  const [demoOn,   setDemoOn]   = useState(() => loadFlag(DEMO_KEY, false))
  const [speaker,  setSpeakerState] = useState(() => localStorage.getItem(SPEAKER_KEY) || 'nemo')
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

  // audioPlayer はモジュールスコープの現在話者しか持たないので、初回マウント時にも
  // 反映する (localStorage から復元した値が既定の 'nemo' と異なる場合に必要)
  useEffect(() => {
    setAudioSpeaker(speaker)
    localStorage.setItem(SPEAKER_KEY, speaker)
  }, [speaker])

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

  const setSpeaker = useCallback((id) => {
    // toggleLayer と同じく、ボタン押下という実ユーザー操作の中で呼ばれるため解除を兼ねる
    unlockAudio().then(() => setAudioState(getAudioState()))
    setSpeakerState(id)
  }, [])

  // overrides ({ clips }) を転送する。これが無いと数値の動的差し替え
  // (VISION.md §7.5。N4 が該当) が voiceQueue まで届かず、常に静的な
  // <ID>.mp3 にフォールバックしたままになる (2026-08-07 発覚・修正)
  const announce     = useCallback((id, overrides) => voiceQueue.announce(id, overrides), [])
  const setCondition = useCallback((id, active) => voiceQueue.setCondition(id, active), [])
  const stopAll      = useCallback((opts) => voiceQueue.stopAll(opts), [])

  return {
    safetyOn, demoOn, toggleLayer,
    speaker, setSpeaker,
    audioState,
    audioReady: audioState === 'running',
    snapshot,
    announce, setCondition, stopAll,
    isConditionActive: voiceQueue.isConditionActive,
  }
}
