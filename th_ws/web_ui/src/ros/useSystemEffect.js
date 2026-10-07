// ros/useSystemEffect.js — subscribes to /system/effect
// (th_system_msgs/StateEffect) for the shell's W-3 guide banner (1b-6, SG-B2).
//
// state_manager publishes every non-self effect here (reliable, depth 10,
// volatile). This hook owns its own rosbridge Topic and hands back the latest
// received message (or null). Dispatch (which name does what) lives in
// shell/effectDispatch.js; banner lifetime in shell/useGuideBanner.js.
//
// Mirrors ros/useWaitClearStatus.js. In TEST_MODE reads
// window.__thTestSystemEffect (seed) + installs window.__thSetTestSystemEffect
// (mutate) for e2e, so specs run with no socket.
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

// TEST_MODE で同じトピックの購読者が複数いても全員に届ける（1b-7: シェルが
// W-3（useGuideBanner）と W-4（useSaveConfirm）で 2 口購読する。後勝ちの単一
// setter だと後からの更新が片方にしか届かない）。実モードは各 hook が自分の
// Topic を持つので関係ない。
const _testSinks = new Set()

export function useSystemEffect(ros) {
  const topicRef = useRef(null)
  const [effect, setEffect] = useState(
    TEST_MODE ? (window.__thTestSystemEffect ?? null) : null)

  // TEST_MODE: let e2e push a new effect after mount (same pattern as
  // useSystemState.js's window.__thSetTestState).
  useEffect(() => {
    if (!TEST_MODE) return undefined
    const sink = (v) => setEffect(v ?? null)
    _testSinks.add(sink)
    window.__thSetTestSystemEffect = (v) => { _testSinks.forEach((fn) => fn(v)) }
    return () => { _testSinks.delete(sink) }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.SYSTEM_EFFECT,
      messageType: MSG_TYPES.STATE_EFFECT,
      queue_length: 10,
    })
    topicRef.current.subscribe((msg) => setEffect(msg ?? null))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return effect
}
