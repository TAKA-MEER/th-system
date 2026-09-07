// ros/usePersonTargets.js — subscribes to /person/targets
// (th_system_msgs/PersonTargets) for the S-20 target-selection radar
// (WP-UI-06). candidate positions are base_link-frame (robot-relative m).
//
// Mirrors ros/useOnsitePins.js: owns its own rosbridge Topic and hands back
// the whole observed message { candidates, selected_index, confidence,
// is_lost, lost_reason } (defaults until a message arrives). In TEST_MODE
// reads window.__thTestPersonTargets (seed) + installs
// window.__thSetTestPersonTargets (mutate) for e2e.
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const DEFAULT_TARGETS = {
  candidates: [],
  selected_index: -1,
  confidence: 0,
  is_lost: false,
  lost_reason: '',
}

export function usePersonTargets(ros) {
  const topicRef = useRef(null)
  const [targets, setTargets] = useState(
    TEST_MODE ? ({ ...DEFAULT_TARGETS, ...(window.__thTestPersonTargets ?? {}) }) : DEFAULT_TARGETS)

  // TEST_MODE: let e2e mutate the seeded targets after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestPersonTargets = (v) => setTargets({ ...DEFAULT_TARGETS, ...(v ?? {}) })
    return () => { delete window.__thSetTestPersonTargets }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.PERSON_TARGETS,
      messageType: MSG_TYPES.PERSON_TARGETS,
      queue_length: 1,   // 最新のみ
    })
    topicRef.current.subscribe((msg) => setTargets(msg ?? DEFAULT_TARGETS))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return targets
}