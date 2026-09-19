// ros/usePersonStatus.js — subscribes to /person/status (th_system_msgs/PersonStatus)
// for the on-site map person marker (MAP-1).
//
// /person/status carries base_link-relative position, confidence, and lost state
// of the currently tracked person. S-20/S-21 did not subscribe to this topic
// (only /person/targets for the radar select).
//
// TEST_MODE: reads window.__thTestPersonStatus (seed) +
// installs window.__thSetTestPersonStatus (mutate) for e2e.
import { useEffect, useRef, useState } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const DEFAULT_PERSON_STATUS = {
  position: { x: 0, y: 0, z: 0 },
  confidence: 0,
  is_lost: true,
  lost_reason: '',
}

const PERSON_STATUS_TOPIC = '/person/status'
const PERSON_STATUS_MSG = 'th_system_msgs/PersonStatus'

export function usePersonStatus(ros) {
  const topicRef = useRef(null)
  const [personStatus, setPersonStatus] = useState(
    TEST_MODE ? (window.__thTestPersonStatus ?? DEFAULT_PERSON_STATUS) : DEFAULT_PERSON_STATUS,
  )

  // TEST_MODE: let e2e mutate the seeded status after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestPersonStatus = (v) => setPersonStatus(v)
    return () => { delete window.__thSetTestPersonStatus }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: PERSON_STATUS_TOPIC,
      messageType: PERSON_STATUS_MSG,
      queue_length: 1,
    })
    topicRef.current.subscribe((msg) => setPersonStatus(msg ?? DEFAULT_PERSON_STATUS))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return personStatus
}
