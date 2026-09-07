// ros/useOnsitePins.js — subscribes to /onsite/pins (th_system_msgs/PinList)
// for the S-20 prep-map pin overlay and the Pins subtab (WP-UI-06).
//
// /onsite/pins is a transient_local publisher (recently registered pins +
// latched list on subscribe). Mirrors ros/useRouteCatalog.js: owns its own
// rosbridge Topic and only hands back the latest pins array (msg.pins); in
// TEST_MODE reads window.__thTestOnsitePins (seed) + installs
// window.__thSetTestOnsitePins (mutate) for e2e.
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useOnsitePins(ros) {
  const topicRef = useRef(null)
  const [pins, setPins] = useState(
    TEST_MODE ? (window.__thTestOnsitePins ?? []) : [])

  // TEST_MODE: let e2e mutate the seeded pins after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestOnsitePins = (v) => setPins(v)
    return () => { delete window.__thSetTestOnsitePins }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.ONSITE_PINS,
      messageType: MSG_TYPES.PIN_LIST,
      queue_length: 1,   // 最新のみ
    })
    topicRef.current.subscribe((msg) => setPins(msg?.pins ?? []))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return pins
}