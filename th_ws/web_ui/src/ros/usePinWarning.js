// ros/usePinWarning.js — subscribes to /onsite/pin_warning
// (th_system_msgs/PinWarning) for the S-20 2点指示ウィザード (WS-9AB).
// pin_registrar publishes this latched (transient_local): a warning that the
// pin the operator just tried to place is too close to a wall / inflation
// band, so Nav2 may not be able to route to it. active:false = no warning.
//
// Mirrors ros/useHomeDeclared.js: topic name is a raw constant because
// /onsite/pin_warning is not in names.json (the WebUI name dictionary). In
// TEST_MODE reads window.__thTestPinWarning (seed) + installs
// window.__thSetTestPinWarning (mutate) for e2e.
import { useEffect, useRef, useState } from 'react'
import { MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const PIN_WARNING_TOPIC = '/onsite/pin_warning'

const INACTIVE = { active: false }

export function usePinWarning(ros) {
  const topicRef = useRef(null)
  const [warn, setWarn] = useState(
    TEST_MODE ? (window.__thTestPinWarning ?? INACTIVE) : INACTIVE)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestPinWarning = (v) => setWarn(v ?? INACTIVE)
    return () => { delete window.__thSetTestPinWarning }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: PIN_WARNING_TOPIC,
      messageType: MSG_TYPES.PIN_WARNING,
      queue_length: 1,
    })
    topicRef.current.subscribe((msg) => setWarn(msg ?? INACTIVE))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return warn
}
