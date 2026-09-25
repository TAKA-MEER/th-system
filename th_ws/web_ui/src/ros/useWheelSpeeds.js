// ros/useWheelSpeeds.js — subscribes to /esp32/wheel_cmd_speed (command) and
// /esp32/wheel_feedback (measured), both th_system_msgs/WheelFeedback, for
// S-30's MOTOR item (WP-UI-08). Keeps the last WHEEL_HISTORY_SEC seconds of
// each as a ring buffer, same shape parts/WheelSpeedView.jsx expects
// ({ measured: [{t,left,right}], command: [{t,left,right}] }).
//
// This does NOT reuse ros/useRosbridge.js, which already subscribes both
// topics into the same shape but owns its own, separate rosbridge
// connection and isn't wired into any current screen (main.jsx no longer
// mounts anything that calls useRosbridge -- see this packet's completion
// report). Following the ros/useLimiterStatus.js / ros/useOdomPose.js
// pattern instead: take the shared `ros` from useSystemState() and own only
// a lightweight Topic subscription.
//
// TEST_MODE mirrors those hooks: window.__thTestWheelSpeeds seeds the
// initial value (same { measured, command } shape), and
// window.__thSetTestWheelSpeeds mutates it after mount.
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const WHEEL_HISTORY_SEC = 15
const EMPTY = { measured: [], command: [] }

export function useWheelSpeeds(ros) {
  const measuredRef = useRef([])
  const commandRef = useRef([])
  const subFeedbackRef = useRef(null)
  const subCmdRef = useRef(null)
  const [data, setData] = useState(
    TEST_MODE ? (window.__thTestWheelSpeeds ?? EMPTY) : EMPTY)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestWheelSpeeds = (v) => setData(v)
    return () => { delete window.__thSetTestWheelSpeeds }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { subFeedbackRef.current = null; subCmdRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined

    const push = (bufRef, msg, key) => {
      const t = Date.now() / 1000
      const buf = bufRef.current
      buf.push({ t, left: msg.left_speed, right: msg.right_speed })
      const cutoff = t - WHEEL_HISTORY_SEC
      while (buf.length && buf[0].t < cutoff) buf.shift()
      setData((prev) => ({ ...prev, [key]: buf.slice() }))
    }

    subFeedbackRef.current = new ROSLIB.Topic({
      ros, name: TOPICS.WHEEL_FEEDBACK, messageType: MSG_TYPES.WHEEL_FEEDBACK,
    })
    subFeedbackRef.current.subscribe((msg) => push(measuredRef, msg, 'measured'))

    subCmdRef.current = new ROSLIB.Topic({
      ros, name: TOPICS.WHEEL_CMD_SPEED, messageType: MSG_TYPES.WHEEL_FEEDBACK,
    })
    subCmdRef.current.subscribe((msg) => push(commandRef, msg, 'command'))

    return () => {
      subFeedbackRef.current?.unsubscribe()
      subCmdRef.current?.unsubscribe()
      subFeedbackRef.current = null
      subCmdRef.current = null
      measuredRef.current = []
      commandRef.current = []
    }
  }, [ros])

  return data
}
