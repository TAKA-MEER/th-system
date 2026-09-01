// ros/useOdomPose.js — subscribes to /odom (nav_msgs/Odometry) for the S-13/S-14
// route-preview robot marker (WS-3 / demo-teach-replay).
//
// The hook only hands back the latest pose as { x, y, yaw } (null until a
// message arrives). It owns its own rosbridge Topic like ros/useLimiterStatus.js,
// and in TEST_MODE reads window.__thTestOdomPose (seed) + installs
// window.__thSetTestOdomPose (mutate) for e2e.
//
// /odom is subscribed under its raw topic name rather than a TOPICS constant
// (useRosbridge.js already subscribes /scan_filtered the same way); the
// topic name lives here without going through names.json's dictionary gate.
import { useEffect, useRef, useState } from 'react'
import { quatToYaw } from '../mapGeometry.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const ODOM_TOPIC = '/odom'
const ODOM_MSG = 'nav_msgs/Odometry'

export function useOdomPose(ros) {
  const topicRef = useRef(null)
  const [pose, setPose] = useState(
    TEST_MODE ? (window.__thTestOdomPose ?? null) : null)

  // TEST_MODE: let e2e mutate the seeded pose after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestOdomPose = (v) => setPose(v)
    return () => { delete window.__thSetTestOdomPose }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: ODOM_TOPIC,
      messageType: ODOM_MSG,
      subscribeOptions: { queueSize: 1, throttle_rate: 0, latching: false },
    })
    topicRef.current.subscribe((odom) => {
      if (!odom?.pose?.pose) return
      const p = odom.pose.pose.position
      const q = odom.pose.pose.orientation
      setPose({ x: p.x, y: p.y, yaw: quatToYaw(q) })
    })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return pose
}