// ros/useOdomTwist.js — subscribes to /odom (nav_msgs/Odometry) twist for the
// S-11 gauge (rotation) and guide line (Spec-webui.md §3.5).
//
// useOdomPose.js already hands back the pose from the same topic; the twist
// (linear.x / angular.z, base_link frame) is what the gauge and the predicted
// path need. It is read from the arriving topic, never invented: null until
// a message arrives. Mirrors the other ros/ hooks: owns its own Topic,
// TEST_MODE reads window.__thTestOdomTwist + installs
// window.__thSetTestOdomTwist for e2e.
import { useEffect, useRef, useState } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const ODOM_TOPIC = '/odom'
const ODOM_MSG = 'nav_msgs/Odometry'

export function useOdomTwist(ros) {
  const topicRef = useRef(null)
  const [twist, setTwist] = useState(
    TEST_MODE ? (window.__thTestOdomTwist ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestOdomTwist = (v) => setTwist(v)
    return () => { delete window.__thSetTestOdomTwist }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: ODOM_TOPIC,
      messageType: ODOM_MSG,
      queue_length: 1,
    })
    topicRef.current.subscribe((odom) => {
      const t = odom?.twist?.twist
      if (!t) return
      setTwist({
        vx: typeof t.linear?.x === 'number' ? t.linear.x : null,
        wz: typeof t.angular?.z === 'number' ? t.angular.z : null,
      })
    })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return twist
}
