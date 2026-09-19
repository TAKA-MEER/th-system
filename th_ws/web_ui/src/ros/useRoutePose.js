// ros/useRoutePose.js — subscribes to /route/robot_pose (geometry_msgs/PoseStamped)
// for the S-13/S-14 route-preview robot marker in the map frame (WS-8B).
//
// Mirrors ros/useOdomPose.js: owns its own rosbridge Topic, and in TEST_MODE
// reads window.__thTestRoutePose (seed) + installs window.__thSetTestRoutePose
// (mutate) for e2e. Hands back the latest pose as { x, y, yaw, frame } (null
// until a message arrives); `frame` is header.frame_id ('map' or 'odom') so the
// route preview can tell whether the map and the pose share a frame. The pose is
// in the map frame when a map is available, otherwise odom.
import { useEffect, useRef, useState } from 'react'
import { quatToYaw } from '../mapGeometry.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const ROUTE_POSE_TOPIC = '/route/robot_pose'
const ROUTE_POSE_MSG = 'geometry_msgs/PoseStamped'

export function useRoutePose(ros) {
  const topicRef = useRef(null)
  const [pose, setPose] = useState(
    TEST_MODE ? (window.__thTestRoutePose ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestRoutePose = (v) => setPose(v)
    return () => { delete window.__thSetTestRoutePose }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: ROUTE_POSE_TOPIC,
      messageType: ROUTE_POSE_MSG,
      queue_length: 1,   // 最新のみ。WiFi が詰まった後に古い pose が連続配信されると位置が飛ぶ
    })
    topicRef.current.subscribe((msg) => {
      if (!msg?.pose) return
      const p = msg.pose.position
      const q = msg.pose.orientation
      setPose({ x: p.x, y: p.y, yaw: quatToYaw(q), frame: msg.header?.frame_id })
    })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return pose
}