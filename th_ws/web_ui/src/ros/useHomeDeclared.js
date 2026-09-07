// ros/useHomeDeclared.js — subscribes to /onsite/home_declared
// (std_msgs/Bool) for the S-21 行き先 tab (WP-UI-07). home_declarer publishes
// this latched (transient_local); the value is the "is the current spot already
// the declared home spot" flag.
//
// Mirrors ros/useOdomPose.js: the topic name lives here as a raw constant
// because /onsite/home_declared is exported by home_declarer but NOT in
// names.json (the WebUI name dictionary), and ros/topics.js TOPICS entries are
// gated by topics-in-dictionary.test.js. In TEST_MODE reads
// window.__thTestHomeDeclared (seed) + installs window.__thSetTestHomeDeclared
// (mutate) for e2e.
import { useEffect, useRef, useState } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const HOME_DECLARED_TOPIC = '/onsite/home_declared'
const HOME_DECLARED_MSG = 'std_msgs/Bool'

export function useHomeDeclared(ros) {
  const topicRef = useRef(null)
  const [declared, setDeclared] = useState(
    TEST_MODE ? (window.__thTestHomeDeclared ?? false) : false)

  // TEST_MODE: let e2e mutate the seeded value after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestHomeDeclared = (v) => setDeclared(!!v)
    return () => { delete window.__thSetTestHomeDeclared }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: HOME_DECLARED_TOPIC,
      messageType: HOME_DECLARED_MSG,
      queue_length: 1,   // latched なので最新 1 件で十分
    })
    topicRef.current.subscribe((msg) => setDeclared(!!msg?.data))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return declared
}