// ros/useReplaySpeedPublisher.js — publishes the S-14 replay-speed ratio to
// /replay/speed_scale (std_msgs/Float32, latched) whenever it changes (WS-9T).
//
// replay_runner subscribes and rebuilds its pure-pursuit params from the ratio
// (route_replay_core.scale_replay_params), so a change takes effect on the next
// 20 Hz control tick -- even mid-run.
//
// The topic name is a module-local constant here rather than in ros/topics.js:
// ros/topics.js's header note sanctions this ("useRosbridge.js が /scan_filtered
// を直書きしている") and it keeps names.json (a generated file) untouched.
//
// Mirrors ros/useActiveScreenPublisher.js: under Playwright there is no
// rosbridge, so `ros` stays null and nothing is published; instead each send is
// recorded on window.__thReplaySpeedPublishes for e2e to inspect.
import { useEffect, useRef } from 'react'

const REPLAY_SPEED_TOPIC = '/replay/speed_scale'
const FLOAT32 = 'std_msgs/Float32'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useReplaySpeedPublisher(ros, ratio) {
  const topicRef = useRef(null)
  const lastSentRef = useRef(null)

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros, name: REPLAY_SPEED_TOPIC, messageType: FLOAT32, latch: true,
    })
    return () => { topicRef.current = null }
  }, [ros])

  useEffect(() => {
    if (typeof ratio !== 'number' || Number.isNaN(ratio)) return
    if (lastSentRef.current === ratio) return
    lastSentRef.current = ratio
    if (TEST_MODE) {
      window.__thReplaySpeedPublishes = window.__thReplaySpeedPublishes ?? []
      window.__thReplaySpeedPublishes.push(ratio)
      return
    }
    if (topicRef.current && window.ROSLIB) {
      topicRef.current.publish(new window.ROSLIB.Message({ data: ratio }))
    }
  }, [ros, ratio])
}
