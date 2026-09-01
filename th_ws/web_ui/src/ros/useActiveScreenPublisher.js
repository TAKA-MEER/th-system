// ros/useActiveScreenPublisher.js — publishes /ui/active_screen at 2Hz
// (DetailedDesign-wp1.md WP-UI-01 §3.1, th_system_msgs/ActiveScreen.msg).
//
// N-15: this topic had a constant in topics.js but nothing ever published
// it. DetailedDesign-names.md §4.1's derive_limits() treats zero interacting
// terminals as "link is down" and falls back to speed_limit=stop / zone=NA /
// auto_brake on -- so with no publisher, that fail-safe default was
// permanent, not just a link-loss reaction.
//
// interacting vs last_input (see the .msg's own comment):
// - interacting: is this tab the foreground tab right now
//   (document.visibilityState === 'visible'). Reset on every visibility
//   change, independent of whether the operator is actually touching
//   anything.
// - last_input: timestamp of the last real pointerdown/keydown/touchstart,
//   so a consumer (th_state) can tell a foregrounded-but-idle terminal from
//   one someone is actually driving. That windowing math (ui_active_window_s)
//   is a consumer concern, not the UI's -- this hook only reports the raw
//   timestamp.
// header.stamp is set fresh on every 2Hz tick; it is the publish time, not
// last_input.
//
// AppShell.jsx is never mounted for ?view=audience (main.jsx renders
// AudienceView instead, which owns its own read-only rosbridge connection
// -- see ros/useRosbridge.js's readOnly option). Since this hook only runs
// from inside AppShell, the audience view can never register as an
// interacting terminal.
import { useEffect, useRef } from 'react'
import { TOPICS, MSG_TYPES } from './topics'
import { getClientId } from './clientId'

const ACTIVE_SCREEN_HZ = 2

// Mirrors ros/useSystemState.js's TEST_MODE: under Playwright there is no
// rosbridge backend at all, so ros stays null forever and no ROSLIB.Topic
// is ever created. e2e/active-screen-published.spec.js instead counts
// attempts recorded on window.__thActiveScreenPublishes.
const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

function toRosTime(ms) {
  const sec = Math.floor(ms / 1000)
  const nanosec = Math.round((ms - sec * 1000) * 1e6)
  return { sec, nanosec }
}

export function useActiveScreenPublisher(ros, screenId) {
  const topicRef = useRef(null)
  const interactingRef = useRef(
    typeof document !== 'undefined' ? document.visibilityState === 'visible' : true)
  const lastInputMsRef = useRef(Date.now())
  const clientIdRef = useRef(null)
  if (clientIdRef.current === null) clientIdRef.current = getClientId()

  // Track foreground/background and real input independently of the
  // publish timer, so each 2Hz tick just reads the latest values.
  useEffect(() => {
    const onVisibility = () => { interactingRef.current = document.visibilityState === 'visible' }
    const onInput = () => { lastInputMsRef.current = Date.now() }
    document.addEventListener('visibilitychange', onVisibility)
    window.addEventListener('pointerdown', onInput)
    window.addEventListener('keydown', onInput)
    window.addEventListener('touchstart', onInput)
    return () => {
      document.removeEventListener('visibilitychange', onVisibility)
      window.removeEventListener('pointerdown', onInput)
      window.removeEventListener('keydown', onInput)
      window.removeEventListener('touchstart', onInput)
    }
  }, [])

  // Real mode: hold a rosbridge Topic handle across re-renders, same
  // pattern as AppShell.jsx's estop_ui publisher.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return
    topicRef.current = new ROSLIB.Topic({
      ros, name: TOPICS.ACTIVE_SCREEN, messageType: MSG_TYPES.ACTIVE_SCREEN,
    })
    return () => { topicRef.current = null }
  }, [ros])

  useEffect(() => {
    if (!screenId) return undefined

    const publishOnce = () => {
      const nowMs = Date.now()
      const msg = {
        header: { stamp: toRosTime(nowMs), frame_id: '' },
        screen_id: screenId,
        client_id: clientIdRef.current,
        interacting: interactingRef.current,
        last_input: toRosTime(lastInputMsRef.current),
      }
      if (TEST_MODE) {
        window.__thActiveScreenPublishes = window.__thActiveScreenPublishes ?? []
        window.__thActiveScreenPublishes.push(msg)
        return
      }
      if (!topicRef.current || !window.ROSLIB) return
      topicRef.current.publish(new window.ROSLIB.Message(msg))
    }

    publishOnce()
    const timer = setInterval(publishOnce, 1000 / ACTIVE_SCREEN_HZ)
    return () => clearInterval(timer)
  }, [screenId])
}
