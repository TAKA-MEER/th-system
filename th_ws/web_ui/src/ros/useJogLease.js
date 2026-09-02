// ros/useJogLease.js — the ONLY place the drive-tab / W-6 manual panel
// publishes anything (DetailedDesign-wp3.md WP-UI-03 §4.2 / §5, and
// DetailedDesign-webui.md §5).
//
// It takes `held` (is the operator touching a stick right now) and
// `cmd = { vx, wz }` (the normalized command already scaled by the chosen
// speed preset ratio; see parts/JogConsole.jsx) and runs two independent
// timers:
//
//   /ui/jog_lease      -> 5 Hz+, best_effort, data = this client's id
//   /cmd_vel_manual_raw -> 10 Hz, reliable, geometry_msgs/Twist
//
// Two timers on purpose (DetailedDesign-wp3.md §6.3 FMEA ①): if the lease
// keeps flowing while the UI freezes, the stick's own pointer events stop
// and /cmd_vel_manual_raw goes silent, letting twist_mux's manual_joy
// timeout cut the drive. Keeping them separate means a stall on one path
// fails into a stop instead of a held command.
//
// On release (held true -> false) it sends a single zero and stops both
// timers. It never sends a `ui.jog.release` trigger -- there is no such
// trigger; the lease simply stops flowing and jog_gate's lease expiry does
// the rest (DetailedDesign-wp3.md §3.1 / webui §5).
//
// Mirrors ros/useActiveScreenPublisher.js's TEST_MODE: under Playwright
// there is no rosbridge, so nothing is published for real; instead every
// send is recorded on window.__thJogPublishes for e2e to count.
import { useEffect, useRef } from 'react'
import { rampToward } from '../parts/stickGeometry.js'
import { TOPICS, MSG_TYPES } from './topics'
import { getClientId } from './clientId'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

// Rate ceilings from DetailedDesign-wp3.md §3.1: lease >= 5 Hz
// (jog_lease_ms is 1200ms, so 200ms is comfortably above it), velocity 10 Hz.
const LEASE_MS = 200
const CMD_MS = 100

// ジョグの加速度制限（2026-09-02 に復活させた）。
// 旧 App.jsx が JOG_LIN_ACCEL=1.0 / JOG_ANG_ACCEL=4.0 を持っており、
// 「無いとスティック/キー入力の変化が瞬時にそのまま速度指令へ反映され機体が
// 激しく揺れる」という実機検証のコメントが残っていた。新 UI へ移す際に
// rampToward だけが stickGeometry.js へ移植され、**呼び出しが失われていた**
// （本番から一度も呼ばれていなかった）。急発進は車輪を滑らせ、/odom は車輪速度
// フィードバック由来なので教示精度そのものを損なう。
const JOG_LIN_ACCEL = 1.0   // m/s^2
const JOG_ANG_ACCEL = 4.0   // rad/s^2

export function useJogLease(ros, held, cmd, enabled = true) {
  const leaseTopicRef = useRef(null)
  const cmdTopicRef = useRef(null)
  const leaseTimerRef = useRef(null)
  const cmdTimerRef = useRef(null)
  // Whether we believe the operator is currently holding. Mirrors `held` but
  // only latches on an accepted transition, so the release branch below can
  // tell "just released" apart from "still released".
  const heldRef = useRef(false)
  // Latest cmd read from inside the interval, so a re-render between ticks
  // never emits a stale value (U3-4: don't close over a stale value).
  const cmdRef = useRef({ vx: 0, wz: 0 })
  const enabledRef = useRef(enabled)

  enabledRef.current = enabled
  cmdRef.current = cmd

  const record = useRef(null)
  if (record.current === null) record.current = (entry) => {
    if (!TEST_MODE) return
    window.__thJogPublishes = window.__thJogPublishes ?? []
    window.__thJogPublishes.push(entry)
  }

  // Hold ROSLIB.Topic handles across re-renders (same pattern as
  // ros/useActiveScreenPublisher.js). In TEST_MODE no Topic is built.
  useEffect(() => {
    if (TEST_MODE || !ros) { leaseTopicRef.current = null; cmdTopicRef.current = null; return }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return
    leaseTopicRef.current = new ROSLIB.Topic({
      ros, name: TOPICS.JOG_LEASE, messageType: MSG_TYPES.STRING,
    })
    cmdTopicRef.current = new ROSLIB.Topic({
      ros, name: TOPICS.CMD_VEL_MANUAL_RAW, messageType: MSG_TYPES.TWIST,
    })
    return () => { leaseTopicRef.current = null; cmdTopicRef.current = null }
  }, [ros])

  // Stop the interval + send a single zero. Used by release, hide, and
  // unmount -- idempotent so repeated calls only ever emit the zero once
  // (guarded by heldRef / publishCountRef... actually the zero is gated by
  // heldRef in the caller; this helper only clears timers).
  const stopTimers = () => {
    if (leaseTimerRef.current) clearInterval(leaseTimerRef.current)
    if (cmdTimerRef.current) clearInterval(cmdTimerRef.current)
    leaseTimerRef.current = null
    cmdTimerRef.current = null
  }

  const publishLease = () => {
    const clientId = getClientId()
    record.current({ topic: TOPICS.JOG_LEASE, data: clientId })
    if (!TEST_MODE && leaseTopicRef.current && window.ROSLIB) {
      leaseTopicRef.current.publish(new window.ROSLIB.Message({ data: clientId }))
    }
  }

  // 直近に実際に送った値。加速度制限の起点になる。
  const sentRef = useRef({ vx: 0, wz: 0 })

  // 引数なし = 定期送信。目標へ加速度制限つきで近づける。
  // 引数あり = 解放・非表示・アンマウント時のゼロなど「今すぐこの値」を意味する
  // 呼び出しなので、**ランプを通さない**。ここを滑らかにすると手を離しても
  // 機体が進み続けることになり、安全上の後退になる。
  const publishCmd = (vx, wz) => {
    let outVx
    let outWz
    if (vx === undefined) {
      outVx = rampToward(sentRef.current.vx, cmdRef.current.vx,
                         JOG_LIN_ACCEL * (CMD_MS / 1000))
      outWz = rampToward(sentRef.current.wz, cmdRef.current.wz,
                         JOG_ANG_ACCEL * (CMD_MS / 1000))
    } else {
      outVx = vx
      outWz = wz
    }
    sentRef.current = { vx: outVx, wz: outWz }
    record.current({ topic: TOPICS.CMD_VEL_MANUAL_RAW, cmd: { vx: outVx, wz: outWz } })
    if (!TEST_MODE && cmdTopicRef.current && window.ROSLIB) {
      cmdTopicRef.current.publish(new window.ROSLIB.Message({
        linear: { x: outVx, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: outWz },
      }))
    }
  }

  // Tab goes to background: someone (or something) else now owns the screen.
  // We must not keep driving while hidden, so treat it exactly like a release
  // -- stop the timers and send one zero (DetailedDesign-wp3.md §6.2 / webui
  // §5 handle the jog_gate side as a normal release). On return to visible
  // the operator has to touch again; we do not auto-resume a stale hold.
  const applyVisibility = useRef(null)
  if (applyVisibility.current === null) {
    applyVisibility.current = () => {
      if (typeof document === 'undefined') return
      if (document.visibilityState !== 'visible' && heldRef.current) {
        heldRef.current = false
        stopTimers()
        publishCmd(0, 0)
      }
    }
  }
  useEffect(() => {
    if (!TEST_MODE && typeof document !== 'undefined') {
      document.addEventListener('visibilitychange', applyVisibility.current)
      return () => document.removeEventListener('visibilitychange', applyVisibility.current)
    }
    return undefined
  }, [])

  // Belt-and-suspenders for U3-1 (DetailedDesign-wp3.md §4.3): the drive-tab
  // / W-6 console also calls its own onRelease on unmount, but if a screen
  // unmounts that hook's state is gone before that callback can flip `held`
  // back. Sending the zero here too makes the unmount path fail-safe
  // regardless of who unmounts.
  useEffect(() => () => {
    if (heldRef.current) {
      heldRef.current = false
      stopTimers()
      publishCmd(0, 0)
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  useEffect(() => {
    if (held && enabledRef.current) {
      if (heldRef.current === false) {
        // fresh hold: kick both timers immediately, then on interval
        heldRef.current = true
        publishLease()
        publishCmd()
        leaseTimerRef.current = setInterval(publishLease, LEASE_MS)
        cmdTimerRef.current = setInterval(() => publishCmd(), CMD_MS)
      }
      return
    }
    if (heldRef.current) {
      // was held, now released (or gated off / hidden) -> stop + single zero
      heldRef.current = false
      stopTimers()
      publishCmd(0, 0)
    }
  }, [held, cmd, enabled])
}
