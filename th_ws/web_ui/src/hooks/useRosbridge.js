// ============================================================
// useRosbridge.js — rosbridge WebSocket 接続フック
// ============================================================
import { useState, useEffect, useRef, useCallback } from 'react'

const MODE_NAMES = {
  0: 'INIT', 1: 'IDLE', 2: 'FOLLOWING',
  3: 'MOVING_TO_PANEL', 4: 'AT_PANEL', 5: 'MANUAL', 6: 'ESTOP'
}

export function useRosbridge(url = 'ws://192.168.137.1:9090') {
  const rosRef  = useRef(null)
  const [connected, setConnected]   = useState(false)
  const [mode, setMode]             = useState(null)
  const [fault, setFault]           = useState({ active: false, fault_type: 'NONE' })
  const [estop, setEstop]           = useState(false)
  const [battVolt, setBattVolt]     = useState(null)

  // ── 接続 ──────────────────────────────────────────────────
  useEffect(() => {
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) { console.error('roslibjs が読み込まれていません'); return }

    const ros = new ROSLIB.Ros({ url })
    rosRef.current = ros

    ros.on('connection', () => {
      console.log('rosbridge 接続')
      setConnected(true)
    })
    ros.on('error',      (e) => { console.error('rosbridge エラー', e); setConnected(false) })
    ros.on('close',      ()  => { console.warn('rosbridge 切断'); setConnected(false) })

    // ── 購読: /robot/mode ──────────────────────────────────
    const subMode = new ROSLIB.Topic({
      ros, name: '/robot/mode',
      messageType: 'th_system_msgs/RobotMode'
    })
    subMode.subscribe((msg) => setMode(msg.mode))

    // ── 購読: /safety/fault ────────────────────────────────
    const subFault = new ROSLIB.Topic({
      ros, name: '/safety/fault',
      messageType: 'th_system_msgs/FaultStatus'
    })
    subFault.subscribe((msg) => setFault(msg))

    // ── 購読: /safety/estop ────────────────────────────────
    const subEstop = new ROSLIB.Topic({
      ros, name: '/safety/estop',
      messageType: 'std_msgs/Bool'
    })
    subEstop.subscribe((msg) => setEstop(msg.data))

    return () => {
      subMode.unsubscribe()
      subFault.unsubscribe()
      subEstop.unsubscribe()
      ros.close()
    }
  }, [url])

  // ── モード変更サービス呼び出し ────────────────────────────
  const requestMode = useCallback((modeNum) => {
    const ROSLIB = window.ROSLIB
    if (!rosRef.current || !ROSLIB) return
    const svc = new ROSLIB.Service({
      ros: rosRef.current,
      name: '/mode_manager/set_mode',
      serviceType: 'th_system_msgs/SetMode'
    })
    svc.callService(
      new ROSLIB.ServiceRequest({ requested_mode: modeNum, requester: 'tablet_ui' }),
      (res) => { if (!res.success) console.warn('モード変更失敗:', res.message) }
    )
  }, [])

  // ── タブレット緊急停止 ────────────────────────────────────
  const tabletEstopRef = useRef(null)
  useEffect(() => {
    if (!rosRef.current || !connected) return
    const ROSLIB = window.ROSLIB
    tabletEstopRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/safety/tablet_estop',
      messageType: 'std_msgs/Bool'
    })
  }, [connected])

  const publishTabletEstop = useCallback((active) => {
    tabletEstopRef.current?.publish(new window.ROSLIB.Message({ data: active }))
  }, [])

  // ── heartbeat 定期送信 ────────────────────────────────────
  useEffect(() => {
    if (!connected) return
    const ROSLIB = window.ROSLIB
    const hbTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/manual/heartbeat',
      messageType: 'std_msgs/Empty'
    })
    const id = setInterval(() => {
      hbTopic.publish(new ROSLIB.Message({}))
    }, 500)   // 500ms = 2Hz
    return () => clearInterval(id)
  }, [connected])

  // ── 手動ゴール送信 ────────────────────────────────────────
  const sendManualGoal = useCallback((x, y) => {
    const ROSLIB = window.ROSLIB
    if (!rosRef.current || !ROSLIB) return
    const topic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/manual/target_pose',
      messageType: 'geometry_msgs/PoseStamped'
    })
    topic.publish(new ROSLIB.Message({
      header: { frame_id: 'base_link' },
      pose: {
        position:    { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 }
      }
    }))
  }, [])

  // ── 配電盤移動サービス ────────────────────────────────────
  const goToPanel = useCallback((panelId) => {
    const ROSLIB = window.ROSLIB
    if (!rosRef.current || !ROSLIB) return
    const svc = new ROSLIB.Service({
      ros: rosRef.current,
      name: '/panel_navigator/go_to_panel',
      serviceType: 'th_system_msgs/GoToPanel'
    })
    svc.callService(
      new ROSLIB.ServiceRequest({ panel_id: panelId }),
      (res) => console.log('GoToPanel:', res)
    )
  }, [])

  return {
    connected, mode, modeName: MODE_NAMES[mode] ?? '---',
    fault, estop,
    requestMode, publishTabletEstop, sendManualGoal, goToPanel,
  }
}
