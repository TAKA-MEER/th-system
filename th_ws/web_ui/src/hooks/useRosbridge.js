// ============================================================
// useRosbridge.js — rosbridge WebSocket 接続フック
// ============================================================
import { useState, useEffect, useRef, useCallback } from 'react'

const MODE_NAMES = {
  0: 'INIT', 1: 'IDLE', 2: 'FOLLOWING',
  3: 'MOVING_TO_PANEL', 4: 'AT_PANEL', 5: 'MANUAL', 6: 'ESTOP',
  7: 'FOLLOWING_MAPLESS'
}

// 既定はページを配信しているホスト (ロボットPC) の rosbridge に接続する。
// 別ホストの rosbridge へ接続する場合は呼び出し側で url を指定する。
export function useRosbridge(url = `ws://${window.location.hostname}:9090`) {
  const rosRef  = useRef(null)
  const [connected, setConnected]   = useState(false)
  const [mode, setMode]             = useState(null)
  const [fault, setFault]           = useState({ active: false, fault_type: 'NONE' })
  const [estop, setEstop]           = useState(false)
  const [battVolt, setBattVolt]     = useState(null)
  const [personStatus, setPersonStatus] = useState(null)
  const [candidates, setCandidates]     = useState([])

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

    // ── 購読: /person/status ───────────────────────────────
    const subPerson = new ROSLIB.Topic({
      ros, name: '/person/status',
      messageType: 'th_system_msgs/PersonStatus'
    })
    subPerson.subscribe((msg) => setPersonStatus(msg))

    // ── 購読: 追従対象候補一覧 ──────────────────────────────
    const subCandidates = new ROSLIB.Topic({
      ros, name: '/sobits_follower/multiple_sensor_person_tracking/person_candidates',
      messageType: 'multiple_sensor_person_tracking/PersonCandidates'
    })
    subCandidates.subscribe((msg) => setCandidates(msg.positions ?? []))

    return () => {
      subMode.unsubscribe()
      subFault.unsubscribe()
      subEstop.unsubscribe()
      subPerson.unsubscribe()
      subCandidates.unsubscribe()
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

  // ── 手動ジョグ (直接速度指令) ──────────────────────────────
  // manual_command_handler の Nav2 ゴール方式は map フレームが必要で、
  // 地図なし運用では機能しない。twist_mux の /cmd_vel_manual (priority 30,
  // timeout 0.5s) へ直接 Twist を流す方式に変更。押している間だけ UI 側が
  // 定期 publish し、timeout で自動停止する。
  const manualCmdRef = useRef(null)
  useEffect(() => {
    if (!rosRef.current || !connected) return
    const ROSLIB = window.ROSLIB
    manualCmdRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/cmd_vel_manual',
      messageType: 'geometry_msgs/Twist'
    })
  }, [connected])

  const publishManualCmd = useCallback((vx, wz) => {
    const ROSLIB = window.ROSLIB
    if (!manualCmdRef.current || !ROSLIB) return
    manualCmdRef.current.publish(new ROSLIB.Message({
      linear:  { x: vx, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: wz }
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

  // ── 追従対象の選択・再登録 ─────────────────────────────────
  const selectTarget = useCallback((candidateIndex) => {
    const ROSLIB = window.ROSLIB
    if (!rosRef.current || !ROSLIB) return
    const svc = new ROSLIB.Service({
      ros: rosRef.current,
      name: '/person_tracker/select_target',
      serviceType: 'multiple_sensor_person_tracking/SelectTarget'
    })
    svc.callService(
      new ROSLIB.ServiceRequest({ candidate_index: candidateIndex, x: 0.0, y: 0.0 }),
      (res) => { if (!res.success) console.warn('ターゲット選択失敗:', res.message) }
    )
  }, [])

  const resetTracking = useCallback(() => {
    const ROSLIB = window.ROSLIB
    if (!rosRef.current || !ROSLIB) return
    const svc = new ROSLIB.Service({
      ros: rosRef.current,
      name: '/person_tracker/reset_tracking',
      serviceType: 'std_srvs/Trigger'
    })
    svc.callService(
      new ROSLIB.ServiceRequest({}),
      (res) => { if (!res.success) console.warn('追従リセット失敗:', res.message) }
    )
  }, [])

  return {
    connected, mode, modeName: MODE_NAMES[mode] ?? '---',
    fault, estop,
    personStatus, candidates, selectTarget, resetTracking,
    requestMode, publishTabletEstop, sendManualGoal, publishManualCmd, goToPanel,
  }
}
