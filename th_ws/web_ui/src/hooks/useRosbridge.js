// ============================================================
// useRosbridge.js — rosbridge WebSocket 接続フック
// ============================================================
import { useState, useEffect, useRef, useCallback } from 'react'

const MODE_NAMES = {
  0: 'INIT', 1: 'IDLE', 2: 'FOLLOWING',
  3: 'MOVING_TO_PANEL', 4: 'AT_PANEL', 5: 'MANUAL', 6: 'ESTOP',
  7: 'FOLLOWING_MAPLESS', 8: 'SUMMONING'
}

// rcl_interfaces/msg/ParameterType の定数
const PARAM_TYPE = {
  BOOL: 1, INTEGER: 2, DOUBLE: 3, STRING: 4,
  BYTE_ARRAY: 5, BOOL_ARRAY: 6, INTEGER_ARRAY: 7, DOUBLE_ARRAY: 8, STRING_ARRAY: 9,
}

function decodeParamValue(pv) {
  switch (pv.type) {
    case PARAM_TYPE.BOOL:          return pv.bool_value
    case PARAM_TYPE.INTEGER:       return pv.integer_value
    case PARAM_TYPE.DOUBLE:        return pv.double_value
    case PARAM_TYPE.STRING:        return pv.string_value
    case PARAM_TYPE.BYTE_ARRAY:    return pv.byte_array_value
    case PARAM_TYPE.BOOL_ARRAY:    return pv.bool_array_value
    case PARAM_TYPE.INTEGER_ARRAY: return pv.integer_array_value
    case PARAM_TYPE.DOUBLE_ARRAY:  return pv.double_array_value
    case PARAM_TYPE.STRING_ARRAY:  return pv.string_array_value
    default: return null
  }
}

// バックエンド側 (config_manager 等) が応答しない場合に UI が無期限に
// フリーズしないようにするタイムアウト (ms)
const TUNABLE_SERVICE_TIMEOUT_MS = 5000

function withTimeout(promise, label) {
  return new Promise((resolve, reject) => {
    const id = setTimeout(
      () => reject(new Error(`${label} がタイムアウトしました`)),
      TUNABLE_SERVICE_TIMEOUT_MS)
    promise.then(
      (v) => { clearTimeout(id); resolve(v) },
      (e) => { clearTimeout(id); reject(e) })
  })
}

function encodeParamValue(value, { isArray = false, isInt = false } = {}) {
  if (isArray) {
    return isInt
      ? { type: PARAM_TYPE.INTEGER_ARRAY, integer_array_value: value }
      : { type: PARAM_TYPE.DOUBLE_ARRAY,  double_array_value: value }
  }
  return isInt
    ? { type: PARAM_TYPE.INTEGER, integer_value: value }
    : { type: PARAM_TYPE.DOUBLE,  double_value: value }
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
  const [mappingActive, setMappingActive] = useState(false)

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

    // ── 購読: /slam_control/mapping_active ─────────────────
    const subMapping = new ROSLIB.Topic({
      ros, name: '/slam_control/mapping_active',
      messageType: 'std_msgs/Bool'
    })
    subMapping.subscribe((msg) => setMappingActive(msg.data))

    return () => {
      subMode.unsubscribe()
      subFault.unsubscribe()
      subEstop.unsubscribe()
      subPerson.unsubscribe()
      subCandidates.unsubscribe()
      subMapping.unsubscribe()
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

  // ── 呼び寄せサービス ──────────────────────────────────────
  const summonRobot = useCallback(() => {
    const ROSLIB = window.ROSLIB
    if (!rosRef.current || !ROSLIB) return
    const svc = new ROSLIB.Service({
      ros: rosRef.current,
      name: '/summon_navigator/call',
      serviceType: 'std_srvs/Trigger'
    })
    svc.callService(
      new ROSLIB.ServiceRequest({}),
      (res) => { if (!res.success) console.warn('呼び寄せ失敗:', res.message) }
    )
  }, [])

  // ── 地図作成 開始/停止 (トグル) ────────────────────────────
  const toggleMapping = useCallback(() => {
    const ROSLIB = window.ROSLIB
    if (!rosRef.current || !ROSLIB) return
    const svc = new ROSLIB.Service({
      ros: rosRef.current,
      name: '/slam_control/toggle_mapping',
      serviceType: 'std_srvs/Trigger'
    })
    svc.callService(
      new ROSLIB.ServiceRequest({}),
      (res) => { if (!res.success) console.warn('地図作成切替失敗:', res.message) }
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

  // ── 設定パネル: パラメータ取得 (rcl_interfaces/GetParameters を対象ノードへ
  // 直接呼び出す。読み取りのみのためモードゲート・config_manager 経由は不要) ──
  const getTunableParams = useCallback((nodeName, paramNames) => {
    const ROSLIB = window.ROSLIB
    return withTimeout(new Promise((resolve, reject) => {
      if (!rosRef.current || !ROSLIB) { reject(new Error('rosbridge 未接続')); return }
      const svc = new ROSLIB.Service({
        ros: rosRef.current,
        name: `/${nodeName}/get_parameters`,
        serviceType: 'rcl_interfaces/GetParameters'
      })
      svc.callService(
        new ROSLIB.ServiceRequest({ names: paramNames }),
        (res) => {
          const out = {}
          paramNames.forEach((name, i) => { out[name] = decodeParamValue(res.values[i]) })
          resolve(out)
        },
        (err) => reject(err)
      )
    }), `${nodeName} のパラメータ取得`)
  }, [])

  // ── 設定パネル: パラメータのライブ反映 (config_manager 経由。
  // IDLE/MANUAL モード以外では config_manager 側が拒否する) ──
  const applyTunableParam = useCallback((nodeName, paramName, value, opts) => {
    const ROSLIB = window.ROSLIB
    return withTimeout(new Promise((resolve, reject) => {
      if (!rosRef.current || !ROSLIB) { reject(new Error('rosbridge 未接続')); return }
      const svc = new ROSLIB.Service({
        ros: rosRef.current,
        name: '/config_manager/set_tunable_params',
        serviceType: 'th_system_msgs/SetTunableParams'
      })
      svc.callService(
        new ROSLIB.ServiceRequest({
          node_name: nodeName,
          parameters: [{ name: paramName, value: encodeParamValue(value, opts) }]
        }),
        (res) => { if (!res.success) console.warn('パラメータ適用失敗:', res.message); resolve(res) },
        (err) => reject(err)
      )
    }), `${nodeName}.${paramName} の適用`)
  }, [])

  // ── 設定パネル: YAML への保存 (config_manager 経由) ──────
  const saveTunableParams = useCallback((nodeName) => {
    const ROSLIB = window.ROSLIB
    return withTimeout(new Promise((resolve, reject) => {
      if (!rosRef.current || !ROSLIB) { reject(new Error('rosbridge 未接続')); return }
      const svc = new ROSLIB.Service({
        ros: rosRef.current,
        name: '/config_manager/save_tunable_params',
        serviceType: 'th_system_msgs/SaveTunableParams'
      })
      svc.callService(
        new ROSLIB.ServiceRequest({ node_name: nodeName }),
        (res) => { if (!res.success) console.warn('パラメータ保存失敗:', res.message); resolve(res) },
        (err) => reject(err)
      )
    }), `${nodeName} の保存`)
  }, [])

  return {
    connected, mode, modeName: MODE_NAMES[mode] ?? '---',
    fault, estop,
    personStatus, candidates, selectTarget, resetTracking,
    requestMode, publishTabletEstop, sendManualGoal, publishManualCmd, goToPanel,
    summonRobot,
    mappingActive, toggleMapping,
    getTunableParams, applyTunableParam, saveTunableParams,
  }
}
