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

// 速度表示カードで保持する履歴の長さ (秒)
const WHEEL_HISTORY_SEC = 15

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
//
// readOnly: 観客向け表示 (VISION.md §6.3) 用。ROS2 への publish を一切
// 行わない。特に /manual/heartbeat を止めることが目的で、これが二重に
// 流れると MANUAL のハートビート源が観客画面にも依存してしまう。
// 既定は false = 従来どおりの操作 UI 向け動作。
// mapThrottleMs: /map (OccupancyGrid) は 1 通で数百 KB になり得る。観客表示は
// 秒単位の更新で十分なので間引いて購読負荷を下げる。0 = 間引かない (既定)。
export function useRosbridge(url = `ws://${window.location.hostname}:9090`,
                             { readOnly = false, mapThrottleMs = 0 } = {}) {
  const rosRef  = useRef(null)
  const [connected, setConnected]   = useState(false)
  const [mode, setMode]             = useState(null)
  const [fault, setFault]           = useState({ active: false, fault_type: 'NONE' })
  const [estop, setEstop]           = useState(false)
  const [battVolt, setBattVolt]     = useState(null)
  const [personStatus, setPersonStatus] = useState(null)
  const [candidates, setCandidates]     = useState([])
  const [mappingActive, setMappingActive] = useState(false)
  // 直近の地図操作の結果文字列 ("OK: ..." / "NG: ...")。slam_control が publish する。
  // サービス応答を取りこぼしても、何が起きたか画面に残るようにするため
  const [slamLastResult, setSlamLastResult] = useState(null)
  const [actionError, setActionError]     = useState(null)  // 直近のサービス呼び出し失敗理由 (WebUI表示用)
  // 直近のサービス呼び出し結果 (音声アナウンスのトリガ用)。actionError と違い成功も記録し、
  // seq が単調増加するため同じ結果が連続してもエッジとして検出できる
  const [lastAction, setLastAction]       = useState(null)  // { seq, kind, ok, message }
  // 追従・捜索・呼び寄せの内部状態 (音声通知のトリガ用)
  const [followStatus, setFollowStatus]   = useState(null)  // th_system_msgs/FollowStatus
  const [searchStatus, setSearchStatus]   = useState(null)  // th_system_msgs/SearchStatus
  const [summonStatus, setSummonStatus]   = useState(null)  // { seq, event, message }
  const summonSeqRef = useRef(0)
  const [mapData, setMapData]             = useState(null)   // 最新の nav_msgs/OccupancyGrid、未受信なら null
  const [robotPose, setRobotPose]         = useState(null)   // map座標系での {x, y, yaw}、未確定なら null
  const [scanData, setScanData]           = useState(null)   // 最新の sensor_msgs/LaserScan (/scan_filtered)、未受信なら null
  const [pathData, setPathData]           = useState(null)   // 最新の nav_msgs/Path (/plan)、未受信なら null
  const [wheelSpeedData, setWheelSpeedData] = useState({ measured: [], command: [] })
  // 左右輪 指令vs実測 速度グラフ用の直近履歴 ({t, left, right} の配列)

  // TF合成用の中間状態 (毎tick再レンダーさせないため ref で保持)
  const mapOdomRef      = useRef(null)  // 最新の map->odom (geometry_msgs/Transform)
  const odomBaseRef     = useRef(null)  // 最新の odom->base_link 相当 (geometry_msgs/Transform)
  const lastPoseEmitRef = useRef(0)     // クライアント側間引き用タイムスタンプ

  const measuredWheelBufRef = useRef([])  // {t, left, right} (受信時刻, 実測 m/s)
  const commandWheelBufRef  = useRef([])  // {t, left, right} (受信時刻, 指令 m/s)

  // サービス応答を購読トピックから復元することはできないため、結果をイベントとして
  // 1本流す。受け手 (useVoiceTriggers) が kind を見て何を鳴らすか決める。
  const actionSeqRef = useRef(0)
  const bumpAction = useCallback((kind, ok, message) => {
    actionSeqRef.current += 1
    setLastAction({ seq: actionSeqRef.current, kind, ok, message: message ?? '' })
  }, [])

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

    // ── 購読: 追従ロジックの内部状態 (音声通知用) ───────────
    // 発行側で「変化時 + 1Hz」に間引いてあるため、ここでの throttle は不要
    const subFollowStatus = new ROSLIB.Topic({
      ros, name: '/follow/status',
      messageType: 'th_system_msgs/FollowStatus'
    })
    subFollowStatus.subscribe((msg) => setFollowStatus(msg))

    // ── 購読: 捜索段階 (音声通知用) ─────────────────────────
    const subSearchStatus = new ROSLIB.Topic({
      ros, name: '/person/search_status',
      messageType: 'th_system_msgs/SearchStatus'
    })
    subSearchStatus.subscribe((msg) => setSearchStatus(msg))

    // ── 購読: 呼び寄せイベント (音声通知用) ─────────────────
    // 到着と中止はどちらも SUMMONING→IDLE 遷移になるため、モードだけでは
    // 区別できない。seq を振ってエッジとして扱えるようにする
    const subSummonStatus = new ROSLIB.Topic({
      ros, name: '/summon_navigator/status',
      messageType: 'th_system_msgs/SummonStatus'
    })
    subSummonStatus.subscribe((msg) => {
      summonSeqRef.current += 1
      setSummonStatus({ seq: summonSeqRef.current, event: msg.event, message: msg.message })
    })

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

    // ── 購読: /slam_control/last_result ────────────────────
    const subSlamResult = new ROSLIB.Topic({
      ros, name: '/slam_control/last_result',
      messageType: 'std_msgs/String'
    })
    subSlamResult.subscribe((msg) => setSlamLastResult(msg.data))

    // ── 購読: /map (SLAM 地図) ──────────────────────────────
    const subMap = new ROSLIB.Topic({
      ros, name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
      ...(mapThrottleMs > 0 ? { throttle_rate: mapThrottleMs } : {}),
    })
    subMap.subscribe((msg) => setMapData(msg))

    // ── 購読: /tf (map->odom, odom->base_link[_footprint] を自前で合成) ──
    // roslib.min.js の ROSLIB.TFClient は tf2_web_republisher (/republish_tfs
    // サービス) 前提の実装で本リポジトリには存在しないノードのため使用不可
    // (subscribe してもコールバックが永久に呼ばれない)。/tf を直接購読し
    // 手動で合成する。child_frame_id は実機では 'base_link' (esp32_bridge)、
    // シムでは 'base_footprint' (gazebo_ros_diff_drive) と異なるが、両者は
    // fixed joint (xyz="0 0 wheel_radius") で X/Y/yaw が完全一致するため
    // 区別せず同一視してよい。
    const subTf = new ROSLIB.Topic({
      ros, name: '/tf',
      messageType: 'tf2_msgs/TFMessage',
    })
    subTf.subscribe((msg) => {
      let updated = false
      msg.transforms.forEach((t) => {
        if (t.child_frame_id === 'odom') { mapOdomRef.current = t.transform; updated = true }
        if (t.child_frame_id === 'base_link' || t.child_frame_id === 'base_footprint') {
          odomBaseRef.current = t.transform; updated = true
        }
      })
      if (!updated || !mapOdomRef.current || !odomBaseRef.current) return

      // /tf は ~50Hz (transform_publish_period: 0.02)。マーカー描画には
      // 過剰なためクライアント側で ~10Hz に間引く
      const now = performance.now()
      if (now - lastPoseEmitRef.current < 100) return
      lastPoseEmitRef.current = now

      const baseInOdom = new ROSLIB.Pose({
        position: odomBaseRef.current.translation,
        orientation: odomBaseRef.current.rotation,
      })
      baseInOdom.applyTransform(new ROSLIB.Transform(mapOdomRef.current))  // map座標系へ

      const q = baseInOdom.orientation
      const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
      setRobotPose({ x: baseInOdom.position.x, y: baseInOdom.position.y, yaw })
    })

    // ── 購読: /scan_filtered (点群表示用。帯域節約のため 5Hz に間引いて配信) ──
    const subScan = new ROSLIB.Topic({
      ros, name: '/scan_filtered',
      messageType: 'sensor_msgs/LaserScan',
      throttle_rate: 200,
    })
    subScan.subscribe((msg) => setScanData(msg))

    // ── 購読: /plan (Nav2 のグローバル経路。ルート表示用) ──
    const subPath = new ROSLIB.Topic({
      ros, name: '/plan',
      messageType: 'nav_msgs/Path',
    })
    subPath.subscribe((msg) => setPathData(msg))

    // ── 購読: 左右輪 指令vs実測 速度 (速度表示カード用、直近 WHEEL_HISTORY_SEC 秒を保持) ──
    const pushWheelSample = (bufRef, msg, key) => {
      const t = Date.now() / 1000
      const buf = bufRef.current
      buf.push({ t, left: msg.left_speed, right: msg.right_speed })
      const cutoff = t - WHEEL_HISTORY_SEC
      while (buf.length && buf[0].t < cutoff) buf.shift()
      setWheelSpeedData((prev) => ({ ...prev, [key]: buf.slice() }))
    }
    const subWheelFeedback = new ROSLIB.Topic({
      ros, name: '/esp32/wheel_feedback',
      messageType: 'th_system_msgs/WheelFeedback',
    })
    subWheelFeedback.subscribe((msg) => pushWheelSample(measuredWheelBufRef, msg, 'measured'))

    const subWheelCmd = new ROSLIB.Topic({
      ros, name: '/esp32/wheel_cmd_speed',
      messageType: 'th_system_msgs/WheelFeedback',
    })
    subWheelCmd.subscribe((msg) => pushWheelSample(commandWheelBufRef, msg, 'command'))

    return () => {
      subMode.unsubscribe()
      subFault.unsubscribe()
      subEstop.unsubscribe()
      subPerson.unsubscribe()
      subFollowStatus.unsubscribe()
      subSearchStatus.unsubscribe()
      subSummonStatus.unsubscribe()
      subCandidates.unsubscribe()
      subMapping.unsubscribe()
      subMap.unsubscribe()
      subTf.unsubscribe()
      subScan.unsubscribe()
      subPath.unsubscribe()
      subWheelFeedback.unsubscribe()
      subWheelCmd.unsubscribe()
      mapOdomRef.current = null
      odomBaseRef.current = null
      measuredWheelBufRef.current = []
      commandWheelBufRef.current = []
      ros.close()
    }
  }, [url, mapThrottleMs])

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
      (res) => {
        bumpAction('set_mode', res.success, res.message)
        if (!res.success) {
          console.warn('モード変更失敗:', res.message)
          setActionError(`モード変更失敗: ${res.message}`)
        } else {
          setActionError(null)
        }
      }
    )
  }, [bumpAction])

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
    if (!connected || readOnly) return
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
  }, [connected, readOnly])

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
      (res) => {
        console.log('GoToPanel:', res)
        bumpAction('go_to_panel', res.success, res.message)
        if (!res.success) setActionError(`配電盤移動失敗: ${res.message}`)
        else setActionError(null)
      }
    )
  }, [bumpAction])

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
      (res) => {
        bumpAction('summon', res.success, res.message)
        if (!res.success) {
          console.warn('呼び寄せ失敗:', res.message)
          setActionError(`呼び寄せ失敗: ${res.message}`)
        } else {
          setActionError(null)
        }
      }
    )
  }, [bumpAction])

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
      (res) => {
        bumpAction('toggle_mapping', res.success, res.message)
        if (!res.success) {
          console.warn('地図作成切替失敗:', res.message)
          setActionError(`地図作成切替失敗: ${res.message}`)
        } else {
          setActionError(null)
        }
      }
    )
  }, [bumpAction])

  // ── 地図操作 (保存 / 破棄) ────────────────────────────────
  // toggle_mapping と同じ Trigger 型で、slam_control が slam_toolbox の
  // save_map / serialize_map / deserialize_map へ転送する。
  const callSlamTrigger = useCallback((service, kind, label) => {
    const ROSLIB = window.ROSLIB
    if (!rosRef.current || !ROSLIB) return
    const svc = new ROSLIB.Service({
      ros: rosRef.current,
      name: service,
      serviceType: 'std_srvs/Trigger'
    })
    svc.callService(
      new ROSLIB.ServiceRequest({}),
      (res) => {
        bumpAction(kind, res.success, res.message)
        if (!res.success) {
          console.warn(`${label}失敗:`, res.message)
          setActionError(`${label}失敗: ${res.message}`)
        } else {
          setActionError(null)
        }
      }
    )
  }, [bumpAction])

  const saveMap = useCallback(
    () => callSlamTrigger('/slam_control/save_map', 'save_map', '地図の保存'),
    [callSlamTrigger])

  const discardMap = useCallback(
    () => callSlamTrigger('/slam_control/discard_map', 'discard_map', '地図の破棄'),
    [callSlamTrigger])

  // ── アクションエラーの手動クリア (WebUI バナーの閉じるボタン用) ──
  const clearActionError = useCallback(() => setActionError(null), [])

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
      (res) => {
        bumpAction('select_target', res.success, res.message)
        if (!res.success) console.warn('ターゲット選択失敗:', res.message)
      }
    )
  }, [bumpAction])

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
    mappingActive, toggleMapping, saveMap, discardMap, slamLastResult,
    actionError, clearActionError, lastAction,
    followStatus, searchStatus, summonStatus,
    mapData, robotPose, scanData, pathData, wheelSpeedData,
    getTunableParams, applyTunableParam, saveTunableParams,
  }
}
