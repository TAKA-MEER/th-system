// ============================================================
// useRosbridge.js — rosbridge WebSocket connection hook
// ============================================================
import { useState, useEffect, useRef, useCallback } from 'react'
import { SLAM_DISCARD_MARKER } from '../i18n/states.js'

const MODE_NAMES = {
  0: 'INIT', 1: 'IDLE', 2: 'FOLLOWING',
  3: 'MOVING_TO_PANEL', 4: 'AT_PANEL', 5: 'MANUAL', 6: 'ESTOP',
  7: 'FOLLOWING_MAPLESS', 8: 'SUMMONING'
}

// rcl_interfaces/msg/ParameterType constants
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

// Timeout (ms) so the UI doesn't freeze forever when the backend
// (config_manager etc.) never responds.
const TUNABLE_SERVICE_TIMEOUT_MS = 5000

// History length (seconds) kept for the wheel-speed display card
const WHEEL_HISTORY_SEC = 15

function withTimeout(promise, label) {
  return new Promise((resolve, reject) => {
    const id = setTimeout(
      () => reject(new Error(`${label} timed out`)),
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

// Defaults to the rosbridge on the host serving this page (the robot PC).
// Pass a different url to connect to a different host's rosbridge.
//
// readOnly: for the audience display (VISION.md §6.3). Never publishes
// anything to ROS2. In particular it stops /manual/heartbeat, so the
// audience view doesn't become a second source of the MANUAL heartbeat.
// Default false = normal behavior for the operator UI.
// mapThrottleMs: /map (OccupancyGrid) can be several hundred KB per message.
// The audience display only needs per-second updates, so this throttles the
// subscription to reduce load. 0 = no throttling (default).
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
  // Latest map-op result string ("OK: ..." / "NG: ..."), published by
  // slam_control. Kept so the outcome survives even if a service response
  // is missed.
  const [slamLastResult, setSlamLastResult] = useState(null)
  const [actionError, setActionError]     = useState(null)  // reason for the most recent failed service call (WebUI display)
  // Result of the most recent service call (for voice-announcement triggers).
  // Unlike actionError this also records success, and seq increases
  // monotonically so repeated identical results still register as an edge.
  const [lastAction, setLastAction]       = useState(null)  // { seq, kind, ok, message }
  // Internal state of follow / search / summon (voice-notification triggers)
  const [followStatus, setFollowStatus]   = useState(null)  // th_system_msgs/FollowStatus
  const [searchStatus, setSearchStatus]   = useState(null)  // th_system_msgs/SearchStatus
  const [summonStatus, setSummonStatus]   = useState(null)  // { seq, event, message }
  const summonSeqRef = useRef(0)
  const [mapData, setMapData]             = useState(null)   // latest nav_msgs/OccupancyGrid, null if none received yet
  const [robotPose, setRobotPose]         = useState(null)   // {x, y, yaw} in the map frame, null if not yet determined
  const [scanData, setScanData]           = useState(null)   // latest sensor_msgs/LaserScan (/scan_filtered), null if none received yet
  const [pathData, setPathData]           = useState(null)   // latest nav_msgs/Path (/plan), null if none received yet
  const [wheelSpeedData, setWheelSpeedData] = useState({ measured: [], command: [] })
  // Recent history ({t, left, right} arrays) for the command-vs-measured
  // left/right wheel speed graph

  // Intermediate state for TF composition (kept in refs so it doesn't
  // trigger a re-render on every tick)
  const mapOdomRef      = useRef(null)  // latest map->odom (geometry_msgs/Transform)
  const odomBaseRef     = useRef(null)  // latest odom->base_link-equivalent (geometry_msgs/Transform)
  const lastPoseEmitRef = useRef(0)     // timestamp for client-side throttling

  const measuredWheelBufRef = useRef([])  // {t, left, right} (receive time, measured m/s)
  const commandWheelBufRef  = useRef([])  // {t, left, right} (receive time, commanded m/s)

  // Service responses can't be reconstructed from a subscribed topic, so
  // results are pushed through as a single event stream. The consumer
  // (useVoiceTriggers) looks at `kind` to decide what to play.
  const actionSeqRef = useRef(0)
  const bumpAction = useCallback((kind, ok, message) => {
    actionSeqRef.current += 1
    setLastAction({ seq: actionSeqRef.current, kind, ok, message: message ?? '' })
  }, [])

  // ── connection ──────────────────────────────────────────────────
  useEffect(() => {
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) { console.error('roslibjs is not loaded'); return }

    const ros = new ROSLIB.Ros({ url })
    rosRef.current = ros

    ros.on('connection', () => {
      console.log('rosbridge connected')
      setConnected(true)
    })
    ros.on('error',      (e) => { console.error('rosbridge error', e); setConnected(false) })
    ros.on('close',      ()  => { console.warn('rosbridge disconnected'); setConnected(false) })

    // ── subscribe: /robot/mode ──────────────────────────────────
    const subMode = new ROSLIB.Topic({
      ros, name: '/robot/mode',
      messageType: 'th_system_msgs/RobotMode'
    })
    subMode.subscribe((msg) => setMode(msg.mode))

    // ── subscribe: /safety/fault ────────────────────────────────
    const subFault = new ROSLIB.Topic({
      ros, name: '/safety/fault',
      messageType: 'th_system_msgs/FaultStatus'
    })
    subFault.subscribe((msg) => setFault(msg))

    // ── subscribe: /safety/estop ────────────────────────────────
    const subEstop = new ROSLIB.Topic({
      ros, name: '/safety/estop',
      messageType: 'std_msgs/Bool'
    })
    subEstop.subscribe((msg) => setEstop(msg.data))

    // ── subscribe: /person/status ───────────────────────────────
    const subPerson = new ROSLIB.Topic({
      ros, name: '/person/status',
      messageType: 'th_system_msgs/PersonStatus'
    })
    subPerson.subscribe((msg) => setPersonStatus(msg))

    // ── subscribe: follow logic internal state (voice notifications) ───
    // The publisher already throttles to "on change + 1Hz", so no
    // additional throttling is needed here
    const subFollowStatus = new ROSLIB.Topic({
      ros, name: '/follow/status',
      messageType: 'th_system_msgs/FollowStatus'
    })
    subFollowStatus.subscribe((msg) => setFollowStatus(msg))

    // ── subscribe: search phase (voice notifications) ─────────────────
    const subSearchStatus = new ROSLIB.Topic({
      ros, name: '/person/search_status',
      messageType: 'th_system_msgs/SearchStatus'
    })
    subSearchStatus.subscribe((msg) => setSearchStatus(msg))

    // ── subscribe: summon events (voice notifications) ─────────────────
    // Arrival and abort both end up as a SUMMONING->IDLE mode transition,
    // so mode alone can't tell them apart. A seq is attached so this can be
    // treated as an edge.
    const subSummonStatus = new ROSLIB.Topic({
      ros, name: '/summon_navigator/status',
      messageType: 'th_system_msgs/SummonStatus'
    })
    subSummonStatus.subscribe((msg) => {
      summonSeqRef.current += 1
      setSummonStatus({ seq: summonSeqRef.current, event: msg.event, message: msg.message })
    })

    // ── subscribe: follow-target candidate list ──────────────────────────
    const subCandidates = new ROSLIB.Topic({
      ros, name: '/sobits_follower/multiple_sensor_person_tracking/person_candidates',
      messageType: 'multiple_sensor_person_tracking/PersonCandidates'
    })
    subCandidates.subscribe((msg) => setCandidates(msg.positions ?? []))

    // ── subscribe: /slam_control/mapping_active ─────────────────
    const subMapping = new ROSLIB.Topic({
      ros, name: '/slam_control/mapping_active',
      messageType: 'std_msgs/Bool'
    })
    subMapping.subscribe((msg) => setMappingActive(msg.data))

    // ── subscribe: /slam_control/last_result ────────────────────
    // The audience page is a separate useRosbridge instance, so discarding
    // the map from the operator UI doesn't propagate directly. This picks
    // up the discard result and syncs both displays. /map itself doesn't
    // arrive right after a discard (slam_toolbox only publishes when it
    // rebuilds from the pose graph), so this can't just wait for a message
    // there.
    //
    // This topic is transient_local, so subscribing later still delivers
    // the last result as one latched message. If that first message were
    // used to clear the map, the map would disappear every time a client
    // reconnects, so the first message is display-only and never triggers
    // the discard side effect.
    let slamResultLatched = false
    const subSlamResult = new ROSLIB.Topic({
      ros, name: '/slam_control/last_result',
      messageType: 'std_msgs/String'
    })
    subSlamResult.subscribe((msg) => {
      setSlamLastResult(msg.data)
      const isFirst = !slamResultLatched
      slamResultLatched = true
      if (!isFirst && msg.data.startsWith('OK') && msg.data.includes(SLAM_DISCARD_MARKER)) {
        setMapData(null)
        setPathData(null)
      }
    })

    // ── subscribe: /map (SLAM map) ──────────────────────────────────
    const subMap = new ROSLIB.Topic({
      ros, name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
      ...(mapThrottleMs > 0 ? { throttle_rate: mapThrottleMs } : {}),
    })
    subMap.subscribe((msg) => setMapData(msg))

    // ── subscribe: /tf (compose map->odom, odom->base_link[_footprint] by hand) ──
    // roslib.min.js's ROSLIB.TFClient assumes tf2_web_republisher (the
    // /republish_tfs service), which this repo doesn't run, so it can't be
    // used here (subscribing would never invoke the callback). /tf is
    // subscribed directly and composed by hand instead. child_frame_id is
    // 'base_link' on real hardware (esp32_bridge) but 'base_footprint' in
    // sim (gazebo_ros_diff_drive); the two can be treated as the same frame
    // since they're joined by a fixed joint (xyz="0 0 wheel_radius") that
    // makes X/Y/yaw identical.
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

      // /tf runs at ~50Hz (transform_publish_period: 0.02). That's more
      // than marker rendering needs, so throttle to ~10Hz client-side
      const now = performance.now()
      if (now - lastPoseEmitRef.current < 100) return
      lastPoseEmitRef.current = now

      const baseInOdom = new ROSLIB.Pose({
        position: odomBaseRef.current.translation,
        orientation: odomBaseRef.current.rotation,
      })
      baseInOdom.applyTransform(new ROSLIB.Transform(mapOdomRef.current))  // into the map frame

      const q = baseInOdom.orientation
      const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
      setRobotPose({ x: baseInOdom.position.x, y: baseInOdom.position.y, yaw })
    })

    // ── subscribe: /scan_filtered (point-cloud display; throttled to 5Hz to save bandwidth) ──
    const subScan = new ROSLIB.Topic({
      ros, name: '/scan_filtered',
      messageType: 'sensor_msgs/LaserScan',
      throttle_rate: 200,
    })
    subScan.subscribe((msg) => setScanData(msg))

    // ── subscribe: /plan (Nav2's global path, for route display) ──
    const subPath = new ROSLIB.Topic({
      ros, name: '/plan',
      messageType: 'nav_msgs/Path',
    })
    subPath.subscribe((msg) => setPathData(msg))

    // ── subscribe: command-vs-measured left/right wheel speed (for the speed card, keeps the last WHEEL_HISTORY_SEC seconds) ──
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

  // ── mode-change service call ────────────────────────────
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
          console.warn('mode change failed:', res.message)
          setActionError(`mode change failed: ${res.message}`)
        } else {
          setActionError(null)
        }
      }
    )
  }, [bumpAction])

  // ── tablet emergency stop ────────────────────────────────────
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

  // ── periodic heartbeat ────────────────────────────────────
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

  // ── manual goal send ────────────────────────────────────────
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

  // ── manual jog (direct velocity command) ──────────────────────────
  // manual_command_handler's Nav2-goal approach needs a map frame, which
  // isn't available in mapless operation. Switched to publishing Twist
  // directly to jog_gate's /cmd_vel_manual_raw (O-6 / WP-SAFE-04), which
  // gate-keeps it to twist_mux's /cmd_vel_manual (priority 30, timeout 0.5s).
  // The UI publishes at a fixed period while held, and the timeout stops
  // the robot automatically.
  const manualCmdRef = useRef(null)
  useEffect(() => {
    if (!rosRef.current || !connected) return
    const ROSLIB = window.ROSLIB
    manualCmdRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: '/cmd_vel_manual_raw',
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

  // ── go-to-panel service ────────────────────────────────────
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
        if (!res.success) setActionError(`go-to-panel failed: ${res.message}`)
        else setActionError(null)
      }
    )
  }, [bumpAction])

  // ── summon service ──────────────────────────────────────
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
          console.warn('summon failed:', res.message)
          setActionError(`summon failed: ${res.message}`)
        } else {
          setActionError(null)
        }
      }
    )
  }, [bumpAction])

  // ── mapping start/stop (toggle) ────────────────────────────
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
          console.warn('mapping toggle failed:', res.message)
          setActionError(`mapping toggle failed: ${res.message}`)
        } else {
          setActionError(null)
        }
      }
    )
  }, [bumpAction])

  // ── map operations (save / discard) ────────────────────────────
  // Same std_srvs/Trigger type as toggle_mapping; slam_control forwards
  // these to slam_toolbox's save_map / serialize_map / deserialize_map.
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
          console.warn(`${label} failed:`, res.message)
          setActionError(`${label} failed: ${res.message}`)
        } else {
          setActionError(null)
        }
      }
    )
  }, [bumpAction])

  const saveMap = useCallback(
    () => callSlamTrigger('/slam_control/save_map', 'save_map', 'map save'),
    [callSlamTrigger])

  // Even a successful discard doesn't make /map arrive automatically:
  // slam_toolbox only rebuilds and publishes from the pose graph on its own
  // map_update_interval, and right after the graph goes empty there's
  // nothing for it to publish (the WebUI keeps showing the last map it
  // received). Once discard succeeds, drop this hook's own copy too.
  const discardMap = useCallback(() => {
    const ROSLIB = window.ROSLIB
    if (!rosRef.current || !ROSLIB) return
    const svc = new ROSLIB.Service({
      ros: rosRef.current,
      name: '/slam_control/discard_map',
      serviceType: 'std_srvs/Trigger'
    })
    svc.callService(
      new ROSLIB.ServiceRequest({}),
      (res) => {
        bumpAction('discard_map', res.success, res.message)
        if (res.success) {
          setMapData(null)
          setPathData(null)
          setActionError(null)
        } else {
          console.warn('map discard failed:', res.message)
          setActionError(`map discard failed: ${res.message}`)
        }
      }
    )
  }, [bumpAction])

  // ── manual clear of the action error (for the WebUI banner's close button) ──
  const clearActionError = useCallback(() => setActionError(null), [])

  // ── follow-target selection / re-registration ─────────────────────────────
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
        if (!res.success) console.warn('target selection failed:', res.message)
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
      (res) => { if (!res.success) console.warn('follow reset failed:', res.message) }
    )
  }, [])

  // ── settings panel: parameter fetch (calls rcl_interfaces/GetParameters
  // directly on the target node; read-only, so no mode gate / config_manager
  // hop is needed) ──
  const getTunableParams = useCallback((nodeName, paramNames) => {
    const ROSLIB = window.ROSLIB
    return withTimeout(new Promise((resolve, reject) => {
      if (!rosRef.current || !ROSLIB) { reject(new Error('rosbridge not connected')); return }
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
    }), `fetching parameters for ${nodeName}`)
  }, [])

  // ── settings panel: live parameter apply (via config_manager;
  // config_manager itself rejects this outside IDLE/MANUAL mode) ──
  const applyTunableParam = useCallback((nodeName, paramName, value, opts) => {
    const ROSLIB = window.ROSLIB
    return withTimeout(new Promise((resolve, reject) => {
      if (!rosRef.current || !ROSLIB) { reject(new Error('rosbridge not connected')); return }
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
        (res) => { if (!res.success) console.warn('parameter apply failed:', res.message); resolve(res) },
        (err) => reject(err)
      )
    }), `applying ${nodeName}.${paramName}`)
  }, [])

  // ── settings panel: save to YAML (via config_manager) ──────
  const saveTunableParams = useCallback((nodeName) => {
    const ROSLIB = window.ROSLIB
    return withTimeout(new Promise((resolve, reject) => {
      if (!rosRef.current || !ROSLIB) { reject(new Error('rosbridge not connected')); return }
      const svc = new ROSLIB.Service({
        ros: rosRef.current,
        name: '/config_manager/save_tunable_params',
        serviceType: 'th_system_msgs/SaveTunableParams'
      })
      svc.callService(
        new ROSLIB.ServiceRequest({ node_name: nodeName }),
        (res) => { if (!res.success) console.warn('parameter save failed:', res.message); resolve(res) },
        (err) => reject(err)
      )
    }), `saving ${nodeName}`)
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
