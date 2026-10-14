// ros/speakerState.js — 機体スピーカー（Spec-safety §11）の純粋部
// (WP-DEV-01B の useDevMode.js / ros/devModeState.js と同じ二層)。
// React を import しないので node --test で直接試験できる。
// 購読と送信の React 部分は ros/useSpeaker.js が持つ。
//
// speaker_announcer が出す /speaker/cue（std_msgs/String に JSON。
// transient_local・2Hz）と、ラズパイの再生が出す /speaker/status
// （std_msgs/String に JSON。transient_local・1Hz）を読む。
// 書きは speaker_announcer/set_parameters への直接呼び出し
// (rcl_interfaces/SetParameters。`ros2 param set` と同じ標準サービス)。
// config_manager 経由ではない — speaker は実行中に変える値で、
// config_manager は停止中しか受け付けないため（useDevMode.js と同じ理由）。
import { encodeParamValue, withTimeout } from './paramCodec.js'

// 接続先ノード（報知判定の正本。値は /speaker/cue が運ぶ）。
export const SPEAKER_NODE = 'speaker_announcer'
export const SPEAKER_PARAM_ENABLED = 'speaker_enabled'
export const SPEAKER_PARAM_VOLUME = 'speaker_volume'
export const SPEAKER_PARAM_TEST = 'test_chime'

export const CUES = ['none', 'drive', 'blocked', 'warning']

// /speaker_announcer/{set,get}_parameters はノード固有の標準サービス名で
// 辞書 (names.json) に無いため、useOnsiteService.js の select_pin と同じく
// ローカル定数で扱い TOPICS/SERVICES には載せない (辞書ゲートの流儀)。
export const SET_PARAMS_SVC = (node) => `/${node}/set_parameters`
export const SET_PARAMS_TYPE = 'rcl_interfaces/SetParameters'

function clampVolume(v) {
  if (typeof v !== 'number' || !Number.isFinite(v)) return null
  return Math.min(100, Math.max(0, Math.round(v)))
}

export function parseSpeakerCue(raw) {
  let obj = raw
  if (typeof raw === 'string') {
    try {
      obj = JSON.parse(raw)
    } catch {
      return null
    }
  }
  if (!obj || typeof obj !== 'object' || Array.isArray(obj)) return null
  if (!CUES.includes(obj.cue)) return null
  const volume = clampVolume(obj.volume)
  if (volume === null) return null
  return {
    cue: obj.cue,
    enabled: obj.enabled === true,
    volume,
    test: obj.test === true,
    rpi_ok: obj.rpi_ok === true,
  }
}

export function parseSpeakerStatus(raw) {
  let obj = raw
  if (typeof raw === 'string') {
    try {
      obj = JSON.parse(raw)
    } catch {
      return null
    }
  }
  if (!obj || typeof obj !== 'object' || Array.isArray(obj)) return null
  if (!CUES.includes(obj.cue)) return null
  const volume = clampVolume(obj.volume)
  if (volume === null) return null
  return {
    cue: obj.cue,
    volume,
    error: typeof obj.error === 'string' ? obj.error : '',
  }
}

// 本番のパラメータ送信 (useSpeaker.js が呼ぶ中身)。
// bool（enabled・test_chime）と int（volume）を送り分ける。
export function setSpeakerParamRequest({ ROSLIB, ros, node, name, value, isBool }) {
  return withTimeout(new Promise((resolve, reject) => {
    if (!ROSLIB) { reject(new Error('roslibjs is not loaded')); return }
    const svc = new ROSLIB.Service({
      ros,
      name: SET_PARAMS_SVC(node),
      serviceType: SET_PARAMS_TYPE,
    })
    svc.callService(
      new ROSLIB.ServiceRequest({
        parameters: [{
          name,
          value: encodeParamValue(value, isBool ? { isBool: true } : { isInt: true }),
        }],
      }),
      (res) => {
        const bad = (res?.results ?? []).filter((r) => !r.successful)
        if (bad.length > 0) console.warn('speaker parameter apply failed:', bad)
        resolve(res)
      },
      (err) => reject(err),
    )
  }), `applying ${node}.${name}`)
}

// 購読の組み立て（topics-defined.test.js の流儀。名前・型が undefined のまま
// 購読を張らないことを unit で縛る）。
export function subscribeSpeaker(ROSLIB, ros, cueTopic, statusTopic, onCue, onStatus) {
  const subs = []
  const mk = (name, cb) => {
    const t = new ROSLIB.Topic({
      ros,
      name,
      messageType: 'std_msgs/String',
      queue_length: 1,
    })
    t.subscribe((msg) => cb(msg?.data ?? null))
    subs.push(t)
    return t
  }
  mk(cueTopic, onCue)
  mk(statusTopic, onStatus)
  return () => subs.forEach((t) => t.unsubscribe?.())
}
