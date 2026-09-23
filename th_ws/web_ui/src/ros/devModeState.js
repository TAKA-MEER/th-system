// ros/devModeState.js — /system/dev_mode (std_msgs/String に JSON) の純粋部
// (WP-DEV-01B)。React を import しないので node --test で直接試験できる
// （th_state/connectivity_core.py と同じ「純粋コア＋薄いフック」の流儀）。
// 購読と送信の React 部分は ros/useDevMode.js が持つ。
//
// connectivity_checker.py が出す JSON の形 (names.md §6.2):
//   {"dev_mode": bool, "ignore": {link,lidar_fault,scan_stop,battery,opcheck,auto_brake: bool},
//    "effective": {...}, "estop_hw_known": bool, "estop_hw_pressed": bool}
import { encodeParamValue, withTimeout } from './paramCodec.js'

// connectivity_checker.py の _DEV_ITEMS・th_safety/dev_mode_core.hpp と揃える。順序は表示順。
export const DEV_ITEMS = ['link', 'lidar_fault', 'scan_stop', 'battery', 'opcheck', 'auto_brake']

// as-built に運用開始を止めるゲートが無く、選んでも何も起きない項目。
// 画面にその旨を出す (brief-DEV-01B §2)。
export const DEV_NO_GATE_ITEMS = ['battery', 'opcheck', 'auto_brake']

// 接続先ノード (開発モードの正本。safety_monitor / obstacle_limiter は
// /system/dev_mode を購読して従う。names.md §1.3)。
export const DEV_MODE_NODE = 'connectivity_checker'

export const DEV_PARAM_MASTER = 'dev_mode'
export const devIgnoreParam = (item) => `dev_ignore_${item}`

function boolMap(src) {
  const out = {}
  for (const item of DEV_ITEMS) out[item] = src?.[item] === true
  return out
}

export function parseDevModeState(raw) {
  let obj = raw
  if (typeof raw === 'string') {
    try {
      obj = JSON.parse(raw)
    } catch {
      return null
    }
  }
  if (!obj || typeof obj !== 'object' || Array.isArray(obj)) return null
  return {
    dev_mode: obj.dev_mode === true,
    ignore: boolMap(obj.ignore),
    effective: boolMap(obj.effective),
    estop_hw_known: obj.estop_hw_known === true,
    estop_hw_pressed: obj.estop_hw_pressed === true,
  }
}

// いま実際に無視している項目名の一覧（表示用）。
export function effectiveItems(state) {
  if (!state) return []
  return DEV_ITEMS.filter((item) => state.effective[item])
}

// /connectivity_checker/{set,get}_parameters はノード固有の標準サービス名で
// 辞書 (names.json) に無いため、useOnsiteService.js の select_pin と同じく
// ローカル定数で扱い TOPICS/SERVICES には載せない (辞書ゲートの流儀)。
export const SET_PARAMS_SVC = (node) => `/${node}/set_parameters`
export const SET_PARAMS_TYPE = 'rcl_interfaces/SetParameters'

// 本番のパラメータ送信 (useDevMode.js の setDevParam が呼ぶ中身)。
// `ros2 param set` と同じ rcl_interfaces/SetParameters への直接呼び出し。
// ROSLIB を引数で受け取る純粋な関数にしてあるので、unit 試験では偽の
// ROSLIB を注入してサービス名・型・ペイロードを検証できる
// （TEST_MODE のモック経路では本番の呼び出しは覆えない）。
export function setBoolParamRequest({ ROSLIB, ros, node, name, value }) {
  return withTimeout(new Promise((resolve, reject) => {
    if (!ROSLIB) { reject(new Error('roslibjs is not loaded')); return }
    const svc = new ROSLIB.Service({
      ros,
      name: SET_PARAMS_SVC(node),
      serviceType: SET_PARAMS_TYPE,
    })
    svc.callService(
      new ROSLIB.ServiceRequest({
        parameters: [{ name, value: encodeParamValue(value, { isBool: true }) }],
      }),
      (res) => {
        const bad = (res?.results ?? []).filter((r) => !r.successful)
        if (bad.length > 0) console.warn('dev parameter apply failed:', bad)
        resolve(res)
      },
      (err) => reject(err),
    )
  }), `applying ${node}.${name}`)
}
