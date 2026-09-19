// ros/devModeState.js — /system/dev_mode (std_msgs/String に JSON) の純粋部
// (WP-DEV-01B)。React を import しないので node --test で直接試験できる
// （th_state/connectivity_core.py と同じ「純粋コア＋薄いフック」の流儀）。
// 購読と送信は ros/useDevMode.js が持つ。
//
// connectivity_checker.py が出す JSON の形 (names.md §6.2):
//   {"dev_mode": bool, "ignore": {link,battery,opcheck,auto_brake: bool},
//    "effective": {...}, "estop_hw_known": bool, "estop_hw_pressed": bool}

// connectivity_checker.py の _DEV_ITEMS と揃える。順序は表示順。
export const DEV_ITEMS = ['link', 'battery', 'opcheck', 'auto_brake']

// as-built に運用開始を止めるゲートが無く、選んでも何も起きない項目。
// 画面にその旨を出す (brief-DEV-01B §2)。
export const DEV_NO_GATE_ITEMS = ['battery', 'opcheck', 'auto_brake']

// 接続先ノード (launch が dev_mode を渡す唯一の相手。names.md §1.3)。
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
