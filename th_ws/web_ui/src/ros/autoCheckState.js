// ros/autoCheckState.js — /opcheck/auto_status (std_msgs/String に JSON) の純粋部。
// React を import しないので node --test で直接試験できる
// （ros/devModeState.js と同じ「純粋コア＋薄いフック」の流儀）。
//
// opcheck_auto.py が出す JSON の形 (names.md §6.4):
//   {"overall": "OK"/"WARN"/"NG"/"CHECKING",
//    "suppressed": bool,
//    "items": {"ESTOP": {"result", "reason"},
//              "IMU": {"result", "reason"},
//              "LIDAR": {"result", "reason"}}}

export const AUTO_CHECK_ITEMS = ['ESTOP', 'IMU', 'LIDAR']

export const AUTO_CHECK_OVERALLS = ['OK', 'WARN', 'NG', 'CHECKING']

function cleanItem(src) {
  const result = typeof src?.result === 'string' ? src.result : 'CHECKING'
  return {
    result: AUTO_CHECK_OVERALLS.includes(result) ? result : 'CHECKING',
    reason: typeof src?.reason === 'string' ? src.reason : '',
  }
}

export function parseAutoCheck(raw) {
  let obj = raw
  if (typeof raw === 'string') {
    try {
      obj = JSON.parse(raw)
    } catch {
      return null
    }
  }
  if (!obj || typeof obj !== 'object' || Array.isArray(obj)) return null
  const overall = AUTO_CHECK_OVERALLS.includes(obj.overall) ? obj.overall : 'CHECKING'
  const items = {}
  for (const key of AUTO_CHECK_ITEMS) {
    items[key] = cleanItem(obj.items?.[key])
  }
  return {
    overall,
    suppressed: obj.suppressed === true,
    items,
  }
}

// S-01 の要約・S-00 の警告表示が使う: 警告を出すべきか。
// suppressed（開発モードで消している）ときは警告を出さない。
export function autoCheckWarns(auto) {
  if (!auto || auto.suppressed) return false
  return auto.overall === 'NG' || auto.overall === 'WARN'
}
