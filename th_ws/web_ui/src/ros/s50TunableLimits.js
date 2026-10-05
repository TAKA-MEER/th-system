// ros/s50TunableLimits.js — S-50 一般タブの registry 連動の上限 (SG-B9)。
//
// 廃止予定ノードの項目のうち上限が registry と食い違うもの
// （`follow_planner_mapless` の `v_max` は UI 上 1.5 まで入れられる）は、
// registry の値を超えて保存できないようにする。上限値そのものは
// `../generated/param_limits.json`（`scripts/gen_param_limits.py` が
// registry.yaml から作る。手で書かない）が持ち、このモジュールは
// 「どのノードのどの項目にどの上限を掛けるか」という対応付けと丸めだけを
// 担う（ROS2 非依存の純粋関数。`node --test` から直接テストする）。
//
// caps の形: `{ [nodeName]: { [paramName]: 上限値 } }`
// （S50Settings.jsx が param_limits.json から組み立てる）。

export function clampTunableParam(caps, nodeName, paramName, value) {
  const cap = caps?.[nodeName]?.[paramName]
  if (cap === undefined) return value
  if (typeof value !== 'number' || !Number.isFinite(value)) return value
  return Math.min(value, cap)
}

// 入力欄の max に使う（UI の NumberField が blur 時にこの範囲へ丸める）。
// registry の上限が欄の既定 max より小さいときだけ絞る。
export function tunableFieldMax(caps, nodeName, paramName, defaultMax) {
  const cap = caps?.[nodeName]?.[paramName]
  if (cap === undefined) return defaultMax
  return Math.min(defaultMax, cap)
}
