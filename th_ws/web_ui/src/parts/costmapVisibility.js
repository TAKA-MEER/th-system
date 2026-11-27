// parts/costmapVisibility.js — 「入れない所」（コストマップ表示／非表示）。
// Spec-webui.md §6.1 の地図の端の道具「入れない所」の入切。全地図で共通で、
// localStorage に置く（読めないときは既定）。
// S-50 の表示設定もこのキー（th.costmapVisible）を共用する。
//
// 既定は「入」（出しておく）。「実際の地図で見て決める」（Spec-webui.md §6.1
// の未決。ブリーフ W0b: 既定を 1 か所の定数にする）が決まったらここだけ直す。

const KEY = 'th.costmapVisible'

export const COSTMAP_DEFAULT_VISIBLE = true

function storage() {
  try {
    return typeof window !== 'undefined' ? window.localStorage : null
  } catch {
    return null
  }
}

export function readCostmapVisible() {
  try {
    const v = storage()?.getItem(KEY) ?? null
    return v === null ? COSTMAP_DEFAULT_VISIBLE : v === '1'
  } catch {
    return COSTMAP_DEFAULT_VISIBLE
  }
}

export function writeCostmapVisible(visible) {
  const next = !!visible
  try {
    storage()?.setItem(KEY, next ? '1' : '0')
  } catch {
    /* ignore */
  }
  return next
}
