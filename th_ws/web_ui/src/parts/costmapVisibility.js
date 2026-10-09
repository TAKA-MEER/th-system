// parts/costmapVisibility.js — S-50 の地図の表示項目（コストマップ表示／非表示）。
// Spec-webui.md §3.15。今後、地図を見やすくする設定を同じ場所に足す。
// fontScale.js と同じ流儀: localStorage が正本（既定は表示）。
// S-20/S-21 の地図タブが useCostmap.js 経由で読む。切り替えは画面を開き直すと反映される。

const KEY = 'th.costmapVisible'

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
    return v === null ? true : v === '1'
  } catch {
    return true
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
