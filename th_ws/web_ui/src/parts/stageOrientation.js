// parts/stageOrientation.js — S-50 の縦横の固定（自動／縦固定／横固定）。
// Spec-webui.md §3.15。fontScale.js と同じ流儀: localStorage が正本。
// shell/stageMetrics.js の stageMetrics(w, h, { lock }) が実効し、
// FixedStage が起動時・変更時に読み直す。変更は「TH_ORIENTATION」
// CustomEvent でも知らせる（同一タブでは storage イベントが飛ばないため）。

const KEY = 'th.stageOrientation'

export const ORIENTATION_AUTO = 'auto'
export const ORIENTATION_LANDSCAPE = 'landscape'
export const ORIENTATION_PORTRAIT = 'portrait'
const KNOWN = [ORIENTATION_AUTO, ORIENTATION_LANDSCAPE, ORIENTATION_PORTRAIT]

function storage() {
  try {
    return typeof window !== 'undefined' ? window.localStorage : null
  } catch {
    return null
  }
}

export function readStageOrientation() {
  try {
    const v = storage()?.getItem(KEY)
    return KNOWN.includes(v) ? v : ORIENTATION_AUTO
  } catch {
    return ORIENTATION_AUTO
  }
}

export function writeStageOrientation(value) {
  const next = KNOWN.includes(value) ? value : ORIENTATION_AUTO
  try {
    storage()?.setItem(KEY, next)
  } catch {
    /* ignore */
  }
  try {
    window.dispatchEvent(new CustomEvent('th:orientation', { detail: next }))
  } catch {
    /* ignore: non-DOM test env */
  }
  return next
}
