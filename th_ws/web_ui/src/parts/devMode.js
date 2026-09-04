// parts/devMode.js — WS-9X: whether the header shows the 開発モード pill.
// Historically driven only by the URL (?dev=1), which can't be toggled from
// the tablet. S-50's 開発モード tab now flips a localStorage flag too; the
// URL still wins so an existing ?dev=1 bookmark keeps working.
//
// This currently only drives Header.jsx's pill. Real dev-only affordances
// (safety-bypass toggles, per-warning suppression) are future work.

const KEY = 'th.devMode'

export function readDevMode() {
  let fromUrl = false
  try {
    fromUrl = new URLSearchParams(window.location.search).get('dev') === '1'
  } catch {
    /* ignore */
  }
  if (fromUrl) return true
  try {
    return window.localStorage.getItem(KEY) === '1'
  } catch {
    return false
  }
}

export const DEV_MODE_EVENT = 'th:devmode'

export function setDevMode(on) {
  try {
    window.localStorage.setItem(KEY, on ? '1' : '0')
  } catch {
    /* ignore */
  }
  // Header.jsx (via AppShell) listens for this so the 開発 pill updates
  // without a full reload.
  try {
    window.dispatchEvent(new CustomEvent(DEV_MODE_EVENT, { detail: readDevMode() }))
  } catch {
    /* ignore */
  }
}
