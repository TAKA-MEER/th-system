// parts/helpPrefs.js — ヘルプ表示の出す・出さない（Spec-webui §3.3.6）。
// 端末ごと（localStorage）。読めないときは「出す」として動く。純関数 + localStorage。
const KEY = 'th.helpShown'

export function readHelpEnabled() {
  try {
    const v = window.localStorage.getItem(KEY)
    return v == null ? true : v === '1'
  } catch {
    return true
  }
}

export function writeHelpEnabled(on) {
  try {
    window.localStorage.setItem(KEY, on ? '1' : '0')
  } catch {
    // 読めない端末では何もしない（表示は「出す」）。
  }
}
