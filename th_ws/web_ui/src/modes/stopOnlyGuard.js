// modes/stopOnlyGuard.js — WebUI 側の「地図・設定値の操作ができるか」判定（補助）。
//
// Spec.md SD-9（2026-10-07 決定）の UI 側コピー。IDLE と PREP の機体が自分で
// 走らない状態（MAPPING／REGISTER／EDIT／SAVED）で、ジョグ中でないときだけ
// true。サーバ側（th_config_manager/stop_only_guard.py）が正本で、ここが緩くても
// サーバが拒否する（安全は server が握る）。未受信・stale（useSystemState の
// 500ms 途絶）の間は押せない（安全側）。
export const STOP_ONLY_PREP_STATES = ['MAPPING', 'REGISTER', 'EDIT', 'SAVED']

export function mapOrParamOpAllowed(state, stale) {
  if (stale || state == null) return false
  if (state.jog_active) return false
  if (state.mode === 'IDLE') return true
  if (state.mode === 'PREP' && STOP_ONLY_PREP_STATES.includes(state.state)) return true
  return false
}

// 当日の「保存した会場地図を開く」。試験画面は待機場所の待機状態（AT_HOME／IDLE_H）で
// 開くので、停止中の許可に加えてここだけ許す（Spec.md SD-9 の 2026-10-09 の例外。
// サーバ側 venue_open_allows と同じ）。
export function venueOpenAllowed(state, stale) {
  if (stale || state == null) return false
  if (state.jog_active) return false
  if (state.mode === 'AT_HOME' && state.state === 'IDLE_H') return true
  return mapOrParamOpAllowed(state, stale)
}
