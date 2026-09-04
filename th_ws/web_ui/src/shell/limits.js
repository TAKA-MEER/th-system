// shell/limits.js — display-only helpers for the shell.
// These decide what the UI *shows*; they are never the source of truth for
// what is *allowed* — th_state (transitions.yaml) is authoritative and can
// still reject a call the UI thought looked fine (DetailedDesign-webui.md
// §4.1: "the UI may pre-decide for convenience, but it is not the
// authority").

// resumeChoices(mode, attributes) -> 'yes_no' | 'ack_only' | 'none'
// Reads generated/attributes.json (built from th_state/config/attributes.yaml
// by scripts/gen_attributes.py). Never hardcode per-mode choices here (U-6).
export function resumeChoices(mode, attributes) {
  return attributes?.[mode]?.resume ?? 'none'
}

// isW1Active(mode, stateName, faultActive) -> bool
//
// W-1 (the fault/estop window, DetailedDesign-webui.md §6) fires in two
// cases that share one window (§6.2 "the same window turns into resume?",
// E-5): mode itself is 'ESTOP' (C-06a/C-06b), or a recoverable fault has
// pushed the *current* mode into PAUSE without changing it (C-03 --
// DetailedDesign-state.md §4.1, WP-UI-01 §11 "W-1 generalization"). Shared by
// shell/Windows.jsx (renders the window) and shell/AppShell.jsx (the
// header's "reopen" badge needs to know the same thing), so it lives here
// rather than being computed twice and risking drift.
export function isW1Active(mode, stateName, faultActive) {
  return mode === 'ESTOP' || (stateName === 'PAUSE' && !!faultActive)
}

// stateToBlueButton(mode, stateName, attributes) -> 'stop' | 'run' | 'check' | null
//
// §4.3 (U-17): exactly one button in the operation card is blue, and it
// reflects "what is happening now" — stopped / confirming-or-selecting /
// running. attributes.json gives us the two states with unambiguous
// meaning (resume_state = the paused/stopped state, run_state = the
// driving state); anything else is treated as an in-between
// selection/setup phase, which the "check" (confirm) slot represents.
export function stateToBlueButton(mode, stateName, attributes) {
  if (!stateName || stateName === 'NONE') return null
  const attrs = attributes?.[mode]
  if (!attrs) return null
  if (stateName === attrs.run_state) return 'run'
  if (stateName === attrs.resume_state) return 'stop'
  return 'check'
}

// operationCardLayout(mode, attributes) -> { stop, check, run, save, manual }
//
// Only `stop`, `run` and `save` can be derived purely from mode + attributes.
// `check` and `manual` depend on which screen is showing the card (e.g.
// S-10 has "confirm", S-11 does not; S-14 has "manual", S-10 does not — see
// DetailedDesign-webui.md §4.2), which isn't known until screens exist
// (WP-UI-02+). Screens should treat this as a starting point and override
// `check` / `manual` explicitly.
// obstacleWarning(limiterStatus) -> { level: 'none'|'warn'|'stop', distance_m } | null
//
// /safety/limiter_status から UI 表示用の障害物警告を返す（WP-TRANSIT-01 §4.1）。
// action 文字列（"PASS" / "CLAMP" / "STOP"）で閾値判定は obstacle_limiter 側が
// 持っており、UI は nearest_obstacle_m をそのまま表示するだけ。
// 未受信（null）のときは安全側として null を返す——"障害物なし" と表示しない
//（DetailedDesign-wp3.md §6.2 / §6.2 の fail-safe）。
export function obstacleWarning(limiterStatus) {
  if (!limiterStatus) return null
  const action = limiterStatus.action
  const level = action === 'STOP' ? 'stop' : action === 'CLAMP' ? 'warn' : 'none'
  return { level, distance_m: limiterStatus.nearest_obstacle_m }
}

export function operationCardLayout(mode, attributes) {
  const attrs = attributes?.[mode]
  if (!attrs) return { stop: false, check: false, run: false, save: false, manual: false }
  return {
    stop: true,
    check: false,
    run: attrs.run_state != null,
    save: !!attrs.has_record,
    manual: false,
  }
}

// ── WS-9R (2026-09-04): 止まっている理由 ──────────────────────────────
//
// 実機フィードバック「謎の一時停止が発生する。画面をスクロールしたりすると
// 復帰する」。速度上限が 0 に落ちても、フォルトでも一時停止でもないので画面に
// 何も出ず、操作者に理由が分からなかった。
//
// stopReason(state, limiterStatus) -> 'estop'|'fault'|'presence'|'obstacle'|'stale'|null
// null は「止められていない」。厳しい順に判定する（複数当てはまるときは
// 操作者が最初に直すべきものを返す）。
export function stopReason(state, limiterStatus, fault) {
  if (!state) return null
  if (state.mode === 'ESTOP' || state.mode === 'CARRY') return 'estop'
  // フォルトで止まっているとき（W-1 の窓が別に出るので、ここでは種別だけ返す）
  if (fault?.active) return 'fault'
  // 在席未確認: derive_limits が 0 台にフェイルセーフした形
  // （zone=NA かつ speed_limit=stop）。WS-9R 以降ここに落ちるのは
  // 「アプリが前面に無い」か「接続が切れている」ときだけ。
  if (state.zone === 'NA' && state.speed_limit === 'stop') return 'presence'
  if (!limiterStatus) return null
  if (limiterStatus.action === 'STOP') return 'obstacle'
  if (limiterStatus.action === 'ZERO_STALE') return 'stale'
  if (limiterStatus.action === 'BLOCKED_UNCALIBRATED') return 'stale'
  return null
}
