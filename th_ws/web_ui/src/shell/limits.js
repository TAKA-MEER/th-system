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

// isW1Active(mode, stateName, faultActive, pauseReason) -> bool
//
// W-1 (the fault/estop window, DetailedDesign-webui.md §6) fires in two
// cases that share one window (§6.2 "the same window turns into resume?",
// E-5): mode itself is 'ESTOP' (C-06a/C-06b), or a recoverable fault has
// pushed the *current* mode into PAUSE without changing it (C-03 --
// DetailedDesign-state.md §4.1, WP-UI-01 §11 "W-1 generalization").
//
// 1b-1 SG-A12: a third case. When the operating terminal leaves mid-drive,
// state_manager parks the mode in PAUSE with pause_reason === 'presence_lost'
// (C-16, Spec-safety.md §6.2.2). There is no fault, so faultActive never
// fires -- and the returning terminal missed the one-shot open_window effect
// entirely. The reason persists in /system/state, so the window is derived
// from state and survives a page reload.
//
// WS-9Z (2026-09-09): call this with the *raw* live faultActive only from
// shell/AppShell.jsx, which latches the result across a fault that clears
// before the next render (safety_monitor's RECOVERABLE faults often clear
// within ~100ms; state_manager's C-03 leaves mode/state parked in PAUSE
// regardless, pending an explicit ui.resume_* this same window offers --
// see AppShell.jsx's faultPauseSeen for why the latch exists and what broke
// without it). AppShell passes the latched result down to Windows.jsx as
// the `w1Active` prop; nothing else should call this function directly, or
// the same real-fault-clears-before-render race reopens (it did once
// already: this function used to be called independently from both files,
// "computed twice ... risking drift" per this comment's own prior wording,
// and only one of the two copies got the fix).
//
// 1b-1 (PREP/RETURN の帰還の一時停止): 第4のケース。PREP の `PREP/PAUSE` は
// 「RETURN（自動帰還）の一時停止」以外に存在せず（ジョグ・回復フォルト・端末離脱・
// 非常停止からの「戻る」で入る）、出る手段は W-1 の「はい」（T-PREP-16→RETURN）／
// 「いいえ」（T-PREP-17→MAPPING）だけ。PREP の run_state は MAPPING で、走行ボタン
// （ui.run）は inert のため、他モードのように「走行」で戻る迂回路が無い。
// pause_reason は jog・空（非常停止から戻った場合）・fault などになるので理由を問わず出す。
// ただしジョグ中（jogActive）は W-1 で手動操作パネルを覆わない。離したら出る。
// attributes.yaml で表せる性質ではない（run_state はあるが走行ボタンが効かない）
// ので、PREP を明示する。他モードのジョグ PAUSE は走行ボタンで戻れるので対象外。
export function isW1Active(mode, stateName, faultActive, pauseReason = '', jogActive = false) {
  return mode === 'ESTOP' || (stateName === 'PAUSE' && !!faultActive)
    || (stateName === 'PAUSE' && pauseReason === 'presence_lost')
    || (mode === 'PREP' && stateName === 'PAUSE' && !jogActive)
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

// showEstopRelease(uiEngaged, mode, estopUi) -> bool
//
// 画面下端の「非常停止を解除」バーを出すか（Spec-safety.md §4.2「解除の配置」。SG-C10）。
// このバーが解除できるのは UI 非常停止（/safety/estop_ui）だけ。重大フォルトで入った
// ESTOP は押しても何も起きない（解除は W-1 の「確認」／「元のモードに戻る」）ので出さない。
//   - 自分が押した（uiEngaged）あいだ
//   - ESTOP で、UI 非常停止が実際に保持されているあいだ（再読込・別の端末が押した場合）
// CARRY は従来どおり、押した直後（uiEngaged）には出る。
export function showEstopRelease(uiEngaged, mode, estopUi) {
  return !!uiEngaged || (mode === 'ESTOP' && !!estopUi)
}

// headerTone(mode, devMode, stale) -> 'estop' | 'carry' | 'dev' | 'normal'
//
// ヘッダの色（SG-C3）。Spec-webui.md §8「非常停止＝赤／解除待ち＝紫」、§5「開発モード中は
// ヘッダの色を変える」。優先は安全側から: 非常停止 > 手押し > 開発モード。
// 通信が古い（stale）ときは機体の状態が分からないので色で断言しない（normal）。
export function headerTone(mode, devMode, stale = false) {
  if (stale) return devMode ? 'dev' : 'normal'
  if (mode === 'ESTOP') return 'estop'
  if (mode === 'CARRY') return 'carry'
  if (devMode) return 'dev'
  return 'normal'
}

// opButtonTone(slot, blue) -> 'current' | 'confirm' | 'outline'
//
// Spec-webui.md §3.3.1（U-17）の色の表:
//   青（塗り）  = いまの状態 … 停止／確認／走行のうち、いま該当している 1 つだけ
//   緑（塗り）  = 押すと確定する動作 … 保存
//   枠のみ      = 押せるが、いまの状態ではない … それ以外（手動もここ）
// slot は 'stop'|'check'|'run'|'save'|'manual'、blue は stateToBlueButton() の結果。
// 色の決め方はここ 1 か所。OperationCard.jsx は返り値をクラスにするだけ
// （画面ごとに決めさせない）。
export function opButtonTone(slot, blue) {
  if (slot === 'save') return 'confirm'
  if ((slot === 'stop' || slot === 'check' || slot === 'run') && blue === slot) return 'current'
  return 'outline'
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
// 2026-09-04 追修正（実機「基本的に常に出ていて邪魔。一部表示を隠してしまう」）:
//   1. **機体が走るはずのときだけ**出す。走らせていない間は速度指令が無いのが
//      normal で、リミッタは常に ZERO_STALE を返す。それを「止まっている理由」
//      として出すと待機中ずっと帯が出たままになる。
//      判定は attributes.yaml の run_state（REPLAY→RUN / TEACH_MANUAL→REC）。
//   2. ZERO_STALE は理由から外した。操作者に打つ手が無いうえ、RUN に入った直後
//      に一瞬出て点滅する。ノードが死んだ場合は経路状態やフォルトが先に出る。
//
// stopReason(state, limiterStatus, fault, attributes)
//   -> 'estop'|'fault'|'presence'|'obstacle'|'uncalibrated'|'blocked'|null
// null は「出さない」。厳しい順に判定する。
export function stopReason(state, limiterStatus, fault, attributes) {
  if (!state) return null
  // 2026-09-09 実機フィードバック「動かない。失敗理由も出ない」: venue_navigator
  // が Nav2 の経路計画そのものに失敗すると mode はそのまま state だけ BLOCKED
  // になる（T-PNAV-04/T-HNAV-04、evt.blocked）。この BLOCKED は
  // attributes[mode].run_state と一致しない（run_state は 'NAV'）ため、下の
  // 走行状態ゲートで弾かれて何も表示されなかった——機体は動こうとして失敗して
  // いるのに、画面には「移動中」の表示すら出ない無反応に見えていた。
  // BLOCKED はどのモードでも「経路計画の失敗」以外の意味を持たないので、
  // ゲートより先にモード非依存で見る。
  if (state.state === 'BLOCKED') return 'blocked'
  // 走るはずでないなら何も出さない（待機・経路選択・準備完了・一時停止中）。
  const runState = attributes?.[state.mode]?.run_state
  if (!runState || state.state !== runState) return null

  // 非常停止（ESTOP / CARRY）の分岐はここに要らない。どちらも attributes.yaml の
  // run_state が null なので、上の走行状態ゲートが先に null を返す。W-1 の窓が
  // 別に説明もする。書いても到達しない＝壊しても誰も気づけないコードになるので
  // 置かない（変異チェックで実際に素通りすることを確認した）。
  // フォルトで止まっているとき（W-1 の窓が別に出るので、ここでは種別だけ返す）
  if (fault?.active) return 'fault'
  // 在席未確認: derive_limits が 0 台にフェイルセーフした形
  // （zone=NA かつ speed_limit=stop）。WS-9R 以降ここに落ちるのは
  // 「アプリが前面に無い」か「接続が切れている」ときだけ。
  if (state.zone === 'NA' && state.speed_limit === 'stop') return 'presence'
  if (!limiterStatus) return null
  if (limiterStatus.action === 'STOP') return 'obstacle'
  if (limiterStatus.action === 'BLOCKED_UNCALIBRATED') return 'uncalibrated'
  return null
}
