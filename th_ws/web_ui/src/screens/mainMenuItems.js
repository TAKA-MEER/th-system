// screens/mainMenuItems.js — pure core for S-01's mode-selection buttons
// (DetailedDesign-wp1.md WP-UI-02 §4.1).
//
// DetailedDesign-wp1.md WP-UI-02: whether a button is pressable is derived
// from mode_entry.yaml plus preconditions; a screen must not hardcode that
// logic itself. menuItems() is that lookup: it takes the generated
// mode_entry.json (built from th_state/config/mode_entry.yaml by
// scripts/gen_mode_entry.py, same flow as generated/attributes.json) plus
// the live SystemState, and returns
// which of the 9 mode-entry buttons S-01 shows are enabled right now, and
// why not when they aren't.
//
// This is a *display convenience*, not the authority (DetailedDesign-webui.md
// §4.1: "the UI may pre-decide for convenience, but it is not the
// authority" -- th_state's guards are what actually accept or reject
// /system/trigger's ui.enter_mode). preconds (routeCount) covers what the
// screen can see; the rest isn't gated here, so a click that looked enabled
// here can still come back rejected; the caller must show reject_reason_key
// from the response in that case (i18n/reasons.js), the same way it shows the
// reasonKey this module hands back for the checks it *can* make locally.

// The 9 buttons S-01 shows (Spec-webui.md §3.2 の階層: 試験場 ▸ ／ 走行 ▸ ／
// 点検と校正 ▸。ライン誘導・電子リードは 2026-11-27 廃止（D7）のため無い）。
// PANEL_NAV / SUMMON / HOME_NAV は IDLE から届くが S-21 内の ui.goto で行くため
// S-01 のボタンにはしない（DetailedDesign-names.md §8.1）。
//
// AT_HOME (2026-09-08, Spec-modes.md §2.3) IS an S-01 button: it is the day-of
// entry point (「試験（当日）」). It used to be a special-cased local flag in
// main.jsx because S-21 had no mode of its own; now that the mode exists the
// button is an ordinary ui.enter_mode like every other one, and the screen is
// derived from /system/state instead of a UI-local flag.
export const MENU_MODES = [
  'FOLLOW', 'MANUAL', 'TEACH_FOLLOW', 'TEACH_MANUAL', 'REPLAY',
  'PREP', 'AT_HOME',
  'OPCHECK', 'CALIB',
]

export const MENU_GROUPS = [
  { key: 'move', modes: ['FOLLOW', 'MANUAL', 'TEACH_FOLLOW', 'TEACH_MANUAL', 'REPLAY'] },
  { key: 'field', modes: ['PREP', 'AT_HOME'] },
  { key: 'maint', modes: ['OPCHECK', 'CALIB'] },
]

/* S-01 のタイル階層（Spec-webui.md §3.2）。一番上は 5 タイル、下の階層は
   「試験場▸」「走行▸」「点検と校正▸」。自律測定・設定は一番上のタイル。 */
export const TOP_TILES = [
  { key: 'field', kind: 'group', icon: 'field', size: 'l' },
  { key: 'drive', kind: 'group', icon: 'drive', size: '' },
  { key: 'maint', kind: 'group', icon: 'maint', size: '' },
  { key: 'auto', kind: 'auto', icon: 'measure', size: 's' },
  { key: 'settings', kind: 'settings', icon: 'gear', size: 's' },
]

export const GROUP_TILES = {
  field: [
    { key: 'AT_HOME', kind: 'mode', mode: 'AT_HOME', icon: 'day', size: 'l' },
    { key: 'PREP', kind: 'mode', mode: 'PREP', icon: 'prep', size: '' },
  ],
  drive: [
    { key: 'FOLLOW', kind: 'mode', mode: 'FOLLOW', icon: 'follow', size: 'l' },
    { key: 'MANUAL', kind: 'mode', mode: 'MANUAL', icon: 'manual', size: 'l' },
    { key: 'REPLAY', kind: 'mode', mode: 'REPLAY', icon: 'replay', size: '' },
    { key: 'TEACH_FOLLOW', kind: 'mode', mode: 'TEACH_FOLLOW', icon: 'teach', size: '' },
    { key: 'TEACH_MANUAL', kind: 'mode', mode: 'TEACH_MANUAL', icon: 'teach', size: '' },
  ],
  maint: [
    { key: 'OPCHECK', kind: 'mode', mode: 'OPCHECK', icon: 'check', size: 'l' },
    { key: 'CALIB', kind: 'mode', mode: 'CALIB', icon: 'calib', size: '' },
  ],
}

/* タイルの大きさの順位は仮（モックアップの既定。Spec-webui.md §3.2）。
   運用で使う頻度を見て決める。大きさはこの 1 か所の表だけが持つ。 */
export const TILE_SIZES = Object.fromEntries([
  ...TOP_TILES.map((t) => [t.key, t.size]),
  ...Object.values(GROUP_TILES).flatMap((tiles) => tiles.map((t) => [t.key, t.size])),
])

export const TILE_ICONS = Object.fromEntries([
  ...TOP_TILES.map((t) => [t.key, t.icon]),
  ...Object.values(GROUP_TILES).flatMap((tiles) => tiles.map((t) => [t.key, t.icon])),
])

// attributes.yaml marks PREP's needs_tracker as "required", but per
// DetailedDesign-state.md §8.2's footnote ※※ that's only true once inside
// PREP's REGISTER sub-state -- not on entry. Gating the S-01 button on
// tracker_enabled would block PREP for a reason that doesn't apply yet, so
// it's excluded from that particular local check (the server-side guard
// still applies once inside PREP; this module only affects display).
const TRACKER_GATE_EXEMPT = new Set(['PREP'])

// menuItems(systemState, modeEntry, attributes, preconds) -> [{mode, enabled, reasonKey}]
//
// preconds は画面が別経路で知る前提条件（1b-11 SG-B13。DetailedDesign-transit.md
// §0.4）。{ routeCount: 経路の本数（/route/catalog） }。undefined（＝まだ分からない）
// ならその前提では止めない（サーバ側のガードが正本で、押せば理由付きで拒否される）。
// ライン誘導・電子リードは 2026-11-27 廃止（D7）。
export function menuItems(systemState, modeEntry, attributes, preconds = {}) {
  const mode = systemState?.mode ?? null

  // M-1: nothing pressable while starting up. Also the fail-safe default
  // when mode is unknown (null) -- treat exactly like INIT, never like "no
  // restriction" (DetailedDesign-wp1.md WP-UI-01 §6.2).
  if (!mode || mode === 'INIT') {
    return MENU_MODES.map((m) => ({ mode: m, enabled: false, reasonKey: null }))
  }

  const allowed = new Set(modeEntry?.[mode] ?? [])
  const routeCount = preconds?.routeCount

  return MENU_MODES.map((m) => {
    if (!allowed.has(m)) {
      return { mode: m, enabled: false, reasonKey: 'mode_entry_denied' }
    }
    // 1b-11 SG-B13: 教示再生は経路 0 本で押せない（no_route_recorded）。
    // 本数がまだ分からない（undefined）間は止めない。
    if (m === 'REPLAY' && typeof routeCount === 'number' && routeCount === 0) {
      return { mode: m, enabled: false, reasonKey: 'no_route_recorded' }
    }
    const attrs = attributes?.[m]
    if (attrs?.needs_tracker === 'required' && !TRACKER_GATE_EXEMPT.has(m) && !systemState.tracker_enabled) {
      return { mode: m, enabled: false, reasonKey: 'tracker_disabled' }
    }
    return { mode: m, enabled: true, reasonKey: null }
  })
}

// 画面の無いモード（MODE_TO_SCREEN に載らないモード）で S-01 が表示されているとき、
// ui.finish で IDLE に戻す導線を出すべきかを返す。IDLE / INIT / null では出さない
// （出す意味が無く、押し間違いで無駄な trigger を投げるだけになる）。
// S-01 が出ている時点で MODE_TO_SCREEN に無いモードだと確定しているため、
// 「IDLE / INIT / null 以外なら true」の単純規則で十分。
export function needsFinishEscape(mode) {
  return mode != null && mode !== 'IDLE' && mode !== 'INIT'
}
