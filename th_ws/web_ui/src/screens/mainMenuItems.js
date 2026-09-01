// screens/mainMenuItems.js — pure core for S-01's mode-selection buttons
// (DetailedDesign-wp1.md WP-UI-02 §4.1).
//
// DetailedDesign-wp1.md WP-UI-02: whether a button is pressable is derived
// from mode_entry.yaml plus preconditions; a screen must not hardcode that
// logic itself. menuItems() is that lookup: it takes the generated
// mode_entry.json (built from th_state/config/mode_entry.yaml by
// scripts/gen_mode_entry.py, same flow as generated/attributes.json) plus
// the live SystemState, and returns
// which of the 10 mode-entry buttons S-01 shows are enabled right now, and
// why not when they aren't.
//
// This is a *display convenience*, not the authority (DetailedDesign-webui.md
// §4.1: "the UI may pre-decide for convenience, but it is not the
// authority" -- th_state's mode_entry_allowed guard is what actually
// accepts or rejects /system/trigger's ui.enter_mode). It intentionally
// does not attempt every precondition mode_entry_allowed checks server-side
// (route existence, device connectivity, ...) -- those aren't visible from
// /system/state, so a click that looked enabled here can still come back
// rejected; the caller must show reject_reason_key from the response in
// that case (i18n/reasons.js), the same way it shows the reasonKey this
// module hands back for the checks it *can* make locally.

// The 10 buttons S-01 shows (Spec-webui.md §3.2's move(7) + field(PREP) +
// maintenance(OPCHECK/CALIB) groups). PANEL_NAV / SUMMON / HOME_NAV are
// also reachable from IDLE per mode_entry.yaml, but that's via ui.goto from
// inside S-21 (the test screen), not an S-01 button
// (DetailedDesign-names.md §8.1); they're deliberately left out of this list.
export const MENU_MODES = [
  'FOLLOW', 'MANUAL', 'TEACH_FOLLOW', 'TEACH_MANUAL', 'REPLAY', 'LINE', 'LEASH',
  'PREP',
  'OPCHECK', 'CALIB',
]

export const MENU_GROUPS = [
  { key: 'move', modes: ['FOLLOW', 'MANUAL', 'TEACH_FOLLOW', 'TEACH_MANUAL', 'REPLAY', 'LINE', 'LEASH'] },
  { key: 'field', modes: ['PREP'] },
  { key: 'maint', modes: ['OPCHECK', 'CALIB'] },
]

// attributes.yaml marks PREP's needs_tracker as "required", but per
// DetailedDesign-state.md §8.2's footnote ※※ that's only true once inside
// PREP's REGISTER sub-state -- not on entry. Gating the S-01 button on
// tracker_enabled would block PREP for a reason that doesn't apply yet, so
// it's excluded from that particular local check (the server-side guard
// still applies once inside PREP; this module only affects display).
const TRACKER_GATE_EXEMPT = new Set(['PREP'])

// menuItems(systemState, modeEntry, attributes) -> [{mode, enabled, reasonKey}]
export function menuItems(systemState, modeEntry, attributes) {
  const mode = systemState?.mode ?? null

  // M-1: nothing pressable while starting up. Also the fail-safe default
  // when mode is unknown (null) -- treat exactly like INIT, never like "no
  // restriction" (DetailedDesign-wp1.md WP-UI-01 §6.2).
  if (!mode || mode === 'INIT') {
    return MENU_MODES.map((m) => ({ mode: m, enabled: false, reasonKey: null }))
  }

  const allowed = new Set(modeEntry?.[mode] ?? [])

  return MENU_MODES.map((m) => {
    if (!allowed.has(m)) {
      return { mode: m, enabled: false, reasonKey: 'mode_entry_denied' }
    }
    const attrs = attributes?.[m]
    if (attrs?.needs_tracker === 'required' && !TRACKER_GATE_EXEMPT.has(m) && !systemState.tracker_enabled) {
      return { mode: m, enabled: false, reasonKey: 'tracker_disabled' }
    }
    return { mode: m, enabled: true, reasonKey: null }
  })
}
