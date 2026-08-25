// shell/Windows.jsx — host for W-1..W-6 (DetailedDesign-webui.md §6).
//
// W-1 (fault/estop) and W-2 (carry) are implemented generically from
// /system/state + /safety/fault + generated/attributes.json, per §6.2
// ("the choices come from state + the attribute table; never hardcoded per
// screen", F-34/U-6). All three are derivable from fields already in this
// packet's interface contract (SystemState.mode/state/estop_ui/estop_hw,
// FaultStatus.active/fault_type), so they work with no screens at all.
//
// W-1 originally only fired for mode === 'ESTOP' (WP-UI-01, before
// /safety/fault was in the interface contract). WP-UI-02 generalizes it: a
// recoverable fault also opens W-1 while it holds the current mode in
// PAUSE (C-03 -- DetailedDesign-state.md §4.1), without changing mode. Both
// cases share the same window (per §6.2 "the same window turns into
// resume?", E-5) and the same resumeChoices(mode, attributes) lookup --
// only the body copy (what happened) and the resolved-condition differ.
//
// W-4 (confirm) is a generic host: shell/confirmWindow.js's context gives a
// screen a `mountNode` DOM ref (rendered here, inside #overlay so it shares
// the header's stacking context -- U-1) to portal its own content into.
// This lets the shutdown flow (WP-UI-02) and later screens' confirm/reject
// dialogs share this file's z-order guarantees without Windows.jsx needing
// to know anything about their content.
//
// W-3 (guide banner), W-5 (route-blocked) and W-6 (manual panel) still need
// screen-supplied content (a guide key, a jog target) no screen provides
// yet. Their DOM mount points are kept here (matching theme.css's
// #winGuide / .win.blocked / #jogWin) so screens can drive them later
// without touching the shell again, but nothing opens them yet.
//
// All of #overlay / #winGuide / #jogWin are direct children of #app; CSS
// `order` (theme.css) puts them in the right stacking position regardless
// of where in the JSX tree they're mounted, so they can all live in this
// one Fragment.
import { useEffect } from 'react'
import { resumeChoices, isW1Active } from './limits.js'
import { faultLabel } from '../i18n/faults.js'
import { reasonLabel } from '../i18n/reasons.js'
import {
  WIN_HIDE_LABEL, WIN_RESUME_ACK, WIN_RESUME_YES, WIN_RESUME_NO,
  WIN_ESTOP_TITLE, WIN_ESTOP_BODY, WIN_ESTOP_HINT, WIN_FAULT_TITLE, WIN_FAULT_HINT,
  WIN_CARRY_TITLE, WIN_CARRY_BODY, WIN_CARRY_HINT, WIN_CARRY_RELEASED,
  WIN_CARRY_RESUME, WIN_CARRY_DISMISS, WIN_CARRY_ESTOP_DISABLED,
} from '../i18n/states.js'

// C-06r's reject_reason_key for a UI estop press rejected during CARRY
// (DetailedDesign-state.md :764). Not /system/trigger's business -- the UI
// estop's own safety path is /safety/estop_ui, published unconditionally
// (DetailedDesign-safety.md §6.2: th_state must never sit on that path).
// state_manager subscribes to that same topic and, on a CARRY-time press,
// republishes this key through SystemState.last_reject_reason; Windows.jsx
// only reads it back out, it never calls a service to get it.
const ESTOP_DISABLED_IN_CARRY = 'estop_disabled_in_carry'

// estopDismissed / setEstopDismissed are lifted to AppShell so the header's
// "reopen" badge (outside this component, in the always-on-top layer) can
// control the same flag. It now doubles as "W-1 dismissed", covering both
// the ESTOP and fault-caused-PAUSE cases below.
export default function Windows({
  mode, stateName, estopUi, estopHw, fault, attributes, onTrigger,
  estopDismissed, setEstopDismissed, confirmOpen, onConfirmMount,
  lastRejectReason,
}) {
  const faultActive = !!fault?.active
  // Mutually exclusive: mode can't be both 'ESTOP' and something else at once.
  const w1IsEstop = mode === 'ESTOP'
  const w1Active = isW1Active(mode, stateName, faultActive)

  // A fresh W-1 occurrence always starts shown (§6.2: "non-display can be
  // done regardless of resolution", but each new fault re-opens it).
  useEffect(() => {
    if (w1Active) setEstopDismissed(false)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [w1IsEstop, stateName, faultActive, setEstopDismissed])

  const w1Open = w1Active && !estopDismissed
  const carryOpen = mode === 'CARRY'
  const hasModal = w1Open || carryOpen || confirmOpen

  // C-09f (ESTOP) / C-04,C-05 (fault-caused PAUSE): once the underlying
  // condition clears, the window becomes a resume confirmation instead of a
  // plain dismiss (§6.2 "the same window turns into 'resume?'", E-5).
  // resumeChoices(mode, ...) is correct for both: in the ESTOP case mode
  // literally is 'ESTOP'; in the fault-caused-PAUSE case mode never changed
  // from whatever it was (C-03's to_mode is '=').
  const w1Resolved = w1Active && (w1IsEstop ? (!estopUi && !estopHw) : !faultActive)
  const w1Resume = resumeChoices(mode, attributes)

  // C-11/C-12: CARRY only offers a way out once the physical button is
  // released.
  const carryHwReleased = mode === 'CARRY' && !estopHw

  const fire = (trigger) => onTrigger?.(trigger)

  return (
    <>
      <div id="overlay" className={hasModal ? 'has-modal' : ''}>
        <div className="backdrop" />

        {w1Active && (
          <div className={`win fault ${w1Open ? 'show' : ''} ${w1Resolved ? 'resolved' : ''}`}>
            <header>{w1IsEstop ? WIN_ESTOP_TITLE : WIN_FAULT_TITLE}</header>
            <div className="bodyw">
              <p>{w1IsEstop ? WIN_ESTOP_BODY : faultLabel(fault?.fault_type)}</p>
              {!w1Resolved && <p className="hint mt">{w1IsEstop ? WIN_ESTOP_HINT : WIN_FAULT_HINT}</p>}
            </div>
            <footer>
              {w1Resolved && w1Resume === 'ack_only' && (
                <button type="button" className="btn primary" onClick={() => fire('ui.resume_ack')}>
                  {WIN_RESUME_ACK}
                </button>
              )}
              {w1Resolved && w1Resume === 'yes_no' && (
                <>
                  <button type="button" className="btn" onClick={() => fire('ui.resume_no')}>
                    {WIN_RESUME_NO}
                  </button>
                  <button type="button" className="btn primary" onClick={() => fire('ui.resume_yes')}>
                    {WIN_RESUME_YES}
                  </button>
                </>
              )}
              <button type="button" className="btn" onClick={() => setEstopDismissed(true)}>
                {WIN_HIDE_LABEL}
              </button>
            </footer>
          </div>
        )}

        {carryOpen && (
          <div className="win carry show">
            <header>{WIN_CARRY_TITLE}</header>
            <div className="bodyw">
              <p>{WIN_CARRY_BODY}</p>
              <p className="hint mt">{WIN_CARRY_HINT}</p>
              {/* C-2: the UI estop button stays visible and clickable in
                  CARRY (it must not be hidden), but it can't do anything
                  while the drive is already cut -- say so plainly. */}
              <p className="hint mt">{WIN_CARRY_ESTOP_DISABLED}</p>
              {lastRejectReason === ESTOP_DISABLED_IN_CARRY && (
                <p className="hint mt">{reasonLabel(lastRejectReason)}</p>
              )}
              {carryHwReleased && (
                <div className="mt">
                  <div className="row"><span className="pill ok">{WIN_CARRY_RELEASED}</span></div>
                  <button type="button" className="btn primary wide mt" onClick={() => fire('ui.carry_resume')}>
                    {WIN_CARRY_RESUME}
                  </button>
                  <button type="button" className="btn wide mt" onClick={() => fire('ui.finish')}>
                    {WIN_CARRY_DISMISS}
                  </button>
                </div>
              )}
            </div>
          </div>
        )}

        {/* W-4 (confirm): generic host. The screen that called
            useConfirmWindow().open() portals its own header/bodyw/footer
            into this node (shell/confirmWindow.js) -- the ref callback
            here is how AppShell learns the mount node exists. */}
        {confirmOpen && <div className="win confirm show" ref={onConfirmMount} />}

        {/* W-5 (route-blocked): opened by screens (WP-UI-03+) */}
      </div>

      {/* W-3 guide banner: mount point only, no screen supplies guide{key} yet */}
      <div id="winGuide" />

      {/* W-6 manual panel: mount point only, no screen supplies a jog target yet */}
      <div id="jogWin" />
    </>
  )
}
