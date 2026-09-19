// shell/Header.jsx — logo / screen name / mode / zone / fault / estop.
// Always the top layer (z-index 100, theme.css #hdr) so the estop button is
// reachable no matter what window is open (DetailedDesign-webui.md §2.1,
// U-1). Never folds the screen name, mode, or the estop button — only the
// project name / zone pill / screen-name tail may shrink (§2.5, U-10).
import { LOGO_DATA_URI } from './logo.js'
import { modeLabel, MODE_UNKNOWN_LABEL } from '../i18n/modes.js'
import { faultLabel } from '../i18n/faults.js'
import {
  CONNECTED_LABEL, DISCONNECTED_LABEL, ESTOP_ACTIVE_LABEL, ESTOP_BUTTON_LABEL,
  PROJECT_NAME_JA, REOPEN_WINDOW_LABEL, ZONE_LABELS, DEV_MODE_LABEL, AP_SPOF_NOTE,
} from '../i18n/states.js'

export default function Header({
  screenName,
  mode,
  stale,
  zone,
  fault,
  devMode,
  estopEngaged,
  onEstopClick,
  faultBadgeVisible,
  onFaultBadgeClick,
}) {
  const modeText = stale ? MODE_UNKNOWN_LABEL : modeLabel(mode)
  const zoneLabels = zone ? ZONE_LABELS[zone] : null

  // C-2 (DetailedDesign-wp1.md WP-CARRY-01 §4.3): in CARRY, C-06r rejects
  // every UI estop press server-side, so `.engaged` (the look a *genuinely
  // accepted* press gets, theme.css #estopBtn.engaged) must not appear --
  // it would visually claim the press worked. The button must also not be
  // `disabled` or hidden (§6.2's safety path -- /safety/estop_ui -- keeps
  // publishing regardless, see AppShell.jsx's publishEstopUi); it only gets
  // a distinct "won't do anything" look via this class.
  const estopClass = mode === 'CARRY' ? 'disabled-in-carry' : (estopEngaged ? 'engaged' : '')

  // Priority: link loss > estop > a recoverable fault (WP-UI-02 generalizes
  // W-1 to fault-caused PAUSE; the header follows the same priority so it
  // never claims normal while a fault window is actually open) > normal.
  let faultText = CONNECTED_LABEL
  let faultDotClass = ''
  if (stale) {
    faultText = DISCONNECTED_LABEL
    faultDotClass = 'ng'
  } else if (mode === 'ESTOP') {
    faultText = ESTOP_ACTIVE_LABEL
    faultDotClass = 'ng'
  } else if (fault?.active) {
    faultText = faultLabel(fault.fault_type)
    faultDotClass = fault.severity === 'CRITICAL' ? 'ng' : 'warn'
  }

  return (
    <header id="hdr">
      <div className="row1">
        <div id="logo"><img src={LOGO_DATA_URI} alt="" /></div>
        <div className="proj">MIRS2602-<b>{PROJECT_NAME_JA}</b></div>
        <div className="spacer" />
        <div id="screenName">{screenName ?? ''}</div>
        {zoneLabels && (
          <div id="zonePill" className="show">
            <span className="z-long">{zoneLabels.long}</span>
            <span className="z-short">{zoneLabels.short}</span>
          </div>
        )}
        {devMode && (
          <div id="devPill" className="show">
            <span className="d-long">{DEV_MODE_LABEL.long}</span>
            <span className="d-short">{DEV_MODE_LABEL.short}</span>
          </div>
        )}
        <div id="modePill" className={stale ? 'unknown' : ''}>{modeText}</div>
      </div>
      <div className="row2">
        <div id="faultBox" title={AP_SPOF_NOTE}>
          <span id="faultDot" className={faultDotClass} />
          <span id="faultTx">{faultText}</span>
          <span className="spacer grow" />
          {faultBadgeVisible && (
            <button id="faultBadge" className="show" onClick={onFaultBadgeClick}>
              {REOPEN_WINDOW_LABEL}
            </button>
          )}
        </div>
        <button
          id="estopBtn"
          className={estopClass}
          onClick={onEstopClick}
        >
          {ESTOP_BUTTON_LABEL}
        </button>
      </div>
    </header>
  )
}
