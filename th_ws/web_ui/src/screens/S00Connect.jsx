// screens/S00Connect.jsx — S-00, the connection-check screen (SCREEN_NAMES.S00
// in i18n/screens.js; DetailedDesign-wp1.md WP-UI-02).
//
// DetailedDesign-state.md §12.2's four required checks (ESP32 feedback,
// ESP32 keepalive loopback, RaspberryPi4/LiDAR, PC nodes) are evaluated by
// connectivity_checker (WP-STATE-03) entirely server-side; it only ever
// surfaces the *result* as evt.link_ok, which folds into SystemState.mode
// leaving 'INIT' (DetailedDesign-state.md §12.1 step 7, T-INIT-01 is the
// only way out of INIT). This packet's interface contract
// (DetailedDesign-wp1.md WP-UI-02 §3.1: "same as WP-UI-01, nothing added")
// gives S-00 no other signal to work from, so it cannot show a genuine
// per-item breakdown -- the four rows below necessarily share one status.
// See this packet's completion report for why, and what a future packet
// would need to add to make the rows independent.
//
// §6.2 fail-safe: while /system/state hasn't arrived yet (or has gone
// stale), or mode is still 'INIT', there is nothing to advance to -- no
// "advance" button is rendered at all (not just disabled).
import { useSystemState } from '../ros/useSystemState.js'
import {
  S00_CHECK_TITLE, S00_COL_DEVICE, S00_COL_REQ, S00_COL_STATUS, S00_REQUIRED,
  S00_MONITOR, S00_ITEMS, S00_STATUS_CHECKING, S00_STATUS_OK, S00_AP_LABEL,
  S00_AP_NOTE, S00_OVERALL_TITLE, S00_READY, S00_CHECKING, S00_ADVANCE,
} from '../i18n/screens.js'
import { DISCONNECTED_LABEL } from '../i18n/states.js'

export default function S00Connect({ onAdvance }) {
  const { state, stale } = useSystemState()
  const mode = state?.mode ?? null
  const ready = !stale && mode != null && mode !== 'INIT'

  return (
    <div className="screen" id="s00">
      <div className="card">
        <h3>{S00_CHECK_TITLE}</h3>
        <table className="lst">
          <tbody>
            <tr>
              <th>{S00_COL_DEVICE}</th>
              <th>{S00_COL_REQ}</th>
              <th className="r">{S00_COL_STATUS}</th>
            </tr>
            {S00_ITEMS.map((item) => (
              <tr key={item.key}>
                <td>{item.label}</td>
                <td><span className="pill">{S00_REQUIRED}</span></td>
                <td className={`r ${ready ? 'tone-ok' : 'tone-warn'}`}>
                  {ready ? S00_STATUS_OK : S00_STATUS_CHECKING}
                </td>
              </tr>
            ))}
            <tr>
              <td>{S00_AP_LABEL}</td>
              <td><span className="pill">{S00_MONITOR}</span></td>
              <td className="r xs mut">{S00_AP_NOTE}</td>
            </tr>
          </tbody>
        </table>
      </div>
      <div className="card">
        <h3>{S00_OVERALL_TITLE}</h3>
        <div className="row">
          <span className={`pill ${ready ? 'ok' : 'warn'}`}>
            {stale ? DISCONNECTED_LABEL : (ready ? S00_READY : S00_CHECKING)}
          </span>
        </div>
        {ready && (
          <button
            type="button"
            className="btn primary wide mt"
            data-testid="s00-advance"
            onClick={onAdvance}
          >
            {S00_ADVANCE}
          </button>
        )}
      </div>
    </div>
  )
}
