// screens/S00Connect.jsx — S-00, the connection-check screen (SCREEN_NAMES.S00
// in i18n/screens.js; DetailedDesign-wp1.md WP-UI-02).
//
// DetailedDesign-state.md §12.2's four required checks (ESP32 feedback,
// ESP32 keepalive loopback, RaspberryPi4/LiDAR, PC nodes) are evaluated by
// connectivity_checker (WP-STATE-03) entirely server-side; it only ever
// surfaces the *result* as evt.link_ok, which folds into SystemState.mode
// leaving 'INIT' (DetailedDesign-state.md §12.1 step 7, T-INIT-01 is the
// only way out of INIT).
//
// 機器別の行（Spec-webui.md §3.1。2026-10-04）: 以前は 4 行とも全体の合否を
// 出すだけだった。connectivity_checker が /system/link_status に項目別の本当の
// 状態（受信間隔・点数・不足ノード）を出すので、各行はそれを表示する。
// 開発モードの link で外していても実際の状態を出し、「無視中」と添える。
//
// §6.2 fail-safe: while /system/state hasn't arrived yet (or has gone
// stale), or mode is still 'INIT', there is nothing to advance to -- no
// "advance" button is rendered at all (not just disabled).
import { useSystemState } from '../ros/useSystemState.js'
import { useAutoCheckStatus } from '../ros/useAutoCheckStatus.js'
import { useLinkStatus } from '../ros/useLinkStatus.js'
import { linkRowView } from '../ros/linkStatusState.js'
import {
  S00_CHECK_TITLE, S00_COL_DEVICE, S00_COL_REQ, S00_COL_STATUS, S00_REQUIRED,
  S00_MONITOR, S00_ITEMS, S00_LINK_TEXT, S00_AP_LABEL,
  S00_AP_NOTE, S00_OVERALL_TITLE, S00_READY, S00_CHECKING, S00_NOT_READY,
  S00_RESTARTING, S00_ADVANCE,
  S00_OPEN_DEV,
  S00_AUTO_TITLE, S00_AUTO_CHECKING, S00_AUTO_OK, S00_AUTO_WARN, S00_AUTO_NG,
  S00_AUTO_SUPPRESSED, S00_AUTO_NOT_YET,
  S00_AUTO_ITEM_LABELS, S00_AUTO_ITEM_ORDER,
  s00AutoResultLabel,
} from '../i18n/screens.js'
import { DISCONNECTED_LABEL } from '../i18n/states.js'

// 自動判定の総合 → pill の色クラスと文言。
function autoOverallView(auto) {
  if (!auto) return { tone: 'warn', label: S00_AUTO_NOT_YET }
  if (auto.suppressed) return { tone: '', label: S00_AUTO_SUPPRESSED }
  switch (auto.overall) {
    case 'OK': return { tone: 'ok', label: S00_AUTO_OK }
    case 'WARN': return { tone: 'warn', label: S00_AUTO_WARN }
    case 'NG': return { tone: 'ng', label: S00_AUTO_NG }
    default: return { tone: 'warn', label: S00_AUTO_CHECKING }
  }
}

function autoItemTone(result) {
  switch (result) {
    case 'OK': return 'tone-ok'
    case 'WARN': return 'tone-warn'
    case 'NG': return 'tone-ng'
    default: return 'tone-warn'
  }
}

// onOpenSettings（2026-09-23）: 機器が揃わず INIT を抜けられないときに、開発モードの
// 設定（S-50 の開発モードタブ）へ入って項目を外し、IDLE へ進むための導線。
export default function S00Connect({ onAdvance, onOpenSettings }) {
  const { state, stale, ros } = useSystemState()
  const { auto } = useAutoCheckStatus(ros)
  const { link } = useLinkStatus(ros)
  const mode = state?.mode ?? null
  const ready = !stale && mode != null && mode !== 'INIT'
  const autoView = autoOverallView(auto)
  // 1b-6 SG-C4: 項目別の状態が届いているのに必須 3 者が揃わないときは
  // 「必須機器が繋がっていません」（従来は「確認中」のままだった）。
  const linkSeen = link != null
  const overallLabel = stale ? DISCONNECTED_LABEL
    : ready ? S00_READY
    : linkSeen ? S00_NOT_READY
    : S00_CHECKING
  const overallTone = ready ? 'ok' : (linkSeen ? 'ng' : 'warn')
  // 1b-6 SG-B7: 再起動中（start.sh が数えた起動回数が 2 以上）の表示。
  // Spec-ops.md §2.4・Spec-webui.md §3.1 の文言どおり。
  const restartAttempt = link?.restart?.attempt
  const restarting = !stale && restartAttempt != null && restartAttempt >= 2

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
            {S00_ITEMS.map((item) => {
              const view = linkRowView(link, item.key, S00_LINK_TEXT)
              return (
                <tr key={item.key}>
                  <td>
                    {item.label}
                    {view.detail && (
                      <div className="xs mut" data-testid={`s00-link-detail-${item.key}`}>{view.detail}</div>
                    )}
                  </td>
                  <td><span className="pill">{S00_REQUIRED}</span></td>
                  <td className={`r ${view.tone}`} data-testid={`s00-link-${item.key}`}>
                    {view.label}
                  </td>
                </tr>
              )
            })}
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
          <span className={`pill ${overallTone}`} data-testid="s00-overall">
            {overallLabel}
          </span>
        </div>
        {restarting && (
          <div className="row mt">
            <span className="pill warn" data-testid="s00-restart">
              {S00_RESTARTING(restartAttempt)}
            </span>
          </div>
        )}
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
        {!ready && onOpenSettings && (
          <button
            type="button"
            className="btn wide mt"
            data-testid="s00-open-dev"
            onClick={onOpenSettings}
          >
            {S00_OPEN_DEV}
          </button>
        )}
      </div>
      <div className="card" data-testid="s00-auto-check">
        <h3>{S00_AUTO_TITLE}</h3>
        <div className="row">
          <span className={`pill ${autoView.tone}`} data-testid="s00-auto-overall">
            {autoView.label}
          </span>
        </div>
        {auto && !auto.suppressed && (
          <table className="lst">
            <tbody>
              {S00_AUTO_ITEM_ORDER.map((key) => {
                const it = auto.items[key] ?? { result: 'CHECKING', reason: '' }
                return (
                  <tr key={key}>
                    <td>{S00_AUTO_ITEM_LABELS[key]}</td>
                    <td className={`r ${autoItemTone(it.result)}`} data-testid={`s00-auto-${key}`}>
                      {s00AutoResultLabel(it.result)}{it.reason ? ` (${it.reason})` : ''}
                    </td>
                  </tr>
                )
              })}
            </tbody>
          </table>
        )}
      </div>
    </div>
  )
}
