// parts/FaultHistoryList.jsx — フォルト履歴の一覧（共通部品。見るだけ）。
// S-50 の「フォルト履歴」タブと、ヘッダの「直近の異常」から開く窓の両方が使う
// （Spec-webui.md §1・§3.15）。新しい順に最大 50 件。継続中は「継続中」＋経過。
import { useEffect, useState } from 'react'
import { faultShortLabel } from '../i18n/faults.js'
import { modeLabel } from '../i18n/modes.js'
import { stateLabel, FH_EMPTY, FH_ONGOING, FH_MODE_STATE, FH_NOTE } from '../i18n/states.js'
import {
  formatClock, formatDuration, recordDurationSec, outcomeLabel, severityLabel,
} from '../ros/faultHistoryState.js'

export default function FaultHistoryList({ records }) {
  const anyOngoing = records.some((r) => r.ongoing)
  const [now, setNow] = useState(() => Date.now() / 1000)
  useEffect(() => {
    if (!anyOngoing) return undefined
    const t = setInterval(() => setNow(Date.now() / 1000), 1000)
    return () => clearInterval(t)
  }, [anyOngoing])

  if (records.length === 0) {
    return <p className="note fh-empty" data-testid="fh-empty">{FH_EMPTY}</p>
  }
  return (
    <div className="fh" data-testid="fh-list">
      <p className="note">{FH_NOTE}</p>
      <ol className="fh-rows">
        {records.map((r, i) => (
          <li
            key={`${r.started_at}-${r.fault_type}-${i}`}
            className={`fh-row ${r.ongoing ? 'ongoing' : ''}`}
            data-testid="fh-row"
            data-ongoing={r.ongoing ? 'true' : 'false'}
          >
            <div className="fh-l1">
              <span className="fh-time" data-testid="fh-time">{formatClock(r.started_at)}</span>
              <b className="fh-type" data-testid="fh-type">{faultShortLabel(r.fault_type)}</b>
              <span className="fh-dur" data-testid="fh-dur">
                {r.ongoing
                  ? `${FH_ONGOING}（${formatDuration(recordDurationSec(r, now))}）`
                  : formatDuration(r.duration_s)}
              </span>
            </div>
            <div className="fh-l2">
              <span data-testid="fh-where">{FH_MODE_STATE(modeLabel(r.mode), stateLabel(r.state))}</span>
              <span className={`fh-sev ${r.severity === 'CRITICAL' ? 'crit' : ''}`} data-testid="fh-sev">
                {severityLabel(r.severity)}
              </span>
              <span data-testid="fh-outcome">{outcomeLabel(r.outcome)}</span>
            </div>
          </li>
        ))}
      </ol>
    </div>
  )
}
