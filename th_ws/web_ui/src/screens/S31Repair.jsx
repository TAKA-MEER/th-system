// screens/S31Repair.jsx — S-31, 故障診断 (Spec-webui.md §3.13).
// 型「手順を進める」。左=だめな項目、右=チェックの並び（押すと ✓）。
// 下端=「やめる」（一覧へ）／「始業点検へ戻る」。
// 「一覧へ戻る」は ui.stop（T-OPC-06: REPAIR -> LIST）で、機体にそのまま届く。
//
// 症状の正本は ros/useOpcheckStatus.js の module-level latch（S-30 が REPAIR へ
// 遷る前にNG が届く。機体の /opcheck/status は UNKNOWN を流し続けるため直に読めない）。
// 手順の正本は th_maintenance/config/repair_hints.yaml → generated/repair_hints.json。
// チェックの ✓ は画面の中だけの控え（記録しない）。
import { useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useOpcheckStatus } from '../ros/useOpcheckStatus.js'
import { checkReasonLabel } from '../i18n/checks.js'
import { stripMotorDir } from '../ros/motorDirs.js'
import ResultMark from '../parts/ResultMark.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import { StepsLayout } from '../shell/Layouts.jsx'
import repairHints from '../generated/repair_hints.json'
import {
  S30_ITEM_LABELS, S31_NO_SYMPTOM, S31_ABORT, S31_TO_CHECK,
  S31_SYMPTOM_TITLE, S31_HINTS_TITLE,
} from '../i18n/screens.js'

// Only ESTOP / MOTOR route to REPAIR (Spec-checks §2.5).
const REPAIRABLE_ITEMS = ['ESTOP', 'MOTOR']

export default function S31Repair() {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const { results } = useOpcheckStatus(ros)
  const disabledAll = stale || state?.mode == null

  const symptoms = REPAIRABLE_ITEMS
    .map((item) => ({ item, verdict: results[item] }))
    .filter(({ verdict }) => verdict?.result === 'NG')

  // チェックの ✓ は画面の中だけの控え（記録しない。§3.13）。
  const [checked, setChecked] = useState(() => new Set())
  function toggleChk(key) {
    setChecked((prev) => {
      const next = new Set(prev)
      if (next.has(key)) next.delete(key)
      else next.add(key)
      return next
    })
  }

  function handleAbort() { sendTrigger('ui.stop') }
  function handleBack() { sendTrigger('ui.stop') }

  const leftList = (
    <>
      <div className="lhd">{S31_SYMPTOM_TITLE}</div>
      {symptoms.length === 0 && <p className="note">{S31_NO_SYMPTOM}</p>}
      {symptoms.map(({ item, verdict }) => (
        <div key={item} className="lrow" data-testid={`s31-symptom-${item}`}>
          <span className="nm">
            {S30_ITEM_LABELS[item]}
            <small>{checkReasonLabel(item, verdict.detail)}</small>
          </span>
          <ResultMark result="NG" testId={`s31-mark-${item}`} />
        </div>
      ))}
    </>
  )

  const bodyContent = (
    <>
      {symptoms.map(({ item, verdict }) => {
        const baseReason = item === 'MOTOR' ? stripMotorDir(verdict.detail).reason : verdict.detail
        const steps = repairHints[item]?.[baseReason]
        if (!steps?.length) return null
        return (
          <div key={item} data-testid={`s31-hints-${item}`}>
            <h3>{S30_ITEM_LABELS[item]}: {S31_HINTS_TITLE}</h3>
            <ol className="s31-hints">
              {steps.map((step, i) => {
                const key = `${item}-${i}`
                const on = checked.has(key)
                return (
                  <li key={step}>
                    <HelpTip helpKey="s31-chk">
                      <button
                        type="button"
                        className={`chk${on ? ' on' : ''}`}
                        data-testid={`s31-chk-${item}-${i}`}
                        aria-pressed={on}
                        disabled={disabledAll}
                        onClick={() => toggleChk(key)}
                      >
                        <span className="bx">{on ? '✓' : ''}</span>
                        <span>{step}</span>
                      </button>
                    </HelpTip>
                  </li>
                )
              })}
            </ol>
          </div>
        )
      })}
      {symptoms.length === 0 && <p className="note">{S31_NO_SYMPTOM}</p>}
    </>
  )

  return (
    <div id="s31" className="screen">
      <StepsLayout
        opbar={(
          <HelpTip helpKey="s31-abort">
            <button type="button" className="btn exit" data-testid="s31-back-top" disabled={disabledAll} onClick={handleAbort}>
              戻る
            </button>
          </HelpTip>
        )}
        list={leftList}
        body={(
          <>
            <div className="phd"><span className="pt">{S31_HINTS_TITLE}</span></div>
            <div data-testid="s31-body">{bodyContent}</div>
          </>
        )}
        foot={(
          <>
            <HelpTip helpKey="s31-abort">
              <button type="button" className="btn primary" data-testid="s31-abort" disabled={disabledAll} onClick={handleAbort}>
                {S31_ABORT}
              </button>
            </HelpTip>
            <span className="grow" />
            <HelpTip helpKey="s31-back">
              <button type="button" className="btn primary" data-testid="s31-back" disabled={disabledAll} onClick={handleBack}>
                {S31_TO_CHECK}
              </button>
            </HelpTip>
          </>
        )}
      />
    </div>
  )
}
