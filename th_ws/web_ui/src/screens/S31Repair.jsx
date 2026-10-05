// screens/S31Repair.jsx — S-31, 故障診断 (SCREEN_NAMES.S31 in
// i18n/screens.js; WP-UI-08). Spec-webui.md §3.13.
//
// Reached when mode=OPCHECK, state=REPAIR (T-OPC-03: ESTOP/MOTOR NG has no
// calibration to route to -- DetailedDesign-maintenance.md §2.6/§6). This
// packet's brief explicitly scopes S-31 down to "最小限: 理由 + 一覧へ戻る"
// -- the full diagnosis screen (repair_hints.yaml driven step lists) is out
// of scope; "一覧へ戻る" is ui.stop (T-OPC-06: REPAIR -> LIST).
//
// WP-MAINT-03 (2026-10-01): 症状ごとに「チェック手順」を出す（DetailedDesign-maintenance.md
// §2.6）。手順は th_maintenance/config/repair_hints.yaml が正本で、
// scripts/gen_repair_hints.py が src/generated/repair_hints.json に変換する
// （コードに文言を埋めない）。理由に対応する手順が無いときは出さない（症状の表示は残る）。
//
// Symptom source: ros/useOpcheckStatus.js's module-level latch, not local
// component state -- this screen mounts fresh (S30Opcheck unmounted when
// the FSM state flips to REPAIR and screenRouting swaps screens), so
// whatever verdict caused the NG has to survive that swap somewhere outside
// React state. See that hook's file header for why the live /opcheck/status
// topic itself can't be trusted here (opcheck_runner keeps republishing
// UNKNOWN after the final verdict -- a fresh subscriber before the next
// non-UNKNOWN tick would see nothing without the latch).
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useOpcheckStatus } from '../ros/useOpcheckStatus.js'
import { checkReasonLabel } from '../i18n/checks.js'
import { stripMotorDir } from '../ros/motorDirs.js'
import { OP_LABELS } from '../i18n/states.js'
import repairHints from '../generated/repair_hints.json'
import {
  S30_ITEM_LABELS, S31_INTRO, S31_SYMPTOM_TITLE, S31_NO_SYMPTOM, S31_BACK_TO_LIST,
  S31_HINTS_TITLE, S31_HINTS_NOTE,
} from '../i18n/screens.js'

// Only ESTOP / MOTOR route to REPAIR (DetailedDesign-maintenance.md §6 "2つ
// の画面の対応関係" -- IMU/LIDAR NG go to their calibration item instead).
const REPAIRABLE_ITEMS = ['ESTOP', 'MOTOR']

export default function S31Repair() {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const { results } = useOpcheckStatus(ros)
  const disabledAll = stale || state?.mode == null

  const symptoms = REPAIRABLE_ITEMS
    .map((item) => ({ item, verdict: results[item] }))
    .filter(({ verdict }) => verdict?.result === 'NG')

  function handleBack() {
    sendTrigger('ui.stop')
  }
  function handleFinish() {
    sendTrigger('ui.finish')
  }

  return (
    <div className="screen" id="s31">
      <div className="top-actions sticky">
        <button type="button" className="btn sm" data-testid="s31-finish"
          disabled={disabledAll} onClick={handleFinish}>
          {OP_LABELS.finish}
        </button>
      </div>
      <div className="card">
        <p className="note">{S31_INTRO}</p>
      </div>
      <div className="card">
        <h3>{S31_SYMPTOM_TITLE}</h3>
        {symptoms.length === 0 && <p className="note">{S31_NO_SYMPTOM}</p>}
        {symptoms.map(({ item, verdict }) => (
          <div key={item} className="row mt" data-testid={`s31-symptom-${item}`}>
            <span className="pill ng">{S30_ITEM_LABELS[item]}</span>
            <span className="grow">{checkReasonLabel(item, verdict.detail)}</span>
          </div>
        ))}
      </div>
      {symptoms.map(({ item, verdict }) => {
        // MOTOR の確定 NG の理由は `DIR:reason` 形（brief-a13・SG-A13）。
        // 手順表（repair_hints.yaml）のキーは素の理由なので剥がして引く。
        // 表示のほうは checkReasonLabel が方向込みで訳す。
        const baseReason = item === 'MOTOR' ? stripMotorDir(verdict.detail).reason : verdict.detail
        const steps = repairHints[item]?.[baseReason]
        if (!steps?.length) return null
        return (
          <div key={item} className="card" data-testid={`s31-hints-${item}`}>
            <h3>{S30_ITEM_LABELS[item]}: {S31_HINTS_TITLE}</h3>
            <ol className="s31-hints">
              {steps.map((step) => <li key={step}>{step}</li>)}
            </ol>
            <p className="note">{S31_HINTS_NOTE}</p>
          </div>
        )
      })}
      <button type="button" className="btn wide primary" disabled={disabledAll}
        data-testid="s31-back" onClick={handleBack}>
        {S31_BACK_TO_LIST}
      </button>
    </div>
  )
}
