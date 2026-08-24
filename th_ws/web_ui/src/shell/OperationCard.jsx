// shell/OperationCard.jsx — the fixed stop/confirm/run/save/manual grid
// (DetailedDesign-webui.md §4). Screens (WP-UI-02+) pass which slots they
// need; this component owns the one rule that must never be violated: at
// most one button is blue, and that blue button is the mode/state's current
// status, decided here — never by the screen (§4.3, U-17, e2e
// one-primary-button.spec.js).
import { operationCardLayout, stateToBlueButton } from './limits.js'
import { OP_LABELS } from '../i18n/states.js'

export default function OperationCard({
  mode,
  stateName,
  attributes,
  slots: slotOverrides,
  runLabel,
  disabled = false,
  onTrigger,
  onManualClick,
}) {
  const layout = operationCardLayout(mode, attributes)
  const slots = { ...layout, ...slotOverrides }
  const blue = stateToBlueButton(mode, stateName, attributes)

  const fire = (trigger) => {
    if (!onTrigger) return
    onTrigger(trigger)
  }

  return (
    <div className="card ops">
      <div className="opsgrid">
        {slots.stop && (
          <button
            type="button"
            className={`btn op-stop ${blue === 'stop' ? 'on' : ''}`}
            disabled={disabled}
            onClick={() => fire('ui.stop')}
          >
            {OP_LABELS.stop}
          </button>
        )}
        {slots.check && (
          <button
            type="button"
            className={`btn op-check ${blue === 'check' ? 'on' : ''}`}
            disabled={disabled}
            onClick={() => fire('ui.confirm')}
          >
            {OP_LABELS.check}
          </button>
        )}
        {slots.run && (
          <button
            type="button"
            className={`btn op-run ${blue === 'run' ? 'on' : ''}`}
            disabled={disabled}
            onClick={() => fire('ui.run')}
          >
            {runLabel ?? OP_LABELS.run}
          </button>
        )}
        {slots.save && (
          <button
            type="button"
            className="btn save op-save"
            disabled={disabled}
            onClick={() => fire('ui.save')}
          >
            {OP_LABELS.save}
          </button>
        )}
        {slots.manual && (
          <button
            type="button"
            className="btn op-manual"
            disabled={disabled}
            onClick={onManualClick}
          >
            {OP_LABELS.manual}
          </button>
        )}
      </div>
    </div>
  )
}
