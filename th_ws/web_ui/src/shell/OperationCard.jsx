// shell/OperationCard.jsx — the fixed stop/confirm/run/save/manual grid
// (DetailedDesign-webui.md §4). Screens (WP-UI-02+) pass which slots they
// need; this component owns the one rule that must never be violated: at
// most one button is blue, and that blue button is the mode/state's current
// status, decided here — never by the screen (§4.3, U-17, e2e
// one-primary-button.spec.js).
//
// brief-onsite-ux UX-2-a: each slot carries a kind shape (shape + inline SVG
// icon, never colour alone). UX-2-b / UX-2-d: when a screen passes
// manualDenyReason, the manual button is disabled and the reason is shown in
// a badge directly beneath it (data-testid="reason-manual").
import { operationCardLayout, stateToBlueButton } from './limits.js'
import { OP_LABELS } from '../i18n/states.js'
import {
  IconArrow, IconJoystick, IconSave, IconStop, OP_BUTTON_KINDS,
} from '../parts/icons.jsx'

export default function OperationCard({
  mode,
  stateName,
  attributes,
  slots: slotOverrides,
  runLabel,
  disabled = false,
  manualDenyReason = null,
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
            className={`btn op-stop ${OP_BUTTON_KINDS.stop} ${blue === 'stop' ? 'on' : ''}`}
            disabled={disabled}
            onClick={() => fire('ui.stop')}
          >
            <IconStop />
            <span>{OP_LABELS.stop}</span>
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
            className={`btn op-run ${OP_BUTTON_KINDS.advance} ${blue === 'run' ? 'on' : ''}`}
            disabled={disabled}
            onClick={() => fire('ui.run')}
          >
            <IconArrow />
            <span>{runLabel ?? OP_LABELS.run}</span>
          </button>
        )}
        {slots.save && (
          <button
            type="button"
            className={`btn save op-save ${OP_BUTTON_KINDS.save}`}
            disabled={disabled}
            onClick={() => fire('ui.save')}
          >
            <IconSave />
            <span>{OP_LABELS.save}</span>
          </button>
        )}
        {slots.manual && (
          <div className="manual-cell">
            <button
              type="button"
              className={`btn op-manual ${OP_BUTTON_KINDS.manual}`}
              disabled={disabled || !!manualDenyReason}
              onClick={onManualClick}
            >
              <IconJoystick />
              <span>{OP_LABELS.manual}</span>
            </button>
            {manualDenyReason && (
              <span className="reason-badge" data-testid="reason-manual">
                {manualDenyReason}
              </span>
            )}
          </div>
        )}
      </div>
    </div>
  )
}