// shell/OperationBar.jsx — fixed top-left of the body: finish / screen-local
// tabs / a status pill (DetailedDesign-webui.md §4). "Finish" lives only
// here, never in the operation card, so a slip can't be mistaken for pause.
import { OP_LABELS } from '../i18n/states.js'

export default function OperationBar({
  onFinish,
  finishDisabled = false,
  tabs,
  activeTab,
  onTabChange,
  statusPill,
  sticky = false,
}) {
  return (
    <div className={`top-actions ${sticky ? 'sticky' : ''}`}>
      <button type="button" className="btn" disabled={finishDisabled} onClick={onFinish}>
        {OP_LABELS.finish}
      </button>
      {tabs && tabs.length > 0 && (
        <div className="tabs" role="tablist">
          {tabs.map((t) => (
            <button
              key={t.id}
              type="button"
              role="tab"
              aria-selected={activeTab === t.id}
              className={`tab ${activeTab === t.id ? 'on' : ''}`}
              onClick={() => onTabChange?.(t.id)}
            >
              {t.label}
            </button>
          ))}
        </div>
      )}
      {statusPill && <span className="grow" />}
      {statusPill}
    </div>
  )
}
