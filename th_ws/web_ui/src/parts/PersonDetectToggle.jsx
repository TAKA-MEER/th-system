// parts/PersonDetectToggle.jsx — 人検出 切│入（Spec-webui §3.5・D9/D35/D96）。
//
// 左上の操作バーに置く（試験準備・試験で同じ場所）。入のときだけ明るく点ける。
// 状態の正本は機体（tracker_enabled）。ここは表示と要求だけを持つ。
export default function PersonDetectToggle({
  enabled = false,
  canToggle = true,
  onToggle,
  testId = 'person-detect',
  labelOff = '切',
  labelOn = '入',
}) {
  return (
    <div className="tracker-toggle" data-testid={testId}>
      <span className="tracker-cap">人検出</span>
      <div className="tracker-sides" role="group" aria-label="人検出">
        <button
          type="button"
          className={`tracker-side off${enabled ? '' : ' on'}`}
          data-testid={`${testId}-off`}
          aria-pressed={!enabled}
          disabled={!canToggle}
          onClick={() => onToggle?.(false)}
        >
          {labelOff}
        </button>
        <button
          type="button"
          className={`tracker-side onside${enabled ? ' on' : ''}`}
          data-testid={`${testId}-on`}
          aria-pressed={!!enabled}
          disabled={!canToggle}
          onClick={() => onToggle?.(true)}
        >
          {labelOn}
        </button>
      </div>
    </div>
  )
}
