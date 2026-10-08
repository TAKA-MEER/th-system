// TestHarness.jsx — NOT a shell/parts/ros file (no U-4 restriction; this is
// dev/e2e-only scaffolding, never real screen content).
//
// Renders OperationCard with every slot enabled so e2e/one-primary-button.spec.js
// can drive /system/state through window.__thSetTestState (see
// ros/useSystemState.js's test hook) and check the "exactly one blue
// button" invariant (DetailedDesign-webui.md §4.3 / U-17) across every
// mode's states -- something no real screen exercises on its own, since a
// given screen only ever shows one mode. Only mounted when
// window.__thTestState is present (Playwright-only; never in a normal
// production load).
import { useSystemState } from './ros/useSystemState.js'
import OperationCard from './shell/OperationCard.jsx'
import attributes from './generated/attributes.json'
import { useJogPanel } from './shell/jogPanel.js'

export default function OperationCardHarness() {
  const { state, stale } = useSystemState()
  const mode = state?.mode ?? null
  const stateName = state?.state ?? null
  // 1b-12 SG-C11: 操作カードの各ボタンで W-6 が閉じることを確かめるため、W-6 を開く
  // 口を置く（本番の画面の「手動」ボタンと同じ jogPanel.open()）。
  const jogPanel = useJogPanel()

  return (
    <div data-testid="opcard-harness">
      <button type="button" className="btn sm" data-testid="harness-open-jog" onClick={() => jogPanel.open()}>
        W-6
      </button>
      <OperationCard
        mode={mode}
        stateName={stateName}
        attributes={attributes}
        slots={{ stop: true, check: true, run: true, save: true, manual: true }}
        disabled={stale}
        onTrigger={() => {}}
        onManualClick={() => {}}
      />
    </div>
  )
}
