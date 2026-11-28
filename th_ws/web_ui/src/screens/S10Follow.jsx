// screens/S10Follow.jsx — S-10, 追従走行 (Spec-webui.md §3.4・§3.5)。
//
// 型「走らせる」（RunLayout）。主表示＝対象選択レーダー（RadarSelect。
// 候補を押して選ぶ・選択中は反転・軌跡）。右列 ① 対象までの距離
// ② 停止│走行（StopRunToggle）③ 向く（薄い緑の別ボタン。ui.confirm）
// ④ スティック常設（JogConsole。触れたらジョグ介入で一時停止）。
// 左上＝終了だけ（人検出は追従系では自動なので切り替えを置かない。
// 自動ブレーキは on_locked で切り替えなし）。タブは使わない。
//
// FSM（th_state/config/transitions.yaml）:
//   FOLLOW: SELECT --ui.select_target{index}--> SELECT（対象確定）
//           SELECT --ui.confirm--> CONFIRM（対象の方を向く。移動しない）
//           CONFIRM --ui.run--> RUN、RUN --ui.stop--> PAUSE
//           PAUSE --ui.run--> RUN、RUN --(evt.target_lost)--> SELECT
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { usePersonTargets } from '../ros/usePersonTargets.js'
import { RunLayout } from '../shell/Layouts.jsx'
import RadarSelect from '../parts/RadarSelect.jsx'
import StopRunToggle from '../parts/StopRunToggle.jsx'
import ReasonButton from '../parts/ReasonButton.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import JogConsole from '../parts/JogConsole.jsx'
import {
  S10_DISTANCE_LABEL, S10_NO_TARGET, S10_DIST,
  S10_RUN_DENY_TARGET, S10_RUN_DENY_FACE,
  S10_FACE_DENY_RUNNING, S10_FACE_DENY_FACED, S10_FACE_DENY_STOPPED,
} from '../i18n/screens.js'
import { OP_LABELS } from '../i18n/states.js'
import './follow.css'

export default function S10Follow({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const personTargets = usePersonTargets(ros)
  const disabledAll = stale || state?.mode == null

  const stateName = state?.state ?? null
  const candidates = personTargets?.candidates ?? []
  const selectedIndex = personTargets?.selected_index ?? -1
  const selected = candidates[selectedIndex] ?? null
  const hasTarget = selected != null

  // 対象までの距離（機体からの直線距離。候補は base_link 相対）。
  // 機体から届かない値は作らず、未選択なら「対象を選んでいません」。
  const dist = hasTarget
    ? Math.hypot(selected?.x ?? 0, selected?.y ?? 0)
    : null

  async function handleFinish() {
    try {
      const res = await sendTrigger('ui.finish')
      if (res?.accepted && onFinish) onFinish()
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  function handleSelectTarget(index) {
    sendTrigger('ui.select_target', { index })
  }

  const stopEnabled = !disabledAll && stateName === 'RUN'
  const runEnabled = !disabledAll && (
    stateName === 'CONFIRM'
    || (stateName === 'PAUSE' && hasTarget)
  )
  const runLabel = stateName === 'PAUSE' ? '再開する' : '走行する'
  const runDenyReason = stateName === 'SELECT' && !hasTarget
    ? S10_RUN_DENY_TARGET
    : (stateName === 'SELECT' ? S10_RUN_DENY_FACE : null)

  const faceEnabled = !disabledAll && stateName === 'SELECT' && hasTarget
  const faceReason = stateName === 'RUN'
    ? S10_FACE_DENY_RUNNING
    : (stateName === 'CONFIRM'
      ? S10_FACE_DENY_FACED
      : (!hasTarget
        ? S10_RUN_DENY_TARGET
        : (stateName === 'PAUSE' ? S10_FACE_DENY_STOPPED : null)))

  return (
    <div id="s10" className="screen">
      <RunLayout
        opbar={(
          <HelpTip helpKey="finish">
            <button
              type="button"
              className="btn exit"
              data-testid="s10-finish"
              disabled={disabledAll}
              onClick={handleFinish}
            >
              <span>{OP_LABELS.finish}</span>
            </button>
          </HelpTip>
        )}
        main={(
          <div className="s10-main">
            {/* レーダー（地図と同じく、カーソルのヘルプを出さない。§3.3.6） */}
            <RadarSelect
              candidates={candidates}
              selectedIndex={selectedIndex}
              isLost={personTargets?.is_lost ?? false}
              confidence={personTargets?.confidence}
              onSelect={handleSelectTarget}
            />
          </div>
        )}
        where={(
          <div className="s10-where" data-testid="s10-distance">
            <HelpTip helpKey="s10-distance">
              <span>
                {dist != null && Number.isFinite(dist) ? (
                  <>
                    <span>{S10_DISTANCE_LABEL} </span>
                    <b data-testid="s10-distance-value">{S10_DIST(dist)}</b>
                  </>
                ) : (
                  <span className="note warn" data-testid="s10-no-target">{S10_NO_TARGET}</span>
                )}
              </span>
            </HelpTip>
          </div>
        )}
        toggle={(
          <StopRunToggle
            testId="s10-stop-run"
            runLabel={runLabel}
            stopEnabled={stopEnabled}
            runEnabled={runEnabled}
            runDenyReason={runDenyReason}
            onStop={() => sendTrigger('ui.stop')}
            onRun={() => sendTrigger('ui.run')}
          />
        )}
        ops={(
          <HelpTip helpKey="look">
            <ReasonButton
              className="look wide"
              data-testid="s10-face"
              disabled={disabledAll || !faceEnabled}
              reason={faceReason}
              reasonTestId="s10-face-reason"
              onClick={() => sendTrigger('ui.confirm')}
            >
              <span>向く</span>
            </ReasonButton>
          </HelpTip>
        )}
        jog={(
          <JogConsole ros={ros} disabled={stale} keyboard />
        )}
      />
    </div>
  )
}
