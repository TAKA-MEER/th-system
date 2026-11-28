// screens/S12TeachFollow.jsx — S-12, 教示（追従）(Spec-webui.md §3.4・§3.6)。
//
// 型「走らせる」（RunLayout）。主表示＝対象選択レーダー＋作成中の地図と
// 経路（OnsiteMap 小さく。押すと大小を入れ替え。入口は中央のカード）。
// 右列 ① 記録の量 ② 停止│走行 ③ 向く・保存 ④ スティック常設。
// 記録中の札はヘッダ（記録中。中立の札＋丸の印。STATE_NAMES の REC）。
//
// FSM（th_state/config/transitions.yaml）:
//   TEACH_FOLLOW: ROUTE_SEL --ui.route_select--> REC、REC --ui.stop--> PAUSE
//           PAUSE --ui.run--> REC、REC/PAUSE --ui.save--> SAVED
//           SAVED からの ui.run／スティックは拒否（記録の確定）
// 対象提示（DF-D-6）：記録開始前に対象をレーダーで提示する（機体は動かさない）。
import { useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { usePersonTargets } from '../ros/usePersonTargets.js'
import { useRouteStatus } from '../ros/useRouteStatus.js'
import { useRouteCatalog } from '../ros/useRouteCatalog.js'
import { useRoutePreview } from '../ros/useRoutePreview.js'
import { useOdomPose } from '../ros/useOdomPose.js'
import { useRouteMap } from '../ros/useRouteMap.js'
import { RunLayout } from '../shell/Layouts.jsx'
import RadarSelect from '../parts/RadarSelect.jsx'
import StopRunToggle from '../parts/StopRunToggle.jsx'
import ReasonButton from '../parts/ReasonButton.jsx'
import OnsiteMap from '../parts/OnsiteMap.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import ArmedButton from '../parts/ArmedButton.jsx'
import JogConsole from '../parts/JogConsole.jsx'
import { reasonLabel } from '../i18n/reasons.js'
import { mapSaveWarningVisible } from './routeMapWarning.js'
import {
  S13_ROUTE_NAME_LABEL, S13_ROUTE_NAME_PLACEHOLDER,
  S13_PAUSED, S13_PAUSE_HINT, S13_SAVED, S13_SAVE_FAILED, S13_MAP_SAVE_WARNING,
  S13_ROUTE_NEW_TITLE, S13_ROUTE_EXISTING_TITLE,
  S13_NEW_START, S13_RESELECT, S13_RESELECT_ARMED,
  S14_EMPTY,
} from '../i18n/screens.js'
import {
  S12_MAP_CAPTION, S12_MAP_SWAP, S12_RUN_DENY_ROUTE, S12_REC_M, S12_REC_TIME, S12_REC_LABEL,
  S10_RUN_DENY_TARGET, S10_FACE_DENY_RUNNING, S12_SAVE_DENY_EMPTY, S12_SAVE_DENY_SAVED,
} from '../i18n/screens.js'
import { OP_LABELS } from '../i18n/states.js'
import './follow.css'

const TEACH_SAVED_FINALIZED = 'teach_saved_finalized'

export default function S12TeachFollow({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const personTargets = usePersonTargets(ros)
  const routeStatus = useRouteStatus(ros)
  const routes = useRouteCatalog(ros)
  const routePreview = useRoutePreview(ros)
  const odomPose = useOdomPose(ros)
  const routeMap = useRouteMap(ros, { clearOnRestart: true })
  const disabledAll = stale || state?.mode == null

  const stateName = state?.state ?? null
  const [name, setName] = useState('')
  const [mapLarge, setMapLarge] = useState(false)

  const paused = stateName === 'PAUSE'
  const savedState = stateName === 'SAVED'
  const selecting = stateName === 'ROUTE_SEL'

  const candidates = personTargets?.candidates ?? []
  const selectedIndex = personTargets?.selected_index ?? -1
  const hasTarget = (candidates[selectedIndex] ?? null) != null

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

  function newRouteId() {
    const trimmed = name.replace(/[\\/]/g, '_').trim()
    return trimmed || 'route_' + Date.now()
  }
  async function handleNewStart() {
    await sendTrigger('ui.route_select', { new: true, id: newRouteId() })
  }
  async function handleReselect(id) {
    await sendTrigger('ui.route_select', { id })
  }
  const newIdPreview = newRouteId()
  const newCollides = routes.some((r) => r.id === newIdPreview)

  const st = routeStatus
  const saved = st?.saved === true
  const mapSaveWarning = mapSaveWarningVisible(st)
  const rejectKey = state?.last_reject_reason ?? ''

  const entry = selecting ? (
    <div className="s12-entry">
      <div className="card">
        <h4>{S13_ROUTE_NEW_TITLE}</h4>
        <div className="row mt">
          <span className="grow sm">{S13_ROUTE_NAME_LABEL}</span>
          <input
            type="text"
            data-testid="s12-route-name"
            value={name}
            placeholder={S13_ROUTE_NAME_PLACEHOLDER}
            onChange={(e) => setName(e.target.value.replace(/[\\/]/g, '_'))}
          />
        </div>
        <div className="mt" data-testid="s12-new-start">
          {newCollides ? (
            <HelpTip helpKey="s12-reselect">
              <ArmedButton
                idleLabel={S13_RESELECT}
                armedLabel={S13_RESELECT_ARMED}
                onConfirm={handleNewStart}
                disabled={disabledAll}
              />
            </HelpTip>
          ) : (
            <HelpTip helpKey="s12-new-start">
              <button type="button" className="btn" disabled={disabledAll} onClick={handleNewStart}>
                {S13_NEW_START}
              </button>
            </HelpTip>
          )}
        </div>
        <h4 className="mt">{S13_ROUTE_EXISTING_TITLE}</h4>
        {routes.length === 0 ? (
          <div className="note mt" data-testid="s12-empty">{S14_EMPTY}</div>
        ) : (
          <div className="lst-scroll">
            <table className="lst">
              <tbody>
                {routes.map((r) => (
                  <tr key={r.id} data-testid="s12-route-row">
                    <td className="sm">{r.name}</td>
                    <td className="r sm">
                      <HelpTip helpKey="s12-reselect">
                        <ArmedButton
                          idleLabel={S13_RESELECT}
                          armedLabel={S13_RESELECT_ARMED}
                          onConfirm={() => handleReselect(r.id)}
                          disabled={disabledAll}
                          className="sm"
                        />
                      </HelpTip>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
      </div>
    </div>
  ) : null

  const mapPanel = (
    <div className={`s12-map ${mapLarge ? 'large' : 'small'}`}>
      <div className="mapWrap">
        <OnsiteMap
          mapData={routeMap}
          pins={[]}
          robotPose={odomPose}
          plannedPath={routePreview ?? []}
          testId="s12-map"
          toolsStyle={mapLarge ? undefined : { display: 'none' }}
        />
      </div>
      <button
        type="button"
        className="btn sm s12-map-swap"
        data-testid="s12-map-swap"
        aria-label={S12_MAP_SWAP}
        onClick={() => setMapLarge((v) => !v)}
      >
        <span>{S12_MAP_CAPTION}</span>
      </button>
    </div>
  )

  return (
    <div id="s12" className="screen">
      <RunLayout
        opbar={(
          <HelpTip helpKey="finish">
            <button
              type="button"
              className="btn exit"
              data-testid="s12-finish"
              disabled={disabledAll}
              onClick={handleFinish}
            >
              <span>{OP_LABELS.finish}</span>
            </button>
          </HelpTip>
        )}
        main={(
          <div className="s12-main">
            <div className={`s12-radar ${mapLarge ? 'small' : 'large'}`}>
              {/* レーダー（地図と同じく、カーソルのヘルプを出さない。§3.3.6） */}
              <RadarSelect
                candidates={candidates}
                selectedIndex={selectedIndex}
                isLost={personTargets?.is_lost ?? false}
                confidence={personTargets?.confidence}
                onSelect={handleSelectTarget}
              />
            </div>
            {/* モックアップでは入口では小地図を出さない。地図は経路選択で作り直すため。 */}
            {!selecting && mapPanel}
            {entry}
          </div>
        )}
        where={(
          <div className="s12-rec" data-testid="s12-record-amount">
            {/* 記録の量は距離と時間を 1 行で（「12.4 m　1:32」。点数・開始時の向きは出さない） */}
            <span>{S12_REC_LABEL}</span>
            <b data-testid="s12-record-amount-value">
              {st?.recorded_m != null ? S12_REC_M(st.recorded_m) + '　' + (st?.elapsed_sec != null ? S12_REC_TIME(st.elapsed_sec) : '--') : '--'}
            </b>
            {paused && <span className="note" data-testid="s12-paused-note">{S13_PAUSED}</span>}
            {paused && <span className="note">{S13_PAUSE_HINT}</span>}
            {savedState && (saved
              ? <span className="note" data-testid="s12-saved">{S13_SAVED}</span>
              : <span className="note" data-testid="s12-save-failed">{S13_SAVE_FAILED}</span>)}
            {savedState && mapSaveWarning && (
              <span className="note warn" role="alert" data-testid="s12-map-save-failed">{S13_MAP_SAVE_WARNING}</span>
            )}
            {rejectKey === TEACH_SAVED_FINALIZED && (
              <span className="note" data-testid="s12-reject-reason">{reasonLabel(rejectKey)}</span>
            )}
          </div>
        )}
        toggle={(
          <StopRunToggle
            testId="s12-stop-run"
            runLabel={paused ? '再開する' : '走行する'}
            stopEnabled={!disabledAll && stateName === 'REC'}
            runEnabled={!disabledAll && stateName === 'PAUSE'}
            runDenyReason={selecting ? S12_RUN_DENY_ROUTE : null}
            onStop={() => sendTrigger('ui.stop')}
            onRun={() => sendTrigger('ui.run')}
          />
        )}
        ops={(
          <div className="s12-ops">
            <HelpTip helpKey="look">
              <ReasonButton
                className="look wide"
                data-testid="s12-face"
                disabled={disabledAll || !(hasTarget && (selecting || paused))}
                reason={
                  stateName === 'REC'
                    ? S10_FACE_DENY_RUNNING
                    : (!hasTarget ? S10_RUN_DENY_TARGET : null)
                }
                reasonTestId="s12-face-reason"
                onClick={() => sendTrigger('ui.confirm')}
              >
                <span>向く</span>
              </ReasonButton>
            </HelpTip>
            <HelpTip helpKey="save">
              <ReasonButton
                className="btn save wide"
                data-testid="s12-save"
                disabled={disabledAll || !(stateName === 'REC' || stateName === 'PAUSE')}
                reason={savedState ? S12_SAVE_DENY_SAVED : (selecting ? S12_SAVE_DENY_EMPTY : null)}
                reasonTestId="s12-save-reason"
                onClick={() => sendTrigger('ui.save')}
              >
                <span>{OP_LABELS.save}</span>
              </ReasonButton>
            </HelpTip>
          </div>
        )}
        jog={(
          <JogConsole ros={ros} disabled={stale} keyboard />
        )}
      />
    </div>
  )
}
