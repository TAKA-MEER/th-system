// screens/S13TeachManual.jsx — S-13, 教示（手動）(Spec-webui.md §3.4・§3.6)。
//
// 型「走らせる」（RunLayout）。主表示＝作成中の地図と経路（OnsiteMap。機体を中心に
// 追う。入口は主表示の中央のカード）。右列 ① 記録の量 ②「停止する」だけ
// ③「保存」 ④ 速度のスライダ＋スティック＋速度の段。タブは使わない。
// 計器とガイド線は S-13 に入れない。終了で未保存があれば W-4（シェル側）。
//
// FSM: TEACH_MANUAL: ROUTE_SEL --ui.route_select--> REC、REC --ui.stop--> PAUSE、
// PAUSE --スティック--> REC、REC/PAUSE --ui.save--> SAVED、* --ui.finish--> IDLE
import { useState, useEffect } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useRouteStatus } from '../ros/useRouteStatus.js'
import { useRouteCatalog } from '../ros/useRouteCatalog.js'
import { useRoutePreview } from '../ros/useRoutePreview.js'
import { useOdomPose } from '../ros/useOdomPose.js'
import { useRouteMap } from '../ros/useRouteMap.js'
import { canToggleAutoBrake } from '../shell/autoBrake.js'
import { RunLayout } from '../shell/Layouts.jsx'
import AutoBrakeLockToggle from '../parts/AutoBrakeLockToggle.jsx'
import StopRunToggle from '../parts/StopRunToggle.jsx'
import ReasonButton from '../parts/ReasonButton.jsx'
import OnsiteMap from '../parts/OnsiteMap.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import ArmedButton from '../parts/ArmedButton.jsx'
import JogConsole from '../parts/JogConsole.jsx'
import { reasonLabel } from '../i18n/reasons.js'
import { mapSaveWarningVisible } from './routeMapWarning.js'
import attributes from '../generated/attributes.json'
import {
  S13_ROUTE_NAME_LABEL, S13_ROUTE_NAME_PLACEHOLDER,
  S13_RECORDING, S13_PAUSED, S13_PAUSE_HINT, S13_SAVED, S13_SAVE_FAILED, S13_MAP_SAVE_WARNING,
  S13_RECORDED_LABEL, S13_POINTS_LABEL, S13_ELAPSED_LABEL, S13_START_YAW_LABEL, S13_SEC, S13_M, S13_DEG,
  S13_ROUTE_NEW_TITLE, S13_ROUTE_EXISTING_TITLE,
  S13_NEW_START, S13_RESELECT, S13_RESELECT_ARMED,
  S14_EMPTY,
} from '../i18n/screens.js'
import { OP_LABELS } from '../i18n/states.js'
import './manualRun.css'

function useAutoBrake(state) {
  const autoBrakeDefault = attributes?.TEACH_MANUAL?.auto_brake_default === 'on'
  const [local, setLocal] = useState(null)
  if (state?.auto_brake != null) return [!!state.auto_brake, setLocal]
  const value = local ?? autoBrakeDefault
  return [value, setLocal]
}

const TEACH_SAVED_FINALIZED = 'teach_saved_finalized'

export default function S13TeachManual({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const routeStatus = useRouteStatus(ros)
  const routes = useRouteCatalog(ros)
  const routePreview = useRoutePreview(ros)
  const odomPose = useOdomPose(ros)
  const routeMap = useRouteMap(ros, { clearOnRestart: true })
  const disabledAll = stale || state?.mode == null

  const stateName = state?.state ?? null
  const modeName = state?.mode ?? null
  const isTeach = modeName === 'TEACH_MANUAL' || modeName === 'TEACH_FOLLOW'
  const [name, setName] = useState('')

  const recording = isTeach && stateName === 'REC'
  const paused = isTeach && stateName === 'PAUSE'
  const savedState = isTeach && stateName === 'SAVED'
  const selecting = stateName === 'ROUTE_SEL'

  const [brake] = useAutoBrake(state)
  const canToggle = !disabledAll && canToggleAutoBrake(state, attributes, stale)

  async function handleFinish() {
    try {
      const res = await sendTrigger('ui.finish')
      if (res?.accepted && onFinish) onFinish()
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
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
    <div className="s13-entry">
      <div className="card">
        <h4>{S13_ROUTE_NEW_TITLE}</h4>
        <div className="row mt">
          <span className="grow sm">{S13_ROUTE_NAME_LABEL}</span>
          <input
            type="text"
            data-testid="s13-route-name"
            value={name}
            placeholder={S13_ROUTE_NAME_PLACEHOLDER}
            onChange={(e) => setName(e.target.value.replace(/[\\/]/g, '_'))}
          />
        </div>
        <div className="mt" data-testid="s13-new-start">
          {newCollides ? (
            <HelpTip helpKey="s13-reselect">
            <ArmedButton
              idleLabel={S13_RESELECT}
              armedLabel={S13_RESELECT_ARMED}
              onConfirm={handleNewStart}
              disabled={disabledAll}
            />
            </HelpTip>
          ) : (
            <HelpTip helpKey="s13-new-start">
            <button type="button" className="btn" disabled={disabledAll} onClick={handleNewStart}>
              {S13_NEW_START}
            </button>
            </HelpTip>
          )}
        </div>
        <h4 className="mt">{S13_ROUTE_EXISTING_TITLE}</h4>
        {routes.length === 0 ? (
          <div className="note mt" data-testid="s13-empty">{S14_EMPTY}</div>
        ) : (
          <div className="lst-scroll">
            <table className="lst">
              <tbody>
                {routes.map((r) => (
                  <tr key={r.id} data-testid="s13-route-row">
                    <td className="sm">{r.name}</td>
                    <td className="r sm">
                      <HelpTip helpKey="s13-reselect">
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

  return (
    <div id="s13" className="screen">
      <RunLayout
        opbar={(
          <>
            <HelpTip helpKey="finish">
              <button
                type="button"
                className="btn exit"
                data-testid="s13-finish"
                disabled={disabledAll}
                onClick={handleFinish}
              >
                <span>{OP_LABELS.finish}</span>
              </button>
            </HelpTip>
            <HelpTip helpKey="brake">
              <AutoBrakeLockToggle brake={brake} canToggle={canToggle} prefix="s13" mode={state?.mode} stateName={stateName} />
            </HelpTip>
            {recording && <span className="pill" data-testid="s13-recording">{S13_RECORDING}</span>}
            {paused && <span className="pill" data-testid="s13-paused">{S13_PAUSED}</span>}
          </>
        )}
        main={(
          <div className="s13-main">
            <OnsiteMap
              mapData={routeMap}
              pins={[]}
              robotPose={odomPose}
              plannedPath={routePreview ?? []}
              testId="s13-map"
            />
            {entry}
          </div>
        )}
        where={(
          <div className="s13-rec" data-testid="s13-record-amount">
            <span>{S13_RECORDED_LABEL} <span className="r" data-testid="s13-recorded-m">{st?.recorded_m != null ? S13_M(st.recorded_m.toFixed(2)) : '--'}</span></span>
            <span>{S13_POINTS_LABEL} <span className="r" data-testid="s13-points">{st?.points != null ? st.points : '--'}</span></span>
            <span>{S13_ELAPSED_LABEL} <span className="r" data-testid="s13-elapsed">{st?.elapsed_sec != null ? S13_SEC(st.elapsed_sec.toFixed(0)) : '--'}</span></span>
            {st?.start_yaw != null && st?.points > 0 && (
              <span>{S13_START_YAW_LABEL} <span className="r" data-testid="s13-start-yaw">{S13_DEG((st.start_yaw * 180 / Math.PI).toFixed(0))}</span></span>
            )}
            {paused && <span className="note">{S13_PAUSE_HINT}</span>}
            {savedState && (saved
              ? <span className="note" data-testid="s13-saved">{S13_SAVED}</span>
              : <span className="note" data-testid="s13-save-failed">{S13_SAVE_FAILED}</span>)}
            {savedState && mapSaveWarning && (
              <span className="note warn" role="alert" data-testid="s13-map-save-failed">{S13_MAP_SAVE_WARNING}</span>
            )}
            {rejectKey === TEACH_SAVED_FINALIZED && (
              <span className="note" data-testid="s13-reject-reason">{reasonLabel(rejectKey)}</span>
            )}
          </div>
        )}
        toggle={(
          <StopRunToggle
            testId="s13-stop-run"
            runLabel={null}
            stopEnabled={!disabledAll}
            onStop={() => sendTrigger('ui.stop')}
          />
        )}
        ops={(
          <HelpTip helpKey="save">
            <ReasonButton
              className="btn save wide"
              data-testid="s13-save"
              reason={null}
              disabled={disabledAll}
              onClick={() => sendTrigger('ui.save')}
            >
              <span>{OP_LABELS.save}</span>
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
