// screens/S13TeachManual.jsx — S-13, 教示（手動）(SCREEN_NAMES.S13 in
// i18n/screens.js; P5 / demo-teach-replay).
//
// Spec-transit.md §0.5 / U-5: 教示（手動）＝手動走行画面 ＋「教示」タブ 1 枚。
// 走行部分（障害物カード／後退カード／手動操作カード）は S-11 と完全に共通（S11Manual と
// 同じ骨格。共通部品化まではしない — デモ優先、コピーした箇所はコメントで明記）。
// 教示タブに入るのは「経路名を入れて記録開始」と「記録の状況（距離・時間・点数・向き）」だけ。
//
// FSM（th_state/config/transitions.yaml、P2〜P4 で配線済み）:
//   TEACH_MANUAL: ROUTE_SEL --ui.route_select {new:true,id}--> REC
//                 REC --ui.stop--> PAUSE、PAUSE/SAVED --ui.jog.hold--> REC（スティック。UI は何もしない）
//                 REC/PAUSE --ui.save--> SAVED、* --ui.finish--> IDLE
//
// 操作カードは stop（ui.stop）と save（ui.save）だけ。走行の契機はスティックだけ
// （S-11 の T1-4 と同じ）。onTrigger を空関数にしない（S-11 の既知バグ。
// この画面では「記録開始」という唯一のボタンが無反応にならないことを
// e2e/s13-record-start-sends-route-select.spec.js で検証する）。
import { useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useLimiterStatus } from '../ros/useLimiterStatus.js'
import { useRouteStatus } from '../ros/useRouteStatus.js'
import { useRoutePreview } from '../ros/useRoutePreview.js'
import { useOdomPose } from '../ros/useOdomPose.js'
import { useRoutePose } from '../ros/useRoutePose.js'
import { useRouteMap } from '../ros/useRouteMap.js'
import RoutePreview from './RoutePreview.jsx'
import OperationCard from '../shell/OperationCard.jsx'
import DriveTab from './driveTab.jsx'
import { obstacleWarning } from '../shell/limits.js'
import attributes from '../generated/attributes.json'
import {
  S11_TAB_DRIVE, S11_OBSTACLE_TITLE, S11_OBSTACLE_WARN, S11_OBSTACLE_STOP,
  S11_OBSTACLE_PASS, S11_OBSTACLE_UNKNOWN, S11_REAR_TITLE, S11_REAR_NOTE,
  S11_MANUAL_TITLE,
  S13_TAB_TEACH, S13_TEACH_TITLE, S13_ROUTE_NAME_LABEL, S13_ROUTE_NAME_PLACEHOLDER,
  S13_RECORD_START, S13_RECORDING, S13_SAVED, S13_RECORDED_LABEL, S13_POINTS_LABEL,
  S13_ELAPSED_LABEL, S13_START_YAW_LABEL, S13_SEC, S13_M, S13_DEG,
  S13_REC_DIRECTION,
} from '../i18n/screens.js'
import { stateLabel } from '../i18n/states.js'
import { OP_LABELS } from '../i18n/states.js'

// 障害物カードの文言。S-11 と共通（Spec-transit §0.5）。
function obstacleText(warn) {
  if (warn === null) return S11_OBSTACLE_UNKNOWN
  if (warn.level === 'warn') return S11_OBSTACLE_WARN(warn.distance_m)
  if (warn.level === 'stop') return S11_OBSTACLE_STOP(warn.distance_m)
  return S11_OBSTACLE_PASS
}

export default function S13TeachManual({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const limiter = useLimiterStatus(ros)
  const routeStatus = useRouteStatus(ros)
  const routePreview = useRoutePreview(ros)
  const odomPose = useOdomPose(ros)
  const routePose = useRoutePose(ros)
  const routeMap = useRouteMap(ros)
  const warn = obstacleWarning(limiter)
  const disabledAll = stale || state?.mode == null

  const [tab, setTab] = useState('drive')
  const [name, setName] = useState('')
  const [recording, setRecording] = useState(false)

  const stateName = state?.state ?? null

  // 終了 → ui.finish → 承認で S-01 へ戻る（S11Manual の handleFinish と同じ）。
  async function handleFinish() {
    try {
      const res = await sendTrigger('ui.finish')
      if (res?.accepted && onFinish) onFinish()
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  // 経路名を入れて「記録開始」→ ui.route_select {new:true,id}。
  // 受理されたら記録中表示に切り替える。
  async function handleRecordStart() {
    const id = 'route_' + Date.now()
    const res = await sendTrigger('ui.route_select', { new: true, id })
    if (res?.accepted) setRecording(true)
  }

  const st = routeStatus
  const saved = st?.state === 'SAVED'

  return (
    <div className="screen two-col" id="s13">
      <div>
        <div className="top-actions sticky">
          <button
            type="button"
            className="btn sm"
            data-testid="s13-finish"
            disabled={disabledAll}
            onClick={handleFinish}
          >
            {OP_LABELS.finish}
          </button>
          <div className="tabs grow" style={{ margin: 0, border: 'none' }} role="tablist">
            <button
              type="button"
              role="tab"
              aria-selected={tab === 'drive'}
              className={`tab ${tab === 'drive' ? 'on' : ''}`}
              onClick={() => setTab('drive')}
            >
              {S11_TAB_DRIVE}
            </button>
            <button
              type="button"
              role="tab"
              aria-selected={tab === 'teach'}
              className={`tab ${tab === 'teach' ? 'on' : ''}`}
              onClick={() => setTab('teach')}
            >
              {S13_TAB_TEACH}
            </button>
          </div>
        </div>

        {tab === 'drive' && (
          <div className="tabpane on">
            {/* 走行タブ: S11Manual と共通（Spec-transit §0.5）。 */}
            <div className="card">
              <h3>{S11_OBSTACLE_TITLE}</h3>
              <div className="note">{obstacleText(warn)}</div>
            </div>
            <div className="card">
              <h3>{S11_REAR_TITLE}</h3>
              <div className="note">{S11_REAR_NOTE}</div>
            </div>
          </div>
        )}

        {tab === 'teach' && (
          <div className="tabpane on">
            <div className="card">
              <h3>{S13_TEACH_TITLE}</h3>
              <div className="note">{S13_REC_DIRECTION}</div>
              {!recording ? (
                <>
                  <div className="row mt">
                    <span className="grow sm">{S13_ROUTE_NAME_LABEL}</span>
                    <input
                      type="text"
                      data-testid="s13-route-name"
                      value={name}
                      placeholder={S13_ROUTE_NAME_PLACEHOLDER}
                      onChange={(e) => setName(e.target.value)}
                    />
                  </div>
                  <button
                    type="button"
                    className="btn mt"
                    data-testid="s13-record-start"
                    disabled={disabledAll || !name.trim()}
                    onClick={handleRecordStart}
                  >
                    {S13_RECORD_START}
                  </button>
                </>
              ) : (
                <div className="note mt" data-testid="s13-recording">{S13_RECORDING}</div>
              )}
            </div>
            {/* WS-3: 教示タブの経路プレビュー（記録中の点列。記録中は target_index=-1、
                /scan_filtered は RoutePreview 内部で購読） */}
            <RoutePreview preview={routePreview} pose={routePose ?? odomPose} mapData={routeMap} />
            <div className="card">
              <h3>{S13_RECORDED_LABEL}</h3>
              {saved ? (
                <div className="note" data-testid="s13-saved">{S13_SAVED}</div>
              ) : (
                <table className="lst">
                  <tbody>
                    <tr>
                      <td>{S13_RECORDED_LABEL}</td>
                      <td className="r" data-testid="s13-recorded-m">
                        {st?.recorded_m != null ? S13_M(st.recorded_m.toFixed(2)) : '--'}
                      </td>
                    </tr>
                    <tr>
                      <td>{S13_POINTS_LABEL}</td>
                      <td className="r" data-testid="s13-points">
                        {st?.points != null ? st.points : '--'}
                      </td>
                    </tr>
                    <tr>
                      <td>{S13_ELAPSED_LABEL}</td>
                      <td className="r" data-testid="s13-elapsed">
                        {st?.elapsed_sec != null ? S13_SEC(st.elapsed_sec.toFixed(0)) : '--'}
                      </td>
                    </tr>
                    {st?.start_yaw != null && (
                      <tr>
                        <td>{S13_START_YAW_LABEL}</td>
                        <td className="r">{S13_DEG(st.start_yaw.toFixed(0))}</td>
                      </tr>
                    )}
                  </tbody>
                </table>
              )}
            </div>
          </div>
        )}
      </div>

      <div>
        {/* 操作カードは両タブ共通で右列先頭（S11 と同じ位置）。save は ui.save、
            stop は ui.stop を送る。onTrigger を空関数にしない。 */}
        <OperationCard
          mode={state?.mode}
          stateName={stateName}
          attributes={attributes}
          slots={{ stop: true, check: false, run: false, save: true, manual: false }}
          disabled={disabledAll}
          onTrigger={(trigger) => sendTrigger(trigger)}
        />
        <div className="card">
          <h3>{S11_MANUAL_TITLE}</h3>
          <DriveTab kind="manual" />
        </div>
        <div className="state">{stateLabel(stateName)}</div>
      </div>
    </div>
  )
}
