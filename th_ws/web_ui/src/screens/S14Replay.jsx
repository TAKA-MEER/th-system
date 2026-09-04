// screens/S14Replay.jsx — S-14, 教示再生 (SCREEN_NAMES.S14 in i18n/screens.js;
// P5 / demo-teach-replay). Spec-webui.md §3.7 の簡略版。
//
// 教示済み経路を /route/catalog（RouteList）から一覧し、選択して ui.route_select
// {id, reverse} で再生を開始する。FSM（th_state/config/transitions.yaml、P2〜P4 で配線済み）:
//   REPLAY: ROUTE_SEL --ui.route_select {id, reverse}--> LOCALIZE
//           LOCALIZE --(replay_runner が evt.localize_done)--> READY
//           READY --ui.run--> RUN、RUN --ui.stop--> PAUSE、PAUSE --ui.run--> RUN
//           RUN --(replay_runner が evt.arrived)--> PAUSE、* --ui.finish--> IDLE
//
// 逆再生はデモ範囲外（ボタンを disabled にしておく）。replay_runner の初期姿勢推定は
// P4 の特例（W-01）で省略済みなので LOCALIZE はほぼ一瞬。
//
// 操作カードは stop（ui.stop）と run（ui.run、ラベル「再生」）だけ。
// onTrigger を空関数にしない（S-11 の既知バグ。この画面では「再生」ボタンが
// 実際に ui.run を送ることを e2e/s14-replay-button-sends-ui-run.spec.js で検証する）。
import { useEffect, useRef, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useRouteCatalog } from '../ros/useRouteCatalog.js'
import { useRouteStatus } from '../ros/useRouteStatus.js'
import { useRoutePreview } from '../ros/useRoutePreview.js'
import { useOdomPose } from '../ros/useOdomPose.js'
import { useRoutePose } from '../ros/useRoutePose.js'
import { useRouteMap } from '../ros/useRouteMap.js'
import { useReplaySpeedPublisher } from '../ros/useReplaySpeedPublisher.js'
import RoutePreview from './RoutePreview.jsx'
import OperationCard from '../shell/OperationCard.jsx'
import ReplaySpeedControl from '../parts/ReplaySpeedControl.jsx'
import { REPLAY_SPEED_RATIOS } from '../parts/replaySpeed.js'
import DriveTab from './driveTab.jsx'
import attributes from '../generated/attributes.json'
import {
  S11_MANUAL_TITLE,
  S14_TAB_REPLAY, S14_SELECT_TITLE, S14_EMPTY, S14_FWD, S14_REV, S14_PROCEED,
  S14_POSE_TITLE, S14_POSE_LOCALIZE, S14_POSE_LOCALIZE_TIMEOUT, S14_POSE_READY, S14_POSE_RUN, S14_POSE_PAUSE, S14_POSE_PAUSE_RESUMABLE,
  S14_LENGTH, S14_POINTS, S14_SPEED_TITLE,
} from '../i18n/screens.js'
import { stateLabel } from '../i18n/states.js'
import { OP_LABELS } from '../i18n/states.js'

// この経路は別のセッションで記録したもので初期姿勢が確定できず、LOCALIZE に
// 留まり続ける（WS-9K-B のガード）。正常時は W-01 により LOCALIZE はほぼ一瞬で
// 抜けるので、ここまで留まるのは異常（＝弾かれている）とみなして手がかりを出す。
const LOCALIZE_TIMEOUT_MS = 20000

// WS-9P: PAUSE の文言は「なぜ止まったか」で変える。終端に達したのなら終了を促し、
// フォルト・ジョグ介入・停止ボタンで止まったのなら「再生で続きから」を案内する。
// arrived は replay_runner が /route/status に載せてくる（RouteStatus.arrived）。
// 状態名だけでは区別できない（T-REPLAY-06/09 と C-01/C-03 が同じ PAUSE に落ちる）。
function poseText(stateName, arrived) {
  if (stateName === 'LOCALIZE') return S14_POSE_LOCALIZE
  if (stateName === 'READY') return S14_POSE_READY
  if (stateName === 'RUN') return S14_POSE_RUN
  if (stateName === 'PAUSE') return arrived ? S14_POSE_PAUSE : S14_POSE_PAUSE_RESUMABLE
  return null
}

export default function S14Replay({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const routes = useRouteCatalog(ros)
  const routeStatus = useRouteStatus(ros)
  const routePreview = useRoutePreview(ros)
  const odomPose = useOdomPose(ros)
  const routePose = useRoutePose(ros)
  const routeMap = useRouteMap(ros)
  const disabledAll = stale || state?.mode == null

  const [selectedId, setSelectedId] = useState(null)
  const [localizeStuck, setLocalizeStuck] = useState(false)
  // WS-9T: 再生速度の比率。既定は中速。走行中に変えても replay_runner が
  // 次ティックで反映する（useReplaySpeedPublisher → /replay/speed_scale）。
  const [speedRatio, setSpeedRatio] = useState(REPLAY_SPEED_RATIOS.mid)
  useReplaySpeedPublisher(ros, speedRatio)

  const stateName = state?.state ?? null
  const pose = poseText(stateName, routeStatus?.arrived === true)
  const empty = routes.length === 0
  const selected = selectedId != null

  // LOCALIZE が一定時間続いたら（＝別セッションの経路で弾かれている）手がかりを出す。
  useEffect(() => {
    if (stateName !== 'LOCALIZE') { setLocalizeStuck(false); return undefined }
    setLocalizeStuck(false)
    const t = setTimeout(() => setLocalizeStuck(true), LOCALIZE_TIMEOUT_MS)
    return () => clearTimeout(t)
  }, [stateName])

  // 終了 → ui.finish → 承認で S-01 へ戻る（S11Manual の handleFinish と同じ）。
  async function handleFinish() {
    try {
      const res = await sendTrigger('ui.finish')
      if (res?.accepted && onFinish) onFinish()
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  // この経路で進む → ui.route_select {id, reverse:false}。逆再生はデモ範囲外。
  async function handleProceed() {
    if (selectedId == null) return
    await sendTrigger('ui.route_select', { id: selectedId, reverse: false })
  }

  return (
    <div className="screen two-col" id="s14">
      <div>
        <div className="top-actions sticky">
          <button
            type="button"
            className="btn sm"
            data-testid="s14-finish"
            disabled={disabledAll}
            onClick={handleFinish}
          >
            {OP_LABELS.finish}
          </button>
          <div className="tabs grow" style={{ margin: 0, border: 'none' }} role="tablist">
            <button type="button" role="tab" aria-selected className="tab on">
              {S14_TAB_REPLAY}
            </button>
          </div>
        </div>

        <div className="tabpane on">
          <div className="card">
            <h3>{S14_SELECT_TITLE}</h3>
            {empty ? (
              <div className="note" data-testid="s14-empty">{S14_EMPTY}</div>
            ) : (
              // 経路が増えても地図を下へ押し出さないよう一覧だけ枠内スクロール
              <div className="lst-scroll">
                <table className="lst">
                  <tbody>
                    {routes.map((r) => (
                      <tr
                        key={r.id}
                        className={selectedId === r.id ? 'sel' : ''}
                        data-testid="s14-route-row"
                      >
                        <td role="radio" aria-checked={selectedId === r.id} className="sm">
                          <button
                            type="button"
                            data-testid={`s14-select-${r.id}`}
                            onClick={() => setSelectedId(r.id)}
                          >
                            {r.name}
                          </button>
                        </td>
                        <td className="r sm">
                          {S14_LENGTH}: {r.length_m != null ? r.length_m.toFixed(2) : '--'}
                          / {S14_POINTS}: {r.point_count}
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            )}
            <div className="row mt">
              <span className="grow sm">{S14_FWD}</span>
              <span className="pill ok">{S14_FWD}</span>
            </div>
            <div className="row mt">
              <span className="grow sm">{S14_REV}</span>
              <button type="button" className="btn sm" disabled>{S14_REV}</button>
            </div>
            <button
              type="button"
              className="btn primary mt"
              data-testid="s14-proceed"
              disabled={disabledAll || empty || !selected}
              onClick={handleProceed}
            >
              {S14_PROCEED}
            </button>
          </div>

          {/* WS-3: 経路プレビュー（再生中の点列＋現在地。targetIndex は /route/status の
              pure-pursuit 目標点。記録中・未走行は -1） */}
          <RoutePreview
            preview={routePreview}
            pose={routePose ?? odomPose}
            mapData={routeMap}
            targetIndex={routeStatus?.target_index ?? -1}
          />

          <div className="card">
            <h3>{S14_POSE_TITLE}</h3>
            <div className="note" data-testid="s14-pose">
              {pose ?? stateLabel(stateName)}
            </div>
            {localizeStuck && (
              <div className="note" data-testid="s14-localize-stuck">{S14_POSE_LOCALIZE_TIMEOUT}</div>
            )}
          </div>
        </div>
      </div>

      <div>
        {/* 操作カードは右列先頭（S11 と同じ位置）。run→ui.run（ラベル「再生」）、
            stop→ui.stop。onTrigger を空関数にしない。 */}
        <OperationCard
          mode={state?.mode}
          stateName={stateName}
          attributes={attributes}
          slots={{ stop: true, check: false, run: true, save: false, manual: false }}
          runLabel={S14_TAB_REPLAY}
          disabled={disabledAll}
          onTrigger={(trigger) => sendTrigger(trigger)}
        />
        <div className="card">
          <h3>{S14_SPEED_TITLE}</h3>
          <ReplaySpeedControl
            value={speedRatio}
            onSelect={setSpeedRatio}
            disabled={disabledAll}
          />
        </div>
        <div className="card">
          <h3>{S11_MANUAL_TITLE}</h3>
          {/* 手動介入用（常設。Spec-transit §0.4） */}
          <DriveTab kind="manual" />
        </div>
        <div className="state">{stateLabel(stateName)}</div>
      </div>
    </div>
  )
}
