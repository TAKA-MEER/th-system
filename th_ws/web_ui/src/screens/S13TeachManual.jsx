// screens/S13TeachManual.jsx — S-13, 教示（手動）(SCREEN_NAMES.S13 in
// i18n/screens.js; P5 / demo-teach-replay).
//
// Spec-transit.md §0.5 / U-5: 教示（手動）＝手動走行画面 ＋「教示」タブ 1 枚。
// 走行部分（障害物カード／後退カード／手動操作カード）は S-11 と完全に共通（S11Manual と
// 同じ骨格。共通部品化まではしない — デモ優先、コピーした箇所はコメントで明記）。
// 教示タブに入るのは「経路の選択（新規／既存の再設定）」と「記録の状況
// （距離・時間・点数・開始時の向き）」だけ（Spec-webui.md §3.6）。
//
// FSM（th_state/config/transitions.yaml、P2〜P4 で配線済み）:
//   TEACH_MANUAL: ROUTE_SEL --ui.route_select {new:true,id}--> REC（新規）
//                 ROUTE_SEL --ui.route_select {id}--> REC（既存の再設定）
//                 REC --ui.stop--> PAUSE、PAUSE --スティック--> REC（UI は何もしない）
//                 REC/PAUSE --ui.save--> SAVED、* --ui.finish--> IDLE
//
// 「記録開始」ボタンは無い（Spec-transit.md §3.2 手順 4）。選択直後から記録が
// 始まる。操作カードは stop（ui.stop）と save（ui.save）だけ。走行の契機は
// スティックだけ（S-11 の T1-4 と同じ）。
import { useState } from 'react'
import { createPortal } from 'react-dom'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useLimiterStatus } from '../ros/useLimiterStatus.js'
import { useRouteStatus } from '../ros/useRouteStatus.js'
import { useRouteCatalog } from '../ros/useRouteCatalog.js'
import { useRoutePreview } from '../ros/useRoutePreview.js'
import { useOdomPose } from '../ros/useOdomPose.js'
import { useRoutePose } from '../ros/useRoutePose.js'
import { useRouteMap } from '../ros/useRouteMap.js'
import { useSetFlag, AUTO_BRAKE_FLAG } from '../ros/useSetFlag.js'
import { useConfirmWindow } from '../shell/confirmWindow.js'
import { canToggleAutoBrake, autoBrakeNeedsConfirm, showApproachWarning } from '../shell/autoBrake.js'
import { reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import RoutePreview from './RoutePreview.jsx'
import OperationCard from '../shell/OperationCard.jsx'
import ArmedButton from '../parts/ArmedButton.jsx'
import DriveTab from './driveTab.jsx'
import { obstacleWarning } from '../shell/limits.js'
import attributes from '../generated/attributes.json'
import {
  S11_TAB_DRIVE, S11_OBSTACLE_TITLE, S11_OBSTACLE_WARN, S11_OBSTACLE_STOP,
  S11_OBSTACLE_PASS, S11_OBSTACLE_UNKNOWN, S11_AUTO_BRAKE_LABEL,
  S11_AUTO_BRAKE_OFF, S11_AUTO_BRAKE_ON, S11_REAR_TITLE, S11_REAR_NOTE,
  S11_MANUAL_TITLE, S11_AUTO_BRAKE_TO_OFF, S11_AUTO_BRAKE_TO_ON,
  S11_AUTO_BRAKE_HINT_OFF, S11_AUTO_BRAKE_CONFIRM_TITLE, S11_AUTO_BRAKE_CONFIRM_BODY,
  S11_AUTO_BRAKE_CONFIRM_YES, S11_AUTO_BRAKE_CONFIRM_NO, S11_APPROACH_WARNING,
  S13_TAB_TEACH, S13_TEACH_TITLE, S13_ROUTE_NAME_LABEL, S13_ROUTE_NAME_PLACEHOLDER,
  S13_RECORDING, S13_PAUSED, S13_PAUSE_HINT, S13_SAVED, S13_SAVE_FAILED, S13_RECORDED_LABEL, S13_POINTS_LABEL,
  S13_ELAPSED_LABEL, S13_START_YAW_LABEL, S13_SEC, S13_M, S13_DEG,
  S13_REC_DIRECTION, S13_ROUTE_NEW_TITLE, S13_ROUTE_EXISTING_TITLE,
  S13_NEW_START, S13_RESELECT, S13_RESELECT_ARMED,
  S14_EMPTY,
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

// 自動ブレーキの表示。S-11 と共通（Spec-transit §0.5。共通部品化まではしない）。
// 既定は attributes.json の TEACH_MANUAL.auto_brake_default から引く——画面に
// ハードコードしない。/system/state.auto_brake があるときはそちらが現在値
// （正本は機体側の state_manager。要求は /system/set_flag auto_brake、
// 受理は機体が決める）。
function useAutoBrake(state) {
  const autoBrakeDefault = attributes?.TEACH_MANUAL?.auto_brake_default === 'on'
  const [local, setLocal] = useState(null)
  if (state?.auto_brake != null) return [!!state.auto_brake, setLocal]
  const value = local ?? autoBrakeDefault
  return [value, setLocal]
}

// 拒否理由キー（th_state/config/transitions.yaml T-TEACH-05/-05J/-05M）。
// SAVED で「走行」・スティックを拒否したときに state_manager が
// /system/state.last_reject_reason に載せる（1b-7 SG-B11）。
const TEACH_SAVED_FINALIZED = 'teach_saved_finalized'

export default function S13TeachManual({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const limiter = useLimiterStatus(ros)
  const routeStatus = useRouteStatus(ros)
  const routes = useRouteCatalog(ros)
  const routePreview = useRoutePreview(ros)
  const odomPose = useOdomPose(ros)
  const routePose = useRoutePose(ros)
  const routeMap = useRouteMap(ros)
  const warn = obstacleWarning(limiter)
  const disabledAll = stale || state?.mode == null

  const stateName = state?.state ?? null
  const modeName = state?.mode ?? null
  const isTeach = modeName === 'TEACH_MANUAL' || modeName === 'TEACH_FOLLOW'

  // 再読み込みで記録関連の状態に戻る（SG-C5）。記録表示自体は下の recording が
  // 機体の状態から出すが、タブがローカル既定の「走行」のままだと教示タブが
  // 閉じていて見えない。初回だけ状態から決める（表示の初期値のみ）。
  const [tab, setTab] = useState(
    isTeach && (stateName === 'REC' || stateName === 'PAUSE' || stateName === 'SAVED')
      ? 'teach' : 'drive')
  const [name, setName] = useState('')

  // 1b-7 SG-B1/SG-C5: 「記録中」は画面ローカルのフラグで決めない。FSM の状態が
  // 教示系の REC のときだけ出す（PAUSE・SAVED・ESTOP・終了後は出さない）。
  // 記録していないのに「記録中」と見える状態を作らない。REC では recorder が
  // 開いていることと一致する（機体側は明示の保存・破棄でのみ記録を閉じる）。
  const recording = isTeach && stateName === 'REC'
  const paused = isTeach && stateName === 'PAUSE'
  const savedState = isTeach && stateName === 'SAVED'
  // 経路選択は ROUTE_SEL のときだけ出す（Spec-transit.md §3.2 手順 2）。
  // REC/PAUSE 中は記録対象が確定済み、SAVED 後は保存＝確定なので選ばせない。
  const selecting = stateName === 'ROUTE_SEL'

  // 1b-15 SG-B10: 自動ブレーキの切替。S-11 と同じ見た目・同じ経路
  // （useSetFlag）。ON→OFF だけ W-4（確認）を挟む。OFF→ON は確認なし。
  // 受理は機体が決める（拒否されたら理由を出す）。画面の表示は /system/state
  // に従う——押した直後に勝手に切り替わった見た目にしない。
  const [brake] = useAutoBrake(state)
  const approach = showApproachWarning(limiter, brake)
  const setFlag = useSetFlag()
  const confirmWindow = useConfirmWindow()
  const [confirmOff, setConfirmOff] = useState(false)
  const [brakeError, setBrakeError] = useState(null)
  const canToggle = !disabledAll && canToggleAutoBrake(state, attributes, stale)

  async function requestAutoBrake(next) {
    setBrakeError(null)
    try {
      const res = await setFlag(AUTO_BRAKE_FLAG, next, 's13-auto-brake')
      if (!res?.accepted) {
        setBrakeError(reasonLabel(res?.reject_reason_key) ?? UNKNOWN_REASON_LABEL)
      }
    } catch {
      setBrakeError(UNKNOWN_REASON_LABEL)
    }
  }

  function handleToggle() {
    const next = !brake
    if (autoBrakeNeedsConfirm(brake, next)) {
      setConfirmOff(true)
      confirmWindow.open()
      return
    }
    requestAutoBrake(next)
  }

  function closeConfirm() {
    setConfirmOff(false)
    confirmWindow.close()
  }

  function confirmYes() {
    closeConfirm()
    requestAutoBrake(false)
  }

  // 終了 → ui.finish → 承認で S-01 へ戻る（S11Manual の handleFinish と同じ）。
  async function handleFinish() {
    try {
      const res = await sendTrigger('ui.finish')
      if (res?.accepted && onFinish) onFinish()
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  // 新規経路の設定 → ui.route_select {new:true,id}（T-TEACH-01）。
  // 選択直後から記録が始まる（Spec-transit.md §3.2 手順 4）。
  // 入力欄の値を id に使う（WS-9K E-1）。空欄のときだけ日時ベースの自動名。
  // / \ は保存側で _ に置換されるので、ここで同じ置換をして画面の表示名と
  // 保存名が食い違わないようにする。
  function newRouteId() {
    const trimmed = name.replace(/[\\/]/g, '_').trim()
    return trimmed || 'route_' + Date.now()
  }

  async function handleNewStart() {
    await sendTrigger('ui.route_select', { new: true, id: newRouteId() })
  }

  // 既存経路の再設定 → ui.route_select {id}（T-TEACH-01。route_arg_valid が
  // 既知 id を通す）。保存すると新版になり旧版を 1 世代残す
  // （Spec-transit.md §3.5）。同名の既存が押し出されるので二段階アーム式
  // （Spec-webui.md §6。S-13 の中だけで使う）。
  async function handleReselect(id) {
    await sendTrigger('ui.route_select', { id })
  }

  // 新規名が既存 id と衝突するときも同名の上書きになるのでアームを挟む。
  const newIdPreview = newRouteId()
  const newCollides = routes.some((r) => r.id === newIdPreview)

  const st = routeStatus
  // WS-9K E-2: 「保存しました」は route_recorder が実際にファイルを書いた
  // （RouteStatus.saved === true）ときだけ出す。FSM の state が SAVED でも
  // 保存に失敗していれば saved は false のままなので、出してはいけない。
  const saved = st?.saved === true

  // 1b-7 SG-B11: SAVED で「走行」・スティックを拒否した理由。state_manager が
  // 全イベントの判定で last_reject_reason をラッチする（jog を含む）ので、
  // スティックの拒否もここから見える。S-11 の brakeError と同じ inline 表示。
  const rejectKey = state?.last_reject_reason ?? ''

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
            {/* 走行タブ: S11Manual と共通（Spec-transit §0.5）。自動ブレーキの
                トグルを含む（Spec-webui.md §3.5。1b-15 の残り）。 */}
            <div className="card">
              <h3>{S11_OBSTACLE_TITLE}</h3>
              <div className="note">
                {approach ? S11_OBSTACLE_WARN(limiter.nearest_obstacle_m) : obstacleText(warn)}
              </div>
              {approach && (
                <div className="note warn mt" role="alert" data-testid="s13-approach-warning">
                  {S11_APPROACH_WARNING}
                </div>
              )}
              <div className="row mt">
                <span className="grow sm">{S11_AUTO_BRAKE_LABEL}</span>
                <span className={`pill ${brake ? 'ok' : ''}`} data-testid="s13-auto-brake">
                  {brake ? S11_AUTO_BRAKE_ON : S11_AUTO_BRAKE_OFF}
                </span>
                <button
                  type="button"
                  className="btn sm"
                  data-testid="s13-auto-brake-toggle"
                  disabled={!canToggle}
                  onClick={handleToggle}
                >
                  {brake ? S11_AUTO_BRAKE_TO_OFF : S11_AUTO_BRAKE_TO_ON}
                </button>
              </div>
              {brake === false && (
                <div className="note mt" data-testid="s13-auto-brake-off-hint">{S11_AUTO_BRAKE_HINT_OFF}</div>
              )}
              {brakeError && (
                <div className="note mt" data-testid="s13-auto-brake-error">{brakeError}</div>
              )}
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
              {selecting ? (
                <>
                  <h4 className="mt">{S13_ROUTE_NEW_TITLE}</h4>
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
                      <ArmedButton
                        idleLabel={S13_RESELECT}
                        armedLabel={S13_RESELECT_ARMED}
                        onConfirm={handleNewStart}
                        disabled={disabledAll}
                      />
                    ) : (
                      <button
                        type="button"
                        className="btn"
                        disabled={disabledAll}
                        onClick={handleNewStart}
                      >
                        {S13_NEW_START}
                      </button>
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
                                <ArmedButton
                                  idleLabel={S13_RESELECT}
                                  armedLabel={S13_RESELECT_ARMED}
                                  onConfirm={() => handleReselect(r.id)}
                                  disabled={disabledAll}
                                  className="sm"
                                />
                              </td>
                            </tr>
                          ))}
                        </tbody>
                      </table>
                    </div>
                  )}
                </>
              ) : recording ? (
                // Spec-webui.md §3.6: 赤い「記録中」インジケータ。
                <div className="note mt tone-ng" data-testid="s13-recording">{S13_RECORDING}</div>
              ) : paused ? (
                <>
                  <div className="note mt" data-testid="s13-paused">{S13_PAUSED}</div>
                  <div className="note mt">{S13_PAUSE_HINT}</div>
                </>
              ) : null}
            </div>
            {/* WS-3: 教示タブの経路プレビュー（記録中の点列。記録中は target_index=-1、
                /scan_filtered は RoutePreview 内部で購読） */}
            <RoutePreview preview={routePreview} pose={routePose ?? odomPose} mapData={routeMap} />
            {(recording || paused) && (
              <div className="card">
                <h3>{S13_RECORDED_LABEL}</h3>
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
                    {st?.start_yaw != null && st?.points > 0 && (
                      <tr>
                        <td>{S13_START_YAW_LABEL}</td>
                        <td className="r" data-testid="s13-start-yaw">
                          {S13_DEG((st.start_yaw * 180 / Math.PI).toFixed(0))}
                        </td>
                      </tr>
                    )}
                  </tbody>
                </table>
              </div>
            )}
            {savedState && (
              <div className="card">
                <h3>{S13_RECORDED_LABEL}</h3>
                {saved ? (
                  <div className="note" data-testid="s13-saved">{S13_SAVED}</div>
                ) : (
                  <div className="note" data-testid="s13-save-failed">{S13_SAVE_FAILED}</div>
                )}
              </div>
            )}
          </div>
        )}
      </div>

      <div>
        {/* 操作カードは両タブ共通で右列先頭（S11 と同じ位置）。save は ui.save、
            stop は ui.stop を送る。 */}
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
        {rejectKey === TEACH_SAVED_FINALIZED && (
          <div className="note mt" data-testid="s13-reject-reason">
            {reasonLabel(rejectKey)}
          </div>
        )}
      </div>

      {confirmOff && confirmWindow.isOpen && confirmWindow.mountNode && createPortal(
        <>
          <header>{S11_AUTO_BRAKE_CONFIRM_TITLE}</header>
          <div className="bodyw" data-testid="s13-auto-brake-confirm">
            <p>{S11_AUTO_BRAKE_CONFIRM_BODY}</p>
          </div>
          <footer>
            <button type="button" className="btn" data-testid="s13-auto-brake-confirm-no" onClick={closeConfirm}>
              {S11_AUTO_BRAKE_CONFIRM_NO}
            </button>
            <button type="button" className="btn danger" data-testid="s13-auto-brake-confirm-yes" onClick={confirmYes}>
              {S11_AUTO_BRAKE_CONFIRM_YES}
            </button>
          </footer>
        </>,
        confirmWindow.mountNode,
      )}
    </div>
  )
}
