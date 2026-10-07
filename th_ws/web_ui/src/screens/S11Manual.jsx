// screens/S11Manual.jsx — S-11, 手動走行 (SCREEN_NAMES.S11 in i18n/screens.js;
// DetailedDesign-wp3.md WP-TRANSIT-01).
//
// "走行タブを差し込んだ画面を 1 枚作り、メインメニューから到達できるようにする。
// 挙動ノードは作らない" (§0). 走行は driveTab kind='manual' が担当し、この
// 画面はその周りに 障害物カード／後退カード／操作カード を置くだけ (4.2)。
//
// 区画 (mockup L620-656 を写したもの):
//   左上 操作バー … 終了 ＋ タブ「走行」1枚
//   障害物カード … /safety/limiter_status の action で分岐する警告文 ＋
//                   自動ブレーキトグル (attributes.json の MANUAL 行から既定を引く)
//   後退カード    … 固定テキスト
//   操作カード    … 「停止」だけ。走行の契機はスティックだけ (T1-4)
//   手動操作カード … driveTab kind='manual' (スティック + 速度プリセット)
//
// 「手動」ボタンを置かない (DetailedDesign-webui.md §4.2): スティックを常設する
// ので触れた瞬間に走行操作が始まるのが唯一の入り口。W-6 は開かない。
//
// 不変条件:
//   T1-1 スティックに触ると PAUSE -> RUN (th_state の T-MANUAL-01 が担当。UI は何もしない)
//   T1-3 速度上限の権威はリミッタ。画面は表示するだけ
//   T1-4 操作カードに「走行」ボタンを置かない
//   T1-5 自動ブレーキの既定は attributes.json の MANUAL.auto_brake_default から引く
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useLimiterStatus } from '../ros/useLimiterStatus.js'
import { useSetFlag, AUTO_BRAKE_FLAG } from '../ros/useSetFlag.js'
import { useConfirmWindow } from '../shell/confirmWindow.js'
import { canToggleAutoBrake, autoBrakeNeedsConfirm, showApproachWarning } from '../shell/autoBrake.js'
import { reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import OperationCard from '../shell/OperationCard.jsx'
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
} from '../i18n/screens.js'
import { stateLabel } from '../i18n/states.js'
import { OP_LABELS } from '../i18n/states.js'
import { useState } from 'react'
import { createPortal } from 'react-dom'

// 自動ブレーキの表示 (T1-5)。既定は attributes.json の
// MANUAL.auto_brake_default から引く——画面にハードコードしない (U-6)。
// /system/state.auto_brake があるときはそちらが現在値（正本は機体側の state_manager。
// 1b-15 SG-B10 で切替の経路ができた。要求は /system/set_flag auto_brake、受理は機体が決める）。
function useAutoBrake(state) {
  const autoBrakeDefault = attributes?.MANUAL?.auto_brake_default === 'on'
  const [local, setLocal] = useState(null)
  if (state?.auto_brake != null) return [!!state.auto_brake, setLocal]
  const value = local ?? autoBrakeDefault
  return [value, setLocal]
}

export default function S11Manual({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const limiter = useLimiterStatus(ros)
  const warn = obstacleWarning(limiter)
  const disabledAll = stale || state?.mode == null

  const [brake] = useAutoBrake(state)
  const approach = showApproachWarning(limiter, brake)
  const stateName = state?.state ?? null

  // 1b-15 SG-B10: 切替。ON→OFF だけ W-4（確認）を挟む。OFF→ON は確認なし。
  // 受理は機体が決める（拒否されたら理由を出す）。画面の表示は /system/state に従う
  // ——押した直後に勝手に切り替わった見た目にしない。
  const setFlag = useSetFlag()
  const confirmWindow = useConfirmWindow()
  const [confirmOff, setConfirmOff] = useState(false)
  const [brakeError, setBrakeError] = useState(null)
  const canToggle = !disabledAll && canToggleAutoBrake(state, attributes, stale)

  async function requestAutoBrake(next) {
    setBrakeError(null)
    try {
      const res = await setFlag(AUTO_BRAKE_FLAG, next, 's11-auto-brake')
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

  // 終了 → ui.finish → 承認で S-01 へ戻る (DetailedDesign-wp3.md §3.2)。
  // 拒否されたら何もしない（遷移表から外れた状態では S-01 に戻れないので、
  // th_state の判断を尊重し S-11 に留まる）。
  async function handleFinish() {
    try {
      const res = await sendTrigger('ui.finish')
      if (res?.accepted && onFinish) onFinish()
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  return (
    <div className="screen two-col" id="s11">
      <div>
        <div className="top-actions sticky">
          <button
            type="button"
            className="btn sm"
            data-testid="s11-finish"
            disabled={disabledAll}
            onClick={handleFinish}
          >
            {OP_LABELS.finish}
          </button>
          <div className="tabs grow" style={{ margin: 0, border: 'none' }}>
            <button type="button" className="tab on">{S11_TAB_DRIVE}</button>
          </div>
        </div>
        <div className="tabpane on">
          <div className="card">
            <h3>{S11_OBSTACLE_TITLE}</h3>
            {/* OFF 中は減速しないので action は PASS のまま。機体が approach_warning を
                立てたら「障害物なし」と出さず、距離つきの警告にする。 */}
            <div className="note">
              {approach ? S11_OBSTACLE_WARN(limiter.nearest_obstacle_m) : obstacleText(warn)}
            </div>
            {approach && (
              <div className="note warn mt" role="alert" data-testid="s11-approach-warning">
                {S11_APPROACH_WARNING}
              </div>
            )}
            <div className="row mt">
              <span className="grow sm">{S11_AUTO_BRAKE_LABEL}</span>
              <span className={`pill ${brake ? 'ok' : ''}`} data-testid="s11-auto-brake">
                {brake ? S11_AUTO_BRAKE_ON : S11_AUTO_BRAKE_OFF}
              </span>
              {/* 1b-15 SG-B10: 切替。手動系とジョグ中だけ押せる（自律系は無効化不可。
                  Spec-safety.md §2.1）。押せない状態では無効にする。 */}
              <button
                type="button"
                className="btn sm"
                data-testid="s11-auto-brake-toggle"
                disabled={!canToggle}
                onClick={handleToggle}
              >
                {brake ? S11_AUTO_BRAKE_TO_OFF : S11_AUTO_BRAKE_TO_ON}
              </button>
            </div>
            {brake === false && (
              <div className="note mt" data-testid="s11-auto-brake-off-hint">{S11_AUTO_BRAKE_HINT_OFF}</div>
            )}
            {brakeError && (
              <div className="note mt" data-testid="s11-auto-brake-error">{brakeError}</div>
            )}
          </div>
          <div className="card">
            <h3>{S11_REAR_TITLE}</h3>
            <div className="note">{S11_REAR_NOTE}</div>
          </div>
        </div>
      </div>
      <div>
        <OperationCard
          mode={state?.mode}
          stateName={stateName}
          attributes={attributes}
          // T1-4: 「停止」だけ。走行の契機はスティックだけ。
          slots={{ stop: true, check: false, run: false, save: false, manual: false }}
          disabled={disabledAll}
          // 「停止」は ui.stop を送る（OperationCard が trigger 名を渡してくる）。
          // ここを空関数にすると、この画面で唯一のボタンが無反応になる。
          onTrigger={(trigger) => sendTrigger(trigger)}
        />
        <div className="card">
          <h3>{S11_MANUAL_TITLE}</h3>
          <DriveTab kind="manual" />
        </div>
        <div className="state">{stateLabel(stateName)}</div>
      </div>

      {confirmOff && confirmWindow.isOpen && confirmWindow.mountNode && createPortal(
        <>
          <header>{S11_AUTO_BRAKE_CONFIRM_TITLE}</header>
          <div className="bodyw" data-testid="s11-auto-brake-confirm">
            <p>{S11_AUTO_BRAKE_CONFIRM_BODY}</p>
          </div>
          <footer>
            <button type="button" className="btn" data-testid="s11-auto-brake-confirm-no" onClick={closeConfirm}>
              {S11_AUTO_BRAKE_CONFIRM_NO}
            </button>
            <button type="button" className="btn danger" data-testid="s11-auto-brake-confirm-yes" onClick={confirmYes}>
              {S11_AUTO_BRAKE_CONFIRM_YES}
            </button>
          </footer>
        </>,
        confirmWindow.mountNode,
      )}
    </div>
  )
}

// /safety/limiter_status が未受信 (warn === null) のときは「不明」を出す
// (安全側。§6.2「障害物なし」と表示しない)。未受信と PASS を区別するのは
// obstacleWarning() が null を返すことで行う。
function obstacleText(warn) {
  if (warn === null) return S11_OBSTACLE_UNKNOWN
  if (warn.level === 'warn') return S11_OBSTACLE_WARN(warn.distance_m)
  if (warn.level === 'stop') return S11_OBSTACLE_STOP(warn.distance_m)
  return S11_OBSTACLE_PASS
}
