// parts/AutoBrakeToggle.jsx — 自動ブレーキの切替（A-15。W-6 と自律系の画面で共用）。
//
// S-11 / S-13 は自前の切替を持つ（触らない）。ここは同じ作法を 1 か所に集めたもの:
// ON→OFF だけ W-4 で確認、OFF→ON は確認なし。要求は /system/set_flag auto_brake
// （ros/useSetFlag.js の本番の経路）、受理は機体が決め、拒否されたら理由を出す。
// 表示は /system/state.auto_brake に従い、押した直後に勝手に切り替わった見た目にしない。
import { useState, useContext } from 'react'
import { createPortal } from 'react-dom'
import { useSetFlag, AUTO_BRAKE_FLAG } from '../ros/useSetFlag.js'
import { ConfirmWindowContext } from '../shell/confirmWindow.js'
import { autoBrakeNeedsConfirm } from '../shell/autoBrake.js'
import { reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import {
  AB_TO_OFF, AB_TO_ON, AB_HINT_OFF, AB_JOG_ONLY_HINT,
  AB_CONFIRM_TITLE, AB_CONFIRM_BODY, AB_CONFIRM_YES, AB_CONFIRM_NO,
} from '../i18n/screens.js'

// brake: /system/state.auto_brake（true/false/null）。canToggle: 押せるか。
// prefix: data-testid の接頭辞。confirmWindow: shell/confirmWindow.js の値
// （W-6 は AppShell の Provider の外にあるので props で受ける。省略時は context）。
// jogOnlyHint: 押せない理由が「ジョグ中だけ」のときの案内を出す（W-6）。
export default function AutoBrakeToggle({
  brake, canToggle, prefix, confirmWindow: confirmProp, jogOnlyHint = false,
}) {
  const ctx = useContext(ConfirmWindowContext)
  const confirmWindow = confirmProp ?? ctx
  const setFlag = useSetFlag()
  const [confirmOff, setConfirmOff] = useState(false)
  const [error, setError] = useState(null)

  async function request(next) {
    setError(null)
    try {
      const res = await setFlag(AUTO_BRAKE_FLAG, next, `${prefix}-auto-brake`)
      if (!res?.accepted) setError(reasonLabel(res?.reject_reason_key) ?? UNKNOWN_REASON_LABEL)
    } catch {
      setError(UNKNOWN_REASON_LABEL)
    }
  }

  function handleToggle() {
    const next = !brake
    if (autoBrakeNeedsConfirm(brake, next)) {
      setConfirmOff(true)
      confirmWindow?.open()
      return
    }
    request(next)
  }

  function closeConfirm() {
    setConfirmOff(false)
    confirmWindow?.close()
  }

  function confirmYes() {
    closeConfirm()
    request(false)
  }

  return (
    <>
      <button
        type="button"
        className="btn sm"
        data-testid={`${prefix}-auto-brake-toggle`}
        disabled={!canToggle || brake == null}
        onClick={handleToggle}
      >
        {brake ? AB_TO_OFF : AB_TO_ON}
      </button>
      {brake === false && (
        <div className="note" data-testid={`${prefix}-auto-brake-off-hint`}>{AB_HINT_OFF}</div>
      )}
      {jogOnlyHint && !canToggle && (
        <div className="note" data-testid={`${prefix}-auto-brake-jog-hint`}>{AB_JOG_ONLY_HINT}</div>
      )}
      {error && <div className="note" data-testid={`${prefix}-auto-brake-error`}>{error}</div>}
      {confirmOff && confirmWindow?.isOpen && confirmWindow.mountNode && createPortal(
        <>
          <header>{AB_CONFIRM_TITLE}</header>
          <div className="bodyw" data-testid={`${prefix}-auto-brake-confirm`}>
            <p>{AB_CONFIRM_BODY}</p>
          </div>
          <footer>
            <button type="button" className="btn" data-testid={`${prefix}-auto-brake-confirm-no`} onClick={closeConfirm}>
              {AB_CONFIRM_NO}
            </button>
            <button type="button" className="btn danger" data-testid={`${prefix}-auto-brake-confirm-yes`} onClick={confirmYes}>
              {AB_CONFIRM_YES}
            </button>
          </footer>
        </>,
        confirmWindow.mountNode,
      )}
    </>
  )
}
