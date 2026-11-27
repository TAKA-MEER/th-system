// parts/AutoBrakeLockToggle.jsx — 自動ブレーキのロック付き切替（Spec-webui §3.5）。
//
// 手動系・界面準備・W-6・開発モードの自律系で同じ部品を使う。
// 要求は /system/set_flag auto_brake（本番の経路）。表示は /system/state.auto_brake に従う。
// ON→OFF は W-4 でなくロックの2回押し（1回目で黄「もう一度押すと外す」、約3秒で戻る）。
// OFF→ON は確認なし。「入」に戻す・状態が変わるとロックがかかり直す。
import React, { useEffect, useRef, useState } from 'react'
import { useSetFlag, AUTO_BRAKE_FLAG } from '../ros/useSetFlag.js'
import { LOCKED, ARMED, LOCK_ARM_MS, nextBrakePress } from '../shell/autoBrakeLock.js'
import { reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import {
  AB_HINT_OFF, AB_LOCK_ON, AB_LOCK_ARMED, AB_LOCK_OPEN, AB_LOCK_OFF,
  W6_AUTO_BRAKE_ON, W6_AUTO_BRAKE_OFF,
} from '../i18n/screens.js'

export default function AutoBrakeLockToggle({
  brake,
  canToggle = false,
  prefix = 'ab',
  statusTestId = null,
  mode = null,
  stateName = null,
}) {
  const setFlag = useSetFlag()
  const [lock, setLock] = useState(LOCKED)
  const [error, setError] = useState(null)
  const timerRef = useRef(null)
  const prevBrakeRef = useRef(brake)
  const prevModeStateRef = useRef(`${mode}/${stateName}`)

  async function request(next) {
    setError(null)
    try {
      const res = await setFlag(AUTO_BRAKE_FLAG, next, `${prefix}-auto-brake`)
      if (!res?.accepted) setError(reasonLabel(res?.reject_reason_key) ?? UNKNOWN_REASON_LABEL)
    } catch {
      setError(UNKNOWN_REASON_LABEL)
    }
  }

  function disarm() {
    clearTimeout(timerRef.current)
    timerRef.current = null
    setLock(LOCKED)
  }

  // 「入」に戻す・状態が変わるとロックがかかり直す。
  useEffect(() => {
    const ms = `${mode}/${stateName}`
    if (prevModeStateRef.current !== ms || (brake === true && prevBrakeRef.current !== true)) {
      disarm()
    }
    prevBrakeRef.current = brake
    prevModeStateRef.current = ms
  }, [brake, mode, stateName])

  useEffect(() => () => clearTimeout(timerRef.current), [])

  function handlePress() {
    const { lock: nextLock, request: next } = nextBrakePress(lock, brake)
    if (nextLock === ARMED) {
      setLock(ARMED)
      timerRef.current = setTimeout(disarm, LOCK_ARM_MS)
      return
    }
    setLock(nextLock)
    if (next != null) request(next)
  }

  const label = brake === true
    ? (lock === ARMED ? AB_LOCK_ARMED : AB_LOCK_ON)
    : brake === false
      ? (lock === ARMED || lock !== LOCKED ? AB_LOCK_OPEN : AB_LOCK_OFF)
      : AB_LOCK_ON
  const lockIcon = lock === LOCKED ? '🔒' : '🔓'

  return (
    <div data-testid={`${prefix}-auto-brake-ctl`}>
      <span className={`pill ${brake ? 'ok' : ''}`} data-testid={statusTestId ?? `${prefix}-auto-brake`} data-state={brake === true ? 'on' : brake === false ? 'off' : 'unknown'}>
        {brake === true ? W6_AUTO_BRAKE_ON : brake === false ? W6_AUTO_BRAKE_OFF : '—'}
      </span>
      <button
        type="button"
        className={`btn sm lockb${lock === ARMED ? ' caution armed' : ''}`}
        data-testid={`${prefix}-auto-brake-toggle`}
        data-lock={lock}
        disabled={!canToggle || brake == null}
        onClick={handlePress}
      >
        <span aria-hidden="true">{lockIcon}</span>
        <span>{label}</span>
      </button>
      {brake === false && (
        <div className="note mt" data-testid={`${prefix}-auto-brake-off-hint`}>{AB_HINT_OFF}</div>
      )}
      {error && <div className="note mt" data-testid={`${prefix}-auto-brake-error`}>{error}</div>}
    </div>
  )
}
