// parts/AutoBrakeLockToggle.jsx — 自動ブレーキのロック付き切替（Spec-webui §3.5・D73）。
//
// 手動系・界面準備・W-6・開発モードの自律系で同じ部品を使う。
// 要求は /system/set_flag auto_brake（本番の経路）。表示は /system/state.auto_brake に従う。
// ON の間は鍵（SVG。絵文字は使わない）の「入」ボタン。1回目で黄「もう一度押すと外す」
// （約3秒で戻る）、2回目で外れて「切│入」の切り替えが出る。「切」を押して OFF。
// OFF の間は「入」をすぐ押せる（ON に戻す）。「入」に戻す・状態が変わるとロックがかかり直す。
import React, { useEffect, useRef, useState } from 'react'
import { useSetFlag, AUTO_BRAKE_FLAG } from '../ros/useSetFlag.js'
import { LOCKED, ARMED, UNLOCKED, LOCK_ARM_MS, nextBrakePress, nextBrakeSide } from '../shell/autoBrakeLock.js'
import { reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import { IconLock } from './icons.jsx'
import {
  AB_HINT_OFF, AB_LOCK_ON, AB_LOCK_ARMED,
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

  function handleLockPress() {
    const { lock: nextLock } = nextBrakePress(lock, brake)
    if (nextLock === ARMED) {
      setLock(ARMED)
      timerRef.current = setTimeout(disarm, LOCK_ARM_MS)
      return
    }
    // 2回目: ロックが外れるだけ。要求は送らない。「切│入」が出る。
    clearTimeout(timerRef.current)
    timerRef.current = null
    setLock(UNLOCKED)
  }

  function handleSide(side) {
    const { lock: nextLock, request: next } = nextBrakeSide(side, brake)
    setLock(nextLock)
    if (next != null) request(next)
  }

  const showLockButton = brake === true && lock !== UNLOCKED
  const disabled = !canToggle || brake == null

  return (
    <div data-testid={`${prefix}-auto-brake-ctl`}>
      <span className={`pill ${brake ? 'ok' : ''}`} data-testid={statusTestId ?? `${prefix}-auto-brake`} data-state={brake === true ? 'on' : brake === false ? 'off' : 'unknown'}>
        {brake === true ? W6_AUTO_BRAKE_ON : brake === false ? W6_AUTO_BRAKE_OFF : '—'}
      </span>
      {showLockButton ? (
        <button
          type="button"
          className={`btn sm lockb${lock === ARMED ? ' caution armed' : ''}`}
          data-testid={`${prefix}-auto-brake-toggle`}
          data-lock={lock}
          disabled={disabled}
          onClick={handleLockPress}
        >
          <IconLock />
          <span>{lock === ARMED ? AB_LOCK_ARMED : AB_LOCK_ON}</span>
        </button>
      ) : (
        <div className="brake-sides" role="group" aria-label="自動ブレーキ">
          <button
            type="button"
            className={`btn sm brake-side off${brake === false ? ' cur' : ''}`}
            data-testid={`${prefix}-auto-brake-off`}
            disabled={disabled || brake !== true}
            onClick={() => handleSide('off')}
          >
            切
          </button>
          <button
            type="button"
            className={`btn sm brake-side on${brake === true ? ' cur' : ''}`}
            data-testid={`${prefix}-auto-brake-on`}
            disabled={disabled || brake !== false}
            onClick={() => handleSide('on')}
          >
            入
          </button>
        </div>
      )}
      {brake === false && (
        <div className="note mt" data-testid={`${prefix}-auto-brake-off-hint`}>{AB_HINT_OFF}</div>
      )}
      {error && <div className="note mt" data-testid={`${prefix}-auto-brake-error`}>{error}</div>}
    </div>
  )
}
