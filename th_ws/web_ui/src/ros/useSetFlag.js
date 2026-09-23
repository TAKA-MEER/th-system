// ros/useSetFlag.js — calls /system/set_flag (th_system_msgs/SetFlag) to
// toggle server-held flags (brief-tracker-default-off §3.4: "人検出を開始/停止").
//
// Only tracker_enabled is used today. The server (state_manager) remains
// authoritative: /system/set_flag can still reject (e.g. tracker_required in
// a "要 person" state), and callers surface `reject_reason_key` via
// i18n/reasons.js when `accepted` comes back false. Mirrors useTrigger.js's
// TEST_MODE: under Playwright there is no rosbridge, so a spec records what
// was sent on `window.__thSetFlagCalls` and can stub a response via
// `window.__thTestSetFlag`. Defaults to a rejected response so a spec that
// doesn't stub behaves like a denial.
import { useCallback } from 'react'
import { useSystemState } from './useSystemState'
import { SERVICES, SRV_TYPES } from './topics'

export const TRACKER_FLAG = 'tracker_enabled'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useSetFlag() {
  const { ros } = useSystemState()

  return useCallback((flag, value, requester = 'web_ui') => {
    if (TEST_MODE) {
      // 何を送ったかを e2e が数えられるように残す（送信経路が切れていると
      // 「ボタンが繋がっていない」に気づけない。変異③の標的）。
      window.__thSetFlagCalls = window.__thSetFlagCalls ?? []
      window.__thSetFlagCalls.push({ flag, value, requester })
      const stub = window.__thTestSetFlag?.[flag]
      if (stub) return Promise.resolve(stub(value))
      return Promise.resolve({ accepted: false, reject_reason_key: null })
    }
    return new Promise((resolve, reject) => {
      const ROSLIB = window.ROSLIB
      if (!ros || !ROSLIB) {
        reject(new Error('rosbridge is not connected'))
        return
      }
      const svc = new ROSLIB.Service({
        ros,
        name: SERVICES.SET_FLAG,
        serviceType: SRV_TYPES.SET_FLAG,
      })
      svc.callService(
        new ROSLIB.ServiceRequest({ flag, value, requester }),
        (res) => resolve(res),
        (err) => reject(err),
      )
    })
  }, [ros])
}