// ros/useTrigger.js — calls /system/trigger (UiTrigger.srv) for every ui.*
// event except ui.jog.hold, which goes over the /ui/jog_lease topic instead
// (DetailedDesign-webui.md §5 / DetailedDesign-names.md §8).
//
// The UI may pre-decide whether a button should look pressable, but that is
// a display convenience only — th_state is authoritative and can still
// reject the call. Callers should surface `reject_reason_key` via
// i18n/reasons.js when `accepted` comes back false.
import { useCallback } from 'react'
import { useSystemState } from './useSystemState'
import { SERVICES, SRV_TYPES } from './topics'

// TEST_MODE: no rosbridge under Playwright, so /system/trigger would always
// reject ("rosbridge is not connected"). Mirror useStdTrigger.js's
// window.__thTestServices: an e2e spec can stub a specific trigger via
// window.__thTestTrigger[triggerName], letting the accepted path (e.g.
// S-01's ui.enter_mode -> S-11) be exercised offline. Defaults to a rejected
// response ({ accepted: false, reject_reason_key: null }), so a spec that
// doesn't stub a trigger behaves like a denial.
const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useTrigger() {
  const { ros } = useSystemState()

  return useCallback((trigger, argJson = {}, requester = 'web_ui') => {
    if (TEST_MODE) {
      // 呼ばれたことを e2e が数えられるように残す（何を送ったかを
      // 検証できないと「ボタンが繋がっていない」に気づけない）。
      window.__thTriggerCalls = window.__thTriggerCalls ?? []
      window.__thTriggerCalls.push({ trigger, argJson, requester })
      const stub = window.__thTestTrigger?.[trigger]
      if (stub) return Promise.resolve(stub)
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
        name: SERVICES.SYSTEM_TRIGGER,
        serviceType: SRV_TYPES.UI_TRIGGER,
      })
      svc.callService(
        new ROSLIB.ServiceRequest({
          trigger,
          arg_json: JSON.stringify(argJson),
          requester,
        }),
        (res) => resolve(res),
        (err) => reject(err),
      )
    })
  }, [ros])
}
