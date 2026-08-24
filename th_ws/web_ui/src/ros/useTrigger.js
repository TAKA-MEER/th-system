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

export function useTrigger() {
  const { ros } = useSystemState()

  return useCallback((trigger, argJson = {}, requester = 'web_ui') => {
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
