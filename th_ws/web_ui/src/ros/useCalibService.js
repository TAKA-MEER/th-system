// ros/useCalibService.js — S-40's three direct calib_runner services
// (WP-MAINT-03): /calib/submit (SubmitCalib), /calib/start (StartCalib),
// /calib/rollback (RollbackCalib).
//
// Item selection / next / abort are NOT here: they go through /system/trigger
// (ui.calib_item / ui.calib_next / ui.abort — ros/useTrigger.js) so the FSM and
// the screen never disagree. These three have no FSM trigger:
//   submit   — the measured value the operator typed (ui.* has no argument slot
//              for a float that th_state would forward; calib_runner owns it)
//   start    — re-run after a verification NG (S2 の再走行。FSM は動かさない)
//   rollback — history generation → current (LIST 専用。calib_runner が state も確認する)
//
// Same TEST_MODE contract as ros/useOpcheckAnswer.js: every call is recorded on
// window.__thCalibCalls ({ service, ...request }) so e2e can prove the buttons are
// really wired to the production send path, and answered from
// window.__thTestCalib[service] (a plain value) or a rejection by default.
import { useCallback, useMemo } from 'react'
import { useSystemState } from './useSystemState'
import { SERVICES, SRV_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useCalibService() {
  const { ros } = useSystemState()

  const call = useCallback((service, name, type, request) => {
    if (TEST_MODE) {
      window.__thCalibCalls = window.__thCalibCalls ?? []
      window.__thCalibCalls.push({ service, ...request })
      const stub = window.__thTestCalib?.[service]
      return Promise.resolve(stub ?? { success: false })
    }
    return new Promise((resolve, reject) => {
      const ROSLIB = window.ROSLIB
      if (!ros || !ROSLIB) {
        reject(new Error('rosbridge is not connected'))
        return
      }
      const svc = new ROSLIB.Service({ ros, name, serviceType: type })
      svc.callService(new ROSLIB.ServiceRequest(request), resolve, reject)
    })
  }, [ros])

  return useMemo(() => ({
    submit: (item, measured, argJson = {}) => call(
      'submit', SERVICES.CALIB_SUBMIT, SRV_TYPES.CALIB_SUBMIT,
      { item, measured, arg_json: JSON.stringify(argJson) }),
    start: (item) => call(
      'start', SERVICES.CALIB_START, SRV_TYPES.CALIB_START, { item }),
    rollback: (item, generation) => call(
      'rollback', SERVICES.CALIB_ROLLBACK, SRV_TYPES.CALIB_ROLLBACK, { item, generation }),
  }), [call])
}
