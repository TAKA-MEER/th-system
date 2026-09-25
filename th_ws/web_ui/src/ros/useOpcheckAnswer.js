// ros/useOpcheckAnswer.js — calls /opcheck/answer (AnswerCheck) for S-30's
// ESTOP item visual-confirm question ("実物と一致していますか" -- WP-UI-08 /
// DetailedDesign-maintenance.md §2.2).
//
// Not a /system/trigger event (there is no ui.answer in transitions.yaml):
// opcheck_runner.py's _on_answer handles this directly, independent of the
// FSM. Same TEST_MODE contract as ros/useOnsiteService.js -- every call is
// recorded on window.__thOpcheckAnswerCalls ({ item, ok }) so e2e can prove
// the "はい/いいえ" buttons are actually wired, and answered from
// window.__thTestOpcheckAnswer (a function) or a rejection by default.
import { useCallback } from 'react'
import { useSystemState } from './useSystemState'
import { SERVICES, SRV_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useOpcheckAnswer() {
  const { ros } = useSystemState()

  return useCallback((item, ok) => {
    if (TEST_MODE) {
      window.__thOpcheckAnswerCalls = window.__thOpcheckAnswerCalls ?? []
      window.__thOpcheckAnswerCalls.push({ item, ok })
      const stub = window.__thTestOpcheckAnswer
      return Promise.resolve(stub ? stub(item, ok) : { accepted: false })
    }
    return new Promise((resolve, reject) => {
      const ROSLIB = window.ROSLIB
      if (!ros || !ROSLIB) {
        reject(new Error('rosbridge is not connected'))
        return
      }
      const svc = new ROSLIB.Service({
        ros, name: SERVICES.OPCHECK_ANSWER, serviceType: SRV_TYPES.OPCHECK_ANSWER,
      })
      svc.callService(new ROSLIB.ServiceRequest({ item, ok }), resolve, reject)
    })
  }, [ros])
}
