// ros/useOnsiteService.js — onsite service calls: S-20's /onsite/two_point
// (TwoPointPress) / /onsite/edit_pin (EditPin), and S-21's /onsite/declare_home
// (DeclareHome) / /onsite/select_pin (GoToPanel) — WP-UI-06 / WP-UI-07.
// These are not /system/trigger events, so they live outside useTrigger.js;
// the response shapes are the message definitions' own
// (accepted/reject_reason_key/yaw and success/message/offset_*).
//
// /onsite/select_pin is exposed by venue_navigator but is NOT in names.json
// (the WebUI name dictionary), so inject it here as a local constant instead of
// ros/topics.js -- same carve-out as useOdomPose.js's ODOM_TOPIC. topics.js
// still carries the message type (SRV_TYPES.ONSITE_SELECT_PIN: the dictionary
// unit test only gates TOPICS / SERVICES name values).
//
// TEST_MODE: same contract as useTrigger.js / useStdTrigger.js — every call
// is recorded on window.__thOnsiteServiceCalls ({ service, request }) and
// answered from a stub read at call time (window.__thTestTwoPoint /
// window.__thTestEditPin / __thTestDeclareHome / __thTestSelectPin). Without a
// stub it resolves as a denial, so a spec that doesn't stub a service behaves
// like a rejection. Recording every call is what lets e2e prove a button is
// actually wired (opencode-bigpickle-evaluation's known hole: an inert button
// passing green).
import { useCallback } from 'react'
import { useSystemState } from './useSystemState'
import { SERVICES, SRV_TYPES } from './topics'

// names.json 辞書ゲート対象外（上記コメント参照）。
const SELECT_PIN_SERVICE = '/onsite/select_pin'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

function defaultStub(service) {
  if (service === SERVICES.ONSITE_TWO_POINT) {
    return { accepted: false, reject_reason_key: null, yaw: 0 }
  }
  return { success: false, message: '' }
}

export function useOnsiteService() {
  const { ros } = useSystemState()

  const callService = useCallback((service, serviceType, request, stubKey) => {
    if (TEST_MODE) {
      window.__thOnsiteServiceCalls = window.__thOnsiteServiceCalls ?? []
      window.__thOnsiteServiceCalls.push({ service, request })
      const stub = window[stubKey]
      return Promise.resolve(stub !== undefined ? stub : defaultStub(service))
    }
    return new Promise((resolve, reject) => {
      const ROSLIB = window.ROSLIB
      if (!ros || !ROSLIB) {
        reject(new Error('rosbridge is not connected'))
        return
      }
      const svc = new ROSLIB.Service({ ros, name: service, serviceType })
      svc.callService(new ROSLIB.ServiceRequest(request), resolve, reject)
    })
  }, [ros])

  return {
    twoPoint: (request) => callService(
      SERVICES.ONSITE_TWO_POINT, SRV_TYPES.ONSITE_TWO_POINT, request, '__thTestTwoPoint'),
    editPin: (request) => callService(
      SERVICES.ONSITE_EDIT_PIN, SRV_TYPES.ONSITE_EDIT_PIN, request, '__thTestEditPin'),
    declareHome: (request) => callService(
      SERVICES.ONSITE_DECLARE_HOME, SRV_TYPES.ONSITE_DECLARE_HOME, request, '__thTestDeclareHome'),
    selectPin: (request) => callService(
      SELECT_PIN_SERVICE, SRV_TYPES.ONSITE_SELECT_PIN, request, '__thTestSelectPin'),
  }
}