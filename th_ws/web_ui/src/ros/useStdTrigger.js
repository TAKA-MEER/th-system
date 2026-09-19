// ros/useStdTrigger.js — calls a std_srvs/Trigger-shaped ROS2 service (no
// request fields; response = {success, message}).
//
// ros/useTrigger.js is hardwired to /system/trigger (th_system_msgs/UiTrigger),
// which doesn't fit /shutdown/prepare / /shutdown/execute
// (DetailedDesign-names.md §5.2: both are plain std_srvs/Trigger). This hook
// covers those two (WP-UI-02, DetailedDesign-state.md §12.5).
//
// Test hook (mirrors ros/useSystemState.js's window.__thTestState): when
// running under Playwright with no rosbridge backend, resolve from
// `window.__thTestServices[serviceName]` instead of opening a socket, so
// e2e/s01-shutdown-flow.spec.js can drive the 5-step flow (§12.5) offline
// (U-3). Defaults to `{ success: true, message: '' }` if a spec doesn't
// configure a stub for the service it calls.
//
// brief-onsite-ux2 F-6: every TEST_MODE call is also recorded on
// window.__thStdTriggerCalls ({ service }) -- same "record every call" idea
// as ros/useOnsiteService.js's __thOnsiteServiceCalls -- so an e2e spec can
// prove the S-20 map gate did NOT call /slam_control/toggle_mapping when
// mapping was already active (calling it there would stop a mapping run
// that started at launch, the exact failure mode F-6 exists to avoid).
import { useCallback } from 'react'
import { useSystemState } from './useSystemState'
import { SRV_TYPES } from './topics'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useStdTrigger(serviceName) {
  const { ros } = useSystemState()

  return useCallback(() => {
    if (TEST_MODE) {
      window.__thStdTriggerCalls = window.__thStdTriggerCalls ?? []
      window.__thStdTriggerCalls.push({ service: serviceName })
      const stub = window.__thTestServices?.[serviceName]
      return Promise.resolve(stub ?? { success: true, message: '' })
    }
    return new Promise((resolve, reject) => {
      const ROSLIB = window.ROSLIB
      if (!ros || !ROSLIB) {
        reject(new Error('rosbridge is not connected'))
        return
      }
      const svc = new ROSLIB.Service({
        ros, name: serviceName, serviceType: SRV_TYPES.STD_TRIGGER,
      })
      svc.callService(new ROSLIB.ServiceRequest({}), (res) => resolve(res), (err) => reject(err))
    })
  }, [ros, serviceName])
}
