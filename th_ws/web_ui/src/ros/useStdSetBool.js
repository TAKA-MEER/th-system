// ros/useStdSetBool.js — std_srvs/SetBool 形のサービス呼び出し（request = { data }）。
// useStdTrigger.js と同じ流儀。A-11: S-20 の「地図作成開始」は冪等な
// /slam_control/set_mapping{data:true} で送る（toggle_mapping は押下が重なると
// 開始のあとに停止へ反転する）。TEST_MODE では呼び出しを
// window.__thStdSetBoolCalls に ({ service, data }) で記録する。
import { useCallback } from 'react'
import { useSystemState } from './useSystemState'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useStdSetBool(serviceName) {
  const { ros } = useSystemState()

  return useCallback((data) => {
    if (TEST_MODE) {
      window.__thStdSetBoolCalls = window.__thStdSetBoolCalls ?? []
      window.__thStdSetBoolCalls.push({ service: serviceName, data: !!data })
      const stub = window.__thTestServices?.[serviceName]
      return Promise.resolve(stub ?? { success: true, message: '' })
    }
    return new Promise((resolve, reject) => {
      const ROSLIB = window.ROSLIB
      if (!ros || !ROSLIB) {
        reject(new Error('rosbridge is not connected'))
        return
      }
      const svc = new ROSLIB.Service({ ros, name: serviceName, serviceType: 'std_srvs/SetBool' })
      svc.callService(new ROSLIB.ServiceRequest({ data: !!data }),
        (res) => resolve(res), (err) => reject(err))
    })
  }, [ros, serviceName])
}
