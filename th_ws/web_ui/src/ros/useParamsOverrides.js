// ros/useParamsOverrides.js — S-50 の速度プリセット区画が /params/get・
// /params/set を呼ぶためのフック（SG-B8。Spec-params.md §6「次の起動から効く」）。
//
// 画面から変えた値は走行中の値に触れず、overrides.yaml に出どころ付きで保存
// される。起動時の設定生成が既定値に重ねるので、効くのは次の起動から。
// /params/get は {effective（いま効いている値）, pending（保存済みで次回から
// 効く上書き値）, origins（出どころ）, restart_required} を返す。
//
// TEST_MODE（window.__thTestState 定義時）は useTunableParams.js と同じ流儀:
// window.__thParamsStubs（{get: {...}}）があればそれを返し、無ければ reject。
// /params/set の呼び出しは window.__thParamsSetCalls に記録する。
import { useMemo } from 'react'
import { SERVICES, SRV_TYPES } from './topics.js'
import { withTimeout } from './paramCodec.js'
import { pendingDiff } from './paramsPending.js'

export { pendingDiff }

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

export function useParamsOverrides(ros) {
  return useMemo(() => {
    if (TEST_MODE || !ros) {
      const stubs = (typeof window !== 'undefined' && window.__thParamsStubs) || null
      if (stubs) {
        return {
          getParams: (names) => Promise.resolve(
            typeof stubs.get === 'function' ? stubs.get(names) : (stubs.get ?? {}),
          ),
          setParams: (values, setBy, reason) => {
            window.__thParamsSetCalls = window.__thParamsSetCalls ?? []
            window.__thParamsSetCalls.push({ values, setBy, reason })
            const stub = stubs.set
            return Promise.resolve(
              typeof stub === 'function' ? stub(values) : (stub ?? { success: true, message: 'OK' }),
            )
          },
        }
      }
      const NO_BACKEND = () => Promise.reject(new Error('rosbridge not connected'))
      return { getParams: NO_BACKEND, setParams: NO_BACKEND }
    }
    const ROSLIB = window.ROSLIB

    const getParams = (names) =>
      withTimeout(new Promise((resolve, reject) => {
        if (!ROSLIB) { reject(new Error('roslibjs is not loaded')); return }
        const svc = new ROSLIB.Service({
          ros,
          name: SERVICES.PARAMS_GET,
          serviceType: SRV_TYPES.PARAMS_GET,
        })
        svc.callService(
          new ROSLIB.ServiceRequest({ names }),
          (res) => {
            try {
              resolve(JSON.parse(res.json))
            } catch (e) {
              reject(e)
            }
          },
          (err) => reject(err),
        )
      }), 'fetching /params/get')

    const setParams = (values, setBy, reason) =>
      withTimeout(new Promise((resolve, reject) => {
        if (!ROSLIB) { reject(new Error('roslibjs is not loaded')); return }
        const svc = new ROSLIB.Service({
          ros,
          name: SERVICES.PARAMS_SET,
          serviceType: SRV_TYPES.PARAMS_SET,
        })
        svc.callService(
          new ROSLIB.ServiceRequest({
            json: JSON.stringify({ values, set_by: setBy, reason }),
          }),
          (res) => resolve(res),
          (err) => reject(err),
        )
      }), 'calling /params/set')

    return { getParams, setParams }
  }, [ros])
}
