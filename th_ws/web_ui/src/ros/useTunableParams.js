// ros/useTunableParams.js — parameter get/apply/save for the S-50 settings
// screen (WS-9X). Mirrors ros/useRouteCatalog.js: takes the shared `ros`
// object from useSystemState(), owns nothing across renders, and does not
// go through the useRosbridge.js monolith.
//
//   getTunableParams(node, names)      -> Promise<{name: value}>   (read-only,
//                                          calls <node>/get_parameters directly)
//   applyTunableParam(node, name, val, opts) -> Promise<res>       (live apply via
//                                          /config_manager/set_tunable_params;
//                                          config_manager re-checks the mode
//                                          server-side and rejects outside
//                                          IDLE/MANUAL)
//   saveTunableParams(node)            -> Promise<res>             (write YAML via
//                                          /config_manager/save_tunable_params)
//
// The three functions come verbatim from useRosbridge.js's settings-panel
// section; only the `ros` handle source differs. In TEST_MODE (e2e) every
// call rejects immediately so no rosbridge round-trip is attempted (U-3).
import { useMemo } from 'react'
import { decodeParamValue, encodeParamValue, withTimeout } from './paramCodec.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const NO_BACKEND = () => Promise.reject(new Error('rosbridge not connected'))

export function useTunableParams(ros) {
  return useMemo(() => {
    // e2e 用の部分スタブ (SG-B9)。`window.__thTunableStubs = { [node]: { values } }`
    // があるノードだけ解決し、無いノードは reject（本番でノードが起動していない
    // ときの `get_parameters` 到達不能と同じ扱い）にする。スタブが無ければ
    // 従来どおり全 reject（TEST_MODE）/ ros 未接続時の reject。
    // apply/save の呼び出しは `window.__thTunableApplyCalls` /
    // `window.__thTunableSaveCalls` に記録し、成功を返す。
    const stubsOf = () => (
      (typeof window !== 'undefined' && window.__thTunableStubs) || null
    )
    if (TEST_MODE || !ros) {
      const stubs = stubsOf()
      if (stubs) {
        return {
          getTunableParams: (nodeName, paramNames) => {
            const stub = stubs[nodeName]
            if (stub && stub.values) {
              const out = {}
              paramNames.forEach((name) => { out[name] = stub.values[name] })
              return Promise.resolve(out)
            }
            return Promise.reject(new Error(`no tunable stub for ${nodeName}`))
          },
          applyTunableParam: (nodeName, paramName, value) => {
            window.__thTunableApplyCalls = window.__thTunableApplyCalls ?? []
            window.__thTunableApplyCalls.push({ nodeName, paramName, value })
            return Promise.resolve({ success: true, message: 'OK' })
          },
          saveTunableParams: (nodeName) => {
            window.__thTunableSaveCalls = window.__thTunableSaveCalls ?? []
            window.__thTunableSaveCalls.push({ nodeName })
            return Promise.resolve({ success: true, message: 'OK' })
          },
        }
      }
      return {
        getTunableParams: NO_BACKEND,
        applyTunableParam: NO_BACKEND,
        saveTunableParams: NO_BACKEND,
      }
    }
    const ROSLIB = window.ROSLIB

    const getTunableParams = (nodeName, paramNames) =>
      withTimeout(new Promise((resolve, reject) => {
        if (!ROSLIB) { reject(new Error('roslibjs is not loaded')); return }
        const svc = new ROSLIB.Service({
          ros,
          name: `/${nodeName}/get_parameters`,
          serviceType: 'rcl_interfaces/GetParameters',
        })
        svc.callService(
          new ROSLIB.ServiceRequest({ names: paramNames }),
          (res) => {
            const out = {}
            paramNames.forEach((name, i) => { out[name] = decodeParamValue(res.values[i]) })
            resolve(out)
          },
          (err) => reject(err),
        )
      }), `fetching parameters for ${nodeName}`)

    const applyTunableParam = (nodeName, paramName, value, opts) =>
      withTimeout(new Promise((resolve, reject) => {
        if (!ROSLIB) { reject(new Error('roslibjs is not loaded')); return }
        const svc = new ROSLIB.Service({
          ros,
          name: '/config_manager/set_tunable_params',
          serviceType: 'th_system_msgs/SetTunableParams',
        })
        svc.callService(
          new ROSLIB.ServiceRequest({
            node_name: nodeName,
            parameters: [{ name: paramName, value: encodeParamValue(value, opts) }],
          }),
          (res) => { if (!res.success) console.warn('parameter apply failed:', res.message); resolve(res) },
          (err) => reject(err),
        )
      }), `applying ${nodeName}.${paramName}`)

    const saveTunableParams = (nodeName) =>
      withTimeout(new Promise((resolve, reject) => {
        if (!ROSLIB) { reject(new Error('roslibjs is not loaded')); return }
        const svc = new ROSLIB.Service({
          ros,
          name: '/config_manager/save_tunable_params',
          serviceType: 'th_system_msgs/SaveTunableParams',
        })
        svc.callService(
          new ROSLIB.ServiceRequest({ node_name: nodeName }),
          (res) => { if (!res.success) console.warn('parameter save failed:', res.message); resolve(res) },
          (err) => reject(err),
        )
      }), `saving ${nodeName}`)

    return { getTunableParams, applyTunableParam, saveTunableParams }
  }, [ros])
}
