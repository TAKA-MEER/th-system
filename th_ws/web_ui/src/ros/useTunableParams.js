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
    if (TEST_MODE || !ros) {
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
