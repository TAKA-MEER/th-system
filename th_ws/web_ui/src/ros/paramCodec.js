// ros/paramCodec.js — rcl_interfaces ParameterValue <-> JS codec, shared by
// ros/useRosbridge.js (audience view) and ros/useTunableParams.js (S-50
// settings screen). Split out of useRosbridge.js by WS-9X so the settings
// screen doesn't have to pull in that 28KB monolith; behaviour is unchanged.

// rcl_interfaces/msg/ParameterType constants
export const PARAM_TYPE = {
  BOOL: 1, INTEGER: 2, DOUBLE: 3, STRING: 4,
  BYTE_ARRAY: 5, BOOL_ARRAY: 6, INTEGER_ARRAY: 7, DOUBLE_ARRAY: 8, STRING_ARRAY: 9,
}

export function decodeParamValue(pv) {
  switch (pv.type) {
    case PARAM_TYPE.BOOL:          return pv.bool_value
    case PARAM_TYPE.INTEGER:       return pv.integer_value
    case PARAM_TYPE.DOUBLE:        return pv.double_value
    case PARAM_TYPE.STRING:        return pv.string_value
    case PARAM_TYPE.BYTE_ARRAY:    return pv.byte_array_value
    case PARAM_TYPE.BOOL_ARRAY:    return pv.bool_array_value
    case PARAM_TYPE.INTEGER_ARRAY: return pv.integer_array_value
    case PARAM_TYPE.DOUBLE_ARRAY:  return pv.double_array_value
    case PARAM_TYPE.STRING_ARRAY:  return pv.string_array_value
    default: return null
  }
}

export function encodeParamValue(value, { isArray = false, isInt = false } = {}) {
  if (isArray) {
    return isInt
      ? { type: PARAM_TYPE.INTEGER_ARRAY, integer_array_value: value }
      : { type: PARAM_TYPE.DOUBLE_ARRAY,  double_array_value: value }
  }
  return isInt
    ? { type: PARAM_TYPE.INTEGER, integer_value: value }
    : { type: PARAM_TYPE.DOUBLE,  double_value: value }
}

// Timeout (ms) so the UI doesn't freeze forever when the backend
// (config_manager etc.) never responds.
export const TUNABLE_SERVICE_TIMEOUT_MS = 5000

export function withTimeout(promise, label) {
  return new Promise((resolve, reject) => {
    const id = setTimeout(
      () => reject(new Error(`${label} timed out`)),
      TUNABLE_SERVICE_TIMEOUT_MS)
    promise.then(
      (v) => { clearTimeout(id); resolve(v) },
      (e) => { clearTimeout(id); reject(e) })
  })
}
