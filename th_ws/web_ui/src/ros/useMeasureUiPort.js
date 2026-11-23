// ros/useMeasureUiPort.js — 測定機能（fk-system）の画面の開く先を読む。
//
// registry の measure_ui_port（given 8091）。camera_ui_url と同じ経路
// （/params/get の effective）で読む（ビルド時生成物の値は読まない。
// S-50 では変えない。読めないときは 8091。
// S-60「測定機能の画面へ」と S-21「測定機能へ」で共用する。
import { useEffect, useState } from 'react'
import { useParamsOverrides } from './useParamsOverrides.js'

export const DEFAULT_MEASURE_UI_PORT = 8091

export function measureUiUrl(hostname, port) {
  const p = (typeof port === 'number' && Number.isFinite(port))
    ? port : DEFAULT_MEASURE_UI_PORT
  return `http://${hostname}:${p}/`
}

export function openMeasureUi(port) {
  window.location.assign(measureUiUrl(window.location.hostname, port))
}

export function useMeasureUiPort(ros) {
  const { getParams } = useParamsOverrides(ros)
  const [measurePort, setMeasurePort] = useState(null)
  useEffect(() => {
    let alive = true
    getParams(['measure_ui_port'])
      .then((payload) => {
        if (alive) setMeasurePort(payload?.effective?.measure_ui_port ?? null)
      })
      .catch(() => { if (alive) setMeasurePort(null) })
    return () => { alive = false }
  }, [getParams])
  return measurePort
}
