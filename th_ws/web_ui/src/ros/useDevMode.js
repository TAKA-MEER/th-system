// ros/useDevMode.js — 開発モードの WebUI 接続 (WP-DEV-01B)。
//
// 読みは /system/dev_mode 購読 (transient_local・後から開いても最新が届く)。
// 画面の表示はこちらが正 (Spec-webui.md §5.1)。
// 書きは /connectivity_checker/set_parameters への直接呼び出し
// (rcl_interfaces/SetParameters。`ros2 param set` と同じ標準サービス)。
// /config_manager/set_tunable_params 経由ではない — config_manager は
// connectivity_checker を知らず「未知のノード」で拒否し、IDLE/MANUAL 外も
// 拒否する。開発モードは INIT を抜けるために要るので二重に使えない。
//
// TEST_MODE (window.__thTestState 定義時) は useLimiterStatus.js と同じ流儀:
// 購読も送信もせず、window.__thTestDevMode を種にし、
// window.__thSetTestDevMode で表示を更新、window.__thDevParamCalls に送信を記録。
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'
import { encodeParamValue, withTimeout } from './paramCodec.js'
import {
  DEV_MODE_NODE, DEV_PARAM_MASTER,
  parseDevModeState,
} from './devModeState.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

// TEST_MODE の同報先（上の useEffect を見る）。本番では使わない。
const testSetters = new Set()

const NO_BACKEND = () => Promise.reject(new Error('rosbridge not connected'))

// /connectivity_checker/{set,get}_parameters はノード固有の標準サービス名で
// 辞書 (names.json) に無いため、useOnsiteService.js の select_pin と同じく
// ローカル定数で扱い TOPICS/SERVICES には載せない (辞書ゲートの流儀)。
const SET_PARAMS_SVC = (node) => `/${node}/set_parameters`
const SET_PARAMS_TYPE = 'rcl_interfaces/SetParameters'

export function useDevMode(ros) {
  const topicRef = useRef(null)
  const [dev, setDev] = useState(
    TEST_MODE ? parseDevModeState(window.__thTestDevMode ?? null) : null)

  // TEST_MODE: e2e が表示を更新する (完了条件 2)。AppShell と S-50 で
  // 二重にマウントされるため、生きている全インスタンスへ同報する
  // （後勝ちの単一キーでは S-50 を開いた途端にヘッダが追従しなくなる）。
  useEffect(() => {
    if (!TEST_MODE) return undefined
    testSetters.add(setDev)
    window.__thSetTestDevMode = (v) => {
      const next = parseDevModeState(v)
      testSetters.forEach((fn) => fn(next))
    }
    return () => {
      testSetters.delete(setDev)
      if (testSetters.size === 0) delete window.__thSetTestDevMode
    }
  }, [])

  // 本番: /system/dev_mode を購読する。std_msgs/String に JSON。
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: TOPICS.DEV_MODE,
      messageType: MSG_TYPES.STRING,
      queue_length: 1,
    })
    topicRef.current.subscribe((msg) => {
      const next = parseDevModeState(msg?.data ?? null)
      if (next) setDev(next)
    })
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  const setDevParam = (name, value, node = DEV_MODE_NODE) => {
    if (TEST_MODE) {
      // useTrigger.js の __thTriggerCalls と同じ狙い: 送ったことを記録する。
      window.__thDevParamCalls = window.__thDevParamCalls ?? []
      window.__thDevParamCalls.push({ node, name, value })
      return Promise.resolve({ successful: true })
    }
    if (!ros) return NO_BACKEND()
    return withTimeout(new Promise((resolve, reject) => {
      const ROSLIB = window.ROSLIB
      if (!ROSLIB) { reject(new Error('roslibjs is not loaded')); return }
      const svc = new ROSLIB.Service({
        ros,
        name: SET_PARAMS_SVC(node),
        serviceType: SET_PARAMS_TYPE,
      })
      svc.callService(
        new ROSLIB.ServiceRequest({
          parameters: [{ name, value: encodeParamValue(value, { isBool: true }) }],
        }),
        (res) => {
          const bad = (res?.results ?? []).filter((r) => !r.successful)
          if (bad.length > 0) console.warn('dev parameter apply failed:', bad)
          resolve(res)
        },
        (err) => reject(err),
      )
    }), `applying ${node}.${name}`)
  }

  return { dev, setDevParam, masterParam: DEV_PARAM_MASTER, node: DEV_MODE_NODE }
}
