// ros/useCalibStatus.js — subscribes to /calib/status (CalibStatus) for S-40
// (WP-MAINT-03). calib_runner publishes it at 2 Hz and on every change.
//
// `detail` is a JSON string on the wire (履歴・最終校正日・警告・IMU の各値・進捗。
// calib_runner._detail 参照)。画面が使いやすいよう、ここで一度だけオブジェクトに
// 解釈して渡す（壊れた JSON は空オブジェクト。落とさない）。
//
// TEST_MODE mirrors ros/useOpcheckStatus.js: window.__thTestCalibStatus seeds the
// initial value, window.__thSetTestCalibStatus mutates it after mount. `detail`
// may be given as an object in tests (it is stringified-free here).
import { useEffect, useRef, useState } from 'react'
import { TOPICS, MSG_TYPES } from './topics'
import { parseCalibDetail } from '../screens/calibCore.js'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

function normalise(msg) {
  if (!msg) return null
  return {
    item: msg.item ?? '',
    step: msg.step ?? '',
    result: msg.result ?? '',
    preview_before: msg.preview_before ?? '',
    preview_after: msg.preview_after ?? '',
    detail: parseCalibDetail(msg.detail),
  }
}

export function useCalibStatus(ros) {
  const topicRef = useRef(null)
  const [status, setStatus] = useState(
    TEST_MODE ? normalise(window.__thTestCalibStatus ?? null) : null)

  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestCalibStatus = (v) => setStatus(normalise(v))
    return () => { delete window.__thSetTestCalibStatus }
  }, [])

  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros, name: TOPICS.CALIB_STATUS, messageType: MSG_TYPES.CALIB_STATUS,
    })
    topicRef.current.subscribe((msg) => setStatus(normalise(msg)))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return { status }
}
