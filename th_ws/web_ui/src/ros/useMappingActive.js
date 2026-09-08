// ros/useMappingActive.js — subscribes to /slam_control/mapping_active
// (std_msgs/Bool, transient_local) for the S-20 地図タブの表示ゲート
// （brief-onsite-ux2 F-6）。slam_control.py が現在の地図作成有無を publish する。
//
// 値は 3 状態: null（まだ 1 通も受信していない＝不明）/ true（作成中）/
// false（停止中）。「地図作成開始」ボタンを押したとき、not-yet-known の状態で
// /slam_control/toggle_mapping を呼ぶと、実は起動時から動いている地図作成
// （enable_route_slam:=true）を誤って止めてしまう恐れがある（F-6 の核心の事故）。
// 呼び出し側（S20Prep.jsx）は null の間はボタンを非活性にして、この race を避ける。
//
// Mirrors ros/useHomeDeclared.js（transient_local Bool の購読）。topic 名は
// names.json（辞書ゲート対象）に無いためローカル定数で持つ。TEST_MODE のシードは
// window.__thTestMappingActive（mutate 用に window.__thSetTestMappingActive も足す）。
import { useEffect, useRef, useState } from 'react'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

const MAPPING_ACTIVE_TOPIC = '/slam_control/mapping_active'
const MAPPING_ACTIVE_MSG = 'std_msgs/Bool'

export function useMappingActive(ros) {
  const topicRef = useRef(null)
  const [active, setActive] = useState(
    TEST_MODE ? (window.__thTestMappingActive ?? null) : null)

  // TEST_MODE: let e2e mutate the seeded value after mount.
  useEffect(() => {
    if (!TEST_MODE) return undefined
    window.__thSetTestMappingActive = (v) => setActive(v == null ? null : !!v)
    return () => { delete window.__thSetTestMappingActive }
  }, [])

  // Real mode: hold a rosbridge Topic across re-renders.
  useEffect(() => {
    if (TEST_MODE || !ros) { topicRef.current = null; return undefined }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return undefined
    topicRef.current = new ROSLIB.Topic({
      ros,
      name: MAPPING_ACTIVE_TOPIC,
      messageType: MAPPING_ACTIVE_MSG,
      queue_length: 1,   // transient_local + 0.5s 再送。最新のみで十分。
    })
    topicRef.current.subscribe((msg) => setActive(!!msg?.data))
    return () => {
      topicRef.current?.unsubscribe()
      topicRef.current = null
    }
  }, [ros])

  return active
}
