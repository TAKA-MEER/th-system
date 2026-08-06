// ============================================================
// captionSink.js — 音を鳴らさずアナウンスを字幕ログに変える
//
// useVoiceTriggers が voice に求めるのは announce / setCondition の 2 つだけ
// (useVoiceTriggers.js 参照)。同じ形のシンクを渡せば、51 件のトリガ判定
// ロジックをそのまま観客表示に流用できる。
//
// **voiceQueue.js / audioPlayer.js を import しないこと。**
// 観客画面は ROS2 スタックのホスト機で動くため、ここから音が出ると
// VISION.md §7.1/§7.2 の「ロボット側スピーカーは持たない」「音声は試験員の
// タブレットに閉じる」という決定に反する。import しないこと自体が
// 「音が出ない」ことの構造的な保証になっている。
// ============================================================

import { useCallback, useRef, useState } from 'react'

import { getAnnouncement } from '../voice/announcements.js'

/** 画面に残す字幕の件数。増やしても古いものは読まれない */
const LOG_LIMIT = 8

export function useCaptionSink() {
  const [log, setLog] = useState([])       // [{ key, id, text, layer, at }] 新しい順
  const [conditions, setConditions] = useState([])  // 継続中の状態 [{ id, text }]

  // 継続状態の立ち上がり判定用。voiceQueue.setCondition と同じ冪等セマンティクス
  const activeRef = useRef(new Set())
  const seqRef = useRef(0)

  const push = useCallback((entry) => {
    seqRef.current += 1
    const item = {
      key:   seqRef.current,
      id:    entry.id,
      text:  entry.text,
      layer: entry.layer,
      at:    new Date(),
    }
    setLog((prev) => [item, ...prev].slice(0, LOG_LIMIT))
  }, [])

  const announce = useCallback((id) => {
    const entry = getAnnouncement(id)
    if (!entry) {
      console.warn('[caption] 未知のアナウンス id:', id)
      return false
    }
    // レイヤのゲートはしない。音声側でデモ実況が既定 OFF なのは
    // 「試験員の耳に何を入れるか」の話で、観客画面には当てはまらない
    push(entry)
    return true
  }, [push])

  // 立ち上がりのみログへ。再発話タイマーは持たない —— 同じ字幕が
  // 10 秒おきに積まれても画面では邪魔になるだけで情報が増えない。
  // 継続していること自体は conditions (常時表示のチップ) で表す
  const setCondition = useCallback((id, active) => {
    const entry = getAnnouncement(id)
    if (!entry) {
      console.warn('[caption] 未知のアナウンス id:', id)
      return
    }
    const set = activeRef.current
    if (active) {
      if (set.has(id)) return          // 冪等: 既に立っていれば何もしない
      set.add(id)
      push(entry)
    } else {
      if (!set.has(id)) return
      set.delete(id)
    }
    setConditions([...set].map((cid) => {
      const e = getAnnouncement(cid)
      return { id: cid, text: e?.text ?? cid }
    }))
  }, [push])

  return { log, conditions, announce, setCondition }
}
