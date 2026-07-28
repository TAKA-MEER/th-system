// ============================================================
// voiceQueue.js — 再生キュー・優先度・再発話 (VISION.md §7.6)
//
// React を一切 import しない。キュー規則という最もバグりやすい部分を
// UI から切り離し、開発パネルやコンソールから直接叩いて追えるようにする。
//
// 公開 API は「単発」と「継続」の2つに分かれる:
//   announce(id)              — モード遷移・サービス応答などの単発イベント
//   setCondition(id, active)  — フォルト継続・切断中などの持続状態。冪等
//
// setCondition が冪等であることが重要で、呼び出し側は毎レンダー同じ真偽値を
// 渡してよい。変化したときだけ効くので、エッジ検出を React 側に持たずに済み、
// StrictMode の二重実行も無害になる。
// ============================================================

import { getAnnouncement, LAYER } from './announcements.js'
import { playEntry } from './audioPlayer.js'

/** 優先度。VISION.md §7.6 は2段階しかなく、レイヤと一対一で対応する */
const LAYER_RANK = {
  [LAYER.SAFETY]: 2,
  [LAYER.DEMO]:   1,
}

// 安全通知が入ったとき、再生中のデモ実況に加えて「待機中の」デモ実況も捨てる。
// VISION.md §7.6 は「再生中のデモ実況は破棄する」としか書いていないが、古い実況を
// 後から鳴らすと安全通知が数秒遅れるため広めに解釈している。違和感があれば false に。
const DISCARD_PENDING_DEMO_ON_SAFETY = true

function createVoiceQueue() {
  let current   = null   // { entry, cancel, token } | null
  let playToken = 0
  const queue      = []                              // 待機中の entry
  const conditions = new Map()                       // id → { timerId, count }
  const layers     = { [LAYER.SAFETY]: true, [LAYER.DEMO]: false }
  let lastDropped  = null                            // { id, reason }

  const subscribers = new Set()
  let snapshot = buildSnapshot()

  function buildSnapshot() {
    return {
      playingId:    current?.entry.id ?? null,
      playingLayer: current?.entry.layer ?? null,
      queueIds:     queue.map((e) => e.id),
      conditionIds: [...conditions.keys()],
      layers:       { ...layers },
      lastDropped,
    }
  }

  function emit() {
    snapshot = buildSnapshot()
    for (const fn of subscribers) fn(snapshot)
  }

  function drop(id, reason) {
    lastDropped = { id, reason, at: Date.now() }
    emit()
  }

  // ── 再生の駆動 ──────────────────────────────────────────
  function startNext() {
    if (current || queue.length === 0) return
    const entry = queue.shift()
    const token = ++playToken
    const handle = playEntry(entry)
    current = { entry, cancel: handle.cancel, token }
    emit()
    handle.done.then(() => {
      // cancelCurrent() は current を同期的に null にするので、
      // 打ち切られた再生の done はここで弾かれる
      if (!current || current.token !== token) return
      current = null
      emit()
      startNext()
    })
  }

  function cancelCurrent() {
    if (!current) return
    const c = current
    current = null
    c.cancel()
  }

  // ── 単発 ────────────────────────────────────────────────
  function announce(id) {
    const entry = getAnnouncement(id)
    if (!entry) {
      console.warn('[voice] 未知のアナウンス id:', id)
      return false
    }
    if (!layers[entry.layer]) {
      drop(id, 'レイヤ無効')
      return false
    }
    // 再発話タイマーとキュー滞留が重なったときの二重登録を防ぐ
    if (queue.some((e) => e.id === id)) {
      drop(id, 'キューに滞留中')
      return false
    }

    const rank = LAYER_RANK[entry.layer]
    if (current && rank > LAYER_RANK[current.entry.layer]) {
      cancelCurrent()
      if (DISCARD_PENDING_DEMO_ON_SAFETY) {
        for (let i = queue.length - 1; i >= 0; i--) {
          if (LAYER_RANK[queue[i].layer] < rank) queue.splice(i, 1)
        }
      }
    }

    queue.push(entry)
    emit()
    startNext()
    return true
  }

  // ── 継続状態 ────────────────────────────────────────────
  function startRepeatTimer(entry) {
    if (!entry.repeatSec) return null
    return setInterval(() => {
      const state = conditions.get(entry.id)
      if (!state) return
      state.count += 1
      announce(entry.id)
      if (entry.repeatMax && state.count >= entry.repeatMax) {
        clearInterval(state.timerId)
        state.timerId = null
      }
    }, entry.repeatSec * 1000)
  }

  function clearTimerFor(id) {
    const state = conditions.get(id)
    if (state?.timerId) {
      clearInterval(state.timerId)
      state.timerId = null
    }
  }

  function setCondition(id, active) {
    const entry = getAnnouncement(id)
    if (!entry) {
      console.warn('[voice] 未知のアナウンス id:', id)
      return
    }

    if (active) {
      if (conditions.has(id)) return          // 冪等: 既に立っていれば何もしない
      conditions.set(id, { timerId: null, count: 0 })
      announce(id)
      const timerId = startRepeatTimer(entry)
      conditions.get(id).timerId = timerId
      emit()
    } else {
      if (!conditions.has(id)) return
      clearTimerFor(id)
      conditions.delete(id)                   // count もここで消える = 上限リセット
      emit()
    }
  }

  // ── レイヤの有効・無効 ──────────────────────────────────
  function setLayerEnabled(layer, enabled) {
    if (layers[layer] === enabled) return
    layers[layer] = enabled

    if (!enabled) {
      // 再生停止・キュー除去・タイマー解除。ただし条件フラグ自体は残す
      // (条件が続いていることと、それを今聞きたいかは別の話)
      if (current?.entry.layer === layer) cancelCurrent()
      for (let i = queue.length - 1; i >= 0; i--) {
        if (queue[i].layer === layer) queue.splice(i, 1)
      }
      for (const id of conditions.keys()) {
        if (getAnnouncement(id)?.layer === layer) clearTimerFor(id)
      }
      emit()
      startNext()
      return
    }

    // 有効化: まだ立っている条件を鳴らし直す。
    // これがないと、LiDAR フォルトが継続している最中に安全通知を ON にしても
    // 次のエッジまで無音のままになり、機能の意味がなくなる。
    // 明示操作による再開なので再発話カウントもリセットする。
    for (const [id, state] of conditions) {
      const entry = getAnnouncement(id)
      if (!entry || entry.layer !== layer) continue
      state.count = 0
      announce(id)
      state.timerId = startRepeatTimer(entry)
    }
    emit()
  }

  function stopAll({ clearConditions = false } = {}) {
    cancelCurrent()
    queue.length = 0
    if (clearConditions) {
      for (const id of [...conditions.keys()]) clearTimerFor(id)
      conditions.clear()
    }
    emit()
  }

  return {
    announce,
    setCondition,
    setLayerEnabled,
    stopAll,
    isConditionActive: (id) => conditions.has(id),
    subscribe(fn) {
      subscribers.add(fn)
      return () => subscribers.delete(fn)
    },
    getSnapshot: () => snapshot,
  }
}

// モジュールスコープのシングルトン。
// main.jsx が React.StrictMode を使っているため、useRef で生成してアンマウントで
// 破棄する作りだと開発時の二重マウントでエンジンが壊れる。
export const voiceQueue = createVoiceQueue()

// HMR 対策。これがないと、編集のたびに前世代モジュールの再発話 setInterval が
// 孤児化して鳴り続ける。
import.meta.hot?.dispose(() => voiceQueue.stopAll({ clearConditions: true }))
