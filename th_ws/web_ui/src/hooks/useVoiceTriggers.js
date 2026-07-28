// ============================================================
// useVoiceTriggers.js — ロボット状態 → アナウンス ID の対応付け
//
// useRosbridge には音声の存在を知らせない。あちらはドメインイベントを流すだけで、
// 「どの状態でどれを鳴らすか」の知識はすべてこのファイルに閉じる。
//
// Tier 1 で自動化できるのは、既存の購読とサービス応答から確実に判定できるものだけ:
//   安全通知 13件中 9件 (A1 A2 A3 B1 B2 B3 D1 D2 D3)
//   デモ実況 36件中 11件 (N4 N5 N23 N24 N29 N30 N31 N34 N36 N38 N40)
// 残りは追従系ノードの状態 publish (Tier 2) 待ち。
// ============================================================

import { useEffect, useRef } from 'react'

import { MODE } from '../robotMode.js'

export function useVoiceTriggers(ros, voice) {
  const {
    connected, mode, fault, estop, estopActive, personStatus, lastAction,
  } = ros

  const faultActive = !!fault?.active
  const faultType   = fault?.fault_type ?? 'NONE'
  // タブレット側の押下 (estopActive) でも即座に鳴らす。OR にしてあるので
  // ロボットからエコーが返ってきたときに二度鳴らない
  const estopAny    = !!estop || !!estopActive
  const isTracked   = !!personStatus && !personStatus.is_lost
  const lostReason  = personStatus?.is_lost ? (personStatus.lost_reason ?? '') : ''

  const { setCondition, announce } = voice

  // ── 継続条件 ────────────────────────────────────────────
  // setCondition は冪等なので、毎レンダー同じ値を渡してよい。
  // エッジ検出が voiceQueue 側にあるおかげで ref が要らず、
  // StrictMode の二重実行も無害になる。
  useEffect(() => {
    setCondition('A1', faultActive && faultType === 'LIDAR_LOST')
    setCondition('A2', faultActive && faultType === 'ESP32_DISCONNECTED')
    setCondition('B1', faultActive && faultType === 'PERSON_TRACKER_LOST' && mode === MODE.IDLE)
  }, [faultActive, faultType, mode, setCondition])

  // ── エッジ ──────────────────────────────────────────────
  const prevRef = useRef(null)
  const firstCaptureDoneRef = useRef(false)

  useEffect(() => {
    const now = { connected, mode, faultActive, estopAny, isTracked, lostReason }
    const prev = prevRef.current
    prevRef.current = now

    // 初回は比較せず保存だけして抜ける。これがないとページ表示直後に
    // connected:false が B2 を、mode:null→IDLE が D3 を誤発火させる
    if (prev === null) return

    // 通信断 (全モード)。VISION.md §7.7 B2
    if (prev.connected && !connected) announce('B2')

    // 接続後に初めて mode を受信したとき。mode_manager は起動 500ms で IDLE に
    // 遷移し transient_local で配信するため、後から接続した WebUI は INIT を
    // 観測できない。よって「INIT→IDLE」ではなく初回受信をもって起動完了とみなす
    if (prev.mode === null && mode !== null && mode !== undefined) announce('D3')

    if (!prev.estopAny && estopAny) announce('A3')
    if (prev.estopAny && !estopAny) announce('D2')
    if (prev.faultActive && !faultActive) announce('D1')

    // 試験員の初回捕捉。再捕捉は N27 だが、捜索中かどうかを区別できないため Tier 2
    if (!prev.isTracked && isTracked && !firstCaptureDoneRef.current) {
      firstCaptureDoneRef.current = true
      announce('N4')
    }

    if (prev.lostReason !== lostReason) {
      if (lostReason === 'DETECTION_LOST')  announce('N23')
      if (lostReason === 'TARGET_SWITCHED') announce('N24')
    }

    if (prev.mode !== mode) {
      if (prev.mode === MODE.IDLE && mode === MODE.FOLLOWING_MAPLESS) announce('N5')
      if (prev.mode === MODE.IDLE && mode === MODE.MANUAL)            announce('N40')
      if (prev.mode === MODE.MOVING_TO_PANEL && mode === MODE.AT_PANEL) announce('N38')
    }
  }, [connected, mode, faultActive, estopAny, isTracked, lostReason, announce])

  // ── サービス応答 ────────────────────────────────────────
  // seq が単調増加するので、同じ結果が連続しても取りこぼさない
  const prevSeqRef = useRef(0)

  useEffect(() => {
    if (!lastAction || lastAction.seq === prevSeqRef.current) return
    prevSeqRef.current = lastAction.seq

    const { kind, ok, message } = lastAction

    if (kind === 'set_mode' && !ok) announce('B3')

    if (kind === 'summon') {
      if (ok) announce('N29')
      // summon_navigator_core.py の日本語メッセージとの部分一致。
      // Python 側の文言を変えると黙って鳴らなくなるため、マニフェスト上でも
      // auto: 'heuristic' として区別してある
      else if (message.includes('見失'))   announce('N30')
      else if (message.includes('信頼度')) announce('N31')
    }

    if (kind === 'go_to_panel') announce(ok ? 'N36' : 'N34')
  }, [lastAction, announce])
}
