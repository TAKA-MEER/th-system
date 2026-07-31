// ============================================================
// useVoiceTriggers.js — ロボット状態 → アナウンス ID の対応付け
//
// useRosbridge には音声の存在を知らせない。あちらはドメインイベントを流すだけで、
// 「どの状態でどれを鳴らすか」の知識はすべてこのファイルに閉じる。
//
// Tier 2 時点の自動化範囲:
//   安全通知 13件すべて
//   デモ実況 36件中 25件
// 残る 11件 (N1 N2 N7 N8 N16 N17 N18 N20 N37 N39 N41) は、観測手段がないか
// 他 ID と同じエッジで発火して区別できないため手動発火のみ。理由は
// announcements.js の note に個別に書いてある。
// ============================================================

import { useEffect, useRef } from 'react'

import { MODE } from '../robotMode.js'

export function useVoiceTriggers(ros, voice) {
  const {
    connected, mode, fault, estop, estopActive, personStatus, lastAction,
    followStatus, searchStatus, summonStatus,
  } = ros

  const faultActive = !!fault?.active
  const faultType   = fault?.fault_type ?? 'NONE'
  // タブレット側の押下 (estopActive) でも即座に鳴らす。OR にしてあるので
  // ロボットからエコーが返ってきたときに二度鳴らない
  const estopAny    = !!estop || !!estopActive
  const isTracked   = !!personStatus && !personStatus.is_lost
  const lostReason  = personStatus?.is_lost ? (personStatus.lost_reason ?? '') : ''

  // 追従ロジックの内部状態 (/follow/status)
  const followPlanner = followStatus?.planner ?? null
  const followState   = followStatus?.state ?? null
  const followReason  = followStatus?.reason ?? null
  const searchPhase   = searchStatus?.phase ?? null

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

  // 追従ロジック由来の持続的な待機状態。
  // C4 は reason が 'retreat_blocked' のときだけで、'no_pose' では鳴らさない。
  // 両者は core では同じ retreat_blocked フラグだったが、costmap 未受信のたびに
  // 「逃げ場なし。離れて」が誤発火するため理由を分けてある。
  useEffect(() => {
    setCondition('C3', searchPhase === 'GAVE_UP')
    setCondition('C4', followReason === 'retreat_blocked')
    setCondition('C7', followReason === 'obstacle_ahead')
    setCondition('C8', followReason === 'no_pose' || followReason === 'no_scan')
  }, [searchPhase, followReason, setCondition])

  // ── エッジ ──────────────────────────────────────────────
  const prevRef = useRef(null)
  const firstCaptureDoneRef = useRef(false)

  useEffect(() => {
    const now = {
      connected, mode, faultActive, estopAny, isTracked, lostReason,
      followPlanner, followState, followReason, searchPhase,
    }
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

    // ── 捜索段階 (予測 → 捜索旋回 → 再捕捉) ────────────────
    if (prev.searchPhase !== searchPhase) {
      if (searchPhase === 'PREDICTING') announce('N25')
      if (searchPhase === 'SEARCHING')  announce('N26')
      // 捜索・予測から NONE に戻る = 再発見。デモの山場。
      // 初回捕捉 (N4) とは区別する必要があるので、直前が捜索系だったことを見る
      if (searchPhase === 'NONE' &&
          (prev.searchPhase === 'SEARCHING' || prev.searchPhase === 'GAVE_UP')) {
        announce('N27')
      }
    }

    // ── 追従ロジックの状態遷移 ──────────────────────────────
    // planner が切り替わった直後は前回値と比較しても意味がないので見送る
    if (prev.followPlanner === followPlanner && followPlanner !== null) {
      if (prev.followState !== followState) {
        if (followPlanner === 'mapless') {
          if (followState === 'TRACKING' && prev.followState === 'STOPPED') {
            // 停止からの復帰は「なぜ止まっていたか」で文言が変わる。
            // 接近停止からの再開 (N10) と障害物解消 (N12) は同じ遷移なので
            // 直前の reason で振り分ける
            if (prev.followReason === 'obstacle_ahead')   announce('N12')
            else if (prev.followReason === 'person_close') announce('N10')
          }
          if (followState === 'TRACKING' && prev.followState === 'INACTIVE') announce('N6')
        } else if (followPlanner === 'map') {
          if (followState === 'PREPARE' && prev.followState === 'TRACKING') announce('N15')
          if (followState === 'EVADING' && prev.followState === 'PREPARE')  announce('N19')
          // 退避帯から追従へ復帰 = 距離を確保できた
          if (followState === 'TRACKING' &&
              (prev.followState === 'PREPARE' || prev.followState === 'EVADING')) {
            announce('N21')
          }
          if (followState === 'TRACKING' && prev.followState === 'INACTIVE') announce('N14')
        }
      }
      // 接近停止に入った瞬間 (mapless)。state は STOPPED のまま reason だけ
      // 変わる経路 (障害物 → 接近) もあるので reason の遷移で見る
      if (prev.followReason !== followReason &&
          followPlanner === 'mapless' && followReason === 'person_close') {
        announce('N9')
      }
    }
  }, [connected, mode, faultActive, estopAny, isTracked, lostReason,
      followPlanner, followState, followReason, searchPhase, announce])

  // ── 呼び寄せイベント ────────────────────────────────────
  // 到着 (N33) と中止 (N35) はどちらも SUMMONING→IDLE 遷移なので、
  // モードだけを見ていると区別できない。専用トピックの event で振り分ける
  const prevSummonSeqRef = useRef(0)

  useEffect(() => {
    if (!summonStatus || summonStatus.seq === prevSummonSeqRef.current) return
    prevSummonSeqRef.current = summonStatus.seq

    switch (summonStatus.event) {
      case 'ACCEPTED':  announce('N32'); break
      case 'ARRIVED':   announce('N33'); break
      case 'CANCELLED': announce('N35'); break
      // REJECTED / FAILED はどちらも「経路なし。中止」で足りる
      case 'REJECTED':
      case 'FAILED':    announce('N34'); break
      default: break
    }
  }, [summonStatus, announce])

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
