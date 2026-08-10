// ============================================================
// useVoiceTriggers.js — ロボット状態 → アナウンス ID の対応付け
//
// useRosbridge には音声の存在を知らせない。あちらはドメインイベントを流すだけで、
// 「どの状態でどれを鳴らすか」の知識はすべてこのファイルに閉じる。
//
// Tier 2 時点の自動化範囲:
//   安全通知 13件すべて
//   デモ実況 39件中 27件
// 残る 12件 (N1 N2 N6 N7 N8 N16 N17 N18 N20 N37 N39 N41) は、観測手段がないか
// 他 ID と同じエッジで発火して区別できないため手動発火のみ。理由は
// announcements.js の note に個別に書いてある。
//
// N4 は距離を実測値で差し替える (数値分割合成。VISION.md §7.5)。他の数値系
// (N7/N15/N16/N19/N20/N36/N37) は同じ仕組みに乗せられるが、値の取得元が
// 未接続のため今は文案の数値のまま
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

    // 人位置捕捉系 (N4・N23〜N27) は追従中 (FOLLOWING / FOLLOWING_MAPLESS) のみ。
    // /person/status と捜索段階はモードに関わらず配信され続けるため、ゲートが
    // ないと IDLE・MANUAL 中の検出でも鳴ってしまう
    const isFollowingMode = mode === MODE.FOLLOWING || mode === MODE.FOLLOWING_MAPLESS

    // 試験員の初回捕捉。再捕捉は N27 だが、捜索中かどうかを区別できないため Tier 2
    if (isFollowingMode && !prev.isTracked && isTracked && !firstCaptureDoneRef.current) {
      firstCaptureDoneRef.current = true
      // 距離を実測値で差し替える (数値分割合成。VISION.md §7.5)。
      // クリップ (0〜20 の整数 + meter) の生成範囲に合わせて 0〜20m にクランプする。
      // isTracked は同じレンダーの personStatus から導出しているため、ここで
      // 参照する personStatus はその isTracked と矛盾しない値になっている
      // (personStatus を effect の依存配列に入れているのもこのため)。
      // それでも欠けていれば overrides 無しで呼び、静的な N4.mp3 にフォールバックする
      const distM = personStatus
        ? Math.hypot(personStatus.position.x, personStatus.position.y) : null
      const distClip = distM === null ? null : String(Math.min(20, Math.max(0, Math.round(distM))))
      announce('N4', distClip ? { clips: ['hokaku', distClip, 'meter'] } : null)
    }

    if (isFollowingMode && prev.lostReason !== lostReason) {
      if (lostReason === 'DETECTION_LOST')  announce('N23')
      if (lostReason === 'TARGET_SWITCHED') announce('N24')
    }

    if (prev.mode !== mode) {
      // モード遷移系は遷移元を問わず発火させる (VISION.md §7.7・2026-08-05)。
      // MANUAL → FOLLOWING_MAPLESS のように IDLE を経由しない遷移もあるため
      if (mode === MODE.FOLLOWING_MAPLESS) announce('N5')
      if (mode === MODE.FOLLOWING)         announce('N14')
      if (mode === MODE.MANUAL)            announce('N40')
      if (mode === MODE.FACING)            announce('N45')
      if (prev.mode === MODE.MOVING_TO_PANEL && mode === MODE.AT_PANEL) announce('N38')

      // 待機へ戻ったこと自体の通知。起動直後 (D3) と E-Stop 解除 (D2) は
      // 専用の文言があるためここでは除く
      if (mode === MODE.IDLE && prev.mode !== MODE.ESTOP) announce('N43')
    }

    // ── 捜索段階 (予測 → 捜索旋回 → 再捕捉) ────────────────
    if (isFollowingMode && prev.searchPhase !== searchPhase) {
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
    // planner が切り替わった直後は前回値と比較しても意味がないので見送る。
    // 追従開始そのもの (N5 / N14) はモード遷移側で拾っているので、ここでは
    // 追従中に起きる状態変化だけを扱う
    if (prev.followPlanner === followPlanner && followPlanner !== null) {
      if (prev.followState !== followState) {
        if (followPlanner === 'mapless') {
          // 接近停止からの復帰。接近時だけ state が STOPPED になる
          if (followState === 'TRACKING' && prev.followState === 'STOPPED' &&
              prev.followReason === 'person_close') {
            announce('N10')
          }
        } else if (followPlanner === 'map') {
          if (followState === 'PREPARE' && prev.followState === 'TRACKING') announce('N15')
          if (followState === 'EVADING' && prev.followState === 'PREPARE')  announce('N19')
          // 退避帯から追従へ復帰 = 距離を確保できた
          if (followState === 'TRACKING' &&
              (prev.followState === 'PREPARE' || prev.followState === 'EVADING')) {
            announce('N21')
          }
        }
      }
      // reason の遷移で見るもの (mapless)。
      // 障害物は state を STOPPED にせず TRACKING のまま reason だけ変える
      // (距離ベースの状態機械と進路チェックが別系統のため。実機で確認済み)。
      // よって障害物の検知・解消は state ではなく reason で判定する必要がある。
      if (prev.followReason !== followReason && followPlanner === 'mapless') {
        if (followReason === 'person_close') announce('N9')
        if (followReason === 'tracking' && prev.followReason === 'obstacle_ahead') {
          announce('N12')
        }
      }
    }
  }, [connected, mode, faultActive, estopAny, isTracked, lostReason, personStatus,
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

    if (kind === 'select_target' && ok) announce('N44')
  }, [lastAction, announce])
}
