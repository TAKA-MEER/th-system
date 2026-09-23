// parts/TrackerControl.jsx — 人検出の開始/停止（brief-tracker-default-off §3.4）。
// S-20 / S-21 の対象選択タブ「中心」に出す共通 UI。実体は
// /system/set_flag tracker_enabled（ros/useSetFlag.js）で、状態の正本は
// th_state（server 側）の tracker_enabled。
//
// - OFF: 中心に「人検出を開始」ボタン + 「人検出が止まっています」
// - ON かつ候補なし: 「起動中…」+「人検出を停止」
// - ON かつ候補あり: 「人検出 動作中」+「人検出を停止」
// - 停止を押せない状態（SUMMON 全状態・PREP/REGISTER）ではボタンを無効化し、
//   理由（TRACKER_STOP_DENIED）を出す。押せない状態なのにボタンが在ることを
//   「理由バッジで先に分かる」形に保つ（ボタンを隠すだけにしない）。
// 日本語は i18n/screens.js の TRACKER_* 定数に置く（parts/ は c4）。
import {
  TRACKER_BOOTING, TRACKER_OFF, TRACKER_ON, TRACKER_START, TRACKER_STOP, TRACKER_STOP_DENIED,
} from '../i18n/screens.js'

export default function TrackerControl({
  enabled = false,
  hasCandidate = false,
  canStop = true,
  stopDeniedReason = null,
  onStart,
  onStop,
  disabled = false,
  testId = 'tracker',
}) {
  if (!enabled) {
    return (
      <div className="card tracker-control" data-testid={`${testId}-tracker-control`}>
        <div className="note" data-testid={`${testId}-tracker-off`}>{TRACKER_OFF}</div>
        <button
          type="button"
          className="btn primary wide mt"
          data-testid={`${testId}-tracker-start`}
          disabled={disabled}
          onClick={onStart}
        >
          <span>{TRACKER_START}</span>
        </button>
      </div>
    )
  }
  return (
    <div className="card tracker-control" data-testid={`${testId}-tracker-control`}>
      <div className="row">
        <span className="grow" data-testid={`${testId}-tracker-status`}>
          {hasCandidate ? TRACKER_ON : TRACKER_BOOTING}
        </span>
        <button
          type="button"
          className="btn sm"
          data-testid={`${testId}-tracker-stop`}
          disabled={disabled || !canStop}
          onClick={onStop}
        >
          <span>{TRACKER_STOP}</span>
        </button>
      </div>
      {!canStop && (
        <div className="note mt" data-testid={`${testId}-tracker-stop-denied`}>
          {stopDeniedReason ?? TRACKER_STOP_DENIED}
        </div>
      )}
    </div>
  )
}