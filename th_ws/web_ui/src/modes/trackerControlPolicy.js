// modes/trackerControlPolicy.js — WebUI 側の「人検出を停止できるか」判定。
//
// th_state（src/th_state/scripts/state_manager.py + src/th_state/th_state/
// tracker_policy.py、_TRACKER_OFF_DENIED）と同じ拒否ポリシーの UI 側コピー。
// ボタンの事前無効化と理由バッジに使う。正本はサーバ側（tracker_off_denied）
// で、ここが緩くても ROS 側が拒否する（安全は server が握る）。
export const trackerStopAllowed = (mode, state) => {
  if (mode === 'SUMMON') return false
  if (mode === 'PREP' && state === 'REGISTER') return false
  return true
}