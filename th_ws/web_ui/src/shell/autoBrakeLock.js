// shell/autoBrakeLock.js — 自動ブレーキのロックの判断（Spec-webui §3.5）。
// 鍵の印の「入」→ 1回目で黄の「もう一度押すと外す」（約3秒で戻る）→
// 2回目で外れて「切│入」→「切」で OFF。ロックの2回押しが ON→OFF の確認（W-4)を兼ねる。
// 「入」に戻す・状態が変わるとロックがかかり直す。純関数。表示専用（許可ではない）。

export const LOCKED = 'locked'
export const ARMED = 'armed'
export const UNLOCKED = 'unlocked'

export const LOCK_ARM_MS = 3000

// 押し上がったときの次状態。{ lock, request } を返す。
// brake: 現在の自動ブレーキ（true/false/null）。
export function nextBrakePress(lock, brake) {
  if (brake === true) {
    if (lock === ARMED) return { lock: UNLOCKED, request: false }
    if (lock === UNLOCKED) return { lock: LOCKED, request: false }
    return { lock: ARMED, request: null }
  }
  if (brake === false) {
    return { lock: LOCKED, request: true }
  }
  // 未受信（null）は押しても何もしない。
  return { lock: LOCKED, request: null }
}

// ロックがかかり直すか（「入」に戻した・状態が変わった）。
export function relocksOn(brake, changed) {
  return brake === true || changed === true
}
