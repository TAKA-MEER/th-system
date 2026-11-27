// shell/autoBrakeLock.js — 自動ブレーキのロックの判断（Spec-webui §3.5・D73）。
//
// 「入」（ロック中）の鍵ボタン → 1回目で黄の「もう一度押すと外す」（約3秒で戻る）→
// 2回目でロックが外れて「切│入」の切り替えが出る → 「切」を押して OFF（3回で切れる）。
// OFF の間は「入」をすぐ押せる。ロックがかかり直すのは「入」に戻したとき・状態が変わったとき。
// この 2 回押しが ON→OFF の確認（W-4）を兼ねる。純関数。表示専用（許可ではない）。

export const LOCKED = 'locked'
export const ARMED = 'armed'
export const UNLOCKED = 'unlocked'

export const LOCK_ARM_MS = 3000

// 鍵ボタンを押したときの次状態。{ lock, request } を返す。
// 鍵ボタンの2回目まででは要求を送らない（ロックだけが外れる）。
export function nextBrakePress(lock, brake) {
  if (brake === true) {
    if (lock === ARMED) return { lock: UNLOCKED, request: null }
    if (lock === UNLOCKED) return { lock: UNLOCKED, request: null }
    return { lock: ARMED, request: null }
  }
  // OFF（または未受信）では鍵ボタンは出ない。ロックは外れたまま。
  return { lock: UNLOCKED, request: null }
}

// ロックが外れた「切│入」の切り替えで、側を押したときの次状態。
// 'off'（切）→ OFF 送信。'on'（入）→ ON 送信し、ロックがかかり直す。
export function nextBrakeSide(side, brake) {
  if (side === 'off') {
    return { lock: UNLOCKED, request: brake === true ? false : null }
  }
  if (side === 'on') {
    return { lock: LOCKED, request: brake === false ? true : null }
  }
  return { lock: UNLOCKED, request: null }
}

// ロックがかかり直すか（「入」に戻した・状態が変わった）。
export function relocksOn(brake, changed) {
  return brake === true || changed === true
}
