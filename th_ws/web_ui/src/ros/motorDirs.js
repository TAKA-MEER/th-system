// ros/motorDirs.js — 始業点検 MOTOR の 4 方向の純ロジック（brief-a13・SG-A13）。
//
// runner は方向別の進捗を CheckStatus.detail の先頭 `done=FORWARD,BACK ...` に載せ、
// 確定 NG の理由を `LEFT:sign_mismatch_L` の形（方向プレフィクス）で載せる
// （msg を変えないため）。ここの 2 関数はその読み取り側。React 非依存なので
// node --test で直接縛る（test/unit/motor-dirs.test.js）。
export const MOTOR_DIRS = ['FORWARD', 'BACK', 'LEFT', 'RIGHT']

// `done=...`（CSV。空もありうる）から済み方向だけを返す。知らない語は捨てる。
export function parseMotorDone(detail) {
  const m = /^done=([^ ]*)/.exec(detail ?? '')
  if (!m) return []
  return m[1].split(',').filter((d) => MOTOR_DIRS.includes(d))
}

// `DIR:reason` を { dir, reason } に割る。前方一致では割らない（`FORWARDX:` は対象外）。
// プレフィクスが無ければ { dir: null, reason: 原文 }。
export function stripMotorDir(detail) {
  const m = /^(FORWARD|BACK|LEFT|RIGHT):(.*)$/.exec(detail ?? '')
  if (!m) return { dir: null, reason: detail ?? '' }
  return { dir: m[1], reason: m[2] }
}
