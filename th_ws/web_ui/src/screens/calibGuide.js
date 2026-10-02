// screens/calibGuide.js — S-30 → S-40 の誘導元の受け渡し（計画書 §6 #11 の残り）。
//
// FSM (T-OPC-07) は OPCHECK/LIST --ui.enter_mode--> CALIB/$initial で、誘導元の
// 項目を運ぶ仕組みを持たない（effects: []）。ROS・FSM を変えず画面側だけで
// 完結させるため、S-30 が「校正へ」を押したときに誘導元をここへ置き、S-40 が
// CALIB/LIST で 1 回だけ読んで消す。対応は Spec-checks.md §4:
// 点検 LIDAR → 校正 BLIND、点検 IMU → 校正 IMU（ESTOP・MOTOR は校正で直らない
// ので故障診断へ行き、ここには置かない）。
//
// モジュール内スロットなのは、S-30 → S-40 が同一ページ内の画面遷移だから
// （リロードを跨がない。sessionStorage は要らない）。
// useOpcheckStatus.js の latched と同じ流儀（マウントを跨いで残る意図的な保持）。
// React 非依存の純ロジックなので node --test で直接縛る（test/unit/calib-guide.test.js）。
//
// 形: { from: 'LIDAR'|'IMU', result: 'NG'|'WARN', target: 'BLIND'|'IMU' }

// 点検の項目 → 校正の項目（Spec-checks.md §4）。
export const OPCHECK_TO_CALIB = { LIDAR: 'BLIND', IMU: 'IMU' }

// 誘導元として受け付ける点検結果（「校正へ」ボタンが出るのは NG・WARN のとき。
// opcheck_runner.py の _NEXT_SCREEN と _publish_fanout 参照。IMU の NG は
// next_screen=repair になるのでボタン自体が出ない）。
const GUIDEABLE_RESULTS = ['NG', 'WARN']

let slot = null

// 置く。対応表に無い項目・対象外の結果は置かず null を返す（ESTOP・MOTOR 等）。
export function setCalibGuide({ from, result }) {
  const target = OPCHECK_TO_CALIB[from]
  if (!target || !GUIDEABLE_RESULTS.includes(result)) return null
  slot = { from, result, target }
  return slot
}

// 見るだけ（消さない）。
export function peekCalibGuide() {
  return slot
}

// 1 回だけ読んで消す。S-40 が CALIB/LIST で使う。
export function takeCalibGuide() {
  const g = slot
  slot = null
  return g
}

export function clearCalibGuide() {
  slot = null
}
