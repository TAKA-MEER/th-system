// shell/autonomous.js — 自律走行中の枠を出すか（Spec-webui §3.3.5）。純関数。
//
// 機体が自分で走っている間だけ、ヘッダの下の画面の縁を緑の枠で囲み点滅する。
// 一時停止中・手動では出さない。どのモード×状態が当たるかはこの表が正。
// 表示専用（許可の判断ではない）。

// mode -> 自律で走っている状態の集合。
//   移動中 NAV/ALIGN（試験の移動・呼び寄せ含む）、自律走行 RUN、
//   教示（追従）の記録 REC（機体が自律で進む記録）、
//   試験準備の待機場所へ戻る途中 RETURN、
//   自律測定の旋回 ADJUST と走行 RUN/NAV。
export const AUTONOMOUS_TABLE = {
  FOLLOW: ['RUN'],
  TEACH_FOLLOW: ['REC'],
  REPLAY: ['RUN'],
  PANEL_NAV: ['NAV', 'ALIGN'],
  HOME_NAV: ['NAV', 'ALIGN'],
  SUMMON: ['NAV', 'ALIGN'],
  AT_HOME: ['NAV', 'ALIGN'],
  PREP: ['RETURN'],
  AUTO_MEASURE: ['RUN', 'NAV', 'ALIGN', 'ADJUST'],
}

export function isAutonomousRun(mode, stateName) {
  if (!mode || !stateName) return false
  const states = AUTONOMOUS_TABLE[mode]
  return !!states && states.includes(stateName)
}
