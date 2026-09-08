// screens/onsiteSteps.js — S-20 / S-21 の手順バー判定の純関数（brief-onsite-ux UX-1）。
//
// UI が「いま何すべきか」を読み取れるように、各画面の作業手順を段に分け、
// 完了条件を既存の購読値から導く。ROS も React も使わないので
// test/unit/onsite-steps.test.js からそのまま検証できる。
//
// 戻り値 { steps: [{ id, labelKey, done }], currentIndex }。
// currentIndex は「未完了の最初の段」（0 始まり）。全部完了なら最後の段を現在扱い。
// labelKey は i18n/screens.js の S20_STEP_* / S21_STEP_* 定数名。id は S20/S21 の
// ラベル対応表（S20_STEP_LABELS / S21_STEP_LABELS）のキーに使う。

// S-21 の「行き先を選ぶ」段の完了判定に使うモード群（S21Test.jsx からも参照する）。
export const NAV_MODES = ['SUMMON', 'PANEL_NAV', 'AT_PANEL', 'HOME_NAV']

function currentIndexOf(done) {
  const i = done.findIndex((d) => !d)
  return i === -1 ? done.length - 1 : i
}

/**
 * S-20 試験準備の 5 段。
 * 段 1「地図を作る」は `mode === 'PREP'`（この画面が出ている時点で満たす）なので
 * 引数に mode を取らず常に完了扱いにする（brief-onsite-ux UX-1-a の表）。
 * @param {{pins?: Array, personTargets?: Object, unsaved?: Array}} _
 *   pins: /onsite/pins（PinList）、personTargets: /person/targets、
 *   unsaved: /system/state の unsaved（配列の生値。未変更なら undefined/falsy）。
 */
export function prepSteps({ pins = [], personTargets = {}, unsaved } = {}) {
  const homeDone = pins.some((p) => p?.kind === 'HOME')
  const panelDone = pins.some((p) => p?.kind === 'PANEL')
  const targetDone = (personTargets.selected_index ?? -1) >= 0
  // 段 5「保存」は「未保存が空」かつ待機場所・配電盤の登録済み。
  const unsavedEmpty = !(Array.isArray(unsaved) && unsaved.length > 0)
  const steps = [
    { id: 'map', labelKey: 'S20_STEP_MAP', done: true },
    { id: 'target', labelKey: 'S20_STEP_TARGET', done: targetDone },
    { id: 'home', labelKey: 'S20_STEP_HOME', done: homeDone },
    { id: 'panel', labelKey: 'S20_STEP_PANEL', done: panelDone },
    { id: 'save', labelKey: 'S20_STEP_SAVE', done: unsavedEmpty && homeDone && panelDone },
  ]
  return { steps, currentIndex: currentIndexOf(steps.map((s) => s.done)) }
}

/**
 * S-21 試験（当日）の 5 段。
 * @param {{homeDeclared?: boolean, mapOpened?: boolean, selectedPinId?: string|null,
 *         mode?: string|null, stateName?: string|null}} _
 *   homeDeclared: /onsite/home_declared、mapOpened: /map_session/open の応答 success
 *   （画面ローカル state で保持。ROS 側に状態が無いため UX-3 の報告対象）、
 *   selectedPinId: ピン行の選択、mode / stateName: /system/state。
 */
export function testSteps({ homeDeclared, mapOpened, selectedPinId, mode, stateName } = {}) {
  const navMode = NAV_MODES.includes(mode)
  const steps = [
    { id: 'open', labelKey: 'S21_STEP_OPEN', done: mapOpened === true },
    { id: 'home', labelKey: 'S21_STEP_HOME', done: homeDeclared === true },
    { id: 'dest', labelKey: 'S21_STEP_DEST', done: selectedPinId != null || navMode },
    { id: 'move', labelKey: 'S21_STEP_MOVE', done: mode === 'AT_PANEL' },
    { id: 'work', labelKey: 'S21_STEP_WORK', done: stateName === 'WORKING' || mode === 'SUMMON' },
  ]
  return { steps, currentIndex: currentIndexOf(steps.map((s) => s.done)) }
}