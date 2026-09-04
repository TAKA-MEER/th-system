// screens/screenRouting.js — pure screen-routing logic, split out of
// main.jsx (WS-9X) so it can be unit-tested without pulling in React / the
// shell / theme.css. main.jsx's Screens() is the only caller.

// names.json の screens に載っている ID (N-15: /ui/active_screen の
// screen_id はこの値をそのまま使う -- names.json に無い値を送ってはいけない)。
export const SCREEN_IDS = {
  S00: 'S-00', S01: 'S-01', S11: 'S-11', S13: 'S-13', S14: 'S-14', S50: 'S-50',
}

// mode -> screen-key map. S-01 sends ui.enter_mode; when th_state accepts
// it, /system/state's new mode lands here and Screens() switches. A mode
// with no mapped screen yet (e.g. FOLLOW -> S-10) is simply absent: S-01
// stays put and the header's mode pill reflects the change.
// P5 / demo-teach-replay: TEACH_MANUAL -> S-13、REPLAY -> S-14。
export const MODE_TO_SCREEN = { MANUAL: 'S11', TEACH_MANUAL: 'S13', REPLAY: 'S14' }

// 表示中の画面は SystemState.mode から導出する（純関数）。
//
// 2026-09-02: 以前は画面がローカル state の一方通行で、ロボット側 FSM が
// 独自にモードを変えたとき（重大フォルトで IDLE へ強制遷移、ESTOP 復帰、
// 他端末の操作、再生終了）画面が取り残され「どこにも行けない」不具合になった。
// 対策: FSM を唯一の真実とし、画面はそこから導出する。ローカルに残す状態は
// 「S-00（接続確認）を通過したか」の 1 つだけ。
// - mode に対応する画面が無ければ S-01。
// - リンクが stale でも S-00 へは戻さない（一瞬の途切れで画面が飛ぶのを防ぐ）。
//
// settingsOpen (WS-9X): S-50 設定は FSM のモードではなく S-01 のサブ画面。
// 「本来なら S-01 を出す」ときだけ S-50 に差し替える。動作系モードに入って
// いれば（MODE_TO_SCREEN にヒット）設定は絶対に出ない = 走行中に設定画面が
// かぶることは構造上あり得ない。
export function resolveScreen({ testScreen, passedConnect, mode, settingsOpen }) {
  // DRIVE_S11 は本番に存在しない e2e 専用の合成画面。モード導出を迂回する。
  if (testScreen === 'DRIVE_S11') return 'DRIVE_S11'
  if (!passedConnect) return 'S00'
  const base = MODE_TO_SCREEN[mode] ?? 'S01'
  return settingsOpen && base === 'S01' ? 'S50' : base
}

// __thTestScreen は「どこから始めるか」の初期値。'S00' 指定なら接続確認から、
// それ以外なら通過済みとして始める。
export function initialPassedConnect(testScreen) {
  if (!testScreen) return false
  return testScreen !== 'S00'
}
