import React, { useState } from 'react'
import ReactDOM from 'react-dom/client'
import AppShell from './shell/AppShell.jsx'
import { SystemStateProvider, useSystemState } from './ros/useSystemState.js'
import AudienceView from './audience/AudienceView.jsx'
import OperationCardHarness from './TestHarness.jsx'
import S00Connect from './screens/S00Connect.jsx'
import S01Main from './screens/S01Main.jsx'
import S11Manual from './screens/S11Manual.jsx'
import S13TeachManual from './screens/S13TeachManual.jsx'
import S14Replay from './screens/S14Replay.jsx'
import DriveTab from './screens/driveTab.jsx'
import { useJogPanel } from './shell/jogPanel.js'
import { SCREEN_NAMES, DRIVE_MANUAL, DRIVE_LEAVE } from './i18n/screens.js'

// ?view=audience で観客向け表示 (VISION.md §6.3)。
// App 側で分岐せずマウントするツリーごと分けている。こうしておくと
// 操作 UI のジョグ用 setInterval・音声・heartbeat が観客画面では
// そもそも起動しない = 走行制御に触れないことが構造で保証される。
const AUDIENCE = new URLSearchParams(window.location.search).get('view') === 'audience'

// window.__thTestState は e2e (Playwright) だけが addInitScript で注入する
// (ros/useSystemState.js のテストフック)。通常の起動では絶対に付かない。
const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined

// window.__thTestScreen: e2e が S-00/S-01 を直接開くためのフック
// (e2e/helpers.js の gotoScreen())。既存 13 件のテスト (WP-UI-01) は
// これを付けずに __thTestState だけを注入するので、そちらは今まで通り
// OperationCardHarness を表示する -- one-primary-button.spec.js が探す
// [data-testid="opcard-harness"] を壊さないため。
const TEST_SCREEN = typeof window !== 'undefined' ? window.__thTestScreen : undefined

// names.json の screens に載っている ID (N-15: /ui/active_screen の
// screen_id はこの値をそのまま使う -- names.json に無い値を送ってはいけない)。
const SCREEN_IDS = { S00: 'S-00', S01: 'S-01', S11: 'S-11', S13: 'S-13', S14: 'S-14' }

// mode -> screen-key map, owned here so Screens() can switch. S-01 sends
// ui.enter_mode; when th_state accepts it, S01Main calls onEnter(mode) which
// reads this map to decide where to go. Later packets add one row per screen
// (DetailedDesign-wp3.md WP-TRANSIT-01 §11 c1: "遷移先を配列化して次の画面で
// 増やしやすくする"). A mode with no mapped screen yet (e.g. FOLLOW -> S-10,
// which is WP-UI-04) is simply absent: S-01 stays put and the header's mode
// pill reflects the change.
// P5 / demo-teach-replay: TEACH_MANUAL -> S-13（教示（手動））、REPLAY -> S-14（教示再生）。
const MODE_TO_SCREEN = { MANUAL: 'S11', TEACH_MANUAL: 'S13', REPLAY: 'S14' }

// e2e 用の DRIVE テスト画面: 常設走行タブ (driveTab) と、W-6 を開く「手動」
// ボタンだけを並べる。runTestDrive() は __thTestScreen === 'DRIVE_S11' を
// 注入してここへ到達する (e2e/helpers.js)。
// 「手動」「離れる」ボタンは AppShell の子として描画されるため、ここで初めて
// useJogPanel() (AppShell が提供する context) を呼べる。
function DriveTestBody({ departed, onDepart }) {
  const jogPanel = useJogPanel()
  return (
    <>
      {/* 「離れる」で DriveTab を外す。外れたことは e2e 側が
          `#body .stick svg` の消失で確かめるので、印は置かない。 */}
      {departed ? null : <DriveTab kind="manual" />}
      <button type="button" className="btn sm" onClick={() => jogPanel.open()} data-testid="open-jog">
        {DRIVE_MANUAL}
      </button>
      <button type="button" className="btn sm" onClick={onDepart} data-testid="leave-drive">
        {DRIVE_LEAVE}
      </button>
    </>
  )
}

function DriveTestScreen() {
  // U3-1 の e2e (stick-unmount-releases.spec.js) 用: 「離れる」で DriveTab を
  // マウントから外し、unmount 時にゼロが 1 回出て送出が止まることを確かめる。
  const [departed, setDeparted] = useState(false)
  return (
    <AppShell screenName={SCREEN_NAMES.S11} screenId={SCREEN_IDS.S11 ?? 'S-11'}>
      <DriveTestBody departed={departed} onDepart={() => setDeparted(true)} />
    </AppShell>
  )
}

// 表示中の画面は SystemState.mode から導出する（純関数。ユニットテスト対象）。
//
// 2026-09-02 修正の要点: 以前は画面が完全にローカル state の一方通行で、
// S-00 の「進む」と S-01 の enter_mode、各画面の「終了」でしか動かなかった。
// そのためロボット側 FSM が独自にモードを変えたとき——重大フォルトで
// 動作系モードから IDLE へ強制遷移した、ESTOP から復帰した、他の端末が
// 操作した、再生が終わった——画面はそのまま取り残された。
// 画面が S-13（教示）なのに FSM は IDLE、という状態では教示系の操作が
// 全部 FSM に拒否され、しかも「終了」も既に IDLE なので効かず、
// **どこにも行けなくなる**（実機で報告された「移動不能」）。
//
// 対策: ロボットの FSM を唯一の真実とし、画面はそこから導出する。
// ローカルに残す状態は「S-00（接続確認）を通過したか」の 1 つだけ。
//
// - mode に対応する画面が無い場合は S-01（メインメニュー）。
// - リンクが stale になっても S-00 へは戻さない。最後に見えていたモードの
//   画面に留まり、切断の告知は W-2（Windows.jsx）に任せる。ここで戻すと
//   一瞬の途切れで画面が飛ぶ（2026-09-01 に直したチラつきの再発）。
export function resolveScreen({ testScreen, passedConnect, mode }) {
  // DRIVE_S11 は本番に存在しない e2e 専用の合成画面なので、これだけは
  // モード導出を迂回する固定の上書きとして扱う。
  if (testScreen === 'DRIVE_S11') return 'DRIVE_S11'
  if (!passedConnect) return 'S00'
  return MODE_TO_SCREEN[mode] ?? 'S01'
}

// __thTestScreen は「どこから始めるか」の初期値であって、以後の遷移を
// 固定するものではない（固定すると S-00 の「進む」が効かなくなる）。
// 'S00' 指定なら接続確認から、それ以外なら通過済みとして始める。
function initialPassedConnect(testScreen) {
  if (!testScreen) return false
  return testScreen !== 'S00'
}

function Screens() {
  const { state } = useSystemState()
  const mode = state?.mode ?? null
  // S-00 は「疎通確認」という UI 上の前段で、FSM のモードではない。
  // 通過したかどうかだけがローカル状態。
  const [passedConnect, setPassedConnect] = useState(() => initialPassedConnect(TEST_SCREEN))

  const screen = resolveScreen({ testScreen: TEST_SCREEN, passedConnect, mode })

  if (screen === 'DRIVE_S11') {
    return <DriveTestScreen />
  }
  if (screen === 'S00') {
    return (
      <AppShell screenName={SCREEN_NAMES.S00} screenId={SCREEN_IDS.S00}>
        <S00Connect onAdvance={() => setPassedConnect(true)} />
      </AppShell>
    )
  }
  // 以下の画面の「終了」は ui.finish を送るだけでよい。受理されれば
  // FSM が IDLE になり、その /system/state を見てここが S-01 に戻す。
  // onFinish を渡して画面側から直接切り替えさせない（それをやると
  // 「画面は戻ったが FSM は戻っていない」という乖離を作り直すことになる）。
  if (screen === 'S11') {
    return (
      <AppShell screenName={SCREEN_NAMES.S11} screenId={SCREEN_IDS.S11}>
        <S11Manual />
      </AppShell>
    )
  }
  if (screen === 'S13') {
    return (
      <AppShell screenName={SCREEN_NAMES.S13} screenId={SCREEN_IDS.S13}>
        <S13TeachManual />
      </AppShell>
    )
  }
  if (screen === 'S14') {
    return (
      <AppShell screenName={SCREEN_NAMES.S14} screenId={SCREEN_IDS.S14}>
        <S14Replay />
      </AppShell>
    )
  }
  return (
    <AppShell screenName={SCREEN_NAMES.S01} screenId={SCREEN_IDS.S01}>
      {/* onEnter は渡さない。ui.enter_mode が受理されれば FSM が
          モードを変え、その /system/state を見てここが画面を切り替える。 */}
      <S01Main />
    </AppShell>
  )
}

// SystemStateProvider はルータより外側に 1 つだけ置く。こうすることで
// (1) Screens() が mode を読んで画面を導出でき、(2) 画面が変わっても
// rosbridge 接続が張り直されない。
// 観客表示 (AudienceView) は useRosbridge を自前で持ち /system/state を
// 使わないので、無駄な接続を増やさないよう Provider の外に置く。
ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    {AUDIENCE ? <AudienceView />
      : (
        <SystemStateProvider>
          {(TEST_MODE && !TEST_SCREEN)
            ? <AppShell><OperationCardHarness /></AppShell>
            : <Screens />}
        </SystemStateProvider>
      )}
  </React.StrictMode>,
)
