import React, { useEffect, useState } from 'react'
import ReactDOM from 'react-dom/client'
import AppShell from './shell/AppShell.jsx'
import FixedStage from './shell/FixedStage.jsx'
import { SystemStateProvider, useSystemState } from './ros/useSystemState.js'
import AudienceView from './audience/AudienceView.jsx'
import OperationCardHarness from './TestHarness.jsx'
import S00Connect from './screens/S00Connect.jsx'
import S01Main from './screens/S01Main.jsx'
import S11Manual from './screens/S11Manual.jsx'
import S13TeachManual from './screens/S13TeachManual.jsx'
import S14Replay from './screens/S14Replay.jsx'
import S50Settings from './screens/S50Settings.jsx'
import DriveTab from './screens/driveTab.jsx'
import { useJogPanel } from './shell/jogPanel.js'
import { SCREEN_NAMES, DRIVE_MANUAL, DRIVE_LEAVE } from './i18n/screens.js'
// 画面ルーティングの純ロジックは screens/screenRouting.js（ユニットテスト対象）。
import {
  SCREEN_IDS, MODE_TO_SCREEN, resolveScreen, initialPassedConnect,
} from './screens/screenRouting.js'

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

function Screens() {
  const { state } = useSystemState()
  const mode = state?.mode ?? null
  // S-00 は「疎通確認」という UI 上の前段で、FSM のモードではない。
  // 通過したかどうかだけがローカル状態。
  const [passedConnect, setPassedConnect] = useState(() => initialPassedConnect(TEST_SCREEN))
  // WS-9X: S-50 設定を開いているか（S-01 のサブ画面。ローカル state）。
  const [settingsOpen, setSettingsOpen] = useState(false)
  // モードが実画面（S-11/S-13/S-14）に対応したら設定は畳む（走行に入った・
  // フォルトで飛んだ 等）。INIT / IDLE は S-01 のままなので開いたままでよい。
  // resolveScreen 側も base !== 'S01' なら settingsOpen を無視するので二重の安全。
  useEffect(() => {
    if (mode && MODE_TO_SCREEN[mode]) setSettingsOpen(false)
  }, [mode])

  const screen = resolveScreen({ testScreen: TEST_SCREEN, passedConnect, mode, settingsOpen })

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
  if (screen === 'S50') {
    return (
      <AppShell screenName={SCREEN_NAMES.S50} screenId={SCREEN_IDS.S50}>
        <S50Settings onBack={() => setSettingsOpen(false)} />
      </AppShell>
    )
  }
  return (
    <AppShell screenName={SCREEN_NAMES.S01} screenId={SCREEN_IDS.S01}>
      {/* onEnter は渡さない。ui.enter_mode が受理されれば FSM が
          モードを変え、その /system/state を見てここが画面を切り替える。
          onOpenSettings は S-50（設定サブ画面）を開くだけ（FSM は動かさない）。 */}
      <S01Main onOpenSettings={() => setSettingsOpen(true)} />
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
          <FixedStage>
            {(TEST_MODE && !TEST_SCREEN)
              ? <AppShell><OperationCardHarness /></AppShell>
              : <Screens />}
          </FixedStage>
        </SystemStateProvider>
      )}
  </React.StrictMode>,
)
