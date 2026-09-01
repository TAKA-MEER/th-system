import React, { useState } from 'react'
import ReactDOM from 'react-dom/client'
import AppShell from './shell/AppShell.jsx'
import AudienceView from './audience/AudienceView.jsx'
import OperationCardHarness from './TestHarness.jsx'
import S00Connect from './screens/S00Connect.jsx'
import S01Main from './screens/S01Main.jsx'
import S11Manual from './screens/S11Manual.jsx'
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
const SCREEN_IDS = { S00: 'S-00', S01: 'S-01', S11: 'S-11' }

// mode -> screen-key map, owned here so Screens() can switch. S-01 sends
// ui.enter_mode; when th_state accepts it, S01Main calls onEnter(mode) which
// reads this map to decide where to go. Later packets add one row per screen
// (DetailedDesign-wp3.md WP-TRANSIT-01 §11 c1: "遷移先を配列化して次の画面で
// 増やしやすくする"). A mode with no mapped screen yet (e.g. FOLLOW -> S-10,
// which is WP-UI-04) is simply absent: S-01 stays put and the header's mode
// pill reflects the change.
const MODE_TO_SCREEN = { MANUAL: 'S11' }

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

// 15 画面のうち S-00 / S-01 は WP-UI-02 で追加された (screens/ 配下)。
// 残りは以降の作業パケットで追加される。画面遷移はまだ本格的なルータでは
// なく、S-00 の「進む」だけがローカルに S-01 へ切り替える
// (DetailedDesign-webui.md §2.1 の遷移図: 走行方式・保守画面はまだ無いため
// ui.enter_mode が受理されてもそこへは進めない -- WP-UI-02 §11 相当)。
function Screens() {
  const [screen, setScreen] = useState(TEST_SCREEN || 'S00')
  if (screen === 'DRIVE_S11') {
    return <DriveTestScreen />
  }
  if (screen === 'S00') {
    return (
      <AppShell screenName={SCREEN_NAMES.S00} screenId={SCREEN_IDS.S00}>
        <S00Connect onAdvance={() => setScreen('S01')} />
      </AppShell>
    )
  }
  if (screen === 'S11') {
    return (
      <AppShell screenName={SCREEN_NAMES.S11} screenId={SCREEN_IDS.S11}>
        <S11Manual onFinish={() => setScreen('S01')} />
      </AppShell>
    )
  }
  return (
    <AppShell screenName={SCREEN_NAMES.S01} screenId={SCREEN_IDS.S01}>
      <S01Main onEnter={(mode) => {
        const next = MODE_TO_SCREEN[mode]
        if (next) setScreen(next)
      }} />
    </AppShell>
  )
}

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    {AUDIENCE ? <AudienceView />
      : (TEST_MODE && !TEST_SCREEN) ? <AppShell><OperationCardHarness /></AppShell>
      : <Screens />}
  </React.StrictMode>,
)
