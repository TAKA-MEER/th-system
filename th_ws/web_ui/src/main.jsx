import React, { useState } from 'react'
import ReactDOM from 'react-dom/client'
import AppShell from './shell/AppShell.jsx'
import AudienceView from './audience/AudienceView.jsx'
import OperationCardHarness from './TestHarness.jsx'
import S00Connect from './screens/S00Connect.jsx'
import S01Main from './screens/S01Main.jsx'
import { SCREEN_NAMES } from './i18n/screens.js'

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
const SCREEN_IDS = { S00: 'S-00', S01: 'S-01' }

// 15 画面のうち S-00 / S-01 は WP-UI-02 で追加された (screens/ 配下)。
// 残りは以降の作業パケットで追加される。画面遷移はまだ本格的なルータでは
// なく、S-00 の「進む」だけがローカルに S-01 へ切り替える
// (DetailedDesign-webui.md §2.1 の遷移図: 走行方式・保守画面はまだ無いため
// ui.enter_mode が受理されてもそこへは進めない -- WP-UI-02 §11 相当)。
function Screens() {
  const [screen, setScreen] = useState(TEST_SCREEN || 'S00')
  if (screen === 'S00') {
    return (
      <AppShell screenName={SCREEN_NAMES.S00} screenId={SCREEN_IDS.S00}>
        <S00Connect onAdvance={() => setScreen('S01')} />
      </AppShell>
    )
  }
  return (
    <AppShell screenName={SCREEN_NAMES.S01} screenId={SCREEN_IDS.S01}>
      <S01Main />
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
