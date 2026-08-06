import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App.jsx'
import AudienceView from './audience/AudienceView.jsx'

// ?view=audience で観客向け表示 (VISION.md §6.3)。
// App 側で分岐せずマウントするツリーごと分けている。こうしておくと
// 操作 UI のジョグ用 setInterval・音声・heartbeat が観客画面では
// そもそも起動しない = 走行制御に触れないことが構造で保証される。
const AUDIENCE = new URLSearchParams(window.location.search).get('view') === 'audience'

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    {AUDIENCE ? <AudienceView /> : <App />}
  </React.StrictMode>,
)
