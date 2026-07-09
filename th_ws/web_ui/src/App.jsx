// ============================================================
// App.jsx — TH システム タブレット UI
// ============================================================
import { useState, useRef } from 'react'
import { useRosbridge } from './hooks/useRosbridge'
import './App.css'

// ROS2 モード定数
const MODE = { INIT:0, IDLE:1, FOLLOWING:2, MOVING_TO_PANEL:3, AT_PANEL:4, MANUAL:5, ESTOP:6, FOLLOWING_MAPLESS:7 }

// 配電盤リスト (panels.yaml と合わせること)
const PANELS = [
  { id: 'panel_01', label: '第1配電盤' },
  { id: 'panel_02', label: '第2配電盤' },
  { id: 'panel_03', label: '第3配電盤' },
]

// ジョグ距離 (m)
const JOG_STEP = 0.3

export default function App() {
  const {
    connected, mode, modeName,
    fault, estop,
    requestMode, publishTabletEstop, sendManualGoal, goToPanel,
  } = useRosbridge()

  const [estopActive, setEstopActive] = useState(false)

  // ── 緊急停止トグル ─────────────────────────────────────────
  const handleEstop = () => {
    const next = !estopActive
    setEstopActive(next)
    publishTabletEstop(next)
  }

  // ── ジョグボタン ──────────────────────────────────────────
  const jog = (dx, dy) => {
    if (mode !== MODE.MANUAL) return
    sendManualGoal(dx, dy)
  }

  // ── モードバッジの色 ──────────────────────────────────────
  const modeColor = {
    INIT: '#888', IDLE: '#2196F3', FOLLOWING: '#4CAF50',
    MOVING_TO_PANEL: '#FF9800', AT_PANEL: '#9C27B0',
    MANUAL: '#00BCD4', ESTOP: '#F44336', FOLLOWING_MAPLESS: '#8BC34A',
  }[modeName] ?? '#888'

  const isFault = fault?.active

  return (
    <div className="app">

      {/* ── ヘッダー ─────────────────────────────────── */}
      <header className="header">
        <span className="title">TH システム</span>
        <span className={`conn-badge ${connected ? 'ok' : 'ng'}`}>
          {connected ? '● 接続中' : '○ 切断'}
        </span>
      </header>

      {/* ── モード表示 ───────────────────────────────── */}
      <div className="mode-bar" style={{ backgroundColor: modeColor }}>
        <span className="mode-label">{modeName}</span>
        {isFault && (
          <span className="fault-badge">⚠ {fault.fault_type}</span>
        )}
      </div>

      {/* ── 緊急停止ボタン ──────────────────────────── */}
      <button
        className={`estop-btn ${estopActive ? 'estop-active' : ''}`}
        onClick={handleEstop}
      >
        {estopActive ? '■ 緊急停止 解除' : '⚠ 緊急停止'}
      </button>

      {/* ── モード切替 ───────────────────────────────── */}
      <section className="card">
        <h2>モード操作</h2>
        <div className="btn-row">
          <button
            className="mode-btn"
            disabled={mode === MODE.FOLLOWING || !connected}
            onClick={() => requestMode(MODE.FOLLOWING)}
          >
            追従開始
          </button>
          <button
            className="mode-btn"
            disabled={mode === MODE.FOLLOWING_MAPLESS || !connected}
            onClick={() => requestMode(MODE.FOLLOWING_MAPLESS)}
          >
            軌跡追従(マップ不要)
          </button>
          <button
            className="mode-btn"
            disabled={mode === MODE.MANUAL || !connected}
            onClick={() => requestMode(MODE.MANUAL)}
          >
            手動操作
          </button>
          <button
            className="mode-btn idle"
            disabled={mode === MODE.IDLE || !connected}
            onClick={() => requestMode(MODE.IDLE)}
          >
            待機 (IDLE)
          </button>
        </div>
      </section>

      {/* ── 手動ジョグ ──────────────────────────────── */}
      <section className={`card ${mode !== MODE.MANUAL ? 'disabled' : ''}`}>
        <h2>手動移動 <span className="note">({JOG_STEP * 100} cm/回)</span></h2>
        <div className="jog-grid">
          <div />
          <button className="jog-btn" onClick={() => jog(JOG_STEP, 0)}>▲</button>
          <div />
          <button className="jog-btn" onClick={() => jog(0,  JOG_STEP)}>◀</button>
          <button className="jog-btn stop" onClick={() => jog(0, 0)}>■</button>
          <button className="jog-btn" onClick={() => jog(0, -JOG_STEP)}>▶</button>
          <div />
          <button className="jog-btn" onClick={() => jog(-JOG_STEP, 0)}>▼</button>
          <div />
        </div>
      </section>

      {/* ── 配電盤移動 ──────────────────────────────── */}
      <section className="card">
        <h2>配電盤へ移動</h2>
        <div className="panel-list">
          {PANELS.map(p => (
            <button
              key={p.id}
              className="panel-btn"
              disabled={!connected || estopActive}
              onClick={() => {
                requestMode(MODE.FOLLOWING)  // FOLLOWING 経由でトリガー
                setTimeout(() => goToPanel(p.id), 300)
              }}
            >
              {p.label}
            </button>
          ))}
        </div>
      </section>

      {/* ── フォルト情報 ─────────────────────────────── */}
      {isFault && (
        <div className="fault-bar">
          <b>⚠ フォルト:</b> {fault.fault_type} — {fault.description}
        </div>
      )}

    </div>
  )
}
