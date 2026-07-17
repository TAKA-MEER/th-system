// ============================================================
// App.jsx — TH システム タブレット UI
// ============================================================
import { useState, useRef, useCallback, useEffect } from 'react'
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

// ジョグ速度 (押している間だけ /cmd_vel_manual に流す)
const JOG_LIN = 0.15   // m/s
const JOG_ANG = 0.6    // rad/s
const JOG_PUB_MS = 100 // publish 周期

// レーダー表示レンジ (m)
const RADAR_RANGE = 3.5

// 追跡中ターゲットと候補の同一判定距離 (m)
const TRACKED_MATCH_DIST = 0.35

// ── 候補レーダー (上から見た図。上=ロボット前方) ─────────────
function CandidateRadar({ candidates, personStatus, onSelect, connected }) {
  const size = 260               // SVG 論理サイズ (px)
  const c = size / 2
  const scale = (size / 2 - 18) / RADAR_RANGE

  // ロボット座標 (x=前方, y=左) → SVG (上=前方, 右=右)
  const toSvg = (p) => ({ sx: c - p.y * scale, sy: c - p.x * scale })

  const tracked = personStatus && !personStatus.is_lost ? personStatus.position : null
  const isTracked = (p) =>
    tracked && Math.hypot(p.x - tracked.x, p.y - tracked.y) < TRACKED_MATCH_DIST

  return (
    <svg
      className="radar"
      viewBox={`0 0 ${size} ${size}`}
      preserveAspectRatio="xMidYMid meet"
    >
      {/* レンジリング (1m 毎) */}
      {[1, 2, 3].map(r => (
        <circle key={r} cx={c} cy={c} r={r * scale} className="radar-ring" />
      ))}
      {[1, 2, 3].map(r => (
        <text key={r} x={c + 4} y={c - r * scale + 12} className="radar-ring-label">{r}m</text>
      ))}
      {/* 前方線 */}
      <line x1={c} y1={c} x2={c} y2={16} className="radar-axis" />
      <text x={c} y={11} textAnchor="middle" className="radar-ring-label">前</text>

      {/* ロボット (中心) */}
      <polygon
        points={`${c},${c - 9} ${c - 7},${c + 7} ${c + 7},${c + 7}`}
        className="radar-robot"
      />

      {/* 候補 */}
      {candidates.map((p, i) => {
        const { sx, sy } = toSvg(p)
        const trackedNow = isTracked(p)
        return (
          <g
            key={i}
            className={`radar-cand ${trackedNow ? 'tracked' : ''} ${connected ? '' : 'off'}`}
            onClick={() => connected && onSelect(i)}
          >
            {trackedNow && <circle cx={sx} cy={sy} r={15} className="radar-tracked-ring" />}
            <circle cx={sx} cy={sy} r={10} className="radar-dot" />
            <text x={sx} y={sy + 4} textAnchor="middle" className="radar-idx">{i + 1}</text>
          </g>
        )
      })}

      {/* 追跡中ターゲットが候補一覧に無い場合も位置を表示 */}
      {tracked && !candidates.some(isTracked) && (() => {
        const { sx, sy } = toSvg(tracked)
        return <circle cx={sx} cy={sy} r={12} className="radar-tracked-ring" />
      })()}
    </svg>
  )
}

export default function App() {
  const {
    connected, mode, modeName,
    fault, estop,
    personStatus, candidates, selectTarget, resetTracking,
    requestMode, publishTabletEstop, publishManualCmd, goToPanel,
  } = useRosbridge()

  const [estopActive, setEstopActive] = useState(false)

  // ── 緊急停止 (発動と解除を分離。発動ボタンは連打しても常に「停止」) ──
  const engageEstop = useCallback(() => {
    setEstopActive(true)
    publishTabletEstop(true)
  }, [publishTabletEstop])

  const releaseEstop = useCallback(() => {
    setEstopActive(false)
    publishTabletEstop(false)
  }, [publishTabletEstop])

  // ── ジョグ: 押している間だけ速度指令を publish ─────────────
  const jogTimerRef = useRef(null)
  const jogStart = useCallback((vx, wz) => {
    if (mode !== MODE.MANUAL) return
    if (jogTimerRef.current) clearInterval(jogTimerRef.current)
    publishManualCmd(vx, wz)
    jogTimerRef.current = setInterval(() => publishManualCmd(vx, wz), JOG_PUB_MS)
  }, [mode, publishManualCmd])

  const jogStop = useCallback(() => {
    if (jogTimerRef.current) {
      clearInterval(jogTimerRef.current)
      jogTimerRef.current = null
    }
    publishManualCmd(0, 0)
  }, [publishManualCmd])

  const jogProps = (vx, wz) => ({
    onPointerDown:   () => jogStart(vx, wz),
    onPointerUp:     jogStop,
    onPointerLeave:  jogStop,
    onPointerCancel: jogStop,
    onContextMenu:   (e) => e.preventDefault(),
  })

  // ── キーボード操作 ─────────────────────────────────────────
  //   Space / Esc : 緊急停止 (発動のみ。解除はキーボード不可 = 誤操作防止)
  //   W/A/S/D or 矢印キー : 手動ジョグ (MANUAL モード時のみ、押している間)
  const activeJogKeyRef = useRef(null)
  useEffect(() => {
    const JOG_KEYS = {
      'w': [JOG_LIN, 0],  'arrowup':    [JOG_LIN, 0],
      's': [-JOG_LIN, 0], 'arrowdown':  [-JOG_LIN, 0],
      'a': [0, JOG_ANG],  'arrowleft':  [0, JOG_ANG],
      'd': [0, -JOG_ANG], 'arrowright': [0, -JOG_ANG],
    }
    const onKeyDown = (e) => {
      const tag = e.target?.tagName
      if (tag === 'INPUT' || tag === 'TEXTAREA') return
      if (e.code === 'Space' || e.key === 'Escape') {
        e.preventDefault()
        engageEstop()
        return
      }
      if (e.repeat) return
      const key = e.key.toLowerCase()
      const cmd = JOG_KEYS[key]
      if (cmd) {
        e.preventDefault()
        activeJogKeyRef.current = key
        jogStart(cmd[0], cmd[1])
      }
    }
    const onKeyUp = (e) => {
      const key = e.key.toLowerCase()
      if (key === activeJogKeyRef.current) {
        activeJogKeyRef.current = null
        jogStop()
      }
    }
    // タブ非表示・フォーカス喪失時もキー押しっぱなし扱いにならないよう停止
    const onBlur = () => {
      if (activeJogKeyRef.current) {
        activeJogKeyRef.current = null
        jogStop()
      }
    }
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    window.addEventListener('blur', onBlur)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
      window.removeEventListener('blur', onBlur)
    }
  }, [engageEstop, jogStart, jogStop])

  // ── モードバッジの色 ──────────────────────────────────────
  const modeColor = {
    INIT: '#888', IDLE: '#2196F3', FOLLOWING: '#4CAF50',
    MOVING_TO_PANEL: '#FF9800', AT_PANEL: '#9C27B0',
    MANUAL: '#00BCD4', ESTOP: '#F44336', FOLLOWING_MAPLESS: '#8BC34A',
  }[modeName] ?? '#888'

  const isFault = fault?.active
  const isTrackedNow = personStatus && !personStatus.is_lost

  return (
    <div className="app">

      {/* ── ヘッダー ─────────────────────────────────── */}
      <header className="header">
        <span className="title">TH システム</span>
        <div className="mode-inline" style={{ backgroundColor: modeColor }}>
          <span className="mode-label">{modeName}</span>
          {isFault && <span className="fault-badge">⚠ {fault.fault_type}</span>}
        </div>
        <span className={`conn-badge ${connected ? 'ok' : 'ng'}`}>
          {connected ? '● 接続中' : '○ 切断'}
        </span>
      </header>

      {/* ── 緊急停止ボタン (発動専用。連打しても常に停止のまま) ── */}
      <button
        className={`estop-btn ${estopActive ? 'estop-active' : ''}`}
        onClick={engageEstop}
      >
        {estopActive ? '■ 緊急停止中 (Space/Esc)' : '⚠ 緊急停止 (Space/Esc)'}
      </button>

      {/* ── フォルト情報 ─────────────────────────────── */}
      {isFault && (
        <div className="fault-bar">
          <b>⚠ フォルト:</b> {fault.fault_type} — {fault.description}
        </div>
      )}

      {/* ── メイングリッド (横長レイアウト) ──────────── */}
      <div className="grid">

        {/* ── 追従ターゲット ─────────────────────────── */}
        <section className="card card-target">
          <h2>
            追従ターゲット{' '}
            <span className={`target-state ${isTrackedNow ? 'ok' : 'ng'}`}>
              {isTrackedNow
                ? `追跡中 (${personStatus.position.x.toFixed(1)}, ${personStatus.position.y.toFixed(1)})`
                : personStatus?.lost_reason === 'TARGET_SWITCHED'
                  ? '⚠ 対象切替の疑い — 選び直してください'
                  : `未追跡${personStatus?.lost_reason ? ` (${personStatus.lost_reason})` : ''}`}
            </span>
          </h2>
          <div className="target-body">
            <CandidateRadar
              candidates={candidates}
              personStatus={personStatus}
              onSelect={selectTarget}
              connected={connected}
            />
            <div className="target-side">
              <p className="note">
                図の●をタップで対象を選択。緑リング = 現在の追跡対象。
              </p>
              <button className="mode-btn" disabled={!connected} onClick={resetTracking}>
                最も近い人を再取得
              </button>
            </div>
          </div>
        </section>

        {/* ── モード切替 ─────────────────────────────── */}
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

        {/* ── 手動ジョグ (押している間だけ動く) ───────── */}
        <section className={`card ${mode !== MODE.MANUAL ? 'disabled' : ''}`}>
          <h2>手動移動 <span className="note">(押している間だけ動く / キー: WASD・矢印)</span></h2>
          <div className="jog-grid">
            <div />
            <button className="jog-btn" {...jogProps(JOG_LIN, 0)}>▲</button>
            <div />
            <button className="jog-btn" {...jogProps(0, JOG_ANG)}>↺</button>
            <button className="jog-btn stop" onClick={jogStop}>■</button>
            <button className="jog-btn" {...jogProps(0, -JOG_ANG)}>↻</button>
            <div />
            <button className="jog-btn" {...jogProps(-JOG_LIN, 0)}>▼</button>
            <div />
          </div>
        </section>

        {/* ── 配電盤移動 ─────────────────────────────── */}
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

        {/* ── 緊急停止の解除 (発動ボタンと離れた位置・見た目も別) ── */}
        {estopActive && (
          <section className="card estop-release-card">
            <h2>緊急停止の解除</h2>
            <p className="note">
              安全を確認してから解除してください。キーボードからは解除できません。
            </p>
            <button className="estop-release-btn" onClick={releaseEstop}>
              ✓ 安全確認済み — 緊急停止を解除する
            </button>
          </section>
        )}

      </div>
    </div>
  )
}
