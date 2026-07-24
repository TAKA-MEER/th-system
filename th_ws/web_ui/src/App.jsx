// ============================================================
// App.jsx — TH システム タブレット UI
// ============================================================
import { useState, useRef, useCallback, useEffect } from 'react'
import { useRosbridge } from './hooks/useRosbridge'
import SettingsPanel from './SettingsPanel'
import './App.css'

// ROS2 モード定数
const MODE = { INIT:0, IDLE:1, FOLLOWING:2, MOVING_TO_PANEL:3, AT_PANEL:4, MANUAL:5, ESTOP:6, FOLLOWING_MAPLESS:7, SUMMONING:8 }

// 配電盤リスト (panels.yaml と合わせること)
const PANELS = [
  { id: 'panel_01', label: '第1配電盤' },
  { id: 'panel_02', label: '第2配電盤' },
  { id: 'panel_03', label: '第3配電盤' },
]

// ジョグ速度 (押している間だけ /cmd_vel_manual に流す)
// 実際の速度は UI の速度設定 (speedPct) を掛けて決まる。100% 時の上限:
const JOG_LIN_MAX = 0.5    // m/s
const JOG_ANG_MAX = 2.0    // rad/s (従来の lin:ang = 1:4 の比を維持)
const JOG_SPEED_MIN = 0.1  // 速度スライダー下限 (10%)
const JOG_PUB_MS = 100     // publish 周期
// 緩旋回 (直進 + 旋回同時) 時の旋回レート。crawler_teleop の緩旋回と同じ 0.5 倍
const JOG_ARC_ANG_SCALE = 0.5
// スティックのデッドゾーン (中心からの倒し量がこの割合未満なら停止)
const STICK_DEADZONE = 0.15
const JOG_PRESETS = [
  { label: '低速', pct: 0.3 },   // 0.15 m/s / 0.6 rad/s (従来の固定値と同じ)
  { label: '中速', pct: 0.6 },   // 0.30 m/s / 1.2 rad/s
  { label: '高速', pct: 1.0 },   // 0.50 m/s / 2.0 rad/s
]

// 速度設定の永続化 (リロード後も維持)
const SPEED_STORAGE_KEY = 'th_jog_speed_pct'
const loadSpeedPct = () => {
  const v = parseFloat(localStorage.getItem(SPEED_STORAGE_KEY))
  return Number.isFinite(v) ? Math.min(1, Math.max(JOG_SPEED_MIN, v)) : 0.3
}

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

// ── スティック位置 → 正規化コマンド ──────────────────────────
// dx: 右+ / dy: 上+ (いずれも -1..1)。8方向セクター (45°幅) で判定:
//   上下 = 直進 / 真横 = 超信地旋回 / 斜め = 緩旋回 (旋回 0.5 倍)
// 倒し量 (デッドゾーン超過分を 0→1 に正規化) が速度の比率になる
function stickToCmd(dx, dy, len) {
  if (len < STICK_DEADZONE) return { vn: 0, wn: 0, label: null }
  const m = Math.min(1, (len - STICK_DEADZONE) / (1 - STICK_DEADZONE))
  const deg = Math.abs(Math.atan2(dx, dy)) * 180 / Math.PI  // 0=上, 90=横, 180=下
  const turn = dx > 0 ? -1 : 1  // 右に倒す = 右旋回 (wz 負)
  if (deg < 22.5)  return { vn:  m, wn: 0,                              label: '前進' }
  if (deg < 67.5)  return { vn:  m, wn: turn * JOG_ARC_ANG_SCALE * m,   label: '緩旋回' }
  if (deg < 112.5) return { vn:  0, wn: turn * m,                       label: '超信地旋回' }
  if (deg < 157.5) return { vn: -m, wn: turn * JOG_ARC_ANG_SCALE * m,   label: '後退緩旋回' }
  return { vn: -m, wn: 0, label: '後退' }
}

// ── バーチャルスティック ─────────────────────────────────────
function VirtualStick({ speedPct, onChange, onRelease }) {
  const size = 240
  const c = size / 2
  const knobR = 30
  const travel = c - knobR - 4   // ノブ中心の可動半径 (px, SVG 論理座標)
  const svgRef = useRef(null)
  const pointerIdRef = useRef(null)
  const [stick, setStick] = useState({ x: 0, y: 0, cmd: null })

  const update = (e) => {
    const rect = svgRef.current.getBoundingClientRect()
    let dx = (e.clientX - (rect.left + rect.width / 2)) / (rect.width / 2)
    let dy = ((rect.top + rect.height / 2) - e.clientY) / (rect.height / 2)
    const len = Math.hypot(dx, dy)
    if (len > 1) { dx /= len; dy /= len }
    const cmd = stickToCmd(dx, dy, Math.min(1, len))
    setStick({ x: dx * travel, y: -dy * travel, cmd })
    onChange(cmd)
  }

  const handleDown = (e) => {
    e.preventDefault()
    e.currentTarget.setPointerCapture(e.pointerId)
    pointerIdRef.current = e.pointerId
    update(e)
  }
  const handleMove = (e) => {
    if (pointerIdRef.current !== e.pointerId) return
    update(e)
  }
  const handleUp = (e) => {
    if (pointerIdRef.current !== e.pointerId) return
    pointerIdRef.current = null
    setStick({ x: 0, y: 0, cmd: null })
    onRelease()
  }

  const cmd = stick.cmd
  return (
    <div className="stick-wrap">
      <svg
        ref={svgRef}
        className="stick-pad"
        viewBox={`0 0 ${size} ${size}`}
        onPointerDown={handleDown}
        onPointerMove={handleMove}
        onPointerUp={handleUp}
        onPointerCancel={handleUp}
        onContextMenu={(e) => e.preventDefault()}
      >
        <circle cx={c} cy={c} r={c - 2} className="stick-ring" />
        {/* セクター境界 (22.5° 起点で45°毎) */}
        {[22.5, 67.5, 112.5, 157.5].map(a => {
          const r = (a * Math.PI) / 180
          const x = Math.sin(r) * (c - 2), y = Math.cos(r) * (c - 2)
          return (
            <g key={a} className="stick-guide">
              <line x1={c - x} y1={c - y} x2={c + x} y2={c + y} />
            </g>
          )
        })}
        <circle cx={c} cy={c} r={(c - 2) * STICK_DEADZONE} className="stick-dead" />
        <text x={c} y={14} textAnchor="middle" className="stick-hint">前</text>
        <text x={c} y={size - 6} textAnchor="middle" className="stick-hint">後</text>
        <text x={10} y={c + 4} textAnchor="middle" className="stick-hint">旋回</text>
        <text x={size - 10} y={c + 4} textAnchor="middle" className="stick-hint">旋回</text>
        <circle
          cx={c + stick.x} cy={c + stick.y} r={knobR}
          className={`stick-knob ${cmd?.label ? 'active' : ''}`}
        />
      </svg>
      <div className={`stick-status ${cmd?.label ? 'active' : ''}`}>
        {cmd?.label
          ? `${cmd.label}  ${(Math.abs(cmd.vn) * JOG_LIN_MAX * speedPct).toFixed(2)} m/s / ` +
            `${(Math.abs(cmd.wn) * JOG_ANG_MAX * speedPct).toFixed(2)} rad/s`
          : '上=前進 / 横=その場旋回 / 斜め=緩旋回'}
      </div>
    </div>
  )
}

export default function App() {
  const {
    connected, mode, modeName,
    fault, estop,
    personStatus, candidates, selectTarget, resetTracking,
    requestMode, publishTabletEstop, publishManualCmd, goToPanel,
    summonRobot,
    mappingActive, toggleMapping,
    getTunableParams, applyTunableParam, saveTunableParams,
  } = useRosbridge()

  const [estopActive, setEstopActive] = useState(false)
  const [settingsOpen, setSettingsOpen] = useState(false)

  // ── ジョグ速度設定 (プリセット + スライダー) ────────────────
  // publish ループは ref を読むため、押しっぱなし中の変更も次周期から反映される
  const [speedPct, setSpeedPct] = useState(loadSpeedPct)
  const speedPctRef = useRef(speedPct)
  useEffect(() => {
    speedPctRef.current = speedPct
    localStorage.setItem(SPEED_STORAGE_KEY, String(speedPct))
  }, [speedPct])

  // ── 緊急停止 (発動と解除を分離。発動ボタンは連打しても常に「停止」) ──
  const engageEstop = useCallback(() => {
    setEstopActive(true)
    publishTabletEstop(true)
  }, [publishTabletEstop])

  const releaseEstop = useCallback(() => {
    setEstopActive(false)
    publishTabletEstop(false)
  }, [publishTabletEstop])

  // ── ジョグ: 正規化コマンド (vn, wn ∈ [-1,1]) を周期 publish ──
  // スティック / キーボードの両入力が jogNormRef を更新し、単一の
  // interval が最新値 × 最新速度設定を publish する (操作中の速度変更も
  // 次周期から反映)。入力がなくなったら (0,0) を publish して停止
  const jogNormRef = useRef({ vn: 0, wn: 0 })
  const jogTimerRef = useRef(null)
  const stickActiveRef = useRef(false)  // ドラッグ中はキーボード入力を無視
  const heldKeysRef = useRef(new Map()) // 押下中のジョグキー → [lin, ang]

  const jogPublish = useCallback(() => {
    const { vn, wn } = jogNormRef.current
    publishManualCmd(
      vn * JOG_LIN_MAX * speedPctRef.current,
      wn * JOG_ANG_MAX * speedPctRef.current,
    )
  }, [publishManualCmd])

  const setJogInput = useCallback((vn, wn) => {
    if (mode !== MODE.MANUAL) return
    jogNormRef.current = { vn, wn }
    jogPublish()
    if (!jogTimerRef.current) jogTimerRef.current = setInterval(jogPublish, JOG_PUB_MS)
  }, [mode, jogPublish])

  const clearJogInput = useCallback(() => {
    jogNormRef.current = { vn: 0, wn: 0 }
    if (jogTimerRef.current) {
      clearInterval(jogTimerRef.current)
      jogTimerRef.current = null
      publishManualCmd(0, 0)
    }
  }, [publishManualCmd])

  // MANUAL から離れたら入力を全クリアして停止 (interval の残留防止)
  useEffect(() => {
    if (mode !== MODE.MANUAL) {
      stickActiveRef.current = false
      heldKeysRef.current.clear()
      clearJogInput()
    }
  }, [mode, clearJogInput])

  // ── スティック入力 ─────────────────────────────────────────
  const stickChange = useCallback((cmd) => {
    stickActiveRef.current = true
    setJogInput(cmd.vn, cmd.wn)
  }, [setJogInput])

  const stickRelease = useCallback(() => {
    stickActiveRef.current = false
    clearJogInput()
  }, [clearJogInput])

  // ── キーボード操作 ─────────────────────────────────────────
  //   Space / Esc : 緊急停止 (発動のみ。解除はキーボード不可 = 誤操作防止)
  //   W/A/S/D or 矢印キー : 手動ジョグ (MANUAL モード時のみ、押している間)
  //   同時押しで合成: W+A = 前進左緩旋回 (旋回 0.5 倍) / A 単独 = 超信地旋回
  useEffect(() => {
    const JOG_KEYS = {   // 値は [lin, ang] の寄与 (-1/0/+1)
      'w': [1, 0],  'arrowup':    [1, 0],
      's': [-1, 0], 'arrowdown':  [-1, 0],
      'a': [0, 1],  'arrowleft':  [0, 1],
      'd': [0, -1], 'arrowright': [0, -1],
    }
    // 押下中キーの寄与を合成して正規化コマンドに変換
    const applyKeys = () => {
      if (stickActiveRef.current) return
      if (heldKeysRef.current.size === 0) {
        clearJogInput()
        return
      }
      let lin = 0, ang = 0
      for (const [l, a] of heldKeysRef.current.values()) { lin += l; ang += a }
      lin = Math.max(-1, Math.min(1, lin))
      ang = Math.max(-1, Math.min(1, ang))
      if (lin !== 0 && ang !== 0) ang *= JOG_ARC_ANG_SCALE  // 緩旋回
      setJogInput(lin, ang)
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
        if (mode !== MODE.MANUAL || stickActiveRef.current) return
        heldKeysRef.current.set(key, cmd)
        applyKeys()
      }
    }
    const onKeyUp = (e) => {
      const key = e.key.toLowerCase()
      if (heldKeysRef.current.delete(key)) applyKeys()
    }
    // タブ非表示・フォーカス喪失時もキー押しっぱなし扱いにならないよう停止
    const onBlur = () => {
      if (heldKeysRef.current.size > 0) {
        heldKeysRef.current.clear()
        clearJogInput()
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
  }, [mode, engageEstop, setJogInput, clearJogInput])

  // ── モードバッジの色 ──────────────────────────────────────
  const modeColor = {
    INIT: '#888', IDLE: '#2196F3', FOLLOWING: '#4CAF50',
    MOVING_TO_PANEL: '#FF9800', AT_PANEL: '#9C27B0',
    MANUAL: '#00BCD4', ESTOP: '#F44336', FOLLOWING_MAPLESS: '#8BC34A',
    SUMMONING: '#E91E63',
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
        <button className="settings-btn" onClick={() => setSettingsOpen(true)} aria-label="設定">
          ⚙
        </button>
      </header>

      <SettingsPanel
        open={settingsOpen}
        onClose={() => setSettingsOpen(false)}
        editable={mode === MODE.IDLE || mode === MODE.MANUAL}
        getTunableParams={getTunableParams}
        applyTunableParam={applyTunableParam}
        saveTunableParams={saveTunableParams}
      />

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
            <button
              className="mode-btn"
              disabled={mode !== MODE.IDLE || !connected}
              onClick={summonRobot}
            >
              呼び寄せ
            </button>
          </div>
        </section>

        {/* ── 地図作成 開始/停止 ─────────────────────── */}
        <section className="card">
          <h2>
            地図作成{' '}
            <span className={`target-state ${mappingActive ? 'ok' : 'ng'}`}>
              {mappingActive ? '作成中' : '停止中'}
            </span>
          </h2>
          <div className="btn-row">
            <button
              className="mode-btn"
              disabled={!connected || (mode !== MODE.IDLE && mode !== MODE.MANUAL)}
              onClick={toggleMapping}
            >
              {mappingActive ? '地図作成停止' : '地図作成開始'}
            </button>
          </div>
        </section>

        {/* ── 手動ジョグ (押している間だけ動く) ───────── */}
        <section className={`card ${mode !== MODE.MANUAL ? 'disabled' : ''}`}>
          <h2>手動移動 <span className="note">(触れている間だけ動く / キー: WASD・矢印, 同時押しで緩旋回)</span></h2>
          <div className="speed-ctrl">
            <div className="speed-label">
              速度: <b>{(JOG_LIN_MAX * speedPct).toFixed(2)} m/s</b>
              <span className="speed-pct">({Math.round(speedPct * 100)}%)</span>
            </div>
            <div className="speed-presets">
              {JOG_PRESETS.map(p => (
                <button
                  key={p.label}
                  className={`speed-preset ${Math.abs(speedPct - p.pct) < 0.001 ? 'active' : ''}`}
                  onClick={() => setSpeedPct(p.pct)}
                >
                  {p.label}
                </button>
              ))}
            </div>
            <input
              type="range"
              className="speed-slider"
              min={JOG_SPEED_MIN * 100}
              max={100}
              step={5}
              value={Math.round(speedPct * 100)}
              onChange={(e) => setSpeedPct(Number(e.target.value) / 100)}
            />
          </div>
          <VirtualStick
            speedPct={speedPct}
            onChange={stickChange}
            onRelease={stickRelease}
          />
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
