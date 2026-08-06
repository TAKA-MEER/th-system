// ============================================================
// App.jsx — TH システム タブレット UI
// ============================================================
import { useState, useRef, useCallback, useEffect } from 'react'
import { useRosbridge } from './hooks/useRosbridge'
import { useVoice } from './hooks/useVoice'
import { useVoiceTriggers } from './hooks/useVoiceTriggers'
import SettingsPanel from './SettingsPanel'
import VoiceDevPanel from './VoiceDevPanel'
import MapView from './MapView'
import WheelSpeedView from './WheelSpeedView'
import { MODE, modeColorOf } from './robotMode'
import { LAYER } from './voice/announcements'
import './App.css'

// 音声テストパネルは ?dev=1 のときだけ出す。URL はリロードなしに変わらないため
// モジュールスコープで一度だけ判定する
const DEV_MODE = new URLSearchParams(window.location.search).get('dev') === '1'

// タブ構成 (VISION.md §6.1)。運用フェーズ単位で分ける。
// サブシステム単位に並べると走行中に不要なカードが画面を埋め、
// 操作に必要なものがスクロールの外へ出てしまう
const TABS = [
  { id: 'operate', label: '運用' },
  { id: 'setup',   label: '準備' },
  { id: 'diag',    label: '診断' },
]

// ジョグ速度 (押している間だけ /cmd_vel_manual に流す)
// 実際の速度は UI の速度設定 (speedPct) を掛けて決まる。100% 時の上限:
const JOG_LIN_MAX = 0.5    // m/s
const JOG_ANG_MAX = 2.0    // rad/s (従来の lin:ang = 1:4 の比を維持)
const JOG_SPEED_MIN = 0.1  // 速度スライダー下限 (10%)
const JOG_PUB_MS = 100     // publish 周期
// ジョグ入力のレート制限 (急加減速防止。実機検証: これが無いとスティック/
// キー入力の変化が瞬時にそのまま速度指令へ反映され機体が激しく揺れる)。
// 離した瞬間の停止は即時 0 publish のままなので安全性は損なわれない。
const JOG_LIN_ACCEL = 1.0  // m/s^2
const JOG_ANG_ACCEL = 4.0  // rad/s^2
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

// current を target へ maxDelta の範囲内で近づける (加減速レート制限)
function rampToward(current, target, maxDelta) {
  if (target > current + maxDelta) return current + maxDelta
  if (target < current - maxDelta) return current - maxDelta
  return target
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

  // ドラッグ中にこのコンポーネントがアンマウントされる (タブ切替など) と
  // pointerup が届かず、最後のジョグ指令が publish され続けてしまう。
  // アンマウント時に確実に停止させる
  const onReleaseRef = useRef(onRelease)
  onReleaseRef.current = onRelease
  useEffect(() => () => {
    if (pointerIdRef.current !== null) onReleaseRef.current()
  }, [])

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
  const ros = useRosbridge()
  const {
    connected, mode, modeName,
    fault, estop,
    personStatus, candidates, selectTarget, resetTracking,
    requestMode, publishTabletEstop, publishManualCmd,
    summonRobot,
    mappingActive, toggleMapping,
    actionError, clearActionError,
    mapData, robotPose, scanData, pathData, wheelSpeedData,
    getTunableParams, applyTunableParam, saveTunableParams,
  } = ros

  const [estopActive, setEstopActive] = useState(false)
  const [settingsOpen, setSettingsOpen] = useState(false)
  const [voiceDevOpen, setVoiceDevOpen] = useState(false)
  const [tab, setTab] = useState('operate')

  // 音声アナウンス (VISION.md §7)。estopActive はローカル state なので
  // ros に混ぜて渡し、タブレット側の押下でも即座に鳴らせるようにする
  const voice = useVoice()
  useVoiceTriggers({ ...ros, estopActive }, voice)

  // ── ジョグ速度設定 (プリセット + スライダー) ────────────────
  // publish ループは ref を読むため、押しっぱなし中の変更も次周期から反映される
  const [speedPct, setSpeedPct] = useState(loadSpeedPct)
  const speedPctRef = useRef(speedPct)
  useEffect(() => {
    speedPctRef.current = speedPct
    localStorage.setItem(SPEED_STORAGE_KEY, String(speedPct))
  }, [speedPct])

  // ── ジョグ: 正規化コマンド (vn, wn ∈ [-1,1]) を周期 publish ──
  // スティック / キーボードの両入力が jogNormRef を更新し、単一の
  // interval が最新値 × 最新速度設定を publish する (操作中の速度変更も
  // 次周期から反映)。入力がなくなったら (0,0) を publish して停止
  const jogNormRef = useRef({ vn: 0, wn: 0 })
  const jogTimerRef = useRef(null)
  const stickActiveRef = useRef(false)  // ドラッグ中はキーボード入力を無視
  const heldKeysRef = useRef(new Map()) // 押下中のジョグキー → [lin, ang]
  // 直近に publish した実際の速度 (目標値へこれをレート制限しながら近づける)
  const currentCmdRef = useRef({ vx: 0, wz: 0 })

  const jogPublish = useCallback(() => {
    const { vn, wn } = jogNormRef.current
    const targetVx = vn * JOG_LIN_MAX * speedPctRef.current
    const targetWz = wn * JOG_ANG_MAX * speedPctRef.current
    const dtSec = JOG_PUB_MS / 1000
    const cur = currentCmdRef.current
    cur.vx = rampToward(cur.vx, targetVx, JOG_LIN_ACCEL * dtSec)
    cur.wz = rampToward(cur.wz, targetWz, JOG_ANG_ACCEL * dtSec)
    publishManualCmd(cur.vx, cur.wz)
  }, [publishManualCmd])

  const setJogInput = useCallback((vn, wn) => {
    if (mode !== MODE.MANUAL) return
    jogNormRef.current = { vn, wn }
    jogPublish()
    if (!jogTimerRef.current) jogTimerRef.current = setInterval(jogPublish, JOG_PUB_MS)
  }, [mode, jogPublish])

  const clearJogInput = useCallback(() => {
    jogNormRef.current = { vn: 0, wn: 0 }
    currentCmdRef.current = { vx: 0, wz: 0 }
    if (jogTimerRef.current) {
      clearInterval(jogTimerRef.current)
      jogTimerRef.current = null
      publishManualCmd(0, 0)
    }
  }, [publishManualCmd])

  // ── 緊急停止 (発動と解除を分離。発動ボタンは連打しても常に「停止」) ──
  // 発動時は mode が ESTOP に切り替わる round-trip を待たず、保持中の
  // ジョグ入力 (キー/スティック) を即座にローカルでクリアする。これが無いと
  // 前進キーを押したまま緊急停止しても、モード遷移が反映されるまでの間
  // /cmd_vel_manual の前進指令送信が続いてしまう (2026-08-06 報告)。
  const engageEstop = useCallback(() => {
    setEstopActive(true)
    // ジョグのゼロ指令をロックより先に出す (mux がまだロックされていない
    // うちに /cmd_vel_manual へゼロを通しておく。ロック自体は esp32_bridge
    // 側でも独立して強制されるため、これは多重防御の一枚)。
    stickActiveRef.current = false
    heldKeysRef.current.clear()
    clearJogInput()
    publishTabletEstop(true)
  }, [publishTabletEstop, clearJogInput])

  const releaseEstop = useCallback(() => {
    setEstopActive(false)
    publishTabletEstop(false)
  }, [publishTabletEstop])

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

  // ── モードバッジの色 (観客表示と共有。robotMode.js) ────────
  const modeColor = modeColorOf(modeName)

  const isFault = fault?.active
  const isTrackedNow = personStatus && !personStatus.is_lost
  // 接続断・フォルトは画面全体で分かる表示にする (VISION.md §6.1)。試験員は
  // 前を歩いていて画面を見ていない前提なので、視線を戻した瞬間に気付ける必要がある
  const alertLevel = !connected ? 'disconnect' : isFault ? 'fault' : null

  return (
    <div className={`app ${alertLevel ? `app-alert app-alert-${alertLevel}` : ''}`}>

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
        {DEV_MODE && (
          <button className="settings-btn" onClick={() => setVoiceDevOpen(true)} aria-label="音声テスト">
            ♪
          </button>
        )}
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

      {DEV_MODE && (
        <VoiceDevPanel
          open={voiceDevOpen}
          onClose={() => setVoiceDevOpen(false)}
          voice={voice}
        />
      )}

      {/* ── 緊急停止ボタン (発動専用。連打しても常に停止のまま) ── */}
      <button
        className={`estop-btn ${estopActive ? 'estop-active' : ''}`}
        onClick={engageEstop}
      >
        {estopActive ? '■ 緊急停止中 (Space/Esc)' : '⚠ 緊急停止 (Space/Esc)'}
      </button>

      {/* ── 警告帯 (常時表示ゾーン。タブに関係なく必ず見える) ───── */}
      {!connected && (
        <div className="alert-bar alert-disconnect">
          <b>⚠ 通信途絶</b> — ロボットと接続できていません。操作は届きません
        </div>
      )}

      {isFault && (
        <div className="alert-bar alert-fault">
          <b>⚠ フォルト:</b> {fault.fault_type} — {fault.description}
        </div>
      )}

      {/* ── 直近の操作エラー (呼び寄せ/地図作成の失敗理由) ── */}
      {actionError && (
        <div className="alert-bar alert-action">
          <span><b>⚠ 操作失敗:</b> {actionError}</span>
          <button className="action-error-close" onClick={clearActionError} aria-label="閉じる">×</button>
        </div>
      )}

      {/* ── タブ (運用フェーズごと。VISION.md §6.1) ────────────── */}
      <nav className="tabs" role="tablist">
        {TABS.map(t => (
          <button
            key={t.id}
            role="tab"
            aria-selected={tab === t.id}
            className={`tab ${tab === t.id ? 'active' : ''}`}
            onClick={() => setTab(t.id)}
          >
            {t.label}
          </button>
        ))}
      </nav>

      {/* ── タブ本体 (ここだけがスクロールする) ─────────────── */}
      <div className="tabpanel" role="tabpanel">

        {/* ══ 運用タブ: 走行中に使うものだけ ══════════════════ */}
        {tab === 'operate' && (
          <div className="grid grid-operate">

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

            {/* ── モード操作 ───────────────────────────────
                当面の運用は軌跡追従 / 手動 / 待機 / 呼び寄せの 4 つで成立する
                (VISION.md §8)。地図あり追従は優先度が低いため副扱いにして、
                同格ボタンが 5 つ横並びになる状態を解消する */}
            <section className="card">
              <h2>モード操作</h2>
              <div className="btn-row">
                <button
                  className="mode-btn primary"
                  disabled={mode === MODE.FOLLOWING_MAPLESS || !connected}
                  onClick={() => requestMode(MODE.FOLLOWING_MAPLESS)}
                >
                  追従開始
                </button>
                <button
                  className="mode-btn primary"
                  disabled={mode === MODE.MANUAL || !connected}
                  onClick={() => requestMode(MODE.MANUAL)}
                >
                  手動操作
                </button>
                <button
                  className="mode-btn primary idle"
                  disabled={mode === MODE.IDLE || !connected}
                  onClick={() => requestMode(MODE.IDLE)}
                >
                  待機 (IDLE)
                </button>
                <button
                  className="mode-btn primary"
                  disabled={mode !== MODE.IDLE || !connected}
                  onClick={summonRobot}
                >
                  呼び寄せ
                </button>
                <button
                  className="mode-btn secondary"
                  disabled={mode === MODE.FOLLOWING || !connected}
                  onClick={() => requestMode(MODE.FOLLOWING)}
                  title="地図あり追従 (実験用・優先度低)"
                >
                  地図あり追従
                  <span className="note"> 実験用</span>
                </button>
              </div>
            </section>

            {/* ── 手動ジョグ (押している間だけ動く) ───────── */}
            <section className={`card card-manual ${mode !== MODE.MANUAL ? 'disabled' : ''}`}>
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

          </div>
        )}

        {/* ══ 準備タブ: 試験場到着時のセットアップ ══════════════ */}
        {tab === 'setup' && (
          <div className="grid">

            <section className="card">
              <h2>
                地図作成{' '}
                <span className={`target-state ${mappingActive ? 'ok' : 'ng'}`}>
                  {mappingActive ? '作成中' : '停止中'}
                </span>
              </h2>
              <p className="note">
                起動直後は停止状態。試験場に着いたら開始し、必要な範囲を走行後に停止する
                （IDLE / MANUAL 中のみ操作可）。
              </p>
              <div className="btn-row">
                <button
                  className="mode-btn primary"
                  disabled={!connected || (mode !== MODE.IDLE && mode !== MODE.MANUAL)}
                  onClick={toggleMapping}
                >
                  {mappingActive ? '地図作成停止' : '地図作成開始'}
                </button>
              </div>
            </section>

            <section className="card">
              <h2>地図</h2>
              <MapView mapData={mapData} robotPose={robotPose} scanData={scanData} pathData={pathData} />
            </section>

          </div>
        )}

        {/* ══ 診断タブ: 不調時の切り分け ═══════════════════════ */}
        {tab === 'diag' && (
          <div className="grid">

            {/* 左右輪 指令vs実測。PID 追従状況の確認用 */}
            <section className="card">
              <h2>車輪速度 <span className="note">(指令=破線 / 実測=実線, 直近15秒)</span></h2>
              <WheelSpeedView wheelSpeedData={wheelSpeedData} />
            </section>

            <section className="card">
              <h2>接続・フォルト</h2>
              <dl className="diag-list">
                <dt>rosbridge</dt>
                <dd className={connected ? 'ok' : 'ng'}>{connected ? '接続中' : '切断'}</dd>
                <dt>モード</dt>
                <dd>{modeName}</dd>
                <dt>フォルト</dt>
                <dd className={isFault ? 'ng' : 'ok'}>
                  {isFault ? `${fault.fault_type} — ${fault.description}` : 'なし'}
                </dd>
                <dt>追跡</dt>
                <dd className={isTrackedNow ? 'ok' : 'ng'}>
                  {isTrackedNow
                    ? `追跡中 (${personStatus.position.x.toFixed(2)}, ${personStatus.position.y.toFixed(2)}) conf ${personStatus.confidence?.toFixed(2) ?? '—'}`
                    : `未追跡${personStatus?.lost_reason ? ` (${personStatus.lost_reason})` : ''}`}
                </dd>
                <dt>候補数</dt>
                <dd>{candidates.length}</dd>
              </dl>
            </section>

            {/* トグルが ON なのに鳴らない、という他に症状の出ない故障が
                あるため AudioContext の状態とキューを必ず出す */}
            <section className="card">
              <h2>
                音声
                <span className={`voice-audio-state ${voice.audioReady ? 'ok' : 'ng'}`}>
                  {voice.audioReady ? '有効' : '未許可 — 画面をタップで有効化'}
                </span>
              </h2>
              <div className="voice-now">
                {voice.snapshot.playingId
                  ? `再生中: ${voice.snapshot.playingId}`
                  : '再生中: —'}
                {voice.snapshot.queueIds.length > 0 && `  (待機 ${voice.snapshot.queueIds.length})`}
              </div>
            </section>

          </div>
        )}

      </div>

      {/* ── 緊急停止の解除 ───────────────────────────────────
          どのタブからでも到達できる必要があるためタブの外に置く。発動ボタン
          (画面最上部) から最も遠い画面最下部に、色も形も変えて配置している。
          連打で発動 → 即解除、を起こさないための距離である */}
      {estopActive && (
        <div className="estop-release-bar">
          <span className="estop-release-note">
            安全を確認してから解除してください（キーボードからは解除できません）
          </span>
          <button className="estop-release-btn" onClick={releaseEstop}>
            ✓ 安全確認済み — 緊急停止を解除する
          </button>
        </div>
      )}

      {/* ── フッター (常時表示。音声レイヤ + クレジット) ─────────
          VOICEVOX Nemo のクレジットは利用規約で必須 (VISION.md §7.5 /
          docs/voice-credits.md)。タブの中に入れると非表示になり得るため
          常時表示ゾーンに置いている。消さないこと */}
      <footer className="footer">
        <div className="footer-voice">
          <button
            className={`voice-toggle ${voice.safetyOn ? 'active' : ''}`}
            onClick={() => voice.toggleLayer(LAYER.SAFETY)}
          >
            ♪ 安全通知
          </button>
          <button
            className={`voice-toggle ${voice.demoOn ? 'active' : ''}`}
            onClick={() => voice.toggleLayer(LAYER.DEMO)}
          >
            ♪ デモ実況
          </button>
        </div>
        {!voice.audioReady && (
          <span className="footer-audio-warn">音声未許可 — 画面をタップ</span>
        )}
        <span className="voice-credit">音声: VOICEVOX Nemo</span>
      </footer>

    </div>
  )
}
