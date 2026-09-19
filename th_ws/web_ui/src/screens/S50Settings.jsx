// screens/S50Settings.jsx — S-50 設定（SCREEN_NAMES.S50; WS-9X）。
// Spec-webui.md §3.15 / docs/plan/spec/mockup/index.html #s50。
//
// S-01 の「保守・設定」カードから開く IDLE のサブ画面。FSM のモードではない
// （main.jsx の Screens() が settingsOpen を持ち、resolveScreen が S-01 のときだけ
// S-50 を返す。動作系モードに入ると自動で閉じる）。
//
// タブ:
//   一般     … 実配線のあるパラメータ調整（follow_planner_mapless / lidar_filter /
//              slam_toolbox）。旧 SettingsPanel.jsx の中身をそのまま移植。
//   表示     … 文字サイズ（localStorage、parts/fontScale.js）
//   開発モード … 開発モード ON/OFF（localStorage、parts/devMode.js。現状はヘッダ表示のみ）
//
// 変更できるのは IDLE / MANUAL のときだけ。UI で disabled にするのに加え、
// config_manager がサーバ側でモードを再確認して拒否する（二重ガード）。
import { useCallback, useEffect, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTunableParams } from '../ros/useTunableParams.js'
import { readFontScale, applyFontScale } from '../parts/fontScale.js'
import { readDevMode, setDevMode } from '../parts/devMode.js'
import {
  S50_BACK, S50_TAB_GENERAL, S50_TAB_DISPLAY, S50_TAB_DEV, S50_GUARD,
  S50_SAVE_YAML, S50_SAVING, S50_SAVED, S50_SAVE_FAILED, S50_LOAD_FAILED,
  S50_SEC_FOLLOW, S50_SEC_LIDAR, S50_SEC_SLAM, S50_SLAM_NOTE,
  S50_FONT_TITLE, S50_FONT_NORMAL, S50_FONT_LARGE, S50_FONT_XLARGE,
  S50_DEV_TITLE, S50_DEV_ENABLE, S50_DEV_DISABLE, S50_DEV_NOTE,
} from '../i18n/screens.js'

// follow_planner_mapless: th_config_manager/tunable_targets.py の params と揃える。
const MAPLESS_FIELDS = [
  { name: 'lookback_distance',             label: '軌跡追従: 遡り距離',   unit: 'm',    min: 0.1, max: 3,    step: 0.05 },
  { name: 'trail_sample_interval_m',       label: '軌跡点の記録間隔',     unit: 'm',    min: 0.01, max: 0.5, step: 0.01 },
  { name: 'trail_max_points',              label: '軌跡の最大保持点数',   unit: '点',   min: 100, max: 5000, step: 100, isInt: true },
  { name: 'stop_distance',                 label: '接近停止距離',         unit: 'm',    min: 0.3, max: 3,    step: 0.05 },
  { name: 'resume_distance',               label: '追従再開距離',         unit: 'm',    min: 0.3, max: 3,    step: 0.05 },
  { name: 'obstacle_check_distance_m',     label: '障害物検知距離',       unit: 'm',    min: 0.1, max: 2,    step: 0.05 },
  { name: 'obstacle_check_half_width_deg', label: '障害物検知角度半幅',   unit: 'deg',  min: 5,   max: 60,   step: 1 },
  { name: 'v_max',                         label: '最高速度',             unit: 'm/s',  min: 0.1, max: 1.5,  step: 0.05 },
  { name: 'k_ang',                         label: '旋回角度ゲイン',       unit: '',     min: 0.5, max: 5,    step: 0.1 },
  { name: 'stop_radius_m',                 label: 'ゴール到達半径',       unit: 'm',    min: 0.05, max: 1,   step: 0.05 },
  { name: 'w_max_rad_s',                   label: '最高旋回速度',         unit: 'rad/s', min: 0.2, max: 3,   step: 0.1 },
  { name: 'max_linear_accel_mps2',         label: '加速度上限',           unit: 'm/s²', min: 0.2, max: 3,    step: 0.1 },
  { name: 'max_linear_decel_mps2',         label: '減速度上限',           unit: 'm/s²', min: 0.2, max: 4,    step: 0.1 },
  { name: 'max_angular_accel_rad_s2',      label: '旋回加速度上限',       unit: 'rad/s²', min: 0.5, max: 8,  step: 0.1 },
]

const BLIND_LABELS = ['右前 開始', '右前 終了', '右後 開始', '右後 終了', '左後 開始', '左後 終了', '左前 開始', '左前 終了']

// slam_toolbox: 再生の自己位置推定（スキャンマッチ）。WS-9W。
const SLAM_FIELDS = [
  { name: 'minimum_travel_distance',            label: '補正間隔（距離）',       unit: 'm',     min: 0.05, max: 1.0,  step: 0.05 },
  { name: 'minimum_travel_heading',             label: '補正間隔（角度）',       unit: 'rad',   min: 0.05, max: 1.0,  step: 0.05 },
  { name: 'correlation_search_space_dimension', label: 'スキャン探索窓（全幅）', unit: 'm',     min: 0.3,  max: 2.0,  step: 0.05 },
  { name: 'correlation_search_space_resolution', label: '探索の刻み',            unit: 'm',     min: 0.005, max: 0.05, step: 0.005 },
  { name: 'link_match_minimum_response_fine',   label: 'マッチ受理の下限',       unit: '',      min: 0.05, max: 0.6,  step: 0.01 },
]

function NumberField({ label, unit, value, min, max, step, disabled, onCommit }) {
  const [local, setLocal] = useState(value ?? '')
  useEffect(() => { setLocal(value ?? '') }, [value])
  return (
    <label className="s50-field">
      <span className="s50-field-label">
        {label}{unit && <span className="mut"> ({unit})</span>}
      </span>
      <input
        type="number"
        min={min} max={max} step={step}
        value={local}
        disabled={disabled}
        onChange={(e) => setLocal(e.target.value)}
        onBlur={() => {
          const n = Number(local)
          // ブラウザは type=number の min/max を入力時に強制しないので、ここで
          // 範囲内に丸めてから送る（範囲外の値が制御ループでゼロ除算・定義域
          // エラーを起こすのを防ぐ）。
          const clamped = Number.isFinite(n) ? Math.min(max, Math.max(min, n)) : value
          setLocal(clamped ?? '')
          if (Number.isFinite(clamped) && clamped !== value) onCommit(clamped)
        }}
      />
    </label>
  )
}

function Section({ title, note, saveKey, status, editable, loading, onSave, children }) {
  return (
    <div className="card">
      <div className="row" style={{ marginBottom: 8 }}>
        <h3 className="grow" style={{ margin: 0 }}>{title}</h3>
        <button
          type="button"
          className="btn save sm"
          disabled={!editable || loading}
          onClick={onSave}
          data-testid={`s50-save-${saveKey}`}
        >
          {S50_SAVE_YAML}
        </button>
      </div>
      {status && <p className="note" data-testid={`s50-status-${saveKey}`}>{status}</p>}
      {note && <p className="note">{note}</p>}
      <div className="s50-grid">{children}</div>
    </div>
  )
}

export default function S50Settings({ onBack }) {
  const { ros, state, stale } = useSystemState()
  const { getTunableParams, applyTunableParam, saveTunableParams } = useTunableParams(ros)

  const mode = state?.mode ?? null
  const editable = !stale && (mode === 'IDLE' || mode === 'MANUAL')

  const [tab, setTab] = useState('general')

  // ── 一般タブ ──
  const [mapless, setMapless] = useState({})
  const [blindRanges, setBlindRanges] = useState(null)
  const [slam, setSlam] = useState({})
  const [status, setStatus] = useState({})
  const [loading, setLoading] = useState(false)

  const reload = useCallback(() => {
    setLoading(true)
    Promise.all([
      getTunableParams('follow_planner_mapless', MAPLESS_FIELDS.map((f) => f.name)),
      getTunableParams('lidar_filter', ['blind_angle_ranges']),
      // slam_toolbox は enable_route_slam のときだけ存在する。取れなくても
      // パネル全体を止めない。
      getTunableParams('slam_toolbox', SLAM_FIELDS.map((f) => f.name)).catch(() => ({})),
    ]).then(([maplessVals, lidarVals, slamVals]) => {
      setMapless(maplessVals)
      setBlindRanges(lidarVals.blind_angle_ranges ?? [])
      setSlam(slamVals ?? {})
    }).catch(() => {
      setStatus((s) => ({ ...s, load: S50_LOAD_FAILED }))
    }).finally(() => setLoading(false))
  }, [getTunableParams])

  useEffect(() => { reload() }, [reload])

  const applyMapless = (name, isInt) => (value) => {
    setMapless((prev) => ({ ...prev, [name]: value }))
    applyTunableParam('follow_planner_mapless', name, value, { isInt }).catch(() => {})
  }
  const applyBlindRange = (index) => (value) => {
    const next = blindRanges.slice()
    next[index] = value
    setBlindRanges(next)
    applyTunableParam('lidar_filter', 'blind_angle_ranges', next, { isArray: true }).catch(() => {})
  }
  const applySlam = (name) => (value) => {
    setSlam((prev) => ({ ...prev, [name]: value }))
    // slam_toolbox はライブ反映しない。値は保持され、YAML 保存 → 次の経路選択で効く。
    applyTunableParam('slam_toolbox', name, value).catch(() => {})
  }

  const save = (nodeName) => {
    setStatus((s) => ({ ...s, [nodeName]: S50_SAVING }))
    saveTunableParams(nodeName)
      .then((res) => setStatus((s) => ({
        ...s, [nodeName]: res.success ? S50_SAVED : `${S50_SAVE_FAILED}: ${res.message}`,
      })))
      .catch((e) => setStatus((s) => ({
        ...s, [nodeName]: `${S50_SAVE_FAILED}: ${e.message ?? e}`,
      })))
  }

  // ── 表示タブ ──
  const [fontScale, setFontScale] = useState(() => readFontScale())
  const chooseFont = (name) => {
    setFontScale(name)
    applyFontScale(name)
  }

  // ── 開発モードタブ ──
  const [dev, setDev] = useState(() => readDevMode())
  const toggleDev = () => {
    const next = !dev
    setDev(next)
    setDevMode(next)
  }

  const FONT_OPTIONS = [
    ['normal', S50_FONT_NORMAL],
    ['large', S50_FONT_LARGE],
    ['xlarge', S50_FONT_XLARGE],
  ]

  return (
    <div className="screen" id="s50">
      <div className="top-actions sticky">
        <button type="button" className="btn sm" onClick={onBack} data-testid="s50-back">
          {S50_BACK}
        </button>
        <div className="tabs grow" style={{ margin: 0, border: 'none' }} role="tablist">
          {[
            ['general', S50_TAB_GENERAL],
            ['display', S50_TAB_DISPLAY],
            ['dev', S50_TAB_DEV],
          ].map(([key, label]) => (
            <button
              key={key}
              type="button"
              role="tab"
              aria-selected={tab === key}
              className={`tab ${tab === key ? 'on' : ''}`}
              onClick={() => setTab(key)}
              data-testid={`s50-tab-${key}`}
            >
              {label}
            </button>
          ))}
        </div>
      </div>

      {!editable && tab === 'general' && (
        <p className="note" data-testid="s50-guard">{S50_GUARD}</p>
      )}

      {tab === 'general' && (
        <div className="tabpane on">
          {status.load && (
            <p className="note" data-testid="s50-load-error" style={{ gridColumn: '1 / -1' }}>
              {status.load}
            </p>
          )}
          <Section
            title={S50_SEC_FOLLOW} saveKey="follow_planner_mapless"
            status={status.follow_planner_mapless} editable={editable} loading={loading}
            onSave={() => save('follow_planner_mapless')}
          >
            {MAPLESS_FIELDS.map((f) => (
              <NumberField
                key={f.name} label={f.label} unit={f.unit}
                min={f.min} max={f.max} step={f.step}
                value={mapless[f.name]}
                disabled={!editable || loading}
                onCommit={applyMapless(f.name, f.isInt)}
              />
            ))}
          </Section>

          <Section
            title={S50_SEC_LIDAR} saveKey="lidar_filter"
            status={status.lidar_filter} editable={editable} loading={loading}
            onSave={() => save('lidar_filter')}
          >
            {BLIND_LABELS.map((label, i) => (
              <NumberField
                key={i} label={label} unit="deg"
                min={0} max={360} step={1}
                value={blindRanges?.[i]}
                disabled={!editable || loading || !blindRanges}
                onCommit={applyBlindRange(i)}
              />
            ))}
          </Section>

          <Section
            title={S50_SEC_SLAM} saveKey="slam_toolbox" note={S50_SLAM_NOTE}
            status={status.slam_toolbox} editable={editable} loading={loading}
            onSave={() => save('slam_toolbox')}
          >
            {SLAM_FIELDS.map((f) => (
              <NumberField
                key={f.name} label={f.label} unit={f.unit}
                min={f.min} max={f.max} step={f.step}
                value={slam[f.name]}
                disabled={!editable || loading || !(f.name in slam)}
                onCommit={applySlam(f.name)}
              />
            ))}
          </Section>
        </div>
      )}

      {tab === 'display' && (
        <div className="tabpane on">
          <div className="card">
            <h3>{S50_FONT_TITLE}</h3>
            <div className="btnrow n3">
              {FONT_OPTIONS.map(([key, label]) => (
                <button
                  key={key}
                  type="button"
                  className={`btn ${fontScale === key ? 'on' : ''}`}
                  onClick={() => chooseFont(key)}
                  data-testid={`s50-font-${key}`}
                >
                  {label}
                </button>
              ))}
            </div>
          </div>
        </div>
      )}

      {tab === 'dev' && (
        <div className="tabpane on">
          <div className="card">
            <h3>{S50_DEV_TITLE}</h3>
            <p className="note">{S50_DEV_NOTE}</p>
            <button
              type="button"
              className={`btn wide ${dev ? 'on' : ''}`}
              onClick={toggleDev}
              data-testid="s50-dev-toggle"
            >
              {dev ? S50_DEV_DISABLE : S50_DEV_ENABLE}
            </button>
          </div>
        </div>
      )}
    </div>
  )
}

// テスト（test_tunable_targets.py）がこの配列名を正規表現で拾う。
// 名前を変えるときは向こうも直すこと。
export { MAPLESS_FIELDS, SLAM_FIELDS, BLIND_LABELS }
