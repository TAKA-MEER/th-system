// ============================================================
// SettingsPanel.jsx — WebUI 設定パネル（パラメータ調整）
//
// 対象: follow_planner_mapless（数値パラメータ全数）と
//       lidar_filter.blind_angle_ranges（VISION.md §6 参照）。
// 変更は IDLE / MANUAL モード中のみ許可する。UI 側で入力を disabled に
// するだけでなく、実際の適用・保存は config_manager ノードがモードを
// 再確認して拒否する（サーバー側の安全ガード。二重チェック）。
// ============================================================
import { useEffect, useState, useCallback } from 'react'

// follow_planner_mapless: ラベル・単位・入力レンジ (th_config_manager の
// tunable_targets.py の params と揃えること)
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
  // update_rate_hz は対象外: 制御ループの周期は起動時に固定されておりライブ変更が
  // 効かない上、内部計算の除数のため 0 を送るとノードがクラッシュする
  // (th_config_manager/tunable_targets.py 参照)
]

const BLIND_LABELS = ['右前 開始', '右前 終了', '右後 開始', '右後 終了', '左後 開始', '左後 終了', '左前 開始', '左前 終了']

function NumberField({ label, unit, value, min, max, step, disabled, onCommit }) {
  const [local, setLocal] = useState(value ?? '')
  useEffect(() => { setLocal(value ?? '') }, [value])
  return (
    <label className="settings-field">
      <span className="settings-field-label">{label}{unit && <span className="settings-field-unit"> ({unit})</span>}</span>
      <input
        type="number"
        min={min} max={max} step={step}
        value={local}
        disabled={disabled}
        onChange={(e) => setLocal(e.target.value)}
        onBlur={() => {
          const n = Number(local)
          // ブラウザは type=number の min/max を入力時に強制しないため、
          // ここで明示的に範囲内へ丸めてから送る（範囲外の値が制御ループの
          // 計算式でゼロ除算・定義域エラーを起こすのを防ぐ）
          const clamped = Number.isFinite(n) ? Math.min(max, Math.max(min, n)) : value
          setLocal(clamped ?? '')
          if (Number.isFinite(clamped) && clamped !== value) onCommit(clamped)
        }}
      />
    </label>
  )
}

export default function SettingsPanel({ open, onClose, editable, getTunableParams, applyTunableParam, saveTunableParams }) {
  const [mapless, setMapless] = useState({})
  const [blindRanges, setBlindRanges] = useState(null)
  const [status, setStatus] = useState({ mapless: '', lidar_filter: '' })
  const [loading, setLoading] = useState(false)

  const reload = useCallback(() => {
    setLoading(true)
    Promise.all([
      getTunableParams('follow_planner_mapless', MAPLESS_FIELDS.map(f => f.name)),
      getTunableParams('lidar_filter', ['blind_angle_ranges']),
    ]).then(([maplessVals, lidarVals]) => {
      setMapless(maplessVals)
      setBlindRanges(lidarVals.blind_angle_ranges ?? [])
    }).catch((e) => {
      console.error('パラメータ取得失敗:', e)
    }).finally(() => setLoading(false))
  }, [getTunableParams])

  useEffect(() => { if (open) reload() }, [open, reload])

  if (!open) return null

  const applyMapless = (name, isInt) => (value) => {
    setMapless((prev) => ({ ...prev, [name]: value }))
    applyTunableParam('follow_planner_mapless', name, value, { isInt })
      .catch((e) => console.error('適用失敗:', e))
  }

  const applyBlindRange = (index) => (value) => {
    const next = blindRanges.slice()
    next[index] = value
    setBlindRanges(next)
    applyTunableParam('lidar_filter', 'blind_angle_ranges', next, { isArray: true })
      .catch((e) => console.error('適用失敗:', e))
  }

  const save = (nodeName) => {
    setStatus((s) => ({ ...s, [nodeName]: '保存中…' }))
    saveTunableParams(nodeName)
      .then((res) => setStatus((s) => ({ ...s, [nodeName]: res.success ? '保存しました' : `失敗: ${res.message}` })))
      .catch((e) => setStatus((s) => ({ ...s, [nodeName]: `失敗: ${e.message ?? e}` })))
  }

  return (
    <div className="settings-overlay" onClick={onClose}>
      <div className="settings-panel" onClick={(e) => e.stopPropagation()}>
        <div className="settings-header">
          <h2>設定・パラメータ調整</h2>
          <button className="settings-close" onClick={onClose}>✕</button>
        </div>

        {!editable && (
          <p className="settings-guard">
            IDLE または MANUAL モード中のみ変更できます（現在は変更不可）
          </p>
        )}

        <section className="settings-section">
          <div className="settings-section-header">
            <h3>軌跡追従（follow_planner_mapless）</h3>
            <button
              className="settings-save-btn"
              disabled={!editable || loading}
              onClick={() => save('follow_planner_mapless')}
            >
              YAML に保存
            </button>
          </div>
          {status.follow_planner_mapless && <p className="settings-status">{status.follow_planner_mapless}</p>}
          <div className="settings-grid">
            {MAPLESS_FIELDS.map((f) => (
              <NumberField
                key={f.name}
                label={f.label}
                unit={f.unit}
                min={f.min} max={f.max} step={f.step}
                value={mapless[f.name]}
                disabled={!editable || loading}
                onCommit={applyMapless(f.name, f.isInt)}
              />
            ))}
          </div>
        </section>

        <section className="settings-section">
          <div className="settings-section-header">
            <h3>LiDAR 死角マスク（lidar_filter）</h3>
            <button
              className="settings-save-btn"
              disabled={!editable || loading}
              onClick={() => save('lidar_filter')}
            >
              YAML に保存
            </button>
          </div>
          {status.lidar_filter && <p className="settings-status">{status.lidar_filter}</p>}
          <div className="settings-grid">
            {BLIND_LABELS.map((label, i) => (
              <NumberField
                key={i}
                label={label}
                unit="deg"
                min={0} max={360} step={1}
                value={blindRanges?.[i]}
                disabled={!editable || loading || !blindRanges}
                onCommit={applyBlindRange(i)}
              />
            ))}
          </div>
        </section>
      </div>
    </div>
  )
}
