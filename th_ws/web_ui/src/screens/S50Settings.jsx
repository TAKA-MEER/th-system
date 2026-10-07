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
//   開発モード … 開発モード ON/OFF と項目別選択（ros/useDevMode.js 経由で
//              connectivity_checker のパラメータに書く。表示の正本は
//              /system/dev_mode。localStorage は見た目の即時反映だけ）
//
// 変更できるのは停止中（IDLE・PREP の地図作業中・非ジョグ）のときだけ
// （Spec.md SD-9。1b-5）。UI で disabled にするのに加え、config_manager が
// サーバ側で同じ条件を再確認して拒否する（二重ガード）。
import { useCallback, useEffect, useState } from 'react'
import { mapOrParamOpAllowed } from '../modes/stopOnlyGuard.js'
import { useSystemState } from '../ros/useSystemState.js'
import { useTunableParams } from '../ros/useTunableParams.js'
import { useParamsOverrides, pendingDiff } from '../ros/useParamsOverrides.js'
import { useTrigger } from '../ros/useTrigger.js'
import { reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import { blindRangesText } from './calibCore.js'
import { useDevMode } from '../ros/useDevMode.js'
import {
  DEV_ITEMS, devIgnoreParam, effectiveItems,
} from '../ros/devModeState.js'
import { readFontScale, applyFontScale } from '../parts/fontScale.js'
import { readDevMode, setDevMode } from '../parts/devMode.js'
import PARAM_LIMITS from '../generated/param_limits.json'
import { clampTunableParam, tunableFieldMax } from '../ros/s50TunableLimits.js'
import {
  S50_BACK, S50_TAB_GENERAL, S50_TAB_DISPLAY, S50_TAB_DEV, S50_GUARD,
  S50_SAVE_YAML, S50_SAVING, S50_SAVED, S50_SAVE_FAILED, S50_LOAD_FAILED,
  S50_SEC_FOLLOW, S50_SEC_LIDAR, S50_SEC_SLAM, S50_SLAM_NOTE,
  S50_SEC_PRESET, S50_PRESET_NOTE, S50_PRESET_SAVE,
  S50_PRESET_REASON_LABEL, S50_PRESET_REASON_NEED, S50_PRESET_NEXT_PREFIX,
  S50_BLIND_GOTO_CALIB, S50_BLIND_NOTE,
  S50_FONT_TITLE, S50_FONT_NORMAL, S50_FONT_LARGE, S50_FONT_XLARGE,
  S50_DEV_TITLE, S50_DEV_ENABLE, S50_DEV_DISABLE, S50_DEV_NOTE,
  S50_DEV_ITEMS_TITLE, S50_DEV_ITEM_LINK, S50_DEV_ITEM_LINK_DESC,
  S50_DEV_ITEM_LIDAR_FAULT, S50_DEV_ITEM_LIDAR_FAULT_DESC,
  S50_DEV_ITEM_SCAN_STOP, S50_DEV_ITEM_SCAN_STOP_DESC,
  S50_DEV_ITEM_BATTERY, S50_DEV_ITEM_OPCHECK, S50_DEV_ITEM_OPCHECK_DESC,
  S50_DEV_ITEM_AUTO_BRAKE,
  S50_DEV_NO_GATE, S50_DEV_EFFECTIVE_TITLE, S50_DEV_NONE,
  S50_DEV_ESTOP_UNKNOWN, S50_DEV_SCOPE_NOTE,
  S50_DEV_SEND_FAILED,
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

// SG-B8: 速度プリセットの割合（registry の speed_preset_*。
// /params/get で効き値を読み、/params/set で出どころ付きで保存する。
// 走行中の値は変えない＝次の起動から効く）。
const PRESET_FIELDS = [
  { name: 'speed_preset_low', label: '低速', unit: '割合', min: 0.05, max: 1, step: 0.01 },
  { name: 'speed_preset_mid', label: '中速', unit: '割合', min: 0.05, max: 1, step: 0.01 },
  { name: 'speed_preset_high', label: '高速', unit: '割合', min: 0.05, max: 1, step: 0.01 },
]
const PRESET_NAMES = PRESET_FIELDS.map((f) => f.name)

// SG-B9: 廃止予定ノードの項目のうち上限が registry と食い違うものは、
// registry の値を超えて保存できないようにする（対象: mapless の v_max。
// 上限値自体は scripts/gen_param_limits.py が registry.yaml から作る）。
const REGISTRY_CAPS = { follow_planner_mapless: { v_max: PARAM_LIMITS.v_max } }

// 一般タブの LiDAR 死角は読み取り専用（2026-10-02 Spec-webui.md §3.15）。
// 変更は校正 S-40 の BLIND 経路だけにするため、編集欄も「YAML に保存」も置かない。
// 値の読み出しは lidar_filter の get_parameters を直接叩く従来の経路のまま
// （表示は config_manager の調整対象でなくても読める）。

// 開発モードの項目メタ（WP-DEV-01B §2）。キーは devModeState.DEV_ITEMS と揃える。
// battery・auto_brake 以外は効く。opcheck は起動時の自動点検の警告を消す
// （判定自体は止めない）。battery・auto_brake は as-built に止める側の
// 仕組みが無く、選んでも何も起きない（DEV_NO_GATE_ITEMS。カード末尾の注記で明示する）。
const DEV_ITEM_META = [
  { item: 'link', label: S50_DEV_ITEM_LINK, desc: S50_DEV_ITEM_LINK_DESC },
  { item: 'lidar_fault', label: S50_DEV_ITEM_LIDAR_FAULT, desc: S50_DEV_ITEM_LIDAR_FAULT_DESC },
  { item: 'scan_stop', label: S50_DEV_ITEM_SCAN_STOP, desc: S50_DEV_ITEM_SCAN_STOP_DESC },
  { item: 'battery', label: S50_DEV_ITEM_BATTERY, desc: '' },
  { item: 'opcheck', label: S50_DEV_ITEM_OPCHECK, desc: S50_DEV_ITEM_OPCHECK_DESC },
  { item: 'auto_brake', label: S50_DEV_ITEM_AUTO_BRAKE, desc: '' },
]
const DEV_ITEM_LABEL = Object.fromEntries(DEV_ITEM_META.map(({ item, label }) => [item, label]))

// いま無視しているものの表示文（/system/dev_mode の effective から作る）。
// 未受信（devState null）のときは通常運用として扱う。
function devEffectiveText(devState) {
  if (!devState?.dev_mode) return S50_DEV_NONE
  const items = effectiveItems(devState)
  if (items.length === 0) return S50_DEV_NONE
  return items.map((item) => DEV_ITEM_LABEL[item] ?? item).join(' / ')
}

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

// initialTab: S-00（疎通確認）から開いたときは 'dev'。INIT では一般タブの
// 読み込み（config_manager）が通らないため、開発モードタブから始める。
export default function S50Settings({ onBack, initialTab = 'general' }) {
  const { ros, state, stale } = useSystemState()
  const { getTunableParams, applyTunableParam, saveTunableParams } = useTunableParams(ros)
  // SG-B8: 速度プリセット区画（/params/get・/params/set。次の起動から効く）。
  const { getParams, setParams } = useParamsOverrides(ros)
  // S-40（校正）への導線。FSM が真実なので画面側で遷移はしない — 受理されれば
  // /system/state が CALIB になり、main.jsx が S-50 を畳んで S-40 を出す。
  // 拒否されたら理由をその場に出す（S-01 の理由ウィンドウと同型。S-30 の
  // 発射しっぱなしにはしない）。
  const sendTrigger = useTrigger()
  const [calibErr, setCalibErr] = useState('')

  // 1b-5: サーバ側（config_manager）と同じ停止中条件に揃える。MANUAL は
  // 拒否されるようになったため外す（以前は IDLE/MANUAL だった）。
  const editable = mapOrParamOpAllowed(state, stale)

  const [tab, setTab] = useState(initialTab)

  // ── 一般タブ ──
  const [mapless, setMapless] = useState({})
  const [blindRanges, setBlindRanges] = useState(null)
  const [slam, setSlam] = useState({})
  const [status, setStatus] = useState({})
  const [loading, setLoading] = useState(false)
  // ── SG-B8: 速度プリセット区画 ──
  // presetEff＝いま効いている値、presetPending＝保存済みで次回から効く上書き、
  // presetDraft＝編集中。保存は /params/set（出どころ付き）で一括し、
  // 走行中の値には触れない（次の起動から効く）。
  const [presetEff, setPresetEff] = useState({})
  const [presetPending, setPresetPending] = useState({})
  const [presetDraft, setPresetDraft] = useState({})
  const [presetReason, setPresetReason] = useState('')

  const reload = useCallback(() => {
    setLoading(true)
    // SG-B9: 起動していないノードの失敗で全体を落とさない。節ごとに成否を
    // 扱い、取れたぶんは出す（本番の bringup では use_stub なしだと
    // follow_planner_mapless が起動しておらず、その取得だけ失敗する）。
    Promise.allSettled([
      getTunableParams('follow_planner_mapless', MAPLESS_FIELDS.map((f) => f.name)),
      getTunableParams('lidar_filter', ['blind_angle_ranges']),
      getTunableParams('slam_toolbox', SLAM_FIELDS.map((f) => f.name)),
    ]).then(([maplessRes, lidarRes, slamRes]) => {
      const loadErr = {}
      if (maplessRes.status === 'fulfilled') {
        setMapless(maplessRes.value)
      } else {
        setMapless({})
        loadErr.follow_planner_mapless = S50_LOAD_FAILED
      }
      if (lidarRes.status === 'fulfilled') {
        setBlindRanges(lidarRes.value.blind_angle_ranges ?? [])
      } else {
        // 読み取り専用表示は従来どおり「取得できませんでした」になる
        //（s50-blind-current。節の注記は出さない）。
        setBlindRanges(null)
      }
      if (slamRes.status === 'fulfilled') {
        setSlam(slamRes.value ?? {})
      } else {
        setSlam({})
        loadErr.slam_toolbox = S50_LOAD_FAILED
      }
      if (Object.keys(loadErr).length > 0) {
        setStatus((s) => ({ ...s, ...loadErr }))
      }
    }).finally(() => setLoading(false))
    // SG-B8: 速度プリセットは /params/get で効き値と保存済み上書きを区別して読む。
    // 取れなくても他区画は出す（節ごとの成否。各区画の保存ボタンだけ無効化）。
    getParams(PRESET_NAMES).then((payload) => {
      setPresetEff(payload?.effective ?? {})
      setPresetPending(payload?.pending ?? {})
      // 編集の出発点は「次に効く値」（保存済みがあればそれ、無ければ効き値）。
      setPresetDraft({ ...(payload?.effective ?? {}), ...(payload?.pending ?? {}) })
    }).catch(() => {
      setStatus((s) => ({ ...s, preset: S50_LOAD_FAILED }))
    })
  }, [getTunableParams, getParams])

  useEffect(() => { reload() }, [reload])

  const applyMapless = (name, isInt) => (value) => {
    // SG-B9: registry の上限を超えて送らない（欄の max と二重の網。
    // 保存はこの生値を YAML に書くので、ここで丸めた値が保存される値になる）。
    const capped = clampTunableParam(REGISTRY_CAPS, 'follow_planner_mapless', name, value)
    setMapless((prev) => ({ ...prev, [name]: capped }))
    applyTunableParam('follow_planner_mapless', name, capped, { isInt }).catch(() => {})
  }
  const gotoCalib = () => {
    setCalibErr('')
    sendTrigger('ui.enter_mode', { mode: 'CALIB' }).then((res) => {
      if (!res?.accepted) setCalibErr(reasonLabel(res?.reject_reason_key) ?? UNKNOWN_REASON_LABEL)
    }).catch(() => setCalibErr(UNKNOWN_REASON_LABEL))
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

  // SG-B8: 速度プリセットの保存（/params/set。出どころ必須＝PT-4）。
  // 即時の set_parameters はしない（走行中の値は変えない。次の起動から効く）。
  const applyPreset = (name) => (value) => {
    setPresetDraft((prev) => ({ ...prev, [name]: value }))
  }
  const savePreset = () => {
    if (!presetReason) {
      setStatus((s) => ({ ...s, preset: S50_PRESET_REASON_NEED }))
      return
    }
    setStatus((s) => ({ ...s, preset: S50_SAVING }))
    // 効き値から変わったものだけ送る（触っていない行に触らない）。
    const values = {}
    for (const name of PRESET_NAMES) {
      if (presetDraft[name] !== presetEff[name]) values[name] = presetDraft[name]
    }
    if (Object.keys(values).length === 0) {
      setStatus((s) => ({ ...s, preset: S50_SAVED }))
      return
    }
    setParams(values, 'web_ui', presetReason)
      .then((res) => {
        if (res?.success) {
          setStatus((s) => ({ ...s, preset: S50_SAVED }))
          reload()
        } else {
          setStatus((s) => ({ ...s, preset: `${S50_SAVE_FAILED}: ${res?.message ?? ''}` }))
        }
      })
      .catch((e) => setStatus((s) => ({
        ...s, preset: `${S50_SAVE_FAILED}: ${e.message ?? e}`,
      })))
  }
  const presetDiffs = pendingDiff(PRESET_NAMES, presetEff, presetPending)

  // ── 表示タブ ──
  const [fontScale, setFontScale] = useState(() => readFontScale())
  const chooseFont = (name) => {
    setFontScale(name)
    applyFontScale(name)
  }

  // ── 開発モードタブ (WP-DEV-01B) ──
  // 表示の正本は /system/dev_mode（useDevMode）。localStorage は押した瞬間の
  // 見目だけに残す（?dev=1 は据え置き）。トグルはどのモードでも押せる
  // （INIT を抜けるために要るので、一般タブのような停止中の縛りは無い）。
  const { dev: devState, setDevParam } = useDevMode(ros)
  const [devLocal, setDevLocal] = useState(() => readDevMode())
  // 既定は全項目 OFF（connectivity_checker の dev_ignore_* 既定と同じ。
  // 開発モードに入っただけでは通常運用と同じ。Spec-safety.md §10）。
  const [devItemsLocal, setDevItemsLocal] = useState(
    () => Object.fromEntries(DEV_ITEMS.map((item) => [item, false])))
  const [devStatus, setDevStatus] = useState('')
  const devShown = devState ? devState.dev_mode : devLocal
  const devItemsShown = devState ? devState.ignore : devItemsLocal
  const toggleDev = () => {
    const next = !devShown
    setDevLocal(next)
    setDevMode(next)
    setDevStatus('')
    setDevParam('dev_mode', next).catch(() => setDevStatus(S50_DEV_SEND_FAILED))
  }
  const toggleDevItem = (item) => {
    const next = !devItemsShown[item]
    setDevItemsLocal((prev) => ({ ...prev, [item]: next }))
    setDevStatus('')
    setDevParam(devIgnoreParam(item), next).catch(() => setDevStatus(S50_DEV_SEND_FAILED))
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
          <Section
            title={S50_SEC_FOLLOW} saveKey="follow_planner_mapless"
            status={status.follow_planner_mapless} editable={editable} loading={loading}
            onSave={() => save('follow_planner_mapless')}
          >
            {MAPLESS_FIELDS.map((f) => (
              <NumberField
                key={f.name} label={f.label} unit={f.unit}
                min={f.min}
                max={tunableFieldMax(REGISTRY_CAPS, 'follow_planner_mapless', f.name, f.max)}
                step={f.step}
                value={mapless[f.name]}
                disabled={!editable || loading}
                onCommit={applyMapless(f.name, f.isInt)}
              />
            ))}
          </Section>

          <div className="card">
            <div className="row" style={{ marginBottom: 8 }}>
              <h3 className="grow" style={{ margin: 0 }}>{S50_SEC_LIDAR}</h3>
              <button
                type="button"
                className="btn sm"
                onClick={gotoCalib}
                data-testid="s50-goto-calib"
              >
                {S50_BLIND_GOTO_CALIB}
              </button>
            </div>
            <p className="note">{S50_BLIND_NOTE}</p>
            <p data-testid="s50-blind-current">
              {blindRanges == null ? (loading ? '' : S50_LOAD_FAILED) : blindRangesText(blindRanges)}
            </p>
            {calibErr && <p className="note" data-testid="s50-calib-err">{calibErr}</p>}
          </div>

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

          <div className="card">
            <div className="row" style={{ marginBottom: 8 }}>
              <h3 className="grow" style={{ margin: 0 }}>{S50_SEC_PRESET}</h3>
              <button
                type="button"
                className="btn save sm"
                disabled={!editable || loading}
                onClick={savePreset}
                data-testid="s50-save-preset"
              >
                {S50_PRESET_SAVE}
              </button>
            </div>
            {status.preset && <p className="note" data-testid="s50-status-preset">{status.preset}</p>}
            <p className="note">{S50_PRESET_NOTE}</p>
            {presetDiffs.length > 0 && (
              <p className="note" data-testid="s50-preset-pending">
                {presetDiffs.map(({ name, effective, pending }) => (
                  `${S50_PRESET_NEXT_PREFIX} ${name}: ${effective} → ${pending}`
                )).join(' ／ ')}
              </p>
            )}
            <div className="s50-grid">
              {PRESET_FIELDS.map((f) => (
                <NumberField
                  key={f.name} label={f.label} unit={f.unit}
                  min={f.min} max={f.max} step={f.step}
                  value={presetDraft[f.name]}
                  disabled={!editable || loading || !(f.name in presetDraft)}
                  onCommit={applyPreset(f.name)}
                />
              ))}
            </div>
            <label className="s50-field">
              <span className="s50-field-label">{S50_PRESET_REASON_LABEL}</span>
              <input
                type="text"
                value={presetReason}
                disabled={!editable || loading}
                onChange={(e) => setPresetReason(e.target.value)}
                data-testid="s50-preset-reason"
              />
            </label>
          </div>
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
              className={`btn wide ${devShown ? 'on' : ''}`}
              onClick={toggleDev}
              data-testid="s50-dev-toggle"
            >
              {devShown ? S50_DEV_DISABLE : S50_DEV_ENABLE}
            </button>
            {devStatus && <p className="note" data-testid="s50-dev-status">{devStatus}</p>}
          </div>
          <div className="card">
            <h3>{S50_DEV_ITEMS_TITLE}</h3>
            <div className="s50-grid">
              {DEV_ITEM_META.map(({ item, label, desc }) => (
                <div key={item}>
                  <button
                    type="button"
                    className={`btn wide ${devItemsShown[item] ? 'on' : ''}`}
                    aria-pressed={!!devItemsShown[item]}
                    onClick={() => toggleDevItem(item)}
                    data-testid={`s50-dev-ignore-${item}`}
                  >
                    {label}
                  </button>
                  {desc && <p className="note">{desc}</p>}
                </div>
              ))}
            </div>
            <p className="note" data-testid="s50-dev-no-gate">{S50_DEV_NO_GATE}</p>
          </div>
          <div className="card">
            <h3>{S50_DEV_EFFECTIVE_TITLE}</h3>
            <p data-testid="s50-dev-effective">{devEffectiveText(devState)}</p>
            {devState?.dev_mode && !devState.estop_hw_known && (
              <p className="note" data-testid="s50-dev-estop-unknown">{S50_DEV_ESTOP_UNKNOWN}</p>
            )}
          </div>
          <p className="note" data-testid="s50-dev-scope">{S50_DEV_SCOPE_NOTE}</p>
        </div>
      )}
    </div>
  )
}

// テスト（test_tunable_targets.py）が MAPLESS_FIELDS / SLAM_FIELDS の配列名を
// 正規表現で拾う。名前を変えるときは向こうも直すこと。
// BLIND_LABELS は廃止（2026-10-02。死角の編集欄を S-50 から外した）。
export { MAPLESS_FIELDS, SLAM_FIELDS }
