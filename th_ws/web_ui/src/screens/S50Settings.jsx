// screens/S50Settings.jsx — S-50 設定（SCREEN_NAMES.S50; WS-9X）。
// Spec-webui.md §3.15（型: 見る・変える。タブは無い）。左に区分 8 つ、右に中身。
// 右の下端の右に「保存」と「いつ効くかの印」（すぐ／次の起動から／
// 次に地図を読み直したときから）。効き方の違う設定は区分を分ける。
// 注記の文は置かない（§0 原則 7）。
//
// S-01 の「設定」タイルから開く IDLE のサブ画面。FSM のモードではない
// （main.jsx の Screens() が settingsOpen を持ち、resolveScreen が S-01 のときだけ
// S-50 を返す。動作系モードに入ると自動で閉じる）。
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
import { useFaultHistory } from '../ros/useFaultHistory.js'
import { useSpeaker } from '../ros/useSpeaker.js'
import FaultHistoryList from '../parts/FaultHistoryList.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import { ViewLayout } from '../shell/Layouts.jsx'
import {
  DEV_ITEMS, devIgnoreParam, effectiveItems,
} from '../ros/devModeState.js'
import { readFontScale, applyFontScale } from '../parts/fontScale.js'
import { readVoiceMaster, applyVoiceMaster } from '../parts/voiceMaster.js'
import {
  readStageOrientation, writeStageOrientation,
  ORIENTATION_AUTO, ORIENTATION_LANDSCAPE, ORIENTATION_PORTRAIT,
} from '../parts/stageOrientation.js'
import { readHelpEnabled, writeHelpEnabled } from '../parts/helpPrefs.js'
import { readDevMode, setDevMode } from '../parts/devMode.js'
import { clampTunableParam } from '../ros/s50TunableLimits.js'
import PARAM_LIMITS from '../generated/param_limits.json'
import { presetOrderError } from '../ros/presetOrder.js'
import {
  S50_BACK, S50_GUARD,
  S50_SAVE_YAML, S50_SAVING, S50_SAVED, S50_SAVE_FAILED, S50_LOAD_FAILED,
  S50_SEC_SLAM, S50_SEC_PRESET, S50_PRESET_SAVE,
  S50_PRESET_REASON_LABEL, S50_PRESET_REASON_NEED, S50_PRESET_NEXT_PREFIX,
  S50_SEC_SPEAKER, S50_SPEAKER_ON, S50_SPEAKER_OFF, S50_SPEAKER_VOLUME, S50_SPEAKER_TEST,
  S50_SPEAKER_TEST_SENT, S50_SPEAKER_SEND_FAILED, S50_SPEAKER_NO_REPLY,
  S50_SPEAKER_PLAYING,
  S50_PRESET_ORDER_REASONS,
  S50_SEC_BRAKE, S50_BRAKE_OUTDOOR, S50_BRAKE_INDOOR,
  S50_BRAKE_OFF, S50_BRAKE_ON,
  S50_SEC_CAMERA, S50_CAMERA_URL_LABEL, S50_CAMERA_BAD_URL,
  S50_BLIND_GOTO_CALIB,
  S50_FONT_TITLE, S50_FONT_NORMAL, S50_FONT_LARGE, S50_FONT_XLARGE,
  S50_VOICE_TITLE, S50_VOICE_ON, S50_VOICE_OFF,
  S50_ORIENTATION_TITLE, S50_ORIENTATION_AUTO, S50_ORIENTATION_PORTRAIT,
  S50_ORIENTATION_LANDSCAPE,
  S50_HELP_TITLE, S50_HELP_SHOW, S50_HELP_HIDE,
  S50_DEV_TITLE, S50_DEV_ENABLE, S50_DEV_DISABLE,
  S50_DEV_ITEMS_TITLE, S50_DEV_ITEM_LINK, S50_DEV_ITEM_LIDAR_FAULT,
  S50_DEV_ITEM_SCAN_STOP, S50_DEV_ITEM_BATTERY, S50_DEV_ITEM_OPCHECK,
  S50_DEV_ITEM_AUTO_BRAKE,
  S50_DEV_NO_GATE, S50_DEV_EFFECTIVE_TITLE, S50_DEV_NONE,
  S50_DEV_SCOPE_NOTE,
  S50_DEV_SEND_FAILED,
  S50_SEC_RUN, S50_SEC_LOC, S50_SEC_NOTIFY, S50_SEC_SCREEN,
  S50_SEC_LINK, S50_SEC_LOG, S50_SEC_BLIND, S50_SEC_DEV,
  S50_WHEN_NEXT_BOOT, S50_WHEN_MAP_RELOAD, S50_WHEN_NOW,
} from '../i18n/screens.js'

// follow_planner_mapless の項目。Spec-webui §3.15 に区分が無いため画面には
// 出さない（th_testing/test/test_tunable_targets.py が互換のために参照）。
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

// SG-C14: 自動ブレーキの既定（registry の auto_brake_default_*。
// /params/get で効き値を読み、/params/set で出どころ付きで保存する。
// 走行中の値は変えない＝次の起動から効く。state_manager が起動時に読む）。
const BRAKE_FIELDS = [
  { name: 'auto_brake_default_outdoor', label: S50_BRAKE_OUTDOOR },
  { name: 'auto_brake_default_indoor', label: S50_BRAKE_INDOOR },
]
const BRAKE_NAMES = BRAKE_FIELDS.map((f) => f.name)
const BRAKE_OPTIONS = [
  { value: 'off', label: S50_BRAKE_OFF },
  { value: 'on', label: S50_BRAKE_ON },
]

// S-21 の「カメラ」ボタンの開く先（Spec-onsite.md §7.2）。文字列で1つだけ。
// /params/get で効き値を読み、/params/set で出どころ付きで保存する
// （走行中の値は変えない。次の起動から効く。機体側も http(s) 以外を弾く）。
const CAMERA_NAMES = ['camera_ui_url']
// 画面側の検査（機体側 params_audit と同じ規則。保存前に理由を出せる）。
const isValidCameraUrl = (value) =>
  typeof value === 'string' && (value.startsWith('http://') || value.startsWith('https://'))

// SG-B9: 廃止予定ノードの項目のうち上限が registry と食い違うものは、
// registry の値を超えて保存できないようにする（対象: mapless の v_max。
// 上限値自体は scripts/gen_param_limits.py が registry.yaml から作る）。
const REGISTRY_CAPS = { follow_planner_mapless: { v_max: PARAM_LIMITS.v_max } }

// 開発モードの項目メタ。キーは devModeState.DEV_ITEMS と揃える。
const DEV_ITEM_META = [
  { item: 'link', label: S50_DEV_ITEM_LINK },
  { item: 'lidar_fault', label: S50_DEV_ITEM_LIDAR_FAULT },
  { item: 'scan_stop', label: S50_DEV_ITEM_SCAN_STOP },
  { item: 'battery', label: S50_DEV_ITEM_BATTERY },
  { item: 'opcheck', label: S50_DEV_ITEM_OPCHECK },
  { item: 'auto_brake', label: S50_DEV_ITEM_AUTO_BRAKE },
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

const SECTIONS = [
  ['run', S50_SEC_RUN, 's50-run'],
  ['loc', S50_SEC_LOC, 's50-loc'],
  ['notify', S50_SEC_NOTIFY, 's50-notify'],
  ['screen', S50_SEC_SCREEN, 's50-screen'],
  ['link', S50_SEC_LINK, 's50-link'],
  ['log', S50_SEC_LOG, 's50-log'],
  ['blind', S50_SEC_BLIND, 's50-blind'],
  ['dev', S50_SEC_DEV, 's50-dev'],
]

const WHEN = {
  run: S50_WHEN_NEXT_BOOT,
  loc: S50_WHEN_MAP_RELOAD,
  notify: S50_WHEN_NOW,
  screen: S50_WHEN_NOW,
  link: S50_WHEN_NEXT_BOOT,
  dev: S50_WHEN_NOW,
}

// initialTab: S-00（疎通確認）から開いたときは 'dev'。INIT では一般タブの
// 読み込み（config_manager）が通らないため、開発モードの区分から始める。
export default function S50Settings({ onBack, initialTab = 'general' }) {
  const { ros, state, stale } = useSystemState()
  const { getTunableParams, applyTunableParam, saveTunableParams } = useTunableParams(ros)
  // SG-B8: 速度プリセット区画（/params/get・/params/set。次の起動から効く）。
  const { getParams, setParams } = useParamsOverrides(ros)
  // S-40（校正）への導線。FSM が真実なので画面側で遷移はしない — 受理されれば
  // /system/state が CALIB になり、main.jsx が S-50 を畳んで S-40 を出す。
  // 拒否されたら理由をその場に出す。
  const sendTrigger = useTrigger()
  const [calibErr, setCalibErr] = useState('')

  // 1b-5: サーバ側（config_manager）と同じ停止中条件に揃える。MANUAL は
  // 拒否されるようになったため外す（以前は IDLE/MANUAL だった）。
  const editable = mapOrParamOpAllowed(state, stale)

  const [section, setSection] = useState(initialTab === 'dev' ? 'dev' : initialTab === 'general' ? 'run' : 'run')
  const [status, setStatus] = useState({})
  const [loading, setLoading] = useState(false)
  const [blindRanges, setBlindRanges] = useState(null)
  const [slam, setSlam] = useState({})
  // presetEff＝いま効いている値、presetPending＝保存済みで次回から効く上書き、
  // presetDraft＝編集中。保存は /params/set（出どころ付き）で一括。
  const [presetEff, setPresetEff] = useState({})
  const [presetPending, setPresetPending] = useState({})
  const [presetDraft, setPresetDraft] = useState({})
  const [presetReason, setPresetReason] = useState('')
  // ── SG-C14: 自動ブレーキ既定（preset と同じ /params 経路）──
  const [brakeEff, setBrakeEff] = useState({})
  const [brakeDraft, setBrakeDraft] = useState({})
  const [brakeReason, setBrakeReason] = useState('')
  // ── S-21 の「カメラ」ボタンの開く先区画（同じ /params 経路）──
  const [cameraEff, setCameraEff] = useState({})
  const [cameraDraft, setCameraDraft] = useState({})
  const [cameraReason, setCameraReason] = useState('')

  const reload = useCallback(() => {
    setLoading(true)
    Promise.allSettled([
      getTunableParams('lidar_filter', ['blind_angle_ranges']),
      getTunableParams('slam_toolbox', SLAM_FIELDS.map((f) => f.name)),
    ]).then(([lidarRes, slamRes]) => {
      if (lidarRes.status === 'fulfilled') {
        setBlindRanges(lidarRes.value.blind_angle_ranges ?? [])
      } else {
        setBlindRanges(null)
      }
      if (slamRes.status === 'fulfilled') {
        setSlam(slamRes.value ?? {})
      } else {
        setSlam({})
        setStatus((s) => ({ ...s, slam_toolbox: S50_LOAD_FAILED }))
      }
    }).finally(() => setLoading(false))
    getParams(PRESET_NAMES).then((payload) => {
      setPresetEff(payload?.effective ?? {})
      setPresetPending(payload?.pending ?? {})
      setPresetDraft({ ...(payload?.effective ?? {}), ...(payload?.pending ?? {}) })
    }).catch(() => {
      setStatus((s) => ({ ...s, preset: S50_LOAD_FAILED }))
    })
    getParams(BRAKE_NAMES).then((payload) => {
      setBrakeEff(payload?.effective ?? {})
      setBrakeDraft({ ...(payload?.effective ?? {}), ...(payload?.pending ?? {}) })
    }).catch(() => {
      setStatus((s) => ({ ...s, brake: S50_LOAD_FAILED }))
    })
    getParams(CAMERA_NAMES).then((payload) => {
      setCameraEff(payload?.effective ?? {})
      setCameraDraft({ ...(payload?.effective ?? {}), ...(payload?.pending ?? {}) })
    }).catch(() => {
      setStatus((s) => ({ ...s, camera: S50_LOAD_FAILED }))
    })
  }, [getTunableParams, getParams])

  useEffect(() => { reload() }, [reload])

  const gotoCalib = () => {
    setCalibErr('')
    sendTrigger('ui.enter_mode', { mode: 'CALIB' }).then((res) => {
      if (!res?.accepted) setCalibErr(reasonLabel(res?.reject_reason_key) ?? UNKNOWN_REASON_LABEL)
    }).catch(() => setCalibErr(UNKNOWN_REASON_LABEL))
  }
  const applySlam = (name) => (value) => {
    setSlam((prev) => ({ ...prev, [name]: value }))
    applyTunableParam('slam_toolbox', name, value).catch(() => {})
  }

  const saveSlam = () => {
    setStatus((s) => ({ ...s, slam_toolbox: S50_SAVING }))
    saveTunableParams('slam_toolbox')
      .then((res) => setStatus((s) => ({
        ...s, slam_toolbox: res.success ? S50_SAVED : `${S50_SAVE_FAILED}: ${res.message}`,
      })))
      .catch((e) => setStatus((s) => ({
        ...s, slam_toolbox: `${S50_SAVE_FAILED}: ${e.message ?? e}`,
      })))
  }

  // SG-B8/SG-C14: 低速≦中速≦高速でなければ保存不可（サーバ側も同じ規則で拒否）。
  const presetOrderKey = presetOrderError(presetDraft)
  const presetOrderReason = presetOrderKey ? (S50_PRESET_ORDER_REASONS[presetOrderKey] ?? presetOrderKey) : ''
  const applyPreset = (name) => (value) => {
    setPresetDraft((prev) => ({ ...prev, [name]: value }))
  }
  const presetDiffs = pendingDiff(PRESET_NAMES, presetEff, presetPending)

  async function saveParamsSet(values, reasonKey) {
    const changed = {}
    for (const [name, draft, eff] of values) {
      if (draft !== eff) changed[name] = draft
    }
    if (Object.keys(changed).length === 0) return { ok: true }
    const res = await setParams(changed, 'web_ui', reasonKey)
    return { ok: !!res?.success, message: res?.message }
  }

  const saveRun = async () => {
    if (presetOrderReason) {
      setStatus((s) => ({ ...s, preset: presetOrderReason }))
      return
    }
    if (!presetReason) {
      setStatus((s) => ({ ...s, preset: S50_PRESET_REASON_NEED }))
      return
    }
    setStatus((s) => ({ ...s, preset: S50_SAVING, brake: S50_SAVING }))
    try {
      const pRes = await saveParamsSet(PRESET_NAMES.map((n) => [n, presetDraft[n], presetEff[n]]), presetReason)
      const bRes = await saveParamsSet(BRAKE_NAMES.map((n) => [n, brakeDraft[n], brakeEff[n]]), brakeReason || presetReason)
      setStatus((s) => ({
        ...s,
        preset: pRes.ok ? S50_SAVED : `${S50_SAVE_FAILED}: ${pRes.message ?? ''}`,
        brake: bRes.ok ? S50_SAVED : `${S50_SAVE_FAILED}: ${bRes.message ?? ''}`,
      }))
      if (pRes.ok && bRes.ok) reload()
    } catch (e) {
      setStatus((s) => ({ ...s, preset: `${S50_SAVE_FAILED}: ${e.message ?? e}` }))
    }
  }

  const applyBrake = (name) => (value) => {
    if (value !== 'on' && value !== 'off') return
    setBrakeDraft((prev) => ({ ...prev, [name]: value }))
  }

  const cameraUrlBad = !isValidCameraUrl(cameraDraft.camera_ui_url)
  const applyCamera = (value) => {
    setCameraDraft((prev) => ({ ...prev, camera_ui_url: value }))
  }
  const saveCamera = () => {
    if (cameraUrlBad) {
      setStatus((s) => ({ ...s, camera: S50_CAMERA_BAD_URL }))
      return
    }
    if (!cameraReason) {
      setStatus((s) => ({ ...s, camera: S50_PRESET_REASON_NEED }))
      return
    }
    setStatus((s) => ({ ...s, camera: S50_SAVING }))
    const values = {}
    for (const name of CAMERA_NAMES) {
      if (cameraDraft[name] !== cameraEff[name]) values[name] = cameraDraft[name]
    }
    if (Object.keys(values).length === 0) {
      setStatus((s) => ({ ...s, camera: S50_SAVED }))
      return
    }
    setParams(values, 'web_ui', cameraReason)
      .then((res) => {
        if (res?.success) {
          setStatus((s) => ({ ...s, camera: S50_SAVED }))
          reload()
        } else {
          setStatus((s) => ({ ...s, camera: `${S50_SAVE_FAILED}: ${res?.message ?? ''}` }))
        }
      })
      .catch((e) => setStatus((s) => ({
        ...s, camera: `${S50_SAVE_FAILED}: ${e.message ?? e}`,
      })))
  }

  // ── 表示 ──
  const [fontScale, setFontScale] = useState(() => readFontScale())
  const chooseFont = (name) => {
    setFontScale(name)
    applyFontScale(name)
  }
  const [voiceOn, setVoiceOn] = useState(() => readVoiceMaster())
  const chooseVoice = (on) => {
    setVoiceOn(on)
    applyVoiceMaster(on)
  }
  const [orientation, setOrientation] = useState(() => readStageOrientation())
  const chooseOrientation = (value) => {
    setOrientation(writeStageOrientation(value))
  }
  const [helpOn, setHelpOn] = useState(() => readHelpEnabled())
  const chooseHelp = (on) => {
    setHelpOn(on)
    writeHelpEnabled(on)
  }

  // ── 開発モード ──
  const { dev: devState, setDevParam } = useDevMode(ros)
  const faultRecords = useFaultHistory(ros)
  const [devLocal, setDevLocal] = useState(() => readDevMode())
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

  // ── 機体のスピーカー区画 (Spec-safety §11) ──
  const { cue: speakerCue, status: speakerStatus, setEnabled, setVolume, testChime } = useSpeaker(ros)
  const [speakerErr, setSpeakerErr] = useState('')
  const [speakerTestSent, setSpeakerTestSent] = useState(false)
  const toggleSpeaker = () => {
    setSpeakerErr('')
    setEnabled(!(speakerCue?.enabled ?? false)).catch(() => setSpeakerErr(S50_SPEAKER_SEND_FAILED))
  }
  const changeSpeakerVolume = (value) => {
    setSpeakerErr('')
    setVolume(value).catch(() => setSpeakerErr(S50_SPEAKER_SEND_FAILED))
  }
  const sendSpeakerTest = () => {
    setSpeakerErr('')
    setSpeakerTestSent(false)
    testChime()
      .then(() => setSpeakerTestSent(true))
      .catch(() => setSpeakerErr(S50_SPEAKER_SEND_FAILED))
  }

  const FONT_OPTIONS = [
    ['normal', S50_FONT_NORMAL],
    ['large', S50_FONT_LARGE],
    ['xlarge', S50_FONT_XLARGE],
  ]

  const opbar = (
    <div className="top-actions" data-pos="opbar">
      <HelpTip helpKey="s50-back">
        <button type="button" className="btn" onClick={onBack} data-testid="s50-back">
          {S50_BACK}
        </button>
      </HelpTip>
    </div>
  )

  const list = (
    <div>
      {SECTIONS.map(([key, label, helpKey]) => (
        <HelpTip key={key} helpKey={helpKey}>
          <button
            type="button"
            className={`lrow ${section === key ? 'on' : ''}`}
            onClick={() => setSection(key)}
            data-testid={`s50-sec-${key}`}
          >
            <span>{label}</span>
          </button>
        </HelpTip>
      ))}
    </div>
  )

  let body = null
  if (section === 'run') {
    body = (
      <div className="grp">
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_SEC_PRESET}</h3>
          {status.preset && <p className="note" data-testid="s50-status-preset">{status.preset}</p>}
          {presetOrderReason && (
            <p className="note" data-testid="s50-preset-order">{presetOrderReason}</p>
          )}
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
        </div>
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_SEC_BRAKE}</h3>
          {status.brake && <p className="note" data-testid="s50-status-brake">{status.brake}</p>}
          <div className="s50-grid">
            {BRAKE_FIELDS.map((f) => (
              <label className="s50-field" key={f.name}>
                <span className="s50-field-label">{f.label}</span>
                <select
                  value={brakeDraft[f.name] ?? ''}
                  disabled={!editable || loading || !(f.name in brakeDraft)}
                  onChange={(e) => applyBrake(f.name)(e.target.value)}
                  data-testid={`s50-brake-${f.name === BRAKE_NAMES[0] ? 'outdoor' : 'indoor'}`}
                >
                  {!(f.name in brakeDraft) && <option value="">—</option>}
                  {BRAKE_OPTIONS.map((o) => (
                    <option key={o.value} value={o.value}>{o.label}</option>
                  ))}
                </select>
              </label>
            ))}
          </div>
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
    )
  } else if (section === 'loc') {
    body = (
      <div className="grp">
        <h3 style={{ marginBottom: 8 }}>{S50_SEC_SLAM}</h3>
        {status.slam_toolbox && <p className="note" data-testid="s50-status-slam_toolbox">{status.slam_toolbox}</p>}
        <div className="s50-grid">
          {SLAM_FIELDS.map((f) => (
            <NumberField
              key={f.name} label={f.label} unit={f.unit}
              min={f.min} max={f.max} step={f.step}
              value={slam[f.name]}
              disabled={!editable || loading || !(f.name in slam)}
              onCommit={applySlam(f.name)}
            />
          ))}
        </div>
      </div>
    )
  } else if (section === 'notify') {
    body = (
      <div className="grp">
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_VOICE_TITLE}</h3>
          <div className="btnrow n2">
            {[
              [true, S50_VOICE_ON, 's50-voice-on'],
              [false, S50_VOICE_OFF, 's50-voice-off'],
            ].map(([value, label, testId]) => (
              <button
                key={testId}
                type="button"
                className={`btn ${voiceOn === value ? 'on' : ''}`}
                onClick={() => chooseVoice(value)}
                data-testid={testId}
              >
                {label}
              </button>
            ))}
          </div>
        </div>
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_SEC_SPEAKER}</h3>
          <button
            type="button"
            className={`btn ${(speakerCue?.enabled ?? false) ? 'on' : ''}`}
            onClick={toggleSpeaker}
            data-testid="s50-speaker-toggle"
          >
            {(speakerCue?.enabled ?? false) ? S50_SPEAKER_ON : S50_SPEAKER_OFF}
          </button>
          <label className="s50-field mt">
            <span className="s50-field-label">
              {S50_SPEAKER_VOLUME} ({speakerCue?.volume ?? '—'})
            </span>
            <input
              type="range"
              min="0"
              max="100"
              step="1"
              value={speakerCue?.volume ?? 60}
              onChange={(e) => changeSpeakerVolume(Number(e.target.value))}
              data-testid="s50-speaker-volume"
            />
          </label>
          <div className="row" style={{ marginTop: 8 }}>
            <HelpTip helpKey="s50-speaker-test">
              <button
                type="button"
                className="btn"
                onClick={sendSpeakerTest}
                data-testid="s50-speaker-test"
              >
                {S50_SPEAKER_TEST}
              </button>
            </HelpTip>
          </div>
          {speakerTestSent && <p className="note" data-testid="s50-speaker-test-sent">{S50_SPEAKER_TEST_SENT}</p>}
          {speakerErr && <p className="note" data-testid="s50-speaker-err">{speakerErr}</p>}
          <p data-testid="s50-speaker-state">
            {speakerCue == null
              ? S50_LOAD_FAILED
              : (!speakerCue.rpi_ok
                ? S50_SPEAKER_NO_REPLY
                : `${S50_SPEAKER_PLAYING}: ${speakerStatus?.cue ?? speakerCue.cue}`
                  + (speakerStatus?.error ? ` (${speakerStatus.error})` : ''))}
          </p>
        </div>
      </div>
    )
  } else if (section === 'screen') {
    body = (
      <div className="grp">
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_ORIENTATION_TITLE}</h3>
          <div className="btnrow n3">
            {[
              [ORIENTATION_AUTO, S50_ORIENTATION_AUTO, 's50-orientation-auto'],
              [ORIENTATION_PORTRAIT, S50_ORIENTATION_PORTRAIT, 's50-orientation-portrait'],
              [ORIENTATION_LANDSCAPE, S50_ORIENTATION_LANDSCAPE, 's50-orientation-landscape'],
            ].map(([value, label, testId]) => (
              <button
                key={testId}
                type="button"
                className={`btn ${orientation === value ? 'on' : ''}`}
                onClick={() => chooseOrientation(value)}
                data-testid={testId}
              >
                {label}
              </button>
            ))}
          </div>
        </div>
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_FONT_TITLE}</h3>
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
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_HELP_TITLE}</h3>
          <div className="btnrow n2">
            {[
              [true, S50_HELP_SHOW, 's50-help-on'],
              [false, S50_HELP_HIDE, 's50-help-off'],
            ].map(([value, label, testId]) => (
              <HelpTip key={testId} helpKey="s50-help">
                <button
                  type="button"
                  className={`btn ${helpOn === value ? 'on' : ''}`}
                  onClick={() => chooseHelp(value)}
                  data-testid={testId}
                >
                  {label}
                </button>
              </HelpTip>
            ))}
          </div>
        </div>
      </div>
    )
  } else if (section === 'link') {
    body = (
      <div className="grp">
        <h3 style={{ marginBottom: 8 }}>{S50_SEC_CAMERA}</h3>
        {status.camera && <p className="note" data-testid="s50-status-camera">{status.camera}</p>}
        <label className="s50-field">
          <span className="s50-field-label">{S50_CAMERA_URL_LABEL}</span>
          <input
            type="text"
            value={cameraDraft.camera_ui_url ?? ''}
            disabled={!editable || loading || !('camera_ui_url' in cameraDraft)}
            onChange={(e) => applyCamera(e.target.value)}
            data-testid="s50-camera-url"
          />
        </label>
        <label className="s50-field">
          <span className="s50-field-label">{S50_PRESET_REASON_LABEL}</span>
          <input
            type="text"
            value={cameraReason}
            disabled={!editable || loading}
            onChange={(e) => setCameraReason(e.target.value)}
            data-testid="s50-camera-reason"
          />
        </label>
      </div>
    )
  } else if (section === 'log') {
    body = (
      <div data-testid="s50-faults">
        <FaultHistoryList records={faultRecords} />
      </div>
    )
  } else if (section === 'blind') {
    body = (
      <div className="grp">
        <p data-testid="s50-blind-current">
          {blindRanges == null ? (loading ? '' : S50_LOAD_FAILED) : blindRangesText(blindRanges)}
        </p>
        <div className="row">
          <HelpTip helpKey="s50-goto-calib">
            <button
              type="button"
              className="btn"
              onClick={gotoCalib}
              data-testid="s50-goto-calib"
            >
              {S50_BLIND_GOTO_CALIB}
            </button>
          </HelpTip>
        </div>
        {calibErr && <p className="note" data-testid="s50-calib-err">{calibErr}</p>}
      </div>
    )
  } else if (section === 'dev') {
    body = (
      <div className="grp">
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_DEV_TITLE}</h3>
          <HelpTip helpKey="s50-dev-toggle">
            <button
              type="button"
              className={`btn wide ${devShown ? 'on' : ''}`}
              onClick={toggleDev}
              data-testid="s50-dev-toggle"
            >
              {devShown ? S50_DEV_DISABLE : S50_DEV_ENABLE}
            </button>
          </HelpTip>
          {devStatus && <p className="note" data-testid="s50-dev-status">{devStatus}</p>}
        </div>
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_DEV_ITEMS_TITLE}</h3>
          <div className="s50-grid s50-dev-items">
            {DEV_ITEM_META.map(({ item, label }) => (
              <button
                key={item}
                type="button"
                className={`btn ${devItemsShown[item] ? 'on' : ''}`}
                aria-pressed={!!devItemsShown[item]}
                onClick={() => toggleDevItem(item)}
                data-testid={`s50-dev-ignore-${item}`}
              >
                {label}
              </button>
            ))}
          </div>
          <p className="note" data-testid="s50-dev-no-gate">{S50_DEV_NO_GATE}</p>
        </div>
        <div>
          <h3 style={{ marginBottom: 8 }}>{S50_DEV_EFFECTIVE_TITLE}</h3>
          <p data-testid="s50-dev-effective">{devEffectiveText(devState)}</p>
          <p className="note" data-testid="s50-dev-scope">{S50_DEV_SCOPE_NOTE}</p>
        </div>
      </div>
    )
  }

  const saveFor = () => {
    if (section === 'run') return { onSave: saveRun }
    if (section === 'loc') return { onSave: saveSlam }
    if (section === 'link') return { onSave: saveCamera }
    return null
  }
  const saveCfg = saveFor()

  const foot = (
    <div data-pos="foot">
      {!editable && section !== 'notify' && section !== 'screen' && section !== 'log' && section !== 'blind' && section !== 'dev' && (
        <p className="note" data-testid="s50-guard">{S50_GUARD}</p>
      )}
      <div className="row">
        <span className="grow" />
        {WHEN[section] && (
          <span className="when" data-testid={`s50-when-${section}`}>{WHEN[section]}</span>
        )}
        {saveCfg && (
          <HelpTip helpKey="s50-save">
            <button
              type="button"
              className="btn save"
              disabled={!editable || loading || (section === 'run' && !!presetOrderReason) || (section === 'link' && cameraUrlBad)}
              onClick={saveCfg.onSave}
              data-testid={section === 'run' ? 's50-save-run' : section === 'loc' ? 's50-save-slam_toolbox' : 's50-save-camera'}
            >
              {S50_PRESET_SAVE}
            </button>
          </HelpTip>
        )}
      </div>
    </div>
  )

  return (
    <div className="screen" id="s50">
      <ViewLayout opbar={opbar} list={list} body={body} foot={foot} />
    </div>
  )
}

// テスト（test_tunable_targets.py）が MAPLESS_FIELDS / SLAM_FIELDS の配列名を
// 正規表現で拾う。名前を変えるときは向こうも直すこと。
// BLIND_LABELS は廃止（2026-10-02。死角の編集欄を S-50 から外した）。
export { MAPLESS_FIELDS, SLAM_FIELDS }
