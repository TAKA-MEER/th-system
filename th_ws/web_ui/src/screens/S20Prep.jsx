// screens/S20Prep.jsx — S-20, 試験準備 (SCREEN_NAMES.S20 in i18n/screens.js;
// Spec-webui.md §3.10。型「走らせる」。タブは無い（2026-11-27 の作り直し）。
//
// 構成（§3.10 の表どおり）:
//   左上の操作バー: 「終了」／人検出 切│入／自動ブレーキ（ロック付き。常に出す）／
//     「未保存」の印。RunLayout の data-pos="opbar"（右列の上）に入る形の骨組みを
//     使う（配置の詳細は src/shell/Layouts.jsx）。
//   入口: 地図作成中（mappingActive===true）でなければ主表示の中央のカード。
//     前回の地図とピンが残っていれば「続きから再開」＋「新しい試験日として
//     開始」（二段階）、無ければ「地図作成を開始」1 つ。どちらも機体は動かない
//     （塗らない）。地図作成中になったら地図を出す（入口かどうかは機体の
//     状態から導く。A-9）。
//   主表示: 作成中の地図（OnsiteMap。地図の端の道具込み）＋レーダーを地図の
//     右上に重ねる（人検出が止まっている間は TrackerControl で開始できる）。
//     地図の左上には「順番を決める」。ピンを押すと吹き出しで編集する。
//   地図の下の帯: 登録（方式の切り替え＋「待機場所を登録」「配電盤を登録」）。
//     登録中・地図タップ・壁に近い・順番決めの手順と警告の帯はこの帯に重ねて
//     出し、地図の大きさを変えない（D62・D66）。
//   右列: ① ピンの件数 ② 停止│走行（予約。待機場所へ戻る途中だけ停止が押せる）
//     ③ 保存（スティックに触れている間は押せない）・待機場所に戻る
//     ④ スティック常設（速度のスライダ＋スティック＋中速・低速）。
//   「次にやること」ボタンは置かない（D87）。地図の修正・LiDAR 平面認識は作らない。
//
// FSM（th_state/config/transitions.yaml T-PREP-01〜18）:
//   MAPPING --ui.register{kind}(target_confident)--> REGISTER（2 点指示）
//           --ui.select_target{index}(candidate_exists)--> set_target
//           --ui.return_home(home_pin_exists)--> RETURN
//           RETURN --ui.stop--> PAUSE（SM-3.1.2-050a。RETURN 以外の停止は inert）
//           PAUSE --ui.resume_yes--> RETURN ／ --ui.resume_no--> MAPPING
//           * --ui.save--> SAVED（+ commit_venue_map effect）
//           * --ui.finish--> IDLE、SAVED --ui.run--> MAPPING
// 2 点指示は /onsite/two_point を index 1→2 で呼ぶ（サーバが evt.register_ok を
// 出して REGISTER を抜け、pin_registrar が place_pin でピンを永続化する。
// UI は待つだけ）。配電盤の順番変更は R-5 の /onsite/edit_pin（move / order_ids）で
// 送る（機体側はマージ済み）。
//
// 終了は ui.finish を送るだけ。受理されれば FSM が IDLE になり main.jsx が S-01 に戻す。
// onTrigger/onClick を空関数にしない（全操作ボタンを e2e/s20-prep.spec.js で検証）。
import { useCallback, useEffect, useRef, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useSetFlag, TRACKER_FLAG } from '../ros/useSetFlag.js'
import { trackerStopAllowed } from '../modes/trackerControlPolicy.js'
import { mapOrParamOpAllowed } from '../modes/stopOnlyGuard.js'
import { useOnsitePins } from '../ros/useOnsitePins.js'
import { usePersonTargets } from '../ros/usePersonTargets.js'
import { usePersonStatus } from '../ros/usePersonStatus.js'
import { useScan } from '../ros/useScan.js'
import { scanToPoints } from '../parts/scanToPoints.js'
import { useOnsiteService } from '../ros/useOnsiteService.js'
import { usePinWarning } from '../ros/usePinWarning.js'
import { useOnsiteMapView } from '../ros/useOnsiteMapView.js'
import { useOnsiteCostmap } from '../ros/useCostmap.js'
import { usePlannedPath } from '../ros/usePlannedPath.js'
import { useRoutePose } from '../ros/useRoutePose.js'
import { useMappingActive } from '../ros/useMappingActive.js'
import { useStdTrigger } from '../ros/useStdTrigger.js'
import { useStdSetBool } from '../ros/useStdSetBool.js'
import { RunLayout } from '../shell/Layouts.jsx'
import JogConsole from '../parts/JogConsole.jsx'
import RadarSelect from '../parts/RadarSelect.jsx'
import TrackerControl from '../parts/TrackerControl.jsx'
import OnsiteMap from '../parts/OnsiteMap.jsx'
import StopRunToggle from '../parts/StopRunToggle.jsx'
import ReasonButton from '../parts/ReasonButton.jsx'
import PersonDetectToggle from '../parts/PersonDetectToggle.jsx'
import AutoBrakeLockToggle from '../parts/AutoBrakeLockToggle.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import ArmedButton from '../parts/ArmedButton.jsx'
import { IconClose, IconPin } from '../parts/icons.jsx'
import { OP_LABELS } from '../i18n/states.js'
import { baseToWorld, quatToYaw } from '../mapGeometry.js'
import { REJECT_REASONS, reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import {
  S20_MAP_ARIA, S20_MAP_NO_POSE, S20_MAP_ROBOT, S20_MAP_TARGET,
  S20_MAPTAP_CANCEL, S20_MAPTAP_CONFIRM, S20_MAPTAP_PREVIEW,
  S20_PIN_CANCEL, S20_PIN_DELETE, S20_PIN_DELETE_ARMED, S20_PIN_RENAME,
  S20_PIN_REREGISTER_HERE, S20_PIN_REREGISTER_MAPTAP, S20_PIN_REREGISTER_NO_POSE,
  S20_PIN_REREGISTER_PERSON, S20_PIN_YAW,
  S20_PINWARN_CANCEL, S20_PINWARN_MSG, S20_PINWARN_PLACE, S20_PINWARN_RETREAT,
  S20_REG_HERE_OK, S20_REGISTER_TITLE,
  S20_RESET_ARMED, S20_RESET_BUSY, S20_RESET_FAIL, S20_RESET_IDLE,
  S20_WIZ_MSG_STEP1, S20_WIZ_MSG_STEP2, S20_WIZ_REGISTER,
  S20_UNSAVED,
  S20_METHOD_PERSON, S20_METHOD_POSE, S20_METHOD_TAP,
  S20_REG_HOME_SHORT, S20_REG_PANEL_SHORT, S20_RESUME, S20_START_MAPPING,
  S20_ENTRY_REMAINING, S20_RETURN_HOME_SHORT, S20_MAP_GATE_MSG,
  S20_PIN_COUNT, S20_RESERVED_NOTE, S20_ORDER_START, S20_ORDER_COUNT,
  S20_ORDER_NEXT, S20_ORDER_DONE, S20_ORDER_COMMIT, S20_ORDER_REDO,
  S20_PIN_CLOSE, S20_PIN_ORDER_POS, S20_PIN_PREV, S20_PIN_NEXT,
  S20_TAPMODE_MSG,
  S20_REASON_NO_HOME, S20_REASON_STICK_SAVE, S20_REASON_SAVE_STOP,
  S20_REASON_REG_TRACKER, S20_REASON_REG_BUSY,
  S20_REASON_ORDER_PANELS, S20_REASON_FIRST, S20_REASON_LAST,
  S20_REASON_REREG_PERSON,
  TRACKER_OFF,
} from '../i18n/screens.js'
import './S20Prep.css'

function yawDeg(pin) {
  if (!pin?.pose?.orientation) return 0
  const deg = (quatToYaw(pin.pose.orientation) * 180) / Math.PI
  return (deg % 360 + 360) % 360
}

const METHODS = [
  { id: 'person', label: S20_METHOD_PERSON },
  { id: 'pose', label: S20_METHOD_POSE },
  { id: 'tap', label: S20_METHOD_TAP },
]

export default function S20Prep() {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  // brief-tracker-default-off §3.4: 人検出の開始/停止（/system/set_flag
  // tracker_enabled）。状態の正本は server（state.tracker_enabled）。
  const setFlag = useSetFlag()
  // fix-setflag-requester: 拒否・通信失敗を押した人に見せる（S-11/S-13 と同じ
  // reasonLabel の流儀。受理されるまで表示は /system/state に従う）。
  const [trackerErr, setTrackerErr] = useState(null)
  async function requestTracker(next) {
    setTrackerErr(null)
    try {
      const res = await setFlag(TRACKER_FLAG, next, 's20-target')
      if (!res?.accepted) {
        setTrackerErr(reasonLabel(res?.reject_reason_key) ?? UNKNOWN_REASON_LABEL)
      }
    } catch {
      setTrackerErr(UNKNOWN_REASON_LABEL)
    }
  }
  const pins = useOnsitePins(ros)
  const personTargets = usePersonTargets(ros)
  const personStatus = usePersonStatus(ros)
  const scan = useScan(ros)
  const routeMap = useOnsiteMapView(ros)
  const routePose = useRoutePose(ros)
  // brief-MAP-COSTMAP: 地図に Nav2 の costmap と直近の経路を重ねる。
  const costmapData = useOnsiteCostmap(ros)
  const plannedPath = usePlannedPath(ros)
  const { twoPoint, editPin, registerPinHere, registerPinMapTap, resolvePin } = useOnsiteService()
  const pinWarn = usePinWarning(ros)
  // /slam_control/mapping_active: 地図作成中か（入口かどうかは機体の状態から導く。A-9）。
  // 押したときは set_mapping{data:true} で開始する（幂等。toggle は押下が重なると反転する）。
  const mappingActive = useMappingActive(ros)
  const setMapping = useStdSetBool('/slam_control/set_mapping')
  const startingRef = useRef(false)
  const [mapUnlocked, setMapUnlocked] = useState(false)
  // WS-9Y: 前回セッションの地図・ピンをまとめて消す「新しい試験日として開始」。
  // discard_map（slam_toolbox 再起動）→ reset_pins（pins.yaml を空にする）の順。
  const discardMap = useStdTrigger('/slam_control/discard_map')
  const resetPins = useStdTrigger('/onsite/reset_pins')
  const [resetBusy, setResetBusy] = useState(false)
  const [resetErr, setResetErr] = useState(null)
  const [returnErr, setReturnErr] = useState(null)
  // スティックに触れている間（JogConsole / キーボード）。保存は押せない（A-9）。
  const [stickHeld, setStickHeld] = useState(false)

  const disabledAll = stale || state?.mode == null
  // 1b-5 (SG-A10): 地図の開始・破棄・保存は停止中だけ（Spec.md SD-9）。サーバ側が
  // 正で、画面は同じ条件で事前に押せなくする。
  const mapOpAllowed = mapOrParamOpAllowed(state, stale)

  // 登録の方式（前回の方式を既定にする。D5）。
  const [method, setMethod] = useState('person')
  // ピン（地図で選択。吹き出しで編集する）。
  const [selectedPinId, setSelectedPinId] = useState(null)
  const [pinNameDraft, setPinNameDraft] = useState('')
  // 2 点指示ウィザード。purpose は押した ui.register の kind（§2.3 の一致規則）。
  const [registerKind, setRegisterKind] = useState('HOME')
  const [wizStep, setWizStep] = useState(1)
  const [wizErr, setWizErr] = useState(null)
  const [wizYaw, setWizYaw] = useState(null)
  // 機体姿勢での登録（REG-2）の結果メッセージ。
  const [hereMsg, setHereMsg] = useState(null)
  // 地図タップ登録（方式 tap）。押してドラッグ（押した所＝位置、離した所＝向き）。
  const [tapMode, setTapMode] = useState(false)
  const [pendingTap, setPendingTap] = useState(null)
  // SG-C7: 確定前の数値補正（Spec-onsite.md §3.7）。
  const [tapDraft, setTapDraft] = useState(null)
  // SG-B16: 地図タップで既存ピンを登録し直すときの対象 id。
  const [reregTargetId, setReregTargetId] = useState(null)
  // 順番を決める（押した順に番号 →「決める」。R-5 の order_ids で機体へ送る）。
  const [ordering, setOrdering] = useState(null)

  const stateName = state?.state ?? null
  const isRegister = stateName === 'REGISTER'
  const homePinExists = pins.some((p) => p.kind === 'HOME')
  const panelPins = pins.filter((p) => p.kind === 'PANEL')
  const unsaved = Array.isArray(state?.unsaved) && state.unsaved.length > 0
  // 入口かどうかは機体の状態（地図作成中か）から導く（A-9）。
  const showMap = mapUnlocked || mappingActive === true
  const returning = stateName === 'RETURN'

  // brief-tracker-default-off §3.4: 人検出フラグと「起動中（候補まだ出ない）」判定。
  const trackerEnabled = state?.tracker_enabled === true
  const hasCandidate = Array.isArray(personTargets.candidates)
    && personTargets.candidates.length > 0
  const trackerCanStop = trackerStopAllowed(state?.mode, stateName)
  const trackerStopDenied = trackerCanStop ? null : undefined

  // MAP-1: 追従対象者を地図上に表示。/person/status は base_link 相対なので
  // baseToWorld() で map 座標に変換する（routePose は map フレーム）。is_lost の
  // ときや routePose が未取得のときは変換できないので null（マーカーを出さない）。
  let personPose = null
  if (routePose && personStatus && personStatus.is_lost === false) {
    const [wx, wy] = baseToWorld(personStatus.position.x, personStatus.position.y, routePose)
    personPose = { x: wx, y: wy }
  }

  // MAP-SCAN: LiDAR の生スキャン（/scan_filtered、base_link 相対）を map 座標へ。
  let scanPoints = null
  if (routePose && scan) {
    scanPoints = scanToPoints(scan).map(({ x, y }) => {
      const [wx, wy] = baseToWorld(x, y, routePose)
      return { x: wx, y: wy }
    })
  }

  // REGISTER を離れたら（成功・拒否・ロストで MAPPING へ戻る）ウィザードを初期化。
  useEffect(() => {
    if (isRegister) return undefined
    setWizStep(1)
    setWizErr(null)
    setWizYaw(null)
    return undefined
  }, [isRegister])

  // 編集中のピンが消えたら（削除・再読込）吹き出しを閉じる。
  const selectedPin = pins.find((p) => p.id === selectedPinId) ?? null
  useEffect(() => {
    if (selectedPinId != null && !pins.some((p) => p.id === selectedPinId)) {
      setSelectedPinId(null)
    }
  }, [pins, selectedPinId])

  // 選択したピンが変わったら名前の入力をそのピンの名前にする。
  useEffect(() => {
    if (selectedPin) setPinNameDraft(selectedPin.name ?? '')
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedPinId])

  const handleSelectPin = useCallback((id) => {
    if (ordering != null) {
      // 順番決め中は吹き出しを出さず、配電盤を押した順に番号を付ける。
      if (pins.some((p) => p.id === id && p.kind === 'PANEL')) {
        setOrdering((ord) => (ord && !ord.includes(id) ? [...ord, id] : ord))
      }
      return
    }
    setSelectedPinId(id)
  }, [ordering, pins])

  // ── 操作 ─────────────────────────────────────────────
  // 「地図作成を開始」／「続きから再開」。mappingActive===false のときだけ
  // set_mapping{data:true} を呼ぶ。起動時から動いている通常ケース（true）は
  // すでに showMap（入口カードは出ない）。
  async function handleStartMapping() {
    if (!mapOpAllowed) return   // 停止中以外は送らない（サーバ側も拒否する）
    if (startingRef.current) return   // 応答待ちの間の重ねて押下は送らない
    if (mappingActive === false) {
      startingRef.current = true
      try {
        const res = await setMapping(true)
        if (res?.success === false) return   // 失敗時は解禁しない（留まる）
      } finally {
        startingRef.current = false
      }
    }
    setMapUnlocked(true)
  }

  // WS-9Y: 「新しい試験日として開始」。discard_map → reset_pins の順に呼ぶ。
  // ArmedButton なのでここに来る時点で操作者の二段階確認は済んでいる。
  async function handleResetVenue() {
    if (!mapOpAllowed) return   // 停止中以外は送らない（サーバ側も拒否する）
    setResetErr(null)
    setResetBusy(true)
    const d = await discardMap()
    if (d?.success === false) {
      setResetErr(d?.message || S20_RESET_FAIL)
      setResetBusy(false)
      return
    }
    const r = await resetPins()
    if (r?.success === false) {
      setResetErr(r?.message || S20_RESET_FAIL)
    }
    setResetBusy(false)
  }

  // 登録（MAPPING で受理されると FSM が REGISTER にする）。方式で送り方が変わる。
  async function handleRegister(kind) {
    setRegisterKind(kind)
    if (method === 'person') {
      await sendTrigger('ui.register', { kind })
    } else if (method === 'pose') {
      await handleRegisterHere(kind)
    } else {
      setTapMode(true)
    }
  }

  // REG-2: 機体姿勢での直接登録（/onsite/register_pin, method=ROBOT_POSE）。
  async function handleRegisterHere(kind) {
    setHereMsg(null)
    const res = await registerPinHere(kind)
    setHereMsg({ ok: !!res?.success, text: res?.message || (res?.success ? S20_REG_HERE_OK(kind) : '') })
  }

  // brief-MAPTAP-FRONTEND: 地図タップのジェスチャー完了。確定ボタンを待つ。
  function handleMapTapConfirm(tap1, tap2) {
    setPendingTap({ tap1, tap2 })
  }

  const previewYawRad = pendingTap
    ? Math.atan2(pendingTap.tap2.y - pendingTap.tap1.y, pendingTap.tap2.x - pendingTap.tap1.x)
    : null

  // SG-C7: pendingTap が立ったら数値補正の初期値を入れる。キャンセル・確定で消す。
  useEffect(() => {
    if (pendingTap) {
      const yaw = Math.atan2(pendingTap.tap2.y - pendingTap.tap1.y,
        pendingTap.tap2.x - pendingTap.tap1.x)
      setTapDraft({
        x: pendingTap.tap1.x.toFixed(2),
        y: pendingTap.tap1.y.toFixed(2),
        deg: Math.round((yaw * 180) / Math.PI),
      })
    } else {
      setTapDraft(null)
    }
  }, [pendingTap])

  // brief-MAPTAP-FRONTEND: 確定ボタン。MAP_TAP でサービスを呼ぶ。
  // SG-C7: 送る値は数値補正の draft（tapDraft が無ければ pendingTap のまま）。
  async function handleMapTapCommit() {
    if (!pendingTap) return
    const draft = tapDraft ?? {
      x: pendingTap.tap1.x, y: pendingTap.tap1.y,
      deg: Math.round((previewYawRad * 180) / Math.PI),
    }
    const tap1 = { x: Number(draft.x), y: Number(draft.y) }
    // 向きを触っていなければドラッグで得た値をそのまま使う（度への丸めで
    // 微小に変えない）。触ったときは②までの距離 L を保ったまま向きだけ変える
    // （L は 2 点の間隔チェックの対象なので変えない）。
    const dragDeg = Math.round((previewYawRad * 180) / Math.PI)
    const yawRad = Number(draft.deg) === dragDeg
      ? previewYawRad : (Number(draft.deg) * Math.PI) / 180
    const span = Math.hypot(pendingTap.tap2.x - pendingTap.tap1.x,
      pendingTap.tap2.y - pendingTap.tap1.y)
    const tap2 = {
      x: tap1.x + span * Math.cos(yawRad),
      y: tap1.y + span * Math.sin(yawRad),
    }
    let res
    if (reregTargetId != null) {
      // SG-B16: 同じ id・同じ名前のまま位置と向きだけ上書き（新しい id を振らない）。
      res = await editPin({
        id: reregTargetId, new_name: '', is_delete: false, update_pose: true,
        x: tap1.x, y: tap1.y, yaw: yawRad,
      })
    } else {
      res = await registerPinMapTap({ kind: registerKind, tap1, tap2 })
    }
    setHereMsg({ ok: !!res?.success, text: res?.message || '' })
    cancelTap()
  }

  function cancelTap() {
    setPendingTap(null)
    setTapMode(false)
    setReregTargetId(null)
  }

  // 2 点指示（index 1 → 2）。拒否されたら理由を出して Step 1 からやり直し（§2.3）。
  async function handleWizPress() {
    setWizErr(null)
    const res = await twoPoint({ purpose: registerKind, index: wizStep })
    if (res?.accepted) {
      if (wizStep === 1) {
        setWizStep(2)
      } else if (res.reject_reason_key === 'pin_close_to_wall') {
        // WS-9AB: 壁に近すぎる。登録は保留され /onsite/pin_warning が立つ。
        setWizYaw(null)
      } else {
        setWizYaw(Math.round(((res.yaw ?? 0) * 180) / Math.PI))
      }
      return
    }
    const key = res?.reject_reason_key
    setWizErr(key ? (REJECT_REASONS[key] ?? key) : null)
  }

  // WS-9AB: 壁近接警告の 3 択。two_point 経由なら resolve で pin_registrar が
  // evt.register_ok / evt.register_rejected を出して FSM が REGISTER を抜け、
  // ウィザードが閉じる。robot_pose 経由は FSM を経由しないのでここで出すだけ。
  async function handlePinWarnResolve(action) {
    const res = await resolvePin(action)
    if (res?.success === false) {
      setHereMsg({ ok: false, text: res?.message || '' })
    } else {
      setHereMsg(null)
    }
  }

  // 1 ボタンで待機場所に戻す（guard: home_pin_exists。無いときは押せない）。
  async function handleReturnHome() {
    setReturnErr(null)
    const res = await sendTrigger('ui.return_home')
    // SM-3.1.2-144: 戻る先の地図が無いと機体が拒否する。理由を出す。
    if (res && res.accepted === false) {
      setReturnErr(reasonLabel(res.reject_reason_key) ?? UNKNOWN_REASON_LABEL)
    }
  }

  function handleFinish() {
    sendTrigger('ui.finish')
  }

  function handleSave() {
    sendTrigger('ui.save')
  }

  // 対象選択（RadarSelect のタップ）。
  function handleSelectTarget(index) {
    sendTrigger('ui.select_target', { index })
  }

  // ── ピン編集（吹き出し） ───────────────────────────────
  async function doPinRename() {
    if (selectedPinId == null) return
    await editPin({ id: selectedPinId, new_name: pinNameDraft, is_delete: false })
    setSelectedPinId(null)
  }

  async function doPinDelete() {
    if (selectedPinId == null) return
    await editPin({ id: selectedPinId, new_name: '', is_delete: true })
    setSelectedPinId(null)
  }

  // R-5（D60）: 配電盤を 1 つ前（move=-1）／後ろ（move=+1）へ動かす。
  async function doPinMove(move) {
    if (selectedPinId == null) return
    await editPin({
      id: selectedPinId, new_name: '', is_delete: false, update_pose: false, move,
    })
  }

  // SG-B16: 同じ id・同じ名前のまま位置と向きを登録し直す。機体の現在姿勢
  // （map 座標の routePose）をそのまま送る。frame が odom のとき・未取得の
  // ときは押せない（map が無いとピンの座標系と合わない）。
  const reregPose = routePose && routePose.frame !== 'odom' ? routePose : null
  const [reregMsg, setReregMsg] = useState(null)

  async function doPinReregister() {
    if (selectedPinId == null || !reregPose) return
    setReregMsg(null)
    const res = await editPin({
      id: selectedPinId, new_name: '', is_delete: false, update_pose: true,
      x: reregPose.x, y: reregPose.y, yaw: reregPose.yaw,
    })
    if (res?.success) {
      setSelectedPinId(null)
    } else {
      setReregMsg(res?.message || null)
    }
  }

  // 順番を決める: 押した順に番号 →「決める」（押さなかった盤は現在の順のまま後ろへ。D64）。
  function startOrdering() {
    setSelectedPinId(null)
    setOrdering([])
  }

  async function commitOrdering() {
    if (!ordering || ordering.length === 0) return
    await editPin({
      id: ordering[0], new_name: '', is_delete: false, update_pose: false,
      order_ids: ordering,
    })
    setOrdering(null)
  }

  // 登録ボタンの押せなさと理由。地図の入口カードが出ている間でも、
  // 機体が MAPPING なら登録は受け付ける（入口カードは表示の段差で、受付の条件ではない）。
  const regBusy = isRegister || pinWarn.active || pendingTap != null
  const regDisabled = disabledAll || regBusy
    || (method === 'person' && !trackerEnabled)
  const regReason = regBusy ? S20_REASON_REG_BUSY
    : (method === 'person' && !trackerEnabled) ? S20_REASON_REG_TRACKER
      : null
  // 保存の押せなさと理由（スティックに触れている間は押せない。A-9）。
  const saveDisabled = disabledAll || stickHeld || !mapOpAllowed
  const saveReason = stickHeld ? S20_REASON_STICK_SAVE
    : !mapOpAllowed ? S20_REASON_SAVE_STOP
      : null
  // 順番の前後（配電盤の並びの中での移動）。
  const panelIndex = selectedPin ? panelPins.findIndex((p) => p.id === selectedPin.id) : -1

  const toolbar = (
    <>
      <HelpTip helpKey="finish">
        <button
          type="button"
          className="btn exit"
          data-testid="s20-finish"
          disabled={disabledAll}
          onClick={handleFinish}
        >
          <IconClose />
          <span>{OP_LABELS.finish}</span>
        </button>
      </HelpTip>
      <PersonDetectToggle
        enabled={trackerEnabled}
        canToggle={!disabledAll}
        onToggle={(next) => requestTracker(next)}
        testId="s20-person-detect"
      />
      <HelpTip helpKey="brake">
        <AutoBrakeLockToggle
          brake={state?.auto_brake ?? null}
          canToggle={!disabledAll}
          prefix="s20"
          mode={state?.mode}
          stateName={stateName}
        />
      </HelpTip>
      {unsaved && <span className="pill ng" data-testid="s20-unsaved">{S20_UNSAVED}</span>}
      {trackerErr && (
        <span className="note" data-testid="s20-tracker-error">{trackerErr}</span>
      )}
    </>
  )

  const entry = !showMap ? (
    <div className="card s20-entry">
      {pins.length > 0 ? (
        <>
          <div className="note" data-testid="s20-map-gate-msg">{S20_ENTRY_REMAINING(pins.length)}</div>
          <HelpTip helpKey="s20-resume">
            <button
              type="button"
              className="btn wide mt"
              data-testid="s20-map-gate-start"
              disabled={disabledAll || mappingActive == null || resetBusy || !mapOpAllowed}
              onClick={handleStartMapping}
            >
              <span>{S20_RESUME}</span>
            </button>
          </HelpTip>
          {resetBusy ? (
            <div className="note mt" data-testid="s20-reset-busy">{S20_RESET_BUSY}</div>
          ) : (
            <div className="well mt" data-testid="s20-reset-venue">
              <HelpTip helpKey="s20-reset">
                <ArmedButton
                  className="wide"
                  idleLabel={S20_RESET_IDLE}
                  armedLabel={S20_RESET_ARMED}
                  disabled={disabledAll || !mapOpAllowed}
                  onConfirm={handleResetVenue}
                />
              </HelpTip>
              {resetErr && <div className="note err mt" data-testid="s20-reset-err">{resetErr}</div>}
            </div>
          )}
        </>
      ) : (
        <>
          <div className="note" data-testid="s20-map-gate-msg">{S20_MAP_GATE_MSG}</div>
          <HelpTip helpKey="s20-resume">
            <button
              type="button"
              className="btn wide mt"
              data-testid="s20-map-gate-start"
              disabled={disabledAll || mappingActive == null || !mapOpAllowed}
              onClick={handleStartMapping}
            >
              <span>{S20_START_MAPPING}</span>
            </button>
          </HelpTip>
        </>
      )}
    </div>
  ) : null

  const balloon = selectedPin && showMap ? (
    <div className="well s20-balloon" data-testid="s20-pin-balloon">
      <div className="row mb">
        <span className="b sm grow" data-testid={`s20-pin-name-${selectedPin.id}`}>
          {selectedPin.name || selectedPin.id}
        </span>
        <span className="sm mut">{S20_PIN_YAW(Math.round(yawDeg(selectedPin)))}</span>
        {selectedPin.kind === 'PANEL' && panelIndex >= 0 && (
          <span className="sm mut" data-testid="s20-pin-order-pos">
            {S20_PIN_ORDER_POS(panelIndex + 1, panelPins.length)}
          </span>
        )}
      </div>
      {selectedPin.kind === 'PANEL' && (
        <div className="btnrow n2 mb">
          <HelpTip helpKey="s20-pin-prev">
            <ReasonButton
              className="sm"
              data-testid="s20-pin-move-prev"
              reason={panelIndex === 0 ? S20_REASON_FIRST : null}
              reasonTestId="s20-pin-move-prev-reason"
              disabled={disabledAll || panelIndex === 0}
              onClick={() => doPinMove(-1)}
            >
              <span>{S20_PIN_PREV}</span>
            </ReasonButton>
          </HelpTip>
          <HelpTip helpKey="s20-pin-next">
            <ReasonButton
              className="sm"
              data-testid="s20-pin-move-next"
              reason={panelIndex === panelPins.length - 1 ? S20_REASON_LAST : null}
              reasonTestId="s20-pin-move-next-reason"
              disabled={disabledAll || panelIndex === panelPins.length - 1}
              onClick={() => doPinMove(1)}
            >
              <span>{S20_PIN_NEXT}</span>
            </ReasonButton>
          </HelpTip>
        </div>
      )}
      <div className="row mb">
        <input
          type="text"
          data-testid="s20-pin-edit-name"
          value={pinNameDraft}
          onChange={(e) => setPinNameDraft(e.target.value)}
          aria-label="ピンの名前"
        />
      </div>
      {reregMsg && <div className="note err mb" data-testid="s20-pin-rereg-msg">{reregMsg}</div>}
      <div className="btnrow n3">
        <HelpTip helpKey="s20-pin-rename">
          <button
            type="button"
            className="btn sm"
            data-testid="s20-pin-rename"
            onClick={doPinRename}
          >
            {S20_PIN_RENAME}
          </button>
        </HelpTip>
        <span data-testid="s20-pin-delete">
          <HelpTip helpKey="s20-pin-delete">
            <ArmedButton
              className="sm"
              idleLabel={S20_PIN_DELETE}
              armedLabel={S20_PIN_DELETE_ARMED}
              onConfirm={doPinDelete}
            />
          </HelpTip>
        </span>
        <button
          type="button"
          className="btn sm"
          data-testid="s20-pin-close"
          onClick={() => setSelectedPinId(null)}
        >
          {S20_PIN_CLOSE}
        </button>
      </div>
      <div className="mt">
        {method === 'person' ? (
          <HelpTip helpKey="s20-pin-reregister">
            <ReasonButton
              className="sm btn-register wide"
              data-testid="s20-pin-reregister"
              reason={S20_REASON_REREG_PERSON}
              reasonTestId="s20-pin-reregister-reason"
              disabled
            >
              <IconPin />
              <span>{S20_PIN_REREGISTER_PERSON}</span>
            </ReasonButton>
          </HelpTip>
        ) : method === 'tap' ? (
          <HelpTip helpKey="s20-pin-reregister">
            <button
              type="button"
              className="btn sm btn-register wide"
              data-testid="s20-pin-reregister-maptap"
              disabled={disabledAll || isRegister || pinWarn.active}
              onClick={() => {
                setReregTargetId(selectedPin.id)
                setSelectedPinId(null)
                setTapMode(true)
              }}
            >
              <IconPin />
              <span>{S20_PIN_REREGISTER_MAPTAP}</span>
            </button>
          </HelpTip>
        ) : (
          <HelpTip helpKey="s20-pin-reregister">
            <button
              type="button"
              className="btn sm btn-register wide"
              data-testid="s20-pin-reregister"
              disabled={disabledAll || !reregPose}
              title={!reregPose ? S20_PIN_REREGISTER_NO_POSE : undefined}
              onClick={doPinReregister}
            >
              <IconPin />
              <span>{S20_PIN_REREGISTER_HERE}</span>
            </button>
          </HelpTip>
        )}
        {!reregPose && method === 'pose' && (
          <div className="sm mut mt" data-testid="s20-pin-rereg-nopose">
            {S20_PIN_REREGISTER_NO_POSE}
          </div>
        )}
      </div>
    </div>
  ) : null

  const main = (
    <>
      <div className="mapWrap s20-mapwrap" onClick={(e) => {
        // 吹き出しの外を押すと閉じる（§2.3）。ピン・吹き出し自身のクリックは除く。
        if (!selectedPinId || tapMode) return
        if (e.target.closest('[data-testid="s20-pin-balloon"]')) return
        if (e.target.closest('[data-testid^="s20-map-pin-"]')) return
        setSelectedPinId(null)
      }}>
        {showMap ? (
          <>
            <OnsiteMap
              mapData={routeMap}
              pins={pins}
              robotPose={routePose}
              personPose={personPose}
              personLabel={S20_MAP_TARGET}
              costmapData={costmapData}
              plannedPath={plannedPath}
              scanPoints={scanPoints}
              selectedPinId={selectedPinId}
              onSelectPin={handleSelectPin}
              ariaLabel={S20_MAP_ARIA}
              noPoseLabel={S20_MAP_NO_POSE}
              robotLabel={S20_MAP_ROBOT}
              testId="s20"
              tapMode={tapMode}
              onTapConfirm={handleMapTapConfirm}
              rotateEnabled
              previewPose={pendingTap ? {
                x: tapDraft != null ? Number(tapDraft.x) : pendingTap.tap1.x,
                y: tapDraft != null ? Number(tapDraft.y) : pendingTap.tap1.y,
                yaw: tapDraft != null
                  ? (Number(tapDraft.deg) * Math.PI) / 180
                  : previewYawRad,
              } : null}
            />
            <div className="s20-radar">
              {trackerEnabled ? (
                <RadarSelect
                  candidates={personTargets.candidates}
                  selectedIndex={personTargets.selected_index}
                  isLost={personTargets.is_lost}
                  confidence={personTargets.confidence}
                  onSelect={handleSelectTarget}
                />
              ) : (
                <TrackerControl
                  enabled={false}
                  hasCandidate={false}
                  canStop={trackerCanStop}
                  stopDeniedReason={trackerStopDenied}
                  onStart={() => requestTracker(true)}
                  onStop={() => requestTracker(false)}
                  disabled={disabledAll}
                  testId="s20"
                />
              )}
            </div>
            <div className="s20-maptools-tl">
              <HelpTip helpKey="s20-order-start">
                <ReasonButton
                  className="sm"
                  data-testid="s20-order-start"
                  reason={panelPins.length < 2 ? S20_REASON_ORDER_PANELS : null}
                  reasonTestId="s20-order-start-reason"
                  disabled={disabledAll || ordering != null || panelPins.length < 2}
                  onClick={startOrdering}
                >
                  <span>{S20_ORDER_START}</span>
                </ReasonButton>
              </HelpTip>
            </div>
            {balloon}
          </>
        ) : entry}
      </div>
      <section className="s20-band" aria-label={S20_REGISTER_TITLE} data-testid="s20-register-band">
        {pinWarn.active ? (
          <div className="well" data-testid="s20-pinwarn">
            <div className="note">
              {S20_PINWARN_MSG(pinWarn.nearest_m?.toFixed(2),
                pinWarn.min_m?.toFixed(2))}
            </div>
            <div className="row">
              <button
                type="button"
                className="btn"
                data-testid="s20-pinwarn-place"
                onClick={() => handlePinWarnResolve('place')}
              >
                {S20_PINWARN_PLACE}
              </button>
              <button
                type="button"
                className="btn primary"
                data-testid="s20-pinwarn-retreat"
                onClick={() => handlePinWarnResolve('retreat')}
              >
                {S20_PINWARN_RETREAT}
              </button>
              <button
                type="button"
                className="btn"
                data-testid="s20-pinwarn-cancel"
                onClick={() => handlePinWarnResolve('cancel')}
              >
                {S20_PINWARN_CANCEL}
              </button>
            </div>
          </div>
        ) : ordering != null ? (
          <div className="well" data-testid="s20-order-band">
            <div className="row mb">
              <span className="b sm" data-testid="s20-order-count">
                {S20_ORDER_COUNT(ordering.length, panelPins.length)}
              </span>
              <span className="grow sm mut">
                {ordering.length < panelPins.length
                  ? S20_ORDER_NEXT(ordering.length + 1)
                  : S20_ORDER_DONE}
              </span>
            </div>
            <div className="btnrow n3">
              <HelpTip helpKey="s20-order-commit">
                <ReasonButton
                  className="btn primary sm"
                  data-testid="s20-order-commit"
                  reason={ordering.length === 0 ? S20_REASON_ORDER_PANELS : null}
                  reasonTestId="s20-order-commit-reason"
                  disabled={ordering.length === 0}
                  onClick={commitOrdering}
                >
                  <span>{S20_ORDER_COMMIT}</span>
                </ReasonButton>
              </HelpTip>
              <button
                type="button"
                className="btn sm"
                data-testid="s20-order-redo"
                disabled={ordering.length === 0}
                onClick={() => setOrdering([])}
              >
                {S20_ORDER_REDO}
              </button>
              <button
                type="button"
                className="btn sm"
                data-testid="s20-order-cancel"
                onClick={() => setOrdering(null)}
              >
                {S20_PIN_CANCEL}
              </button>
            </div>
          </div>
        ) : isRegister ? (
          <div className="well" data-testid="s20-wizard">
            <div className="row mb">
              <span className="s20-steps" aria-hidden="true">
                <span className="s20-step done" />
                <span className={`s20-step${wizStep === 2 ? ' cur' : ''}`} />
              </span>
              <span className="grow sm mut">{wizStep === 1 ? S20_WIZ_MSG_STEP1 : S20_WIZ_MSG_STEP2}</span>
            </div>
            {wizErr && <div className="note" data-testid="s20-wiz-err">{wizErr}</div>}
            {wizYaw != null ? (
              <div className="note" data-testid="s20-wiz-done">{S20_PIN_YAW(wizYaw)}</div>
            ) : (
              <button
                type="button"
                className="btn primary wide"
                data-testid="s20-wiz-press"
                disabled={disabledAll}
                onClick={handleWizPress}
              >
                {S20_WIZ_REGISTER}
              </button>
            )}
          </div>
        ) : tapMode && !pendingTap ? (
          <div className="well" data-testid="s20-tapmode">
            <div className="note mb">{S20_TAPMODE_MSG}</div>
            <button
              type="button"
              className="btn sm"
              data-testid="s20-tapmode-cancel"
              onClick={cancelTap}
            >
              {S20_PIN_CANCEL}
            </button>
          </div>
        ) : pendingTap ? (
          <div className="well" data-testid="s20-maptap-confirm-card">
            <div className="note" data-testid="s20-maptap-preview">
              {S20_MAPTAP_PREVIEW(pendingTap.tap1.x, pendingTap.tap1.y,
                Math.round((previewYawRad * 180) / Math.PI))}
            </div>
            {/* SG-C7: 数値で補正する入力欄（Spec-onsite.md §3.7）。 */}
            <div className="row mb" data-testid="s20-maptap-edit">
              <label className="sm">x[m]
                <input
                  type="number" step="0.01"
                  data-testid="s20-maptap-x"
                  value={tapDraft?.x ?? ''}
                  onChange={(e) => setTapDraft((d) => ({ ...d, x: e.target.value }))}
                />
              </label>
              <label className="sm">y[m]
                <input
                  type="number" step="0.01"
                  data-testid="s20-maptap-y"
                  value={tapDraft?.y ?? ''}
                  onChange={(e) => setTapDraft((d) => ({ ...d, y: e.target.value }))}
                />
              </label>
              <label className="sm">向き[°]
                <input
                  type="number" step="1"
                  data-testid="s20-maptap-yaw"
                  value={tapDraft?.deg ?? ''}
                  onChange={(e) => setTapDraft((d) => ({ ...d, deg: e.target.value }))}
                />
              </label>
            </div>
            <div className="row">
              <button
                type="button"
                className="btn primary"
                data-testid="s20-maptap-confirm"
                onClick={handleMapTapCommit}
              >
                {S20_MAPTAP_CONFIRM}
              </button>
              <button
                type="button"
                className="btn"
                data-testid="s20-maptap-cancel"
                onClick={cancelTap}
              >
                {S20_MAPTAP_CANCEL}
              </button>
            </div>
          </div>
        ) : (
          <>
            <div className="btnrow n3 mb" role="group" aria-label="登録の方式">
              {METHODS.map((m) => (
                <HelpTip key={m.id} helpKey="s20-method">
                  <button
                    type="button"
                    className={`btn sm${method === m.id ? ' on' : ''}`}
                    data-testid={`s20-method-${m.id}`}
                    aria-pressed={method === m.id}
                    disabled={regBusy}
                    onClick={() => setMethod(m.id)}
                  >
                    <span>{m.label}</span>
                  </button>
                </HelpTip>
              ))}
            </div>
            {method === 'person' && !trackerEnabled && (
              <div className="note mb" data-testid="s20-reg-tracker-off">{TRACKER_OFF}</div>
            )}
            <div className="btnrow n2">
              <HelpTip helpKey="s20-reg-home">
                <ReasonButton
                  className="sm btn-register"
                  data-testid="s20-reg-home"
                  reason={regReason}
                  reasonTestId="s20-reg-home-reason"
                  disabled={regDisabled}
                  onClick={() => handleRegister('HOME')}
                >
                  <IconPin />
                  <span>{S20_REG_HOME_SHORT}</span>
                </ReasonButton>
              </HelpTip>
              <HelpTip helpKey="s20-reg-panel">
                <ReasonButton
                  className="sm btn-register"
                  data-testid="s20-reg-panel"
                  reason={regReason}
                  reasonTestId="s20-reg-panel-reason"
                  disabled={regDisabled}
                  onClick={() => handleRegister('PANEL')}
                >
                  <IconPin />
                  <span>{S20_REG_PANEL_SHORT}</span>
                </ReasonButton>
              </HelpTip>
            </div>
          </>
        )}
        {hereMsg && (
          <div
            className={`note mt${hereMsg.ok ? '' : ' err'}`}
            data-testid="s20-reg-here-msg"
          >
            {hereMsg.text}
          </div>
        )}
      </section>
    </>
  )

  return (
    <div id="s20" className="screen">
      <RunLayout
        opbar={toolbar}
        main={main}
        where={(
          <div className="s20-where" data-testid="s20-pin-count">
            <span className="b">{S20_PIN_COUNT(pins.length)}</span>
          </div>
        )}
        toggle={(
          <div className="s20-toggle">
            <StopRunToggle
              testId="s20-stop-run"
              reserved={!returning}
              stopEnabled={returning}
              runEnabled={false}
              onStop={() => sendTrigger('ui.stop')}
              onRun={() => sendTrigger('ui.run')}
            />
            {!returning && <div className="sm mut s20-reserved-note">{S20_RESERVED_NOTE}</div>}
          </div>
        )}
        ops={(
          <div className="s20-ops">
            <HelpTip helpKey="save">
              <ReasonButton
                className="sm btn save wide"
                data-testid="s20-save"
                reason={saveReason}
                reasonTestId="s20-save-reason"
                disabled={saveDisabled}
                onClick={handleSave}
              >
                <span>{OP_LABELS.save}</span>
              </ReasonButton>
            </HelpTip>
            <HelpTip helpKey="s20-return-home">
              <ReasonButton
                className="sm wide mt"
                data-testid="s20-return-home"
                reason={!homePinExists ? S20_REASON_NO_HOME : null}
                reasonTestId="s20-return-home-reason"
                disabled={disabledAll || !homePinExists}
                onClick={handleReturnHome}
              >
                <span>{S20_RETURN_HOME_SHORT}</span>
              </ReasonButton>
            </HelpTip>
            {returnErr && (
              <div className="note err mt" role="alert" data-testid="s20-return-home-err">
                {returnErr}
              </div>
            )}
          </div>
        )}
        jog={(
          <JogConsole
            ros={ros}
            inZone
            keyboard
            disabled={disabledAll}
            onHeldChange={setStickHeld}
          />
        )}
      />
    </div>
  )
}
