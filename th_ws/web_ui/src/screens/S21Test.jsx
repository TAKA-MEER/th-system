// screens/S21Test.jsx — S-21, 試験 (Spec-webui.md §3.11。型「走らせる」。タブは無い)。
//
// 右列のボタンはどの状態でも同じ 9 個・同じ位置（終了・停止│走行・手動・中断・
// 待機場所・次の配電盤・呼び寄せ・測定機能へ・カメラ）。終了と人検出は左上の
// 操作バー。地図のピンを押すと吹き出しに「行き先にする」、押すとそのまま出発。
// 待機場所の宣言は無い（D58）。地図が出せない間は主表示の中央に「会場地図を開く」。
//
// FSM（th_state/config/transitions.yaml）:
//   C-15:      IDLE --ui.goto{kind:PANEL|HOME|SUMMON}--> *（goto_allowed）
//   T-PNAV/T-HNAV: PANEL_NAV / HOME_NAV（--ui.stop/run--> PAUSE/NAV、着地で
//               AT_PANEL / IDLE）
//   T-ATP-01/02（ui.working）: AT_PANEL IDLE_P <-> WORKING。「測定機能へ」が
//               停止命令を立て、「停止を解く」が W-4 の確認のあと解く。
//   T-ATP-05:   AT_PANEL(IDLE_P) --ui.goto--> $arg.kind
//   T-NAV-goto-reject: PANEL_NAV / SUMMON / HOME_NAV の全状態（PAUSE 含む）で ui.goto は拒否（SM-3.1.2-118）
//   T-SUM-01:   SUMMON --evt.two_point_done--> WAIT_CLEAR（2 点指示は
//               /onsite/two_point を purpose='SUMMON' index 1→2 で呼ぶ）
//   T-SUM-05:   SUMMON(WAIT_CLEAR) --ui.abort--> POINT
//   T-SUM-15:   SUMMON(POINT) --ui.select_target--> set_target
//   ui.save は出さない（地図更新は使えない間は出さない。§3.3 (c)）。
import { useEffect, useRef, useState } from 'react'
import { createPortal } from 'react-dom'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useParamsOverrides } from '../ros/useParamsOverrides.js'
import { useSetFlag, TRACKER_FLAG } from '../ros/useSetFlag.js'
import { useOnsitePins } from '../ros/useOnsitePins.js'
import { usePersonTargets } from '../ros/usePersonTargets.js'
import { usePersonStatus } from '../ros/usePersonStatus.js'
import { useScan } from '../ros/useScan.js'
import { useOnsiteService } from '../ros/useOnsiteService.js'
import { useMapSessionStatus } from '../ros/useMapSessionStatus.js'
import { resolveMapOpened } from '../ros/mapSessionState.js'
import { useWaitClearStatus } from '../ros/useWaitClearStatus.js'
import { useOnsiteMapView } from '../ros/useOnsiteMapView.js'
import { useOnsiteCostmap } from '../ros/useCostmap.js'
import { usePlannedPath } from '../ros/usePlannedPath.js'
import { useRoutePose } from '../ros/useRoutePose.js'
import { useJogPanel } from '../shell/jogPanel.js'
import { venueOpenAllowed } from '../modes/stopOnlyGuard.js'
import { useConfirmWindow } from '../shell/confirmWindow.js'
import { RunLayout } from '../shell/Layouts.jsx'
import StopRunToggle from '../parts/StopRunToggle.jsx'
import ReasonButton from '../parts/ReasonButton.jsx'
import PersonDetectToggle from '../parts/PersonDetectToggle.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import WindowShell from '../shell/WindowShell.jsx'
import AutoBrakeDevCard from '../parts/AutoBrakeDevCard.jsx'
import RadarSelect from '../parts/RadarSelect.jsx'
import OnsiteMap from '../parts/OnsiteMap.jsx'
import { scanToPoints } from '../parts/scanToPoints.js'
import { IconArrow, IconClose, IconStop } from '../parts/icons.jsx'
import attributes from '../generated/attributes.json'
import {
  NAV_MODES, nextPanelPin, onsiteReasons, abortAvailable,
  remainingToDest, runBlockedByVenue,
} from './onsiteSteps.js'
import { baseToWorld } from '../mapGeometry.js'
import { REJECT_REASONS, reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import { OP_LABELS } from '../i18n/states.js'
import {
  BADGE_JOG_DENIED, BADGE_NO_HOME_PIN, BADGE_NO_PANEL_PIN,
  BADGE_GOTO_PAUSED, BADGE_GOTO_MOVING, BADGE_VENUE_NOT_LOADED,
  S20_MAP_ARIA, S20_MAP_NO_POSE, S20_MAP_ROBOT, S20_MAP_TARGET,
  S20_WIZ_STEP, S20_PIN_YAW,
  S21_SELECT_HINT,
  S21_MAP_LOADING,
  S21_OPEN_VENUE_MAP, S21_OPEN_VENUE_HINT,
  S21_DEST_HOME, S21_DEST_NEXT_PANEL, S21_DEST_SUMMON_HERE,
  S21_PIN_GO, S21_MEASURE, S21_UNSTOP,
  S21_UNSTOP_TITLE, S21_UNSTOP_BODY, S21_UNSTOP_YES, S21_UNSTOP_NO,
  S21_MEASURE_AT_PANEL_ONLY, S21_UNSTOP_ADJUST,
  S21_ABORT_NONE, S21_RECORD, S21_NO_TARGET,
  S21_BLOCKED_TITLE, S21_BLOCKED_BODY, S21_BLOCKED_REROUTE, S21_BLOCKED_ABORT,
  S21_WHERE_REMAIN, S21_WHERE_AT_PANEL, S21_WHERE_TO, S21_SUMMON_POS,
  S21_WIZ_STEP1, S21_WIZ_STEP2,
  S21_WAIT_TITLE, S21_WAIT_DIST, S21_WAIT_CLEARING, S21_WAIT_CANCEL,
  S21_CAMERA, S21_CAMERA_NO_URL, S21_CAMERA_BAD_URL,
  TRACKER_OFF, TRACKER_STOP_DENIED,
} from '../i18n/screens.js'

// onsiteReasons() は画面ローカルの意味キーだけを返す。文言解決はここで行う。
// working_in_progress / wait_clear_active は guards.py に実在するキーなので
// その 2 つだけ reasons.js の文言をそのまま使う（新設しない）。
const ONSITE_BADGE_TEXT = {
  jog_denied: BADGE_JOG_DENIED,
  no_panel_pin: BADGE_NO_PANEL_PIN,
  no_home_pin: BADGE_NO_HOME_PIN,
  goto_paused: BADGE_GOTO_PAUSED,
  goto_moving: BADGE_GOTO_MOVING,
  venue_not_loaded: BADGE_VENUE_NOT_LOADED,
  working_in_progress: REJECT_REASONS.working_in_progress,
  wait_clear_active: REJECT_REASONS.wait_clear_active,
}

// SG-C8: 退避待ちバーの全体時間は画面に直書きしない（remaining_sec の初回値）。
function waitBarPct(wait, totalSec) {
  if (wait.verdict === 'OK') return 100
  if (wait.verdict === 'NOT_CLEAR') return 4
  if (!(totalSec > 0)) return 4
  const remain = Math.max(0, wait.remaining_sec)
  return Math.max(4, Math.min(96, 100 - (remain / totalSec) * 100))
}

const MOVING_STATES = ['NAV', 'ALIGN', 'BLOCKED']

export default function S21Test({ onExit }) {
  const { ros, state, stale } = useSystemState()
  // 1b-5 (SG-A6): 会場地図の読み直しは停止中だけ（Spec.md SD-9）。
  const mapOpAllowed = venueOpenAllowed(state, stale)
  const sendTrigger = useTrigger()
  const setFlag = useSetFlag()
  const [trackerErr, setTrackerErr] = useState(null)
  async function requestTracker(next) {
    setTrackerErr(null)
    try {
      const res = await setFlag(TRACKER_FLAG, next, 's21-target')
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
  const costmapData = useOnsiteCostmap(ros)
  const plannedPath = usePlannedPath(ros)
  const mapSession = useMapSessionStatus(ros)
  const wait = useWaitClearStatus(ros)
  const { twoPoint, selectPin, openVenueMap } = useOnsiteService()
  const jogPanel = useJogPanel()
  const confirmWindow = useConfirmWindow()

  const disabledAll = stale || state?.mode == null
  const mode = state?.mode ?? null
  const stateName = state?.state ?? null
  const isNavMode = NAV_MODES.includes(mode)
  const working = stateName === 'WORKING'
  const showWizard = mode === 'SUMMON' && stateName === 'POINT'
  const showWait = mode === 'SUMMON' && stateName === 'WAIT_CLEAR'
  const moving = isNavMode && MOVING_STATES.includes(stateName)

  const trackerEnabled = state?.tracker_enabled === true

  let personPose = null
  if (routePose && personStatus && personStatus.is_lost === false) {
    const [wx, wy] = baseToWorld(personStatus.position.x, personStatus.position.y, routePose)
    personPose = { x: wx, y: wy }
  }

  let scanPoints = null
  if (routePose && scan) {
    scanPoints = scanToPoints(scan).map(({ x, y }) => {
      const [wx, wy] = baseToWorld(x, y, routePose)
      return { x: wx, y: wy }
    })
  }

  // 吹き出し（ピン選択）と行き先（右列 ①の「どこへ」）。
  const [selectedPinId, setSelectedPinId] = useState(null)
  const [destPinId, setDestPinId] = useState(null)
  const [goErr, setGoErr] = useState(null)
  const [unstopOpen, setUnstopOpen] = useState(false)

  // 2 点指示ウィザード（SUMMON/POINT。purpose='SUMMON'）。
  const [wizStep, setWizStep] = useState(1)
  const [wizErr, setWizErr] = useState(null)
  const [wizYaw, setWizYaw] = useState(null)

  // SG-C8: 退避待ちバーの全体時間。
  const waitTotalRef = useRef(0)
  // 会場地図の読み直し（/map_session/open）の結果メッセージ。
  const [venueMsg, setVenueMsg] = useState(null)
  // reload は最大 45s+30s かかる間「読み込んでいます…」を出す。
  const [venueBusy, setVenueBusy] = useState(false)

  useEffect(() => {
    if (showWizard) return undefined
    setWizStep(1)
    setWizErr(null)
    setWizYaw(null)
    return undefined
  }, [showWizard])

  useEffect(() => {
    if (showWait && wait.remaining_sec > waitTotalRef.current) {
      waitTotalRef.current = wait.remaining_sec
    }
    if (!showWait) {
      waitTotalRef.current = 0
    }
  }, [showWait, wait.remaining_sec])

  // ── 操作 ─────────────────────────────────────────────
  function handleFinish() {
    sendTrigger('ui.finish')
    onExit?.()
  }

  // 他の操作を押したら W-6 を閉じる（W-6 を開いたまま別操作で移らない）。
  function fire(trigger, arg) {
    jogPanel.close()
    sendTrigger(trigger, arg)
  }

  // ピンの吹き出し「行き先にする」→ そのまま出発。
  async function handlePinGo(pin) {
    setGoErr(null)
    if (destMoveReason) {
      setGoErr(destMoveReason)
      return
    }
    if (pin.kind === 'HOME') {
      setDestPinId(pin.id)
      setSelectedPinId(null)
      fire('ui.goto', { kind: 'HOME' })
      return
    }
    const res = await selectPin({ panel_id: pin.id })
    if (res?.success) {
      setDestPinId(pin.id)
      setSelectedPinId(null)
      fire('ui.goto', { kind: 'PANEL' })
    } else {
      setGoErr(res?.message ?? null)
    }
  }

  // 「次の配電盤」→ 順番が次に来る盤へそのまま出発。
  async function handleNextPanel() {
    const next = nextPanelPin(pins, routePose)
    if (!next) return
    setGoErr(null)
    if (destMoveReason) {
      setGoErr(destMoveReason)
      return
    }
    const res = await selectPin({ panel_id: next.id })
    if (res?.success) {
      setDestPinId(next.id)
      fire('ui.goto', { kind: 'PANEL' })
    } else {
      setGoErr(res?.message ?? null)
    }
  }

  function gotoDest(kind) {
    setSelectedPinId(null)
    if (kind === 'HOME') {
      const home = pins.find((p) => p?.kind === 'HOME')
      if (home) setDestPinId(home.id)
    }
    fire('ui.goto', { kind })
  }

  function startSummon() {
    setDestPinId(null)
    fire('ui.goto', { kind: 'SUMMON' })
  }

  async function handleWizPress() {
    setWizErr(null)
    const res = await twoPoint({ purpose: 'SUMMON', index: wizStep })
    if (res?.accepted) {
      if (wizStep === 1) {
        setWizStep(2)
      } else {
        setWizYaw(Math.round(((res.yaw ?? 0) * 180) / Math.PI))
      }
      return
    }
    const key = res?.reject_reason_key
    setWizErr(key ? (REJECT_REASONS[key] ?? key) : null)
  }

  function cancelWait() {
    sendTrigger('ui.abort')
  }

  // 呼び寄せ対象（RadarSelect）。SUMMON/POINT のときだけ送る（T-SUM-15）。
  function handleSelectTarget(index) {
    if (mode !== 'SUMMON' || stateName !== 'POINT') return
    sendTrigger('ui.select_target', { index })
  }

  // S-21 の「カメラ」ボタン（Spec-onsite.md §7.2）。開く先は /params/get の
  // effective.camera_ui_url（S-50 の保存が次の起動に反映される）。
  const { getParams } = useParamsOverrides(ros)
  const [cameraUrl, setCameraUrl] = useState(null)
  useEffect(() => {
    let alive = true
    getParams(['camera_ui_url'])
      .then((payload) => {
        if (alive) setCameraUrl(payload?.effective?.camera_ui_url ?? null)
      })
      .catch(() => { if (alive) setCameraUrl(null) })
    return () => { alive = false }
  }, [getParams])
  const cameraUrlValid =
    typeof cameraUrl === 'string' && (cameraUrl.startsWith('http://') || cameraUrl.startsWith('https://'))
  const cameraUrlReason = cameraUrlValid
    ? null
    : (typeof cameraUrl === 'string' && cameraUrl.trim() !== '' ? S21_CAMERA_BAD_URL : S21_CAMERA_NO_URL)
  const cameraOpenable = mode === 'AT_PANEL' && !disabledAll && cameraUrlValid

  function openCamera() {
    if (!cameraOpenable) return
    window.open(cameraUrl, '_blank', 'noopener')
  }

  // 「測定機能へ」→ 本システムの停止命令（WORKING）。測定機能の画面は別系統の
  // 画面で開く（このボタンは機体への停止命令だけを送る。T-ATP-01）。
  function startMeasure() {
    fire('ui.working')
  }

  // 「停止を解く」→ W-4 の確認のあと停止命令を解く（T-ATP-02 → IDLE_P）。
  function openUnstop() {
    setUnstopOpen(true)
    confirmWindow.open()
  }
  function closeUnstop() {
    setUnstopOpen(false)
    confirmWindow.close()
  }
  function confirmUnstop() {
    closeUnstop()
    fire('ui.working')
  }

  // 保存した会場地図を開く（/map_session/open, slot:VENUE / mode:reload）。
  // 停止中以外は送らない（サーバ側も拒否する。二重の守り）。
  async function handleOpenVenueMap() {
    if (!mapOpAllowed) return
    setVenueMsg(null)
    setVenueBusy(true)
    const res = await openVenueMap()
    setVenueMsg({ ok: !!res?.success, text: res?.message ?? '' })
    setVenueBusy(false)
  }

  // ── 理由バッジ（画面だけで分かる押せない理由）──
  const venueUnloaded = mapSession?.slot === 'VENUE' && mapSession?.mode === 'UNLOADED'
  const reasons = onsiteReasons({
    mode, stateName, attributes, pins, working, venueUnloaded,
  })
  const runBlocked = runBlockedByVenue(mode, stateName, venueUnloaded)
  const manualReason = reasons.manual ? (ONSITE_BADGE_TEXT[reasons.manual] ?? reasons.manual) : null
  const destHomeReason = reasons.destHome ? (ONSITE_BADGE_TEXT[reasons.destHome] ?? reasons.destHome) : null
  const destNextPanelReason = reasons.destNextPanel
    ? (ONSITE_BADGE_TEXT[reasons.destNextPanel] ?? reasons.destNextPanel) : null
  const destSummonHereReason = reasons.destSummonHere
    ? (ONSITE_BADGE_TEXT[reasons.destSummonHere] ?? reasons.destSummonHere) : null
  const destMoveReason = reasons.destMove
    ? (ONSITE_BADGE_TEXT[reasons.destMove] ?? reasons.destMove) : null

  const mapOpened = resolveMapOpened(mapSession, venueMsg?.ok)

  // 停止│走行：走行中は停止、PAUSE からは再開（会場地図が無い間は押せない）。
  const stopEnabled = !disabledAll && moving
  const runEnabled = !disabledAll && isNavMode && stateName === 'PAUSE' && !runBlocked
  const runLabel = stateName === 'PAUSE' ? '再開する' : '走行する'

  // 中断：移動中・一時停止中・経路が塞がれた間・呼び寄せの位置指定中に効く。
  const abortOk = abortAvailable(mode, stateName)
  const abortDisabled = disabledAll || !abortOk

  // 右列 ①「どこへ・あとどれだけ」。
  const destPin = pins.find((p) => p.id === destPinId) ?? null
  const remain = remainingToDest({ plannedPath, robotPose: routePose, destPin })
  let whereText
  if (isNavMode && MOVING_STATES.includes(stateName)) {
    whereText = destPin ? S21_WHERE_TO(destPin.name || destPin.id) : S21_SUMMON_POS
  } else if (mode === 'AT_PANEL') {
    const cur = destPin ?? nextPanelPin(pins, routePose) ?? pins.find((p) => p?.kind === 'PANEL')
    whereText = cur ? S21_WHERE_AT_PANEL(cur.name || cur.id) : S21_SELECT_HINT
  } else if (mode === 'SUMMON') {
    whereText = S21_SUMMON_POS
  } else {
    whereText = mode === 'AT_HOME' ? S21_DEST_HOME : S21_SELECT_HINT
  }
  const whereRemain = (isNavMode && remain != null) ? S21_WHERE_REMAIN(remain) : null

  // 呼び寄せ：対象を選ばなければ記録しない（対象が見えないまま実行させない）。
  const targetSelected = (personTargets.selected_index ?? -1) >= 0
  const wizDisabled = disabledAll || !trackerEnabled || !targetSelected

  const selectedPin = pins.find((p) => p.id === selectedPinId) ?? null

  // ピン以外を押したら吹き出しを消す（取り消し）。
  function onMapBackgroundClick(e) {
    if (e.target.closest('[data-testid^="s21-map-pin-"]')) return
    if (e.target.closest('[data-testid="s21-pin-bubble"]')) return
    setSelectedPinId(null)
  }

  const opbar = (
    <div className="top-actions">
      <HelpTip helpKey="finish">
        <button
          type="button"
          className="btn exit"
          data-testid="s21-finish"
          disabled={disabledAll}
          onClick={handleFinish}
        >
          <IconClose />
          <span>{OP_LABELS.finish}</span>
        </button>
      </HelpTip>
      <PersonDetectToggle
        enabled={trackerEnabled}
        canToggle={!disabledAll && !(mode === 'SUMMON' && trackerEnabled)}
        onToggle={requestTracker}
        testId="s21-tracker"
      />
      {mode === 'SUMMON' && trackerEnabled && (
        <span className="note" data-testid="s21-tracker-stop-denied">{TRACKER_STOP_DENIED}</span>
      )}
      {trackerErr && (
        <div className="note" data-testid="s21-tracker-error">{trackerErr}</div>
      )}
    </div>
  )

  const main = (
    <>
      {!mapOpened && (
        <div className="card" data-testid="s21-map-gate">
          <div className="note" data-testid="s21-map-gate-msg">{S21_OPEN_VENUE_HINT}</div>
          {venueBusy ? (
            <div className="note" data-testid="s21-map-gate-busy">{S21_MAP_LOADING}</div>
          ) : (
            <HelpTip helpKey="venueOpen">
              <ReasonButton
                className="wide mt"
                data-testid="s21-map-gate-open"
                disabled={disabledAll || !mapOpAllowed}
                reason={(!mapOpAllowed && !disabledAll) ? S21_MAP_LOADING : null}
                reasonTestId="s21-map-gate-reason"
                onClick={() => handleOpenVenueMap()}
              >
                <IconArrow />
                <span>{S21_OPEN_VENUE_MAP}</span>
              </ReasonButton>
            </HelpTip>
          )}
          {venueMsg && !venueMsg.ok && (
            <div className="note mt err" data-testid="s21-open-venue-msg">{venueMsg.text}</div>
          )}
        </div>
      )}

      {mapOpened && (
        <div className="mapWrap" onClick={onMapBackgroundClick}>
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
            onSelectPin={setSelectedPinId}
            ariaLabel={S20_MAP_ARIA}
            noPoseLabel={S20_MAP_NO_POSE}
            robotLabel={S20_MAP_ROBOT}
            overlayText={venueBusy ? S21_MAP_LOADING : null}
            testId="s21"
            blocked={isNavMode && stateName === 'BLOCKED'}
          />
          {selectedPin && (
            <div className="card s21-pin-bubble" data-testid="s21-pin-bubble">
              <span className="b sm" data-testid="s21-pin-bubble-name">
                {selectedPin.name || selectedPin.id}
              </span>
              <HelpTip helpKey="pinGo">
                <ReasonButton
                  className="go"
                  data-testid="s21-pin-go"
                  disabled={disabledAll || !!destMoveReason}
                  reason={destMoveReason}
                  reasonTestId="s21-pin-go-reason"
                  onClick={() => handlePinGo(selectedPin)}
                >
                  <IconArrow />
                  <span>{S21_PIN_GO}</span>
                </ReasonButton>
              </HelpTip>
            </div>
          )}
          {goErr && (
            <div className="note err" data-testid="s21-go-err">{goErr}</div>
          )}
          {/* 呼び寄せ：主表示にレーダーを重ねる（§3.11）。手順の帯は地図の下。 */}
          {showWizard && trackerEnabled && (
            <div className="s21-radar-ovr">
              <RadarSelect
                candidates={personTargets.candidates}
                selectedIndex={personTargets.selected_index}
                isLost={personTargets.is_lost}
                confidence={personTargets.confidence}
                onSelect={handleSelectTarget}
              />
            </div>
          )}
        </div>
      )}

      {(showWizard || showWait) && (
        <div className="s21-summon-band" data-testid="s21-summon-band">
          {!trackerEnabled && (
            <div className="note" data-testid="s21-summon-tracker-off">{TRACKER_OFF}</div>
          )}
          {trackerEnabled && showWizard && (
            <div className="well" data-testid="s21-wizard">
                <div className="row mb">
                  <span className="b sm">{S20_WIZ_STEP(wizStep)}</span>
                  <span className="grow sm mut">{wizStep === 1 ? S21_WIZ_STEP1 : S21_WIZ_STEP2}</span>
                </div>
                {wizErr && <div className="note" data-testid="s21-wiz-err">{wizErr}</div>}
                {wizYaw != null ? (
                  <div className="note" data-testid="s21-wiz-done">{S20_PIN_YAW(wizYaw)}</div>
                ) : (
                  <HelpTip helpKey="record">
                    <ReasonButton
                      className="primary wide"
                      data-testid="s21-wiz-press"
                      disabled={wizDisabled}
                      reason={(!targetSelected && trackerEnabled) ? S21_NO_TARGET : null}
                      reasonTestId="s21-wiz-reason"
                      onClick={handleWizPress}
                    >
                      <span>{S21_RECORD}</span>
                    </ReasonButton>
                  </HelpTip>
                )}
              </div>
          )}
          {showWait && (
            <div className="well" data-testid="s21-clear-box">
              <div className="b sm mb">{S21_WAIT_TITLE}</div>
              <div className="row mb">
                <div className={`s21-bar grow ${wait.verdict === 'OK' ? 'ok' : (wait.verdict === 'NOT_CLEAR' ? 'nope' : '')}`} data-testid="s21-clear-bar">
                  <i style={{ width: `${waitBarPct(wait, waitTotalRef.current)}%` }} />
                </div>
                <span className="mono b" data-testid="s21-clear-dist">{S21_WAIT_DIST(wait.distance_m)}</span>
              </div>
              <div className="row">
                <span className="grow sm mut" data-testid="s21-clear-hint">{S21_WAIT_CLEARING}</span>
                <button
                  type="button"
                  className="btn sm"
                  data-testid="s21-wait-cancel"
                  disabled={disabledAll}
                  onClick={cancelWait}
                >
                  {S21_WAIT_CANCEL}
                </button>
              </div>
            </div>
          )}
        </div>
      )}

      {isNavMode && stateName === 'BLOCKED' && (
        <WindowShell
          className="blocked-win"
          testId="s21-blocked"
          title={S21_BLOCKED_TITLE}
          footer={<>
            <HelpTip helpKey="reroute">
              <button
                type="button"
                className="btn primary"
                data-testid="s21-blocked-reroute"
                onClick={() => fire('ui.reroute')}
              >
                {S21_BLOCKED_REROUTE}
              </button>
            </HelpTip>
            <HelpTip helpKey="abort">
              <button
                type="button"
                className="btn danger"
                data-testid="s21-blocked-abort"
                onClick={() => fire('ui.abort')}
              >
                {S21_BLOCKED_ABORT}
              </button>
            </HelpTip>
          </>}
        >
          <p data-testid="s21-blocked-msg">
            {destPin ? `${S21_WHERE_TO(destPin.name || destPin.id)}　` : ''}{S21_BLOCKED_BODY}
          </p>
        </WindowShell>
      )}
    </>
  )

  const where = (
    <div className="s21-where" data-testid="s21-where">
      <span>{whereText}</span>
      {whereRemain && <b data-testid="s21-where-remain">{whereRemain}</b>}
    </div>
  )

  const toggle = (
    <StopRunToggle
      stopLabel="停止する"
      runLabel={runLabel}
      stopEnabled={stopEnabled}
      runEnabled={runEnabled}
      runDenyReason={runDisabledReason()}
      onStop={() => fire('ui.stop')}
      onRun={() => fire('ui.run')}
      testId="stop-run-toggle"
    />
  )

  function runDisabledReason() {
    if (disabledAll || !isNavMode || stateName !== 'PAUSE') return null
    if (runBlocked) return BADGE_VENUE_NOT_LOADED
    return null
  }

  const nextPin = nextPanelPin(pins, routePose)
  const ops = (
    <>
      <HelpTip helpKey="nextPanel">
        <ReasonButton
          className={`${nextPin && !disabledAll && !destNextPanelReason ? "go go-paint wide" : "go wide"}`}
          data-testid="s21-dest-next-panel"
          disabled={disabledAll || !!destNextPanelReason || !nextPin}
          reason={destNextPanelReason}
          reasonTestId="reason-s21-dest-next-panel"
          onClick={handleNextPanel}
        >
          <IconArrow />
          <span>
            {S21_DEST_NEXT_PANEL}
            <small data-testid="s21-next-panel-name">{nextPin ? `　${nextPin.name || nextPin.id} へ` : ''}</small>
          </span>
        </ReasonButton>
      </HelpTip>
      <div className="btnrow n2">
        <HelpTip helpKey="destHome">
          <ReasonButton
            className="go"
            data-testid="s21-dest-home"
            disabled={disabledAll || !!destHomeReason}
            reason={destHomeReason}
            reasonTestId="reason-s21-dest-home"
            onClick={() => gotoDest('HOME')}
          >
            <IconArrow />
            <span>{S21_DEST_HOME}</span>
          </ReasonButton>
        </HelpTip>
        <HelpTip helpKey="summon">
          <ReasonButton
            className="go"
            data-testid="s21-dest-summon-here"
            disabled={disabledAll || !trackerEnabled || !!destSummonHereReason}
            reason={!trackerEnabled ? TRACKER_OFF : destSummonHereReason}
            reasonTestId="reason-s21-dest-summon-here"
            onClick={startSummon}
          >
            <IconArrow />
            <span>{S21_DEST_SUMMON_HERE}</span>
          </ReasonButton>
        </HelpTip>
      </div>
      <div className="btnrow n2">
        {working ? (
          <HelpTip helpKey="unstop">
            <ReasonButton
              className="stop-unstop"
              data-testid="s21-unstop"
              disabled={disabledAll || stateName === 'ADJUST'}
              reason={stateName === 'ADJUST' ? S21_UNSTOP_ADJUST : null}
              reasonTestId="s21-unstop-reason"
              onClick={openUnstop}
            >
              <IconStop />
              <span>{S21_UNSTOP}</span>
            </ReasonButton>
          </HelpTip>
        ) : (
          <HelpTip helpKey="measure">
            <ReasonButton
              className="measure"
              data-testid="s21-measure"
              disabled={disabledAll || mode !== 'AT_PANEL'}
              reason={mode !== 'AT_PANEL' ? S21_MEASURE_AT_PANEL_ONLY : null}
              reasonTestId="s21-measure-reason"
              onClick={startMeasure}
            >
              <IconArrow />
              <span>{S21_MEASURE}</span>
            </ReasonButton>
          </HelpTip>
        )}
        <HelpTip helpKey="camera">
          <ReasonButton
            className="camera"
            data-testid="s21-camera-open"
            disabled={!cameraOpenable}
            reason={(mode === 'AT_PANEL' && !cameraOpenable) ? cameraUrlReason : (mode !== 'AT_PANEL' ? S21_MEASURE_AT_PANEL_ONLY : null)}
            reasonTestId="s21-camera-reason"
            onClick={openCamera}
          >
            <IconArrow />
            <span>{S21_CAMERA}</span>
          </ReasonButton>
        </HelpTip>
      </div>
    </>
  )

  const jog = (
    <div className="btnrow n2">
      <HelpTip helpKey="manual">
        <ReasonButton
          className="manual"
          data-testid="s21-manual"
          disabled={disabledAll || !!manualReason}
          reason={manualReason}
          reasonTestId="reason-manual"
          onClick={() => jogPanel.open()}
        >
          <span>{OP_LABELS.manual}</span>
        </ReasonButton>
      </HelpTip>
      <HelpTip helpKey="abort">
        <ReasonButton
          className="danger abort"
          data-testid="s21-abort"
          disabled={abortDisabled}
          reason={!abortOk ? S21_ABORT_NONE : null}
          reasonTestId="s21-abort-reason"
          onClick={() => fire('ui.abort')}
        >
          <IconStop />
          <span>{S21_BLOCKED_ABORT}</span>
        </ReasonButton>
      </HelpTip>
    </div>
  )

  return (
    <>
      {/* 左上の操作バーは画面の左上に横 1 列（§3.3・§3.11）。RunLayout の opbar 枠は
          右列の先頭なので、ここでは骨組みの外に置き、枠には空を渡す。 */}
      <RunLayout id="s21" opbar={opbar} main={main} where={where} toggle={toggle} ops={ops} jog={jog} />
      {/* A-15: 開発モードで項目 auto_brake を選んでいるときだけ出る。 */}
      <AutoBrakeDevCard prefix="s21" />
      {unstopOpen && confirmWindow.isOpen && confirmWindow.mountNode && createPortal(
        <>
          <header>{S21_UNSTOP_TITLE}</header>
          <div className="bodyw"><p>{S21_UNSTOP_BODY}</p></div>
          <footer>
            <button type="button" className="btn" onClick={closeUnstop}>{S21_UNSTOP_NO}</button>
            <button type="button" className="btn primary" data-testid="s21-unstop-confirm" onClick={confirmUnstop}>{S21_UNSTOP_YES}</button>
          </footer>
        </>,
        confirmWindow.mountNode,
      )}
    </>
  )
}

