// screens/S14Replay.jsx — S-14, 教示再生 (SCREEN_NAMES.S14 in i18n/screens.js).
// Spec-webui.md §3.7。型「走らせる」（RunLayout）。タブは無い。
//
// 入口は主表示の中央のカード（経路の一覧・順再生│逆再生・「この経路で進む」）。
// 主表示は OnsiteMap（地図・経路・始点・終点・機体。読み込み中は地図の上）。
// 右列 ① 確度の印／ずれ／終点、② 停止│再生、③ 完全グローバル・再生速度・
// 経路を選び直す、④ 手動（W-6）。地図の更新の切り替えは出さない（使えない間）。
//
// FSM（th_state/config/transitions.yaml）:
//   REPLAY: ROUTE_SEL --ui.route_select {id, reverse}--> LOCALIZE
//           LOCALIZE --(evt.localize_done)--> READY
//           READY --ui.run--> RUN、RUN --ui.stop--> PAUSE、PAUSE --ui.run--> RUN
//           RUN --(evt.arrived)--> PAUSE、PAUSE --ui.route_select--> LOCALIZE（選び直す）
//           * --ui.finish--> IDLE（終了）
import { useEffect, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useRouteCatalog } from '../ros/useRouteCatalog.js'
import { useRouteStatus } from '../ros/useRouteStatus.js'
import { useRoutePreview } from '../ros/useRoutePreview.js'
import { useOdomPose } from '../ros/useOdomPose.js'
import { useRoutePose } from '../ros/useRoutePose.js'
import { useRouteMap } from '../ros/useRouteMap.js'
import { useScan } from '../ros/useScan.js'
import { useReplaySpeedPublisher } from '../ros/useReplaySpeedPublisher.js'
import { useJogPanel } from '../shell/jogPanel.js'
import { RunLayout } from '../shell/Layouts.jsx'
import StopRunToggle from '../parts/StopRunToggle.jsx'
import ReasonButton from '../parts/ReasonButton.jsx'
import ResultMark from '../parts/ResultMark.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import AutoBrakeDevCard from '../parts/AutoBrakeDevCard.jsx'
import ReplaySpeedControl from '../parts/ReplaySpeedControl.jsx'
import { REPLAY_SPEED_RATIOS } from '../parts/replaySpeed.js'
import OnsiteMap from '../parts/OnsiteMap.jsx'
import { scanToPoints } from '../parts/scanToPoints.js'
import { baseToWorld } from '../mapGeometry.js'
import { IconClose } from '../parts/icons.jsx'
import './S14Replay.css'
import {
  S14_SELECT_TITLE, S14_EMPTY, S14_FWD, S14_REV, S14_PROCEED,
  S14_POSE_PAUSE, S14_POSE_PAUSE_RESUMABLE,
  S14_GLOBAL_BUTTON, S14_NO_MAP_GUIDE, S14_NO_MAP_TAG, S14_NO_MAP_LINE,
  S14_RESELECT, S14_RESELECT_REASON, S14_SELECT_REASON, S14_GLOBAL_REASON,
  S14_END, S14_START_LABEL,
  S14_MAP_LOADING, S14_QUALITY_SEARCHING, S14_POSE_LOCALIZE_TIMEOUT,
  S14_XTRACK_TITLE, S14_XTRACK_EMPTY, S14_XTRACK_VALUE,
} from '../i18n/screens.js'
import {
  localizeQualityView, localizeQualityText, replayRunGate, mapLoadingVisible,
} from './s14Localize.js'
import { noMapRoute } from './routeMapWarning.js'
import { OP_LABELS } from '../i18n/states.js'
import { S20_MAP_ARIA, S20_MAP_NO_POSE, S20_MAP_ROBOT } from '../i18n/screens.js'

// LOCALIZE が長引く = 地図の読み直しに失敗等（slam_toolbox 再起動で ~45s かかりうる）。
const LOCALIZE_TIMEOUT_MS = 60000

// 確度の印（§3.3.1。✓＝正常／!＝要確認／×＝だめ。モックアップの search は !）。
const QUALITY_MARK = {
  high: 'OK', low: 'WARN', failed: 'NG', off_route: 'NG', searching: 'WARN',
}

export default function S14Replay({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const routes = useRouteCatalog(ros)
  const routeStatus = useRouteStatus(ros)
  const routePreview = useRoutePreview(ros)
  const odomPose = useOdomPose(ros)
  const routePose = useRoutePose(ros)
  const routeMap = useRouteMap(ros)
  const scan = useScan(ros)
  const jogPanel = useJogPanel()
  const disabledAll = stale || state?.mode == null

  const [selectedId, setSelectedId] = useState(null)
  const [reverse, setReverse] = useState(false)
  const [localizeStuck, setLocalizeStuck] = useState(false)
  // 一時停止中だけ、選択カードをもう一度出す（「経路を選び直す」。T-REPLAY-11）。
  const [reselecting, setReselecting] = useState(false)
  const [speedRatio, setSpeedRatio] = useState(REPLAY_SPEED_RATIOS.mid)
  useReplaySpeedPublisher(ros, speedRatio)

  const stateName = state?.state ?? null
  const empty = routes.length === 0
  const selecting = stateName === 'ROUTE_SEL' || reselecting

  useEffect(() => {
    if (stateName !== 'PAUSE') setReselecting(false)
  }, [stateName])

  const qualityView = localizeQualityView(routeStatus?.localize_quality)
  const qualityText = localizeQualityText(qualityView.level)
  const showQuality = qualityView.visible
    && (stateName === 'LOCALIZE' || stateName === 'READY')
  const noMap = noMapRoute(routeStatus)
  const showNoMapGuide = noMap
    && (stateName === 'LOCALIZE' || stateName === 'READY')
  const showStuckNote = localizeStuck
    && qualityView.level !== 'searching'
    && qualityView.level !== 'failed'
    && qualityView.level !== 'off_route'
  const runGate = replayRunGate(stateName, routeStatus?.localize_quality)
  const mapLoading = mapLoadingVisible(
    stateName, routeStatus?.localize_quality, routeMap != null)

  useEffect(() => {
    if (stateName !== 'LOCALIZE') { setLocalizeStuck(false); return undefined }
    setLocalizeStuck(false)
    const t = setTimeout(() => setLocalizeStuck(true), LOCALIZE_TIMEOUT_MS)
    return () => clearTimeout(t)
  }, [stateName])

  async function handleFinish() {
    try {
      const res = await sendTrigger('ui.finish')
      if (res?.accepted && onFinish) onFinish()
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  async function handleProceed() {
    if (selectedId == null) return
    await sendTrigger('ui.route_select', { id: selectedId, reverse })
  }

  async function handleGlobalLocalize() {
    try {
      await sendTrigger('ui.localize_global')
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  // 地図の上で重ねるもの：経路・始点・終点・機体・スキャン（§3.7・§6.1）。
  const robotPose = routePose ?? odomPose
  let scanPoints = null
  if (robotPose && scan) {
    scanPoints = scanToPoints(scan).map(({ x, y }) => {
      const [wx, wy] = baseToWorld(x, y, robotPose)
      return { x: wx, y: wy }
    })
  }
  const previewPts = Array.isArray(routePreview) ? routePreview : []
  const routeEnds = previewPts.length > 0 ? {
    start: { x: previewPts[0].x, y: previewPts[0].y },
    end: { x: previewPts[previewPts.length - 1].x, y: previewPts[previewPts.length - 1].y },
    startLabel: S14_START_LABEL,
    endLabel: S14_END,
  } : null

  // 地図の画像が無い経路の印は、読み込んだ経路（/route/status の has_map）だけで分かる。
  // 一覧のほかの経路は機体が送っていないので作らない（値を出さない）。
  const currentNoMap = routeStatus?.has_map === false
    && routeStatus?.current?.id != null
  const selectedNoMap = currentNoMap && selectedId === routeStatus?.current?.id

  const arrived = routeStatus?.arrived === true
  const currentRouteName = routeStatus?.current?.name
    ?? routes.find((r) => r.id === selectedId)?.name
    ?? routes.find((r) => r.id === routeStatus?.current?.id)?.name
    ?? null

  const mapEl = (
    <div className="mapWrap" data-testid="s14-map-wrap">
      <OnsiteMap
        mapData={routeMap}
        pins={[]}
        robotPose={robotPose}
        plannedPath={previewPts}
        scanPoints={scanPoints}
        routeEnds={routeEnds}
        ariaLabel={S20_MAP_ARIA}
        noPoseLabel={S20_MAP_NO_POSE}
        robotLabel={S20_MAP_ROBOT}
        overlayText={mapLoading ? S14_MAP_LOADING : null}
        testId="s14"
      />
      {selecting && (
        <div className="card s14-select-card" data-testid="s14-select-card">
          <h3>{S14_SELECT_TITLE}</h3>
          {empty ? (
            <div className="note" data-testid="s14-empty">{S14_EMPTY}</div>
          ) : (
            <div className="rtl" data-testid="s14-route-list">
              {routes.map((r) => (
                <div
                  key={r.id}
                  role="radio"
                  aria-checked={selectedId === r.id}
                  tabIndex={0}
                  className={`rt${selectedId === r.id ? ' on' : ''}`}
                  data-testid={`s14-select-${r.id}`}
                  onClick={() => setSelectedId(r.id)}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter' || e.key === ' ') {
                      e.preventDefault()
                      setSelectedId(r.id)
                    }
                  }}
                >
                  <span>{r.name}</span>
                  {currentNoMap && r.id === routeStatus?.current?.id && (
                    <span className="tagx" data-testid="s14-route-no-map">{S14_NO_MAP_TAG}</span>
                  )}
                  <small>{`${r.length_m != null ? r.length_m.toFixed(2) : '--'} m`}</small>
                </div>
              ))}
            </div>
          )}
          {selectedNoMap && (
            <div className="note mt" data-testid="s14-select-no-map">{S14_NO_MAP_LINE}</div>
          )}
          <div className="row mt">
            <HelpTip helpKey="s14-dir">
              <button
                type="button"
                className={`btn sm${!reverse ? ' on' : ''}`}
                data-testid="s14-dir-fwd"
                aria-pressed={!reverse}
                disabled={disabledAll}
                onClick={() => setReverse(false)}
              >
                {S14_FWD}
              </button>
            </HelpTip>
            <HelpTip helpKey="s14-dir">
              <button
                type="button"
                className={`btn sm${reverse ? ' on' : ''}`}
                data-testid="s14-dir-rev"
                aria-pressed={reverse}
                disabled={disabledAll}
                onClick={() => setReverse(true)}
              >
                {S14_REV}
              </button>
            </HelpTip>
          </div>
          <HelpTip helpKey="s14-proceed">
            <button
              type="button"
              className="btn mt"
              data-testid="s14-proceed"
              disabled={disabledAll || empty || selectedId == null}
              onClick={handleProceed}
            >
              {S14_PROCEED}
            </button>
          </HelpTip>
        </div>
      )}
    </div>
  )

  const opbar = (
    <div className="top-actions">
      <HelpTip helpKey="finish">
        <button
          type="button"
          className="btn exit"
          data-testid="s14-finish"
          disabled={disabledAll}
          onClick={handleFinish}
        >
          <IconClose />
          <span>{OP_LABELS.finish}</span>
        </button>
      </HelpTip>
    </div>
  )

  const where = (
    <div className="s14-where" data-testid="s14-where">
      {showQuality && (
        <span className="mkline">
          <ResultMark result={QUALITY_MARK[qualityView.level]} testId="s14-quality-mark" />
          <span data-testid="s14-localize-quality">{qualityText}</span>
        </span>
      )}
      {showNoMapGuide && (
        <span className="note warn" role="alert" data-testid="s14-no-map">
          {S14_NO_MAP_GUIDE}
        </span>
      )}
      {showStuckNote && (
        <span className="note" data-testid="s14-localize-stuck">{S14_POSE_LOCALIZE_TIMEOUT}</span>
      )}
      {(stateName === 'RUN' || (stateName === 'PAUSE' && !arrived)) && (
        <>
          <span>{S14_XTRACK_TITLE}</span>
          <b data-testid="s14-xtrack">
            {routeStatus?.cross_track_m != null && routeStatus?.cross_track_max_m != null
              ? S14_XTRACK_VALUE(routeStatus.cross_track_m, routeStatus.cross_track_max_m)
              : S14_XTRACK_EMPTY}
          </b>
        </>
      )}
      {stateName === 'PAUSE' && arrived && (
        <span data-testid="s14-ended">
          {currentRouteName && <span>{currentRouteName}</span>}{' '}
          <b>{S14_END}</b>
        </span>
      )}
      {stateName === 'PAUSE' && arrived && (
        <span className="s14-guide" data-testid="s14-pause-guide">{S14_POSE_PAUSE}</span>
      )}
      {stateName === 'PAUSE' && !arrived && (
        <span className="s14-guide" data-testid="s14-pause-guide">{S14_POSE_PAUSE_RESUMABLE}</span>
      )}
    </div>
  )

  const runLabel = stateName === 'PAUSE' ? '再開する' : '再生する'
  const stopEnabled = !disabledAll && stateName === 'RUN'
  const runEnabled = !disabledAll && !selecting && !arrived
    && (stateName === 'READY' || stateName === 'PAUSE') && runGate.enabled
  // 終端（arrived）の理由は ① の「終点」＋終了の案内で示す。吹き出しを出すと
  // トグルが高くなって下の列が動くので、ここでは出さない。
  const runDenyReason = selecting
    ? S14_SELECT_REASON
    : (stateName === 'LOCALIZE' && !runGate.enabled ? runGate.reason : null)
  const toggle = (
    <StopRunToggle
      stopLabel="停止する"
      runLabel={runLabel}
      stopEnabled={stopEnabled}
      runEnabled={runEnabled}
      runDenyReason={runDenyReason}
      onStop={() => sendTrigger('ui.stop')}
      onRun={() => sendTrigger('ui.run')}
    />
  )

  const globalDisabled = disabledAll || stateName !== 'LOCALIZE'
    || qualityView.level === 'searching' || noMap
  const globalReason = stateName !== 'LOCALIZE'
    ? S14_GLOBAL_REASON
    : (noMap ? S14_NO_MAP_GUIDE : (qualityView.level === 'searching' ? S14_QUALITY_SEARCHING : null))
  const ops = (
    <>
      <HelpTip helpKey="s14-global">
        <ReasonButton
          className="wide"
          data-testid="s14-localize-global"
          disabled={globalDisabled}
          reason={globalReason}
          reasonTestId="s14-global-reason"
          onClick={handleGlobalLocalize}
        >
          <span>{S14_GLOBAL_BUTTON}</span>
        </ReasonButton>
      </HelpTip>
      <HelpTip helpKey="s14-speed">
        <ReplaySpeedControl
          value={speedRatio}
          onSelect={setSpeedRatio}
          disabled={disabledAll}
        />
      </HelpTip>
      <HelpTip helpKey="s14-reselect">
        <ReasonButton
          className="wide"
          data-testid="s14-reselect"
          disabled={disabledAll || stateName !== 'PAUSE' || reselecting}
          reason={stateName !== 'PAUSE' ? S14_RESELECT_REASON : null}
          reasonTestId="s14-reselect-reason"
          onClick={() => setReselecting(true)}
        >
          <span>{S14_RESELECT}</span>
        </ReasonButton>
      </HelpTip>
    </>
  )

  const jog = (
    <div className="btnrow n2">
      <HelpTip helpKey="manual">
        <ReasonButton
          className="manual"
          data-testid="s14-manual"
          disabled={disabledAll}
          onClick={() => jogPanel.open()}
        >
          <span>{OP_LABELS.manual}</span>
        </ReasonButton>
      </HelpTip>
    </div>
  )

  return (
    <>
      <RunLayout id="s14" opbar={opbar} main={mapEl} where={where} toggle={toggle} ops={ops} jog={jog} />
      {/* A-15: 開発モードで項目 auto_brake を選んでいるときだけ出る。 */}
      <AutoBrakeDevCard prefix="s14" />
    </>
  )
}
