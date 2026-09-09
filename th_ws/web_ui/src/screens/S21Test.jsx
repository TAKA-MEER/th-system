// screens/S21Test.jsx — S-21, 試験 (SCREEN_NAMES.S21 in i18n/screens.js;
// brief-UI-S21 / WP-UI-07). Spec-webui.md §3.11 / mockup 1055〜1153 行。
//
// FSM（th_state/config/transitions.yaml）:
//   C-15:      IDLE --ui.goto{kind:PANEL|HOME|SUMMON}--> *（goto_allowed）
//   T-PNAV/T-HNAV: PANEL_NAV / HOME_NAV（--ui.stop/run--> PAUSE/NAV、着地で
//               AT_PANEL / IDLE）
//   T-ATP-01/02: AT_PANEL --ui.working<--> IDLE_P ⇄ WORKING
//   T-ATP-05:   AT_PANEL(IDLE_P/WORKING) --ui.goto--> $arg.kind
//   T-SUM-01:   SUMMON --evt.two_point_done--> WAIT_CLEAR（2 点指示は
//               /onsite/two_point を purpose='SUMMON' index 1→2 で呼ぶ）
//   T-SUM-05:   SUMMON(WAIT_CLEAR) --ui.abort--> POINT
//   T-SUM-15:   SUMMON(POINT) --ui.select_target--> set_target
//   ui.save は map_update=true のときだけ（地図更新 OFF に固定したので構造的に非表示）。
//
// 地図更新 OFF 固定（§2.2）: その有効化は W-10（map_editor）側の仕事なので、
// この画面では「地図の更新 OFF」を非活性で表示し、「保存」は出さない。
// 2 点指示ウィザード・地図／対象選択タブは S-20 の定数をそのまま流用する。
// 呼び寄せ先の選択は /person/targets を RadarSelect で表示し、
// SUMMON のときだけ ui.select_target を送る（FSM に定義されたモード限定）。
// 終了は ui.finish を送るだけ。受理されれば FSM が IDLE になり main.jsx が S-01 に戻す。
import { useEffect, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useOnsitePins } from '../ros/useOnsitePins.js'
import { usePersonTargets } from '../ros/usePersonTargets.js'
import { useOnsiteService } from '../ros/useOnsiteService.js'
import { useHomeDeclared } from '../ros/useHomeDeclared.js'
import { useWaitClearStatus } from '../ros/useWaitClearStatus.js'
import { useOnsiteMapView } from '../ros/useOnsiteMapView.js'
import { useRoutePose } from '../ros/useRoutePose.js'
import { useJogPanel } from '../shell/jogPanel.js'
import RadarSelect from '../parts/RadarSelect.jsx'
import OnsiteMap from '../parts/OnsiteMap.jsx'
import StepBar from '../parts/StepBar.jsx'
import {
  IconArrow, IconClose, IconPin, IconStop, OP_BUTTON_KINDS,
} from '../parts/icons.jsx'
import OperationCard from '../shell/OperationCard.jsx'
import attributes from '../generated/attributes.json'
import { NAV_MODES, testSteps, onsiteReasons } from './onsiteSteps.js'
import { REJECT_REASONS } from '../i18n/reasons.js'
import { OP_LABELS, stateLabel } from '../i18n/states.js'
import {
  BADGE_JOG_DENIED, BADGE_NO_HOME_PIN, BADGE_NO_PANEL_PIN,
  S20_MAP_ARIA, S20_MAP_NO_POSE, S20_MAP_ROBOT, S20_MAP_TITLE,
  S20_TAB_MAP, S20_TAB_TARGET,
  S20_WIZ_MSG, S20_WIZ_REGISTER, S20_WIZ_STEP, S20_PIN_YAW,
  S21_ATPANEL_TITLE, S21_DEST_HOME, S21_DEST_NEXT_PANEL, S21_DEST_SUMMON_HERE,
  S21_HOME_DECLARED, S21_HOME_DECLARE, S21_HOME_FORCE_DECLARE,
  S21_HOME_RETRY_LATER, S21_HOME_UNDECLARED, S21_MAP_GATE_MSG, S21_MAP_UPDATE, S21_MAP_UPDATE_NOTE, S21_MAP_UPDATE_OFF,
  S21_NEXT_DECLARE_HOME, S21_NEXT_OPEN_VENUE, S21_NEXT_PICK_DEST,
  S21_NEXT_STOP, S21_NEXT_SUMMON, S21_NEXT_WORK,
  S21_NEXT_DEST_TITLE, S21_OPEN_VENUE_DONE, S21_OPEN_VENUE_MAP,
  S21_PICK_PANEL_HINT, S21_PIN_MOVE, S21_PINS_EMPTY,
  S21_OPEN_VENUE_FAIL, S21_PREP_TITLE, S21_SELECT_HINT, S21_STEP_DEST, S21_STEP_HOME, S21_STEP_MOVE,
  S21_STEP_OPEN, S21_STEP_WORK, S21_SUBTAB_ATPANEL, S21_SUBTAB_DEST,
  S21_SUBTAB_SUMMON, S21_SUMMON_START, S21_SUMMON_TITLE,
  S21_WAIT_CANCEL, S21_WAIT_CLEARING, S21_WAIT_DIST, S21_WAIT_TITLE,
  S21_WORKING, S21_WORK_ON, S21_WORK_OFF,
} from '../i18n/screens.js'

// brief-onsite-ux-fix UX-6-a: onsiteReasons() は画面ローカルの意味キーだけを
// 返す（i18n/reasons.js を経由しない）。文言解決はここで行う。working_in_progress /
// wait_clear_active は guards.py に実在する reject_reason_key なので、その 2 つ
// だけ i18n/reasons.js の文言をそのまま使う（新設しない）。jog_denied /
// no_panel_pin / no_home_pin は画面専用なので i18n/screens.js の BADGE_* を使う。
const ONSITE_BADGE_TEXT = {
  jog_denied: BADGE_JOG_DENIED,
  no_panel_pin: BADGE_NO_PANEL_PIN,
  no_home_pin: BADGE_NO_HOME_PIN,
  working_in_progress: REJECT_REASONS.working_in_progress,
  wait_clear_active: REJECT_REASONS.wait_clear_active,
}

// brief-onsite-ux UX-1: testSteps() が返す段 id → 表示ラベル（i18n の定数）。
const S21_STEP_LABELS = {
  open: S21_STEP_OPEN,
  home: S21_STEP_HOME,
  dest: S21_STEP_DEST,
  move: S21_STEP_MOVE,
  work: S21_STEP_WORK,
}

// 退避待ちの想定タイムアウト。バー幅の計算にだけ使い、実挙動には関与しない
// （真の値は wait_clear_gate が持つ。残り時間の減少が「そのまま進捗」に見える係数）。
const WAIT_BAR_SEC = 15

// UX-2-a: 次操作ボタンにも形の種別（OP_BUTTON_KINDS）とアイコンを付ける。
const NEXT_ACTION_ICONS = {
  advance: <IconArrow />,
  register: <IconPin />,
  stop: <IconStop />,
}

function waitBarPct(wait) {
  if (wait.verdict === 'OK') return 100
  if (wait.verdict === 'NOT_CLEAR') return 4
  const remain = Math.max(0, wait.remaining_sec)
  return Math.max(4, Math.min(96, 100 - (remain / WAIT_BAR_SEC) * 100))
}

export default function S21Test({ onExit }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const pins = useOnsitePins(ros)
  const personTargets = usePersonTargets(ros)
  const routeMap = useOnsiteMapView(ros)
  const routePose = useRoutePose(ros)
  const homeDeclared = useHomeDeclared(ros)
  const wait = useWaitClearStatus(ros)
  const { twoPoint, selectPin, declareHome, openVenueMap } = useOnsiteService()
  const jogPanel = useJogPanel()

  const disabledAll = stale || state?.mode == null
  const mode = state?.mode ?? null
  const stateName = state?.state ?? null
  const isNavMode = NAV_MODES.includes(mode)
  const mapUpdate = !!state?.map_update
  const working = stateName === 'WORKING'
  const showWizard = mode === 'SUMMON' && stateName === 'POINT'
  const showWait = mode === 'SUMMON' && stateName === 'WAIT_CLEAR'

  // 左列タブ（地図 / 対象選択）と右列サブタブ（行き先 / 呼び寄せ / 配電盤前）。
  const [tab, setTab] = useState('map')
  const [subtab, setSubtab] = useState('dest')
  const [selectedPinId, setSelectedPinId] = useState(null)
  // 「次の配電盤」でピン一覧を促す。ピン行の「移動」で解除する。
  const [panelPick, setPanelPick] = useState(false)

  // 2 点指示ウィザード（SUMMON/POINT。purpose='SUMMON'）。
  const [wizStep, setWizStep] = useState(1)
  const [wizErr, setWizErr] = useState(null)
  const [wizYaw, setWizYaw] = useState(null)

  // 待機場所の宣言エラー（success=false のときのメッセージと再宣言）。
  const [homeErr, setHomeErr] = useState(null)
  // 会場地図の読み直し（/map_session/open）の結果メッセージ（success/message）。
  const [venueMsg, setVenueMsg] = useState(null)
  // ピン選択（select_pin）の失敗メッセージ。
  const [goErr, setGoErr] = useState(null)

  useEffect(() => {
    if (showWizard) return undefined
    setWizStep(1)
    setWizErr(null)
    setWizYaw(null)
    return undefined
  }, [showWizard])

  // ── 操作 ─────────────────────────────────────────────
  // brief-UI-S21-entry: 「終了」は ui.finish を送り、そのあと main.jsx 経由で
  // onsiteTestOpen を閉じてもらう（onExit）。閉じないと FSM が IDLE に戻った
  // あとも S-21 が出たままになる。
  function handleFinish() {
    sendTrigger('ui.finish')
    onExit?.()
  }

  // 行き先を選んで ui.goto（C-15: IDLE、T-ATP-05: AT_PANEL）。
  function gotoDest(kind) {
    setPanelPick(false)
    sendTrigger('ui.goto', { kind })
  }

  // 「次の配電盤」はピン一覧に返して「移動」を押させるだけ（§2.2 の解釈）。
  function pickNextPanel() {
    setSubtab('dest')
    setPanelPick(true)
  }

  // ピン行の「移動」: ゴールを保留（select_pin）→ 成功したら ui.goto{PANEL}。
  async function handleMove(pin) {
    setGoErr(null)
    const res = await selectPin({ panel_id: pin.name ?? pin.id })
    if (res?.success) {
      setPanelPick(false)
      sendTrigger('ui.goto', { kind: 'PANEL' })
    } else {
      setGoErr(res?.message ?? null)
    }
  }

  // その場で呼ぶ（ui.goto → SUMMON/POINT。2 点指示ウィザードへ）。
  function startSummon() {
    setSubtab('summon')
    sendTrigger('ui.goto', { kind: 'SUMMON' })
  }

  // 2 点指示（index 1 → 2）。拒否されたら理由を出して Step 1 からやり直し。
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

  // 退避待ちを中止（T-SUM-05: WAIT_CLEAR → POINT）。
  function cancelWait() {
    sendTrigger('ui.abort')
  }

  // 配電盤前での作業中トグル（T-ATP-01/02: IDLE_P ⇄ WORKING）。
  function toggleWorking() {
    sendTrigger('ui.working')
  }

  // 待機場所の宣言（home_declarer）。失敗時は force 再宣言を促す（§2.3）。
  async function handleDeclareHome(force) {
    setHomeErr(null)
    const res = await declareHome({ force })
    if (!res?.success) {
      setHomeErr(res?.message ?? null)
    }
  }

  // 保存した会場地図を開く（/map_session/open, slot:VENUE / mode:reload）。
  // 当日の手順の先頭（brief-onsite-fix E）。success / message を画面に出す。
  async function handleOpenVenueMap() {
    setVenueMsg(null)
    const res = await openVenueMap()
    setVenueMsg({ ok: !!res?.success, text: res?.message ?? '' })
  }

  // 呼び寄せ対象（RadarSelect）。SUMMON のときだけ set_target（T-SUM-15）。
  function handleSelectTarget(index) {
    if (mode !== 'SUMMON') return
    sendTrigger('ui.select_target', { index })
  }

  const stateText = isNavMode ? stateLabel(stateName) : S21_SELECT_HINT
  const pendingYaw = showWizard && wizYaw != null

  // UX-2-b/UX-2-d/UX-6-b: この画面から「押せない」ことが画面だけでも分かる理由のバッジ。
  const reasons = onsiteReasons({
    mode, stateName, attributes, pins, working,
  })
  const manualDenyReason = reasons.manual ? (ONSITE_BADGE_TEXT[reasons.manual] ?? reasons.manual) : null
  const destHomeReason = reasons.destHome ? (ONSITE_BADGE_TEXT[reasons.destHome] ?? reasons.destHome) : null
  const destNextPanelReason = reasons.destNextPanel
    ? (ONSITE_BADGE_TEXT[reasons.destNextPanel] ?? reasons.destNextPanel) : null
  const destSummonHereReason = reasons.destSummonHere
    ? (ONSITE_BADGE_TEXT[reasons.destSummonHere] ?? reasons.destSummonHere) : null

  // ── 手順バー（UX-1）と「次にやること」ボタン ──
  // 段 1「会場地図を開く」だけは /map_session/open の応答 success を画面ローカル
  // state（venueMsg.ok）で保持する（ROS 側に状態が無いため。UX-3 の報告対象）。
  // brief-onsite-ux2 F-6: この mapOpened を地図タブの表示ゲートにも使う
  // （「会場地図を開く」まで地図を出さない）。
  const mapOpened = venueMsg?.ok === true
  const { steps, currentIndex } = testSteps({
    homeDeclared,
    mapOpened,
    selectedPinId,
    mode,
    stateName,
  })
  const stepViews = steps.map((s) => ({ ...s, label: S21_STEP_LABELS[s.id] ?? s.id }))

  // 現在段の操作を 1 個だけ出す。押すと必要なサブタブへ自動で切り替わる。
  // 2 点指示ウィザード（POINT）と退避待ち（WAIT_CLEAR）は最優先で見せ、隠す。
  // kind は OP_BUTTON_KINDS に通す（進む=塗り、登録=枠線水色、停止=丸太枠赤）。
  const nextActions = {
    0: { kind: 'advance', label: S21_NEXT_OPEN_VENUE, run: () => handleOpenVenueMap() },
    1: { kind: 'register', label: S21_NEXT_DECLARE_HOME, run: () => { setSubtab('dest'); handleDeclareHome(false) } },
    2: { kind: 'advance', label: S21_NEXT_PICK_DEST, run: () => { setSubtab('dest'); setPanelPick(true) } },
    3: { kind: 'stop', label: S21_NEXT_STOP, run: () => sendTrigger('ui.stop') },
    // 段 5: 作業中（WORKING⇄IDLE_P のトグル）か、呼び寄せ（SUMMON ならサブタブへ誘導）。
    4: (working || mode === 'AT_PANEL')
      ? { kind: 'advance', label: S21_NEXT_WORK, run: toggleWorking }
      : { kind: 'advance', label: S21_NEXT_SUMMON, run: () => setSubtab('summon') },
  }
  const nextAction = (showWizard || showWait) ? null : (nextActions[currentIndex] ?? null)

  return (
    <div className="screen two-col" id="s21">
      <div className="stepbar-cell">
        <StepBar steps={stepViews} currentIndex={currentIndex} testId="s21" />
      </div>
      <div className="left-col">
        <div className="top-actions sticky">
          {/* brief-onsite-ux2 F-1: 終了は S-11/S-13/S-14 と同じ左上（top-actions
              sticky の先頭）。前回のブリーフで右列右下（.finish-row）に置いた
              のが誤りだったので統一する。 */}
          <button
            type="button"
            className={`btn sm ${OP_BUTTON_KINDS.finish}`}
            data-testid="s21-finish"
            disabled={disabledAll}
            onClick={handleFinish}
          >
            <IconClose />
            <span>{OP_LABELS.finish}</span>
          </button>
          <div className="tabs grow" style={{ margin: 0, border: 'none' }} role="tablist">
            <button
              type="button"
              role="tab"
              aria-selected={tab === 'map'}
              className={`tab ${tab === 'map' ? 'on' : ''}`}
              data-testid="s21-tab-map"
              onClick={() => setTab('map')}
            >
              {S20_TAB_MAP}
            </button>
            <button
              type="button"
              role="tab"
              aria-selected={tab === 'target'}
              className={`tab ${tab === 'target' ? 'on' : ''}`}
              data-testid="s21-tab-target"
              onClick={() => setTab('target')}
            >
              {S20_TAB_TARGET}
            </button>
          </div>
        </div>

        {tab === 'map' && !mapOpened && (
          <div className="tabpane on">
            <div className="card">
              <h3>{S20_MAP_TITLE}</h3>
              <div className="note" data-testid="s21-map-gate-msg">{S21_MAP_GATE_MSG}</div>
              <button
                type="button"
                className={`btn wide mt ${OP_BUTTON_KINDS.advance}`}
                data-testid="s21-map-gate-open"
                disabled={disabledAll}
                onClick={() => handleOpenVenueMap()}
              >
                <IconArrow />
                <span>{S21_OPEN_VENUE_MAP}</span>
              </button>
            </div>
          </div>
        )}

        {tab === 'map' && mapOpened && (
          <div className="tabpane on">
            <div className="card">
              <h3>{S20_MAP_TITLE}</h3>
              <div className="mapWrap">
                <OnsiteMap
                  mapData={routeMap}
                  pins={pins}
                  robotPose={routePose}
                  selectedPinId={selectedPinId}
                  onSelectPin={setSelectedPinId}
                  ariaLabel={S20_MAP_ARIA}
                  noPoseLabel={S20_MAP_NO_POSE}
                  robotLabel={S20_MAP_ROBOT}
                  testId="s21"
                />
              </div>
            </div>
          </div>
        )}

        {tab === 'target' && (
          <div className="tabpane on">
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

      <div>
        {nextAction && (
          <button
            type="button"
            className={`next-action ${OP_BUTTON_KINDS[nextAction.kind] ?? ''}`.trim()}
            data-testid="s21-next-action"
            disabled={disabledAll}
            onClick={nextAction.run}
          >
            {NEXT_ACTION_ICONS[nextAction.kind]}
            <span>{nextAction.label}</span>
          </button>
        )}
        <OperationCard
          mode={mode}
          stateName={stateName}
          attributes={attributes}
          // brief-onsite-ux-fix UX-6-c: 「次にやること」が保存のときは操作カード
          // 側の保存を出さない（S-21 の次操作に保存は現状無いが S-20 と揃えておく）。
          //
          // 2026-09-09 実機で確認: run を false で潰していたため、PANEL_NAV /
          // SUMMON / HOME_NAV でジョグ後に PAUSE に落ちると「走行」ボタンが
          // どこにも無く詰んでいた（T-PNAV-*/T-SUM-*/T-HNAV-* の PAUSE
          // --ui.run--> NAV が唯一の脱出路）。AT_PANEL / AT_HOME はリース満了で
          // 自動的に IDLE_P / IDLE_H へ戻る（override_common）ので run は要らず、
          // operationCardLayout の既定（attrs.run_state != null）が
          // モードごとに正しく出し分ける（S-20 と同じ「stop と同じ扱いにして
          // FSM に判断させる」流儀）。
          slots={{
            stop: true, check: false, save: mapUpdate && nextAction?.kind !== 'save', manual: true,
          }}
          disabled={disabledAll}
          manualDenyReason={manualDenyReason}
          onTrigger={(trigger) => sendTrigger(trigger)}
          onManualClick={() => jogPanel.open()}
        />
        {/* UX-5: 状態表示と「地図の更新 OFF」の印を 1 行に並べる。地図の更新は
            W-10 により構造的に常時 OFF なので、行を占める操作としては出さない。 */}
        <div className="row s21-state-row">
          <span className="state grow" data-testid="s21-state">{stateText}</span>
          <span className="pill" data-testid="s21-map-update-pill" title={S21_MAP_UPDATE_NOTE}>
            {S21_MAP_UPDATE} {S21_MAP_UPDATE_OFF}
          </span>
        </div>

        <div className="subtabs">
          <div className="tabs" role="tablist">
            <button
              type="button"
              role="tab"
              aria-selected={subtab === 'dest'}
              className={`tab ${subtab === 'dest' ? 'on' : ''}`}
              data-testid="s21-subtab-dest"
              onClick={() => setSubtab('dest')}
            >
              {S21_SUBTAB_DEST}
            </button>
            <button
              type="button"
              role="tab"
              aria-selected={subtab === 'summon'}
              className={`tab ${subtab === 'summon' ? 'on' : ''}`}
              data-testid="s21-subtab-summon"
              onClick={() => setSubtab('summon')}
            >
              {S21_SUBTAB_SUMMON}
            </button>
            <button
              type="button"
              role="tab"
              aria-selected={subtab === 'atp'}
              className={`tab ${subtab === 'atp' ? 'on' : ''}`}
              data-testid="s21-subtab-atp"
              onClick={() => setSubtab('atp')}
            >
              {S21_SUBTAB_ATPANEL}
            </button>
          </div>

          {subtab === 'dest' && (
            <div className="tabpane on">
              <div className="card">
                <h3>{S21_PREP_TITLE}</h3>
                <div className="row mb s21-prep-row">
                  <button
                    type="button"
                    className="btn sm grow"
                    data-testid="s21-open-venue-map"
                    disabled={disabledAll}
                    onClick={() => handleOpenVenueMap()}
                  >
                    {S21_OPEN_VENUE_MAP}
                  </button>
                  <span className={`pill ${homeDeclared ? 'ok' : 'ng'}`} data-testid="s21-home-pill">
                    {homeDeclared ? S21_HOME_DECLARED : S21_HOME_UNDECLARED}
                  </span>
                  <button
                    type="button"
                    className="btn sm btn-register"
                    data-testid="s21-home-declare"
                    disabled={disabledAll || homeDeclared}
                    onClick={() => handleDeclareHome(false)}
                  >
                    <IconPin />
                    <span>{S21_HOME_DECLARE}</span>
                  </button>
                </div>
                {venueMsg && (
                  <div className={`note mb${venueMsg.ok ? '' : ' err'}`} data-testid="s21-open-venue-msg">
                    {venueMsg.text || (venueMsg.ok ? S21_OPEN_VENUE_DONE : S21_OPEN_VENUE_FAIL)}
                  </div>
                )}
                {homeErr && (
                  <div className="note mb" data-testid="s21-home-err">
                    <div className="mb">{homeErr}</div>
                    <div className="btnrow n2">
                      <button
                        type="button"
                        className="btn sm"
                        data-testid="s21-home-force"
                        onClick={() => handleDeclareHome(true)}
                      >
                        {S21_HOME_FORCE_DECLARE}
                      </button>
                      <button
                        type="button"
                        className="btn sm"
                        data-testid="s21-home-err-close"
                        onClick={() => setHomeErr(null)}
                      >
                        {S21_HOME_RETRY_LATER}
                      </button>
                    </div>
                  </div>
                )}
                {pins.length === 0 ? (
                  <div className="note" data-testid="s21-pins-empty">{S21_PINS_EMPTY}</div>
                ) : (
                  <>
                    <div className="lst-scroll">
                      <table className="lst">
                        <tbody>
                          {pins.map((pin) => (
                            <tr
                              key={pin.id}
                              className={pin.id === selectedPinId ? 'sel' : ''}
                              data-testid="s21-pin-row"
                              onClick={() => setSelectedPinId(pin.id)}
                            >
                              <td className="sm" data-testid={`s21-pin-name-${pin.id}`}>
                                {pin.name ?? pin.id}
                              </td>
                              <td className="r">
                                <button
                                  type="button"
                                  className="btn sm"
                                  data-testid={`s21-pin-move-${pin.id}`}
                                  disabled={disabledAll}
                                  onClick={(e) => { e.stopPropagation(); handleMove(pin) }}
                                >
                                  {S21_PIN_MOVE}
                                </button>
                              </td>
                            </tr>
                          ))}
                        </tbody>
                      </table>
                    </div>
                    {panelPick && (
                      <div className="note mt" data-testid="s21-panel-pick-hint">{S21_PICK_PANEL_HINT}</div>
                    )}
                    {goErr && (
                      <div className="note mt" data-testid="s21-go-err">{goErr}</div>
                    )}
                  </>
                )}
              </div>

              <div className="card">
                <h3>{S21_NEXT_DEST_TITLE}</h3>
                <div className="btnrow n3">
                  <div>
                    <button
                      type="button"
                      className={`btn ${OP_BUTTON_KINDS.advance}`}
                      data-testid="s21-dest-home"
                      disabled={disabledAll || !!destHomeReason}
                      onClick={() => gotoDest('HOME')}
                    >
                      <IconArrow />
                      <span>{S21_DEST_HOME}</span>
                    </button>
                    {destHomeReason && (
                      <span className="reason-badge" data-testid="reason-s21-dest-home">
                        {destHomeReason}
                      </span>
                    )}
                  </div>
                  <div>
                    <button
                      type="button"
                      className={`btn ${OP_BUTTON_KINDS.advance}`}
                      data-testid="s21-dest-next-panel"
                      disabled={disabledAll || !!destNextPanelReason}
                      onClick={pickNextPanel}
                    >
                      <IconArrow />
                      <span>{S21_DEST_NEXT_PANEL}</span>
                    </button>
                    {destNextPanelReason && (
                      <span className="reason-badge" data-testid="reason-s21-dest-next-panel">
                        {destNextPanelReason}
                      </span>
                    )}
                  </div>
                  <div>
                    <button
                      type="button"
                      className={`btn ${OP_BUTTON_KINDS.advance}`}
                      data-testid="s21-dest-summon-here"
                      disabled={disabledAll || !!destSummonHereReason}
                      onClick={startSummon}
                    >
                      <IconArrow />
                      <span>{S21_DEST_SUMMON_HERE}</span>
                    </button>
                    {destSummonHereReason && (
                      <span className="reason-badge" data-testid="reason-s21-dest-summon-here">
                        {destSummonHereReason}
                      </span>
                    )}
                  </div>
                </div>
              </div>
            </div>
          )}

          {subtab === 'summon' && (
            <div className="tabpane on">
              <div className="card">
                <h3>{S21_SUMMON_TITLE}</h3>
                <button
                  type="button"
                  className="btn primary wide"
                  data-testid="s21-summon-start"
                  disabled={disabledAll}
                  onClick={startSummon}
                >
                  <IconArrow />
                  <span>{S21_SUMMON_START}</span>
                </button>

                {showWizard && (
                  <div className="well mt" data-testid="s21-wizard">
                    <div className="row mb">
                      <span className="b sm">{S20_WIZ_STEP(wizStep)}</span>
                      <span className="grow sm mut">{S20_WIZ_MSG}</span>
                    </div>
                    {wizErr && <div className="note" data-testid="s21-wiz-err">{wizErr}</div>}
                    {pendingYaw ? (
                      <div className="note" data-testid="s21-wiz-done">{S20_PIN_YAW(wizYaw)}</div>
                    ) : (
                      <button
                        type="button"
                        className="btn primary wide"
                        data-testid="s21-wiz-press"
                        disabled={disabledAll}
                        onClick={handleWizPress}
                      >
                        {S20_WIZ_REGISTER}
                      </button>
                    )}
                  </div>
                )}

                {showWait && (
                  <div className="well mt" data-testid="s21-clear-box">
                    <div className="b sm mb">{S21_WAIT_TITLE}</div>
                    <div className="row mb">
                      <div className={`s21-bar grow ${wait.verdict === 'OK' ? 'ok' : (wait.verdict === 'NOT_CLEAR' ? 'nope' : '')}`} data-testid="s21-clear-bar">
                        <i style={{ width: `${waitBarPct(wait)}%` }} />
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
            </div>
          )}

          {subtab === 'atp' && (
            <div className="tabpane on">
              <div className="card">
                <h3>{S21_ATPANEL_TITLE}</h3>
                <div className="row">
                  <span className="grow sm">{S21_WORKING}</span>
                  <button
                    type="button"
                    className="btn"
                    data-testid="s21-work-toggle"
                    disabled={disabledAll || mode !== 'AT_PANEL'}
                    onClick={toggleWorking}
                  >
                    {working ? S21_WORK_ON : S21_WORK_OFF}
                  </button>
                </div>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}