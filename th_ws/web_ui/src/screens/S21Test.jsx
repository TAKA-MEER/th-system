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
import { useOdomPose } from '../ros/useOdomPose.js'
import { useJogPanel } from '../shell/jogPanel.js'
import RadarSelect from '../parts/RadarSelect.jsx'
import OperationCard from '../shell/OperationCard.jsx'
import attributes from '../generated/attributes.json'
import { REJECT_REASONS } from '../i18n/reasons.js'
import { OP_LABELS, stateLabel } from '../i18n/states.js'
import {
  S20_MAP_ARIA, S20_MAP_NO_POSE, S20_MAP_ROBOT, S20_MAP_TITLE,
  S20_TAB_MAP, S20_TAB_TARGET,
  S20_WIZ_MSG, S20_WIZ_REGISTER, S20_WIZ_STEP, S20_PIN_YAW,
  S21_ATPANEL_TITLE, S21_DEST_HOME, S21_DEST_NEXT_PANEL, S21_DEST_SUMMON_HERE,
  S21_HOME_DECLARED, S21_HOME_DECLARE, S21_HOME_FORCE_DECLARE,
  S21_HOME_RETRY_LATER, S21_HOME_UNDECLARED, S21_MAP_UPDATE, S21_MAP_UPDATE_OFF,
  S21_NEXT_DEST_TITLE, S21_NOW_AT_HOME, S21_PICK_PANEL_HINT, S21_PIN_MOVE,
  S21_PREP_TITLE, S21_SELECT_HINT, S21_SUBTAB_ATPANEL, S21_SUBTAB_DEST,
  S21_SUBTAB_SUMMON, S21_SUMMON_START, S21_SUMMON_TITLE, S21_TARGET_HINT,
  S21_WAIT_CANCEL, S21_WAIT_CLEARING, S21_WAIT_DIST, S21_WAIT_TITLE,
  S21_WORKING, S21_WORK_ON, S21_WORK_OFF,
} from '../i18n/screens.js'

const VIEW_W = 340
const VIEW_H = 250
const CX = VIEW_W / 2
const CY = VIEW_H / 2
// 1 m = 24 px（S-20 と同じ。SVG はデモ用の等方スケール。原点はビューの中心）。
const PX_PER_M = 24
// 退避待ちの想定タイムアウト。バー幅の計算にだけ使い、実挙動には関与しない
// （真の値は wait_clear_gate が持つ。残り時間の減少が「そのまま進捗」に見える係数）。
const WAIT_BAR_SEC = 15

const NAV_MODES = ['SUMMON', 'PANEL_NAV', 'AT_PANEL', 'HOME_NAV']

function toSvg(x, y) {
  return [CX + x * PX_PER_M, CY - y * PX_PER_M]
}

function waitBarPct(wait) {
  if (wait.verdict === 'OK') return 100
  if (wait.verdict === 'NOT_CLEAR') return 4
  const remain = Math.max(0, wait.remaining_sec)
  return Math.max(4, Math.min(96, 100 - (remain / WAIT_BAR_SEC) * 100))
}

export default function S21Test() {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const pins = useOnsitePins(ros)
  const personTargets = usePersonTargets(ros)
  const odomPose = useOdomPose(ros)
  const homeDeclared = useHomeDeclared(ros)
  const wait = useWaitClearStatus(ros)
  const { twoPoint, selectPin, declareHome } = useOnsiteService()
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
  function handleFinish() {
    sendTrigger('ui.finish')
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

  // 呼び寄せ対象（RadarSelect）。SUMMON のときだけ set_target（T-SUM-15）。
  function handleSelectTarget(index) {
    if (mode !== 'SUMMON') return
    sendTrigger('ui.select_target', { index })
  }

  const stateText = isNavMode ? stateLabel(stateName) : S21_SELECT_HINT
  const pendingYaw = showWizard && wizYaw != null

  return (
    <div className="screen two-col" id="s21">
      <div>
        <div className="top-actions sticky">
          <button
            type="button"
            className="btn sm"
            data-testid="s21-finish"
            disabled={disabledAll}
            onClick={handleFinish}
          >
            {OP_LABELS.finish}
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
          {!isNavMode && (
            <span className="pill" data-testid="s21-select-hint">{S21_SELECT_HINT}</span>
          )}
        </div>

        {tab === 'map' && (
          <div className="tabpane on">
            <div className="card">
              <h3>{S20_MAP_TITLE}</h3>
              <div className="mapWrap">
                <svg viewBox={`0 0 ${VIEW_W} ${VIEW_H}`} aria-label={S20_MAP_ARIA} data-testid="s21-map">
                  <rect width={VIEW_W} height={VIEW_H} fill="#20242c" />
                  <path d="M30 20 h280 v210 h-280 z" fill="#3c424e" />
                  <path
                    d="M30 20 h280 v210 h-280 z" fill="none" stroke="#e8ecf2" strokeWidth="4"
                    strokeLinecap="round" opacity=".85"
                  />
                  {pins.map((pin) => {
                    const [px, py] = toSvg(pin.pose?.position?.x ?? 0, pin.pose?.position?.y ?? 0)
                    const sel = pin.id === selectedPinId
                    const home = pin.kind === 'HOME'
                    return (
                      <g
                        key={pin.id}
                        className={`s21-map-pin${sel ? ' sel' : ''}`}
                        transform={`translate(${px} ${py})`}
                        role="button"
                        tabIndex={0}
                        data-testid={`s21-map-pin-${pin.id}`}
                        onClick={() => setSelectedPinId(pin.id)}
                        onKeyDown={(e) => {
                          if (e.key === 'Enter' || e.key === ' ') {
                            e.preventDefault(); setSelectedPinId(pin.id)
                          }
                        }}
                      >
                        <circle r={10} fill={home ? '#2196f3' : '#e91e63'} stroke={sel ? '#ffd54f' : (home ? '#90caf9' : '#f8bbd0')} strokeWidth={2} />
                        <text y={26} textAnchor="middle" fontSize="10" fill={home ? '#90caf9' : '#f8bbd0'}>
                          {pin.name ?? pin.id}
                        </text>
                      </g>
                    )
                  })}
                  {odomPose ? (
                    <g
                      className="s21-map-robot"
                      transform={`translate(${toSvg(odomPose.x, odomPose.y)[0]} ${toSvg(odomPose.x, odomPose.y)[1]}) rotate(${(odomPose.yaw * 180) / Math.PI})`}
                      data-testid="s21-map-robot"
                    >
                      <title>{S20_MAP_ROBOT}</title>
                      <circle r={9} fill="#43a047" stroke="#c8e6c9" strokeWidth={2} />
                      <line x1={0} y1={0} x2={14} y2={0} stroke="#c8e6c9" strokeWidth={3} strokeLinecap="round" />
                    </g>
                  ) : (
                    <text x={CX} y={CY} textAnchor="middle" fill="#9aa4b2" fontSize="11" data-testid="s21-map-no-pose">
                      {S20_MAP_NO_POSE}
                    </text>
                  )}
                </svg>
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
            <div className="hint mt" data-testid="s21-target-hint">{S21_TARGET_HINT}</div>
          </div>
        )}
      </div>

      <div>
        <OperationCard
          mode={mode}
          stateName={stateName}
          attributes={attributes}
          slots={{ stop: true, check: false, run: false, save: mapUpdate, manual: true }}
          disabled={disabledAll}
          onTrigger={(trigger) => sendTrigger(trigger)}
          onManualClick={() => jogPanel.open()}
        />
        <div className="state" data-testid="s21-state">{stateText}</div>
        <div className="s21-map-row">
          <span className="grow sm">{S21_MAP_UPDATE}</span>
          <button
            type="button"
            className="btn sm"
            data-testid="s21-map-update"
            disabled
            title={S21_MAP_UPDATE_OFF}
          >
            {S21_MAP_UPDATE_OFF}
          </button>
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
                <div className="row mb">
                  <span className="grow sm">{S21_NOW_AT_HOME}</span>
                  <span className={`pill ${homeDeclared ? 'ok' : 'ng'}`} data-testid="s21-home-pill">
                    {homeDeclared ? S21_HOME_DECLARED : S21_HOME_UNDECLARED}
                  </span>
                  <button
                    type="button"
                    className="btn sm"
                    data-testid="s21-home-declare"
                    disabled={disabledAll || homeDeclared}
                    onClick={() => handleDeclareHome(false)}
                  >
                    {S21_HOME_DECLARE}
                  </button>
                </div>
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
                  <div className="note" data-testid="s21-pins-empty">{S21_PREP_TITLE}</div>
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
                  <button
                    type="button"
                    className="btn"
                    data-testid="s21-dest-home"
                    disabled={disabledAll}
                    onClick={() => gotoDest('HOME')}
                  >
                    {S21_DEST_HOME}
                  </button>
                  <button
                    type="button"
                    className="btn"
                    data-testid="s21-dest-next-panel"
                    disabled={disabledAll}
                    onClick={pickNextPanel}
                  >
                    {S21_DEST_NEXT_PANEL}
                  </button>
                  <button
                    type="button"
                    className="btn"
                    data-testid="s21-dest-summon-here"
                    disabled={disabledAll}
                    onClick={startSummon}
                  >
                    {S21_DEST_SUMMON_HERE}
                  </button>
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
                  {S21_SUMMON_START}
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