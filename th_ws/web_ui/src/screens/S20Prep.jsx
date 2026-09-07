// screens/S20Prep.jsx — S-20, 試験準備 (SCREEN_NAMES.S20 in i18n/screens.js;
// brief-UI-S20 / WP-UI-06). Spec-webui.md §3.10 / mockup 957〜1054 行。
//
// FSM（th_state/config/transitions.yaml T-PREP-01〜15）:
//   MAPPING --ui.register{kind}(target_confident)--> REGISTER（2 点指示）
//           --ui.select_target{index}(candidate_exists)--> set_target
//           --ui.return_home(home_pin_exists)--> RETURN
//           --ui.stop--> PAUSE、PAUSE --ui.run--> $prev_sub
//           * --ui.save--> SAVED（+ commit_venue_map effect）
//           * --ui.finish--> IDLE、SAVED --ui.run--> MAPPING
// 2 点指示は /onsite/two_point を index 1→2 で呼ぶ（サーバが evt.register_ok を出して
// REGISTER を抜け、pin_registrar が place_pin でピンを永続化する。UI は待つだけ）。
// ピンは /onsite/pins（PinList、map フレーム）を地図タブと「ピン」サブタブに使い、
// /onsite/edit_pin で改名／削除する。
//
// デモ範囲外として出さない: LiDAR 平面認識（W-08）、「地図の修正」サブタブ（W-10、
// map_editor 未実装）、占有格子の完全描画（ピン＋現在位置のオーバーレイまでで可）。
// 地図フレームと odom フレームはマップ作成開始時に一致する（デモ視点）扱いとし、
// /odom（useOdomPose）をそのまま地図座標として描く。
//
// 終了は ui.finish を送るだけ。受理されれば FSM が IDLE になり main.jsx が S-01 に戻す。
// onTrigger を空関数にしない（全操作ボタンを e2e/s20-prep.spec.js で検証）。
import { useEffect, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useOnsitePins } from '../ros/useOnsitePins.js'
import { usePersonTargets } from '../ros/usePersonTargets.js'
import { useOnsiteService } from '../ros/useOnsiteService.js'
import { useOdomPose } from '../ros/useOdomPose.js'
import { useJogPanel } from '../shell/jogPanel.js'
import RadarSelect from '../parts/RadarSelect.jsx'
import OperationCard from '../shell/OperationCard.jsx'
import attributes from '../generated/attributes.json'
import { quatToYaw } from '../mapGeometry.js'
import { REJECT_REASONS } from '../i18n/reasons.js'
import { OP_LABELS, stateLabel } from '../i18n/states.js'
import {
  S20_MAP_ARIA, S20_MAP_NO_POSE, S20_MAP_ROBOT, S20_MAP_TITLE,
  S20_PIN_CANCEL, S20_PIN_DELETE, S20_PIN_EDIT, S20_PIN_RENAME,
  S20_PINS_TITLE, S20_PIN_YAW, S20_REG_HOME, S20_REGISTER_TITLE, S20_REG_PANEL,
  S20_RETURN_HOME, S20_SUBTAB_PINS, S20_SUBTAB_REGISTER,
  S20_TAB_MAP, S20_TAB_TARGET, S20_TARGET_HINT, S20_UNSAVED,
  S20_WIZ_MSG, S20_WIZ_REGISTER, S20_WIZ_STEP,
} from '../i18n/screens.js'

const VIEW_W = 340
const VIEW_H = 250
const CX = VIEW_W / 2
const CY = VIEW_H / 2
// 1 m = 24 px（SVG はデモ用の等方スケール。原点はビューの中心）。
const PX_PER_M = 24

function toSvg(x, y) {
  return [CX + x * PX_PER_M, CY - y * PX_PER_M]
}

function yawDeg(pin) {
  if (!pin?.pose?.orientation) return 0
  const deg = (quatToYaw(pin.pose.orientation) * 180) / Math.PI
  return (deg % 360 + 360) % 360
}

export default function S20Prep() {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const pins = useOnsitePins(ros)
  const personTargets = usePersonTargets(ros)
  const odomPose = useOdomPose(ros)
  const { twoPoint, editPin } = useOnsiteService()
  const jogPanel = useJogPanel()

  const disabledAll = stale || state?.mode == null

  // 左列タブ（地図 / 対象選択）と右列サブタブ（登録 / ピン）。
  const [tab, setTab] = useState('map')
  const [subtab, setSubtab] = useState('register')

  // ピン（地図タップと「ピン」サブタブで選択を連動させる）。
  const [selectedPinId, setSelectedPinId] = useState(null)
  const [editingPinId, setEditingPinId] = useState(null)
  const [pinNameDraft, setPinNameDraft] = useState('')

  // 2 点指示ウィザード。purpose は押した ui.register の kind（§2.3 の一致規則）。
  const [registerKind, setRegisterKind] = useState('HOME')
  const [wizStep, setWizStep] = useState(1)
  const [wizErr, setWizErr] = useState(null)
  const [wizYaw, setWizYaw] = useState(null)

  const stateName = state?.state ?? null
  const isRegister = stateName === 'REGISTER'
  const homePinExists = pins.some((p) => p.kind === 'HOME')
  const unsaved = Array.isArray(state?.unsaved) && state.unsaved.length > 0

  // REGISTER を離れたら（成功・拒否・ロストで MAPPING へ戻る）ウィザードを初期化。
  useEffect(() => {
    if (isRegister) return undefined
    setWizStep(1)
    setWizErr(null)
    setWizYaw(null)
    return undefined
  }, [isRegister])

  // 編集中のピンが消えたら（削除・再読込）エディタを閉じる。
  useEffect(() => {
    if (editingPinId != null && !pins.some((p) => p.id === editingPinId)) {
      setEditingPinId(null)
    }
  }, [pins, editingPinId])

  // ── 操作 ─────────────────────────────────────────────
  // 登録（MAPPING で受理されると FSM が REGISTER にする）。
  async function handleRegister(kind) {
    setRegisterKind(kind)
    await sendTrigger('ui.register', { kind })
  }

  // 2 点指示（index 1 → 2）。拒否されたら理由を出して Step 1 からやり直し（§2.3）。
  async function handleWizPress() {
    setWizErr(null)
    const res = await twoPoint({ purpose: registerKind, index: wizStep })
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

  // 1 ボタンで待機場所に戻す（guard: home_pin_exists。無いときは非活性）。
  async function handleReturnHome() {
    await sendTrigger('ui.return_home')
  }

  function handleFinish() {
    sendTrigger('ui.finish')
  }

  // 対象選択（RadarSelect のタップ）。
  function handleSelectTarget(index) {
    sendTrigger('ui.select_target', { index })
  }

  // ── ピン編集 ─────────────────────────────────────────
  function beginPinEdit(pin) {
    setEditingPinId(pin.id)
    setPinNameDraft(pin.name ?? '')
  }

  async function doPinRename() {
    if (editingPinId == null) return
    await editPin({ id: editingPinId, new_name: pinNameDraft, is_delete: false })
    setEditingPinId(null)
  }

  async function doPinDelete() {
    if (editingPinId == null) return
    await editPin({ id: editingPinId, new_name: '', is_delete: true })
    setEditingPinId(null)
  }

  const editingPin = pins.find((p) => p.id === editingPinId) ?? null

  return (
    <div className="screen two-col" id="s20">
      <div>
        <div className="top-actions sticky">
          <button
            type="button"
            className="btn sm"
            data-testid="s20-finish"
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
              data-testid="s20-tab-map"
              onClick={() => setTab('map')}
            >
              {S20_TAB_MAP}
            </button>
            <button
              type="button"
              role="tab"
              aria-selected={tab === 'target'}
              className={`tab ${tab === 'target' ? 'on' : ''}`}
              data-testid="s20-tab-target"
              onClick={() => setTab('target')}
            >
              {S20_TAB_TARGET}
            </button>
          </div>
          {unsaved && <span className="pill ng" data-testid="s20-unsaved">{S20_UNSAVED}</span>}
        </div>

        {tab === 'map' && (
          <div className="tabpane on">
            <div className="card">
              <h3>{S20_MAP_TITLE}</h3>
              <div className="mapWrap">
                <svg viewBox={`0 0 ${VIEW_W} ${VIEW_H}`} aria-label={S20_MAP_ARIA} data-testid="s20-map">
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
                        className={`s20-map-pin${sel ? ' sel' : ''}`}
                        transform={`translate(${px} ${py})`}
                        role="button"
                        tabIndex={0}
                        data-testid={`s20-map-pin-${pin.id}`}
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
                      className="s20-map-robot"
                      transform={`translate(${toSvg(odomPose.x, odomPose.y)[0]} ${toSvg(odomPose.x, odomPose.y)[1]}) rotate(${(odomPose.yaw * 180) / Math.PI})`}
                      data-testid="s20-map-robot"
                    >
                      <title>{S20_MAP_ROBOT}</title>
                      <circle r={9} fill="#43a047" stroke="#c8e6c9" strokeWidth={2} />
                      <line x1={0} y1={0} x2={14} y2={0} stroke="#c8e6c9" strokeWidth={3} strokeLinecap="round" />
                    </g>
                  ) : (
                    <text x={CX} y={CY} textAnchor="middle" fill="#9aa4b2" fontSize="11" data-testid="s20-map-no-pose">
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
            <div className="hint mt" data-testid="s20-target-hint">{S20_TARGET_HINT}</div>
          </div>
        )}
      </div>

      <div>
        <OperationCard
          mode={state?.mode}
          stateName={stateName}
          attributes={attributes}
          slots={{ stop: true, check: false, run: false, save: true, manual: true }}
          disabled={disabledAll}
          onTrigger={(trigger) => sendTrigger(trigger)}
          onManualClick={() => jogPanel.open()}
        />
        <div className="state" data-testid="s20-state">{stateLabel(stateName)}</div>

        <div className="subtabs">
          <div className="tabs" role="tablist">
            <button
              type="button"
              role="tab"
              aria-selected={subtab === 'register'}
              className={`tab ${subtab === 'register' ? 'on' : ''}`}
              data-testid="s20-subtab-register"
              onClick={() => setSubtab('register')}
            >
              {S20_SUBTAB_REGISTER}
            </button>
            <button
              type="button"
              role="tab"
              aria-selected={subtab === 'pins'}
              className={`tab ${subtab === 'pins' ? 'on' : ''}`}
              data-testid="s20-subtab-pins"
              onClick={() => setSubtab('pins')}
            >
              {S20_SUBTAB_PINS}
            </button>
          </div>

          {subtab === 'register' && (
            <div className="tabpane on">
              <div className="card">
                <h3>{S20_REGISTER_TITLE}</h3>
                <div className="btnrow n2 mb">
                  <button
                    type="button"
                    className="btn sm"
                    data-testid="s20-reg-home"
                    disabled={disabledAll}
                    onClick={() => handleRegister('HOME')}
                  >
                    {S20_REG_HOME}
                  </button>
                  <button
                    type="button"
                    className="btn sm"
                    data-testid="s20-reg-panel"
                    disabled={disabledAll}
                    onClick={() => handleRegister('PANEL')}
                  >
                    {S20_REG_PANEL}
                  </button>
                </div>
                {isRegister && (
                  <div className="well" data-testid="s20-wizard">
                    <div className="row mb">
                      <span className="b sm">{S20_WIZ_STEP(wizStep)}</span>
                      <span className="grow sm mut">{S20_WIZ_MSG}</span>
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
                )}
              </div>
            </div>
          )}

          {subtab === 'pins' && (
            <div className="tabpane on">
              <div className="card">
                <h3>{S20_PINS_TITLE}</h3>
                {pins.length === 0 ? (
                  <div className="note" data-testid="s20-pins-empty">{S20_PIN_EDIT}</div>
                ) : (
                  <div className="lst-scroll">
                    <table className="lst">
                      <tbody>
                        {pins.map((pin) => (
                          <tr
                            key={pin.id}
                            className={pin.id === selectedPinId ? 'sel' : ''}
                            data-testid="s20-pin-row"
                            onClick={() => setSelectedPinId(pin.id)}
                          >
                            <td className="sm" data-testid={`s20-pin-name-${pin.id}`}>
                              {pin.name ?? pin.id}
                            </td>
                            <td className="r mono xs">{S20_PIN_YAW(Math.round(yawDeg(pin)))}</td>
                            <td className="r">
                              <button
                                type="button"
                                className="btn sm"
                                data-testid={`s20-pin-edit-${pin.id}`}
                                onClick={(e) => { e.stopPropagation(); beginPinEdit(pin) }}
                              >
                                {S20_PIN_EDIT}
                              </button>
                            </td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                )}
                {editingPin && (
                  <div className="well mt" data-testid="s20-pin-editor">
                    <div className="row mb">
                      <input
                        type="text"
                        data-testid="s20-pin-edit-name"
                        value={pinNameDraft}
                        onChange={(e) => setPinNameDraft(e.target.value)}
                      />
                    </div>
                    <div className="btnrow n3">
                      <button
                        type="button"
                        className="btn sm"
                        data-testid="s20-pin-rename"
                        onClick={doPinRename}
                      >
                        {S20_PIN_RENAME}
                      </button>
                      <button
                        type="button"
                        className="btn sm danger"
                        data-testid="s20-pin-delete"
                        onClick={doPinDelete}
                      >
                        {S20_PIN_DELETE}
                      </button>
                      <button
                        type="button"
                        className="btn sm"
                        data-testid="s20-pin-cancel"
                        onClick={() => setEditingPinId(null)}
                      >
                        {S20_PIN_CANCEL}
                      </button>
                    </div>
                  </div>
                )}
              </div>
              <div className="card" data-testid="s20-return-home-card">
                <button
                  type="button"
                  className="btn wide"
                  data-testid="s20-return-home"
                  disabled={disabledAll || !homePinExists}
                  onClick={handleReturnHome}
                >
                  {S20_RETURN_HOME}
                </button>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}