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
import { usePersonStatus } from '../ros/usePersonStatus.js'
import { useScan } from '../ros/useScan.js'
import { scanToPoints } from '../parts/scanToPoints.js'
import { useOnsiteService } from '../ros/useOnsiteService.js'
import { useOnsiteMapView } from '../ros/useOnsiteMapView.js'
import { useOnsiteCostmap } from '../ros/useCostmap.js'
import { usePlannedPath } from '../ros/usePlannedPath.js'
import { useRoutePose } from '../ros/useRoutePose.js'
import { useMappingActive } from '../ros/useMappingActive.js'
import { useStdTrigger } from '../ros/useStdTrigger.js'
import { useJogPanel } from '../shell/jogPanel.js'
import RadarSelect from '../parts/RadarSelect.jsx'
import OnsiteMap from '../parts/OnsiteMap.jsx'
import StepBar from '../parts/StepBar.jsx'
import ArmedButton from '../parts/ArmedButton.jsx'
import {
  IconArrow, IconClose, IconPin, IconSave, IconStop, OP_BUTTON_KINDS,
} from '../parts/icons.jsx'
import OperationCard from '../shell/OperationCard.jsx'
import attributes from '../generated/attributes.json'
import { prepSteps, onsiteReasons } from './onsiteSteps.js'
import { baseToWorld, quatToYaw } from '../mapGeometry.js'
import { REJECT_REASONS } from '../i18n/reasons.js'
import { OP_LABELS, stateLabel } from '../i18n/states.js'
import {
  BADGE_JOG_DENIED,
  S20_MAP_ARIA, S20_MAP_GATE_BUTTON, S20_MAP_GATE_MSG, S20_MAP_NO_POSE, S20_MAP_ROBOT, S20_MAP_TARGET, S20_MAP_TITLE,
  S20_NEXT_REG_HOME, S20_NEXT_REG_PANEL, S20_NEXT_SAVE, S20_NEXT_SELECT_TARGET, S20_NEXT_START_MAPPING,
  S20_PIN_CANCEL, S20_PIN_DELETE, S20_PIN_EDIT, S20_PIN_RENAME,
  S20_PINS_TITLE, S20_PIN_YAW, S20_REG_HOME, S20_REG_HOME_HERE, S20_REG_HERE_NOTE,
  S20_REG_HERE_OK, S20_REGISTER_TITLE, S20_REG_PANEL, S20_REG_PANEL_HERE,
  S20_RESET_ARMED, S20_RESET_BANNER, S20_RESET_BUSY, S20_RESET_FAIL, S20_RESET_IDLE,
  S20_RETURN_HOME, S20_STEP_HOME, S20_STEP_MAP, S20_STEP_PANEL, S20_STEP_SAVE, S20_STEP_TARGET,
  S20_SUBTAB_PINS, S20_SUBTAB_REGISTER,
  S20_TAB_MAP, S20_TAB_TARGET, S20_UNSAVED,
  S20_WIZ_MSG_STEP1, S20_WIZ_MSG_STEP2, S20_WIZ_REGISTER, S20_WIZ_STEP,
} from '../i18n/screens.js'

// brief-onsite-ux UX-1: prepSteps() が返す段 id → 表示ラベル（i18n の定数）。
const S20_STEP_LABELS = {
  map: S20_STEP_MAP,
  target: S20_STEP_TARGET,
  home: S20_STEP_HOME,
  panel: S20_STEP_PANEL,
  save: S20_STEP_SAVE,
}

// UX-2-a: 次操作ボタンにも形の種別（OP_BUTTON_KINDS）とアイコンを付ける。
const NEXT_ACTION_ICONS = {
  advance: <IconArrow />,
  register: <IconPin />,
  save: <IconSave />,
  stop: <IconStop />,
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
  const personStatus = usePersonStatus(ros)
  const scan = useScan(ros)
  const routeMap = useOnsiteMapView(ros)
  const routePose = useRoutePose(ros)
  // brief-MAP-COSTMAP: 地図タブに Nav2 の costmap と直近の経路を重ねる。
  const costmapData = useOnsiteCostmap(ros)
  const plannedPath = usePlannedPath(ros)
  const { twoPoint, editPin, registerPinHere } = useOnsiteService()
  const jogPanel = useJogPanel()
  // brief-onsite-ux2 F-6: 地図タブの表示ゲート。/slam_control/mapping_active
  // は起動コマンド（enable_route_slam:=true）により起動直後から true のことが
  // 多いので、「地図作成開始」を押したときは
  //   mappingActive === false なら toggle_mapping を呼んで開始する
  //   mappingActive === true（起動時から動いている通常ケース）なら
  //     サービスは呼ばず表示だけ解禁する（呼ぶと動いている地図作成を止めてしまう）
  // mappingActive が null（初回メッセージ未受信）の間は、この判定ができず
  // 誤って停止させる恐れがあるためボタンを非活性にする。
  const mappingActive = useMappingActive(ros)
  const toggleMapping = useStdTrigger('/slam_control/toggle_mapping')
  const [mapUnlocked, setMapUnlocked] = useState(false)
  // WS-9Y: 前回セッションの地図・ピンをまとめて消す「新しい試験日として開始」。
  // discard_map（slam_toolbox 再起動）→ reset_pins（pins.yaml を空にする）の順。
  const discardMap = useStdTrigger('/slam_control/discard_map')
  const resetPins = useStdTrigger('/onsite/reset_pins')
  const [resetBusy, setResetBusy] = useState(false)
  const [resetErr, setResetErr] = useState(null)

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
  // 機体姿勢での登録（REG-2）の結果メッセージ（success/message を venueMsg と同じ
  // パターンで出す。2 点指示ウィザードとは独立）。
  const [hereMsg, setHereMsg] = useState(null)

  const stateName = state?.state ?? null
  const isRegister = stateName === 'REGISTER'
  const homePinExists = pins.some((p) => p.kind === 'HOME')
  const unsaved = Array.isArray(state?.unsaved) && state.unsaved.length > 0

  // MAP-1: 追従対象者を地図上に表示。/person/status は base_link 相対なので
  // baseToWorld() で map 座標に変換する（routePose は map フレーム）。is_lost の
  // ときや routePose が未取得のときは変換できないので null（マーカーを出さない）。
  let personPose = null
  if (routePose && personStatus && personStatus.is_lost === false) {
    const [wx, wy] = baseToWorld(personStatus.position.x, personStatus.position.y, routePose)
    personPose = { x: wx, y: wy }
  }

  // MAP-SCAN: LiDAR の生スキャン（/scan_filtered、base_link 相対）を map 座標へ。
  // routePose が未取得のときは変換できないので null（personPose と同じガード）。
  let scanPoints = null
  if (routePose && scan) {
    scanPoints = scanToPoints(scan).map(({ x, y }) => {
      const [wx, wy] = baseToWorld(x, y, routePose)
      return { x: wx, y: wy }
    })
  }

  // ── 手順バー（UX-1）と「次にやること」ボタン ──
  // brief-onsite-ux2 F-6: 段 1（地図を作る）の完了条件は mapUnlocked（表示解禁）
  // に変えた（以前は mode==='PREP' で常に完了扱い）。現在段＝未完了の最初の段。
  // 全部完了なら最後の段。
  const { steps, currentIndex } = prepSteps({
    pins, personTargets, unsaved: state?.unsaved, mapRevealed: mapUnlocked,
  })
  const stepViews = steps.map((s) => ({ ...s, label: S20_STEP_LABELS[s.id] ?? s.id }))

  // 現在段に対応する「1 個で済む」操作。押すと必要なタブへ自動で切り替わる。
  // 2 点指示ウィザードが開いている（REGISTER）ときはウィザード優先で隠す。
  // kind は OP_BUTTON_KINDS に通す（進む=塗り、登録・保存=枠線）。
  const nextActions = {
    0: { kind: 'advance', label: S20_NEXT_START_MAPPING, run: () => { setTab('map'); handleStartMapping() } },
    1: { kind: 'advance', label: S20_NEXT_SELECT_TARGET, run: () => setTab('target') },
    2: { kind: 'register', label: S20_NEXT_REG_HOME, run: () => { setSubtab('register'); handleRegister('HOME') } },
    3: { kind: 'register', label: S20_NEXT_REG_PANEL, run: () => { setSubtab('register'); handleRegister('PANEL') } },
    4: { kind: 'save', label: S20_NEXT_SAVE, run: () => sendTrigger('ui.save') },
  }
  const nextAction = isRegister ? null : (nextActions[currentIndex] ?? null)

  // UX-2-b/UX-2-d: この画面から「押せない」ことが画面だけでも分かる理由のバッジ。
  // brief-onsite-ux-fix UX-6-a: onsiteReasons() は i18n/reasons.js を経由しない
  // （この画面で出うるのは jog_denied だけなので i18n/screens.js の BADGE_* を直接使う）。
  const reasons = onsiteReasons({ mode: state?.mode, attributes })
  const manualDenyReason = reasons.manual === 'jog_denied' ? BADGE_JOG_DENIED : (reasons.manual ?? null)

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
  // brief-onsite-ux2 F-6:「地図作成開始」。mappingActive===false のときだけ
  // toggle_mapping を呼ぶ。起動時から動いている通常ケース（true）はサービスを
  // 呼ばず表示だけ解禁する（呼ぶと動いている地図作成を止めてしまう）。
  async function handleStartMapping() {
    if (mappingActive === false) {
      const res = await toggleMapping()
      if (res?.success === false) return   // 失敗時は解禁しない（留まる）
    }
    setMapUnlocked(true)
  }

  // WS-9Y: 「新しい試験日として開始」。discard_map（slam_toolbox 再起動、respawn は
  // 待たない）→ reset_pins（pins.yaml を空にする）の順に呼ぶ。ArmedButton なので
  // ここに来る時点で操作者の二段階確認は済んでいる。
  async function handleResetVenue() {
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

  // 登録（MAPPING で受理されると FSM が REGISTER にする）。
  async function handleRegister(kind) {
    setRegisterKind(kind)
    await sendTrigger('ui.register', { kind })
  }

  // REG-2: 機体姿勢での直接登録（/onsite/register_pin, method=ROBOT_POSE）。
  // 2 点指示ウィザードを開始/変更せず、応答の success/message を出すだけ。
  async function handleRegisterHere(kind) {
    setHereMsg(null)
    const res = await registerPinHere(kind)
    setHereMsg({ ok: !!res?.success, text: res?.message || (res?.success ? S20_REG_HERE_OK(kind) : '') })
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
      <div className="stepbar-cell">
        <StepBar steps={stepViews} currentIndex={currentIndex} testId="s20" />
      </div>
      <div className="left-col">
        <div className="top-actions sticky">
          {/* brief-onsite-ux2 F-1: 終了は S-11/S-13/S-14 と同じ左上（top-actions
              sticky の先頭）。前回のブリーフで右列右下（.finish-row）に置いた
              のが誤りだったので統一する。 */}
          <button
            type="button"
            className={`btn sm ${OP_BUTTON_KINDS.finish}`}
            data-testid="s20-finish"
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

        {tab === 'map' && !mapUnlocked && (
          <div className="tabpane on">
            <div className="card">
              <h3>{S20_MAP_TITLE}</h3>
              <div className="note" data-testid="s20-map-gate-msg">{S20_MAP_GATE_MSG}</div>
              <button
                type="button"
                className={`btn wide mt ${OP_BUTTON_KINDS.advance}`}
                data-testid="s20-map-gate-start"
                disabled={disabledAll || mappingActive == null || resetBusy}
                onClick={handleStartMapping}
              >
                <IconArrow />
                <span>{S20_MAP_GATE_BUTTON}</span>
              </button>
              {/* WS-9Y: 前回セッションのピンが残っているときだけ、まとめて消す
                  経路を出す。自動では消さない（Spec-onsite §4.0.1「判定は
                  自動化しない」と同じ考え方）。 */}
              {pins.length > 0 && (
                <div className="well mt" data-testid="s20-reset-venue">
                  {resetBusy ? (
                    <div className="note" data-testid="s20-reset-busy">{S20_RESET_BUSY}</div>
                  ) : (
                    <>
                      <div className="sm mut mb">{S20_RESET_BANNER}</div>
                      <ArmedButton
                        className="wide"
                        idleLabel={S20_RESET_IDLE}
                        armedLabel={S20_RESET_ARMED}
                        disabled={disabledAll}
                        onConfirm={handleResetVenue}
                      />
                      {resetErr && <div className="note err mt" data-testid="s20-reset-err">{resetErr}</div>}
                    </>
                  )}
                </div>
              )}
            </div>
          </div>
        )}

        {tab === 'map' && mapUnlocked && (
          <div className="tabpane on">
            <div className="card">
              <h3>{S20_MAP_TITLE}</h3>
              <div className="mapWrap">
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
                  testId="s20"
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
            data-testid="s20-next-action"
            disabled={disabledAll}
            onClick={nextAction.run}
          >
            {NEXT_ACTION_ICONS[nextAction.kind]}
            <span>{nextAction.label}</span>
          </button>
        )}
        <OperationCard
          mode={state?.mode}
          stateName={stateName}
          attributes={attributes}
          // brief-onsite-ux-fix UX-6-c: 「次にやること」が保存のときは操作カード
          // 側の保存を出さない（同じ操作の重複表示をやめる）。
          //
          // 「停止」「走行」は PREP では inert（将来の追従走行の停止/開始に予約。
          // 2026-09-10 WS-9AA。PREP は PAUSE を持たず、ジョグを握っても状態を
          // 保つ）。ボタンは operationCardLayout の既定どおり両方出すが、押しても
          // 状態は変わらない。対象選択（ui.select_target、guard は
          // mode=PREP state=MAPPING）は走行/停止に関係なく MAPPING で常に通る。
          slots={{
            stop: true, check: false, save: nextAction?.kind !== 'save', manual: true,
          }}
          disabled={disabledAll}
          manualDenyReason={manualDenyReason}
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
                    className="btn sm btn-register"
                    data-testid="s20-reg-home"
                    disabled={disabledAll}
                    onClick={() => handleRegister('HOME')}
                  >
                    <IconPin />
                    <span>{S20_REG_HOME}</span>
                  </button>
                  <button
                    type="button"
                    className="btn sm btn-register"
                    data-testid="s20-reg-panel"
                    disabled={disabledAll}
                    onClick={() => handleRegister('PANEL')}
                  >
                    <IconPin />
                    <span>{S20_REG_PANEL}</span>
                  </button>
                </div>
                <div className="btnrow n2 mb">
                  <button
                    type="button"
                    className="btn sm btn-register"
                    data-testid="s20-reg-home-here"
                    // isRegister（2 点指示ウィザードが開いている）間は無効化する。
                    // ROBOT_POSE はバックエンドで _place_pin_effect() を直接呼ぶため、
                    // 2 点指示の受付状態（_accepting/_p1/_yaw）を横から上書きして
                    // ウィザードを壊してしまう（2026-09-09 レビューで確認）。
                    disabled={disabledAll || isRegister}
                    onClick={() => handleRegisterHere('HOME')}
                  >
                    <IconPin />
                    <span>{S20_REG_HOME_HERE}</span>
                  </button>
                  <button
                    type="button"
                    className="btn sm btn-register"
                    data-testid="s20-reg-panel-here"
                    disabled={disabledAll || isRegister}
                    onClick={() => handleRegisterHere('PANEL')}
                  >
                    <IconPin />
                    <span>{S20_REG_PANEL_HERE}</span>
                  </button>
                </div>
                {/* REG-2: 2 点指示と機体姿勢の違いを一目で分かる短い注記 */}
                <div className="sm mut mb" data-testid="s20-reg-note">{S20_REG_HERE_NOTE}</div>
                {hereMsg && (
                  <div
                    className={`note mb${hereMsg.ok ? '' : ' err'}`}
                    data-testid="s20-reg-here-msg"
                  >
                    {hereMsg.text}
                  </div>
                )}
                {isRegister && (
                  <div className="well" data-testid="s20-wizard">
                    <div className="row mb">
                      <span className="b sm">{S20_WIZ_STEP(wizStep)}</span>
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
                              {pin.name || pin.id}
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