// screens/S30Opcheck.jsx — S-30, 始業点検 (Spec-webui.md §3.12).
// 型「手順を進める」（shell/Layouts.jsx の StepsLayout）。
// 左=一覧（結果の印・時刻・印ごとの数）＋手順の文、右=項目ごとの手順。
// 下端=左「やめる」（実行中だけ）／右「確かめる」（押しても機体は動かない）。
// 実行中でも別の項目・「戻る」でやめられる（SM-3.1.2-116/-117）。
// 非常停止の項目を確かめている間は非常停止の窓を出さない: OPCHECK の
// RUNNING_CHECK は元々 W-1 の条件（mode=ESTOP / PAUSE）に当たらないので
// 追加の設定は要らない（物理ボタンの押下は機体側で入力として扱う SM-3.1.2-089）.
//
// FSM (th_state/config/transitions.yaml T-OPC-01〜10):
//   LIST --ui.check_item--> RUNNING_CHECK (start_monitor)
//   RUNNING_CHECK --evt.check_result--> LIST (record_result)
//   RUNNING_CHECK --ui.check_item--> RUNNING_CHECK (abort + start_monitor)
//   RUNNING_CHECK --ui.abort--> LIST (abort_check)
//   REPAIR --ui.stop--> LIST
//   LIST --ui.enter_mode{CALIB}--> CALIB
import { useEffect, useRef, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useOpcheckStatus } from '../ros/useOpcheckStatus.js'
import { useOpcheckAnswer } from '../ros/useOpcheckAnswer.js'
import { useMotorHold } from '../ros/useMotorHold.js'
import { useWheelSpeeds } from '../ros/useWheelSpeeds.js'
import { useImuCalib } from '../ros/useImuCalib.js'
import { useScan } from '../ros/useScan.js'
import { TOPICS } from '../ros/topics.js'
import { useTunableParams } from '../ros/useTunableParams.js'
import { rangesFromFlat } from './calibCore.js'
import ScanMaskView from '../parts/ScanMaskView.jsx'
import HoldButton from '../parts/HoldButton.jsx'
import WheelSpeedView from '../parts/WheelSpeedView.jsx'
import ResultMark from '../parts/ResultMark.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import ReasonButton from '../parts/ReasonButton.jsx'
import { StepsLayout } from '../shell/Layouts.jsx'
import { setCalibGuide, clearCalibGuide } from './calibGuide.js'
import { checkReasonLabel } from '../i18n/checks.js'
import { parseBasis } from '../ros/checkBasis.js'
import { MOTOR_DIRS, parseMotorDone } from '../ros/motorDirs.js'
import { REJECT_REASONS } from '../i18n/reasons.js'
import {
  S30_ITEM_ORDER, S30_ITEM_LABELS, S30_RESULT_OK, S30_RESULT_WARN, S30_RESULT_NG, S30_RESULT_UNKNOWN,
  S30_DETAIL_PLACEHOLDER, S30_ABORT_NOTE, S30_GOTO_CALIB, S30_GOTO_REPAIR, S30_RESULT_TITLE, S30_RESULT_BACK,
  S30_ESTOP_PRESSED, S30_ESTOP_RELEASED,
  S30_MOTOR_FORWARD, S30_MOTOR_BACK, S30_MOTOR_LEFT, S30_MOTOR_RIGHT, S30_MOTOR_HOLD_NOTE,
  S30_STATUS_DETAIL_TITLE, S30_STATUS_WAITING,
  S30_IMU_CALIB_TITLE, S30_IMU_CALIB_LABELS, S30_LIDAR_VIEW_TITLE, S30_LIDAR_NO_SCAN, S30_LIDAR_MASK_UNKNOWN,
  S30_BASIS_TITLE, S30_BASIS_VALUE, S30_BASIS_THRESHOLD, S30_BASIS_LABELS,
  s30BasisValueText, s30BasisThresholdText,
  S30_BACK, S30_CONFIRM, S30_ABORT2, S30_STEPS, s30StepIndex,
} from '../i18n/screens.js'
import { WIN_RESUME_YES, WIN_RESUME_NO } from '../i18n/states.js'

function resultKind(result) {
  return result ?? 'UNKNOWN'
}

// 判定の根拠（測った値と基準）。外れた値だけ黄の地（tr.out）。
function BasisList({ basis }) {
  if (!basis || basis.length === 0) return null
  return (
    <div className="s30-basis" data-testid="s30-basis">
      <table className="tbl">
        <thead><tr><th>測ったもの</th><th className="num">{S30_BASIS_VALUE}</th><th>{S30_BASIS_THRESHOLD}</th><th /></tr></thead>
        <tbody>
          {basis.map((b) => (
            <tr key={b.key} className={b.ok ? '' : 'out'} data-testid={`s30-basis-${b.key}`} data-ok={b.ok ? 'true' : 'false'}>
              <td>{S30_BASIS_LABELS[b.key] ?? b.key}</td>
              <td className="num">{s30BasisValueText(b)}</td>
              <td>{s30BasisThresholdText(b)}</td>
              <td><ResultMark result={b.ok ? 'OK' : 'NG'} small testId={`s30-basis-mark-${b.key}`} /></td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}

// 誘導（結果の表の下。右下の「確かめる」の場所には出さない。§3.3 (a)）。
function GotoRow({ item, nextScreen, onGotoCalib, onGotoRepair }) {
  if (nextScreen === 'imu_calib' || nextScreen === 'lidar_calib') {
    return (
      <div className="row mt">
        <HelpTip helpKey="s30-goto">
          <button type="button" className="btn primary" data-testid="s30-goto-calib" onClick={onGotoCalib}>
            {S30_GOTO_CALIB}
          </button>
        </HelpTip>
      </div>
    )
  }
  if (nextScreen === 'repair') {
    return (
      <div className="row mt">
        <HelpTip helpKey="s30-goto">
          <button type="button" className="btn primary" data-testid="s30-goto-repair" onClick={onGotoRepair}>
            {S30_GOTO_REPAIR}
          </button>
        </HelpTip>
      </div>
    )
  }
  return null
}

function VerdictPanel({ item, verdict, onGotoCalib, onGotoRepair, onBack }) {
  return (
    <div data-testid={onBack ? 's30-held-result' : 's30-verdict'}>
      {onBack && <h3>{S30_RESULT_TITLE}</h3>}
      <div className="row mt">
        <ResultMark result={resultKind(verdict.result)} testId="s30-verdict-mark" />
        <span className="note" style={{ display: 'inline' }}>{verdict.result === 'OK' ? S30_RESULT_OK : verdict.result === 'WARN' ? S30_RESULT_WARN : verdict.result === 'NG' ? S30_RESULT_NG : S30_RESULT_UNKNOWN}</span>
        {verdict.detail && <span>{checkReasonLabel(item, verdict.detail)}</span>}
      </div>
      <BasisList basis={verdict.basis} />
      <GotoRow item={item} nextScreen={verdict.next_screen} onGotoCalib={onGotoCalib} onGotoRepair={onGotoRepair} />
      {onBack && (
        <button type="button" className="btn mt" data-testid="s30-held-back" onClick={onBack}>
          {S30_RESULT_BACK}
        </button>
      )}
    </div>
  )
}

function useStickyVerdict(status, item) {
  const [verdict, setVerdict] = useState(null)
  useEffect(() => {
    if (status?.item === item && status.result && status.result !== 'UNKNOWN') {
      setVerdict({ result: status.result, detail: status.detail, next_screen: status.next_screen, basis: parseBasis(status.basis_json) })
    }
  }, [status, item])
  return verdict
}

function EstopPict({ pushed }) {
  return (
    <svg viewBox="0 0 200 170" aria-hidden="true">
      <rect x="30" y={pushed ? 96 : 80} width="140" height={pushed ? 40 : 56} rx="10" fill="#b71c1c" />
      <rect x="20" y="130" width="160" height="30" rx="6" fill="#ffd100" stroke="#161b22" strokeWidth="2" />
      {pushed
        ? <path d="M100 10v60M80 50l20 22 20-22" stroke="currentColor" strokeWidth="8" fill="none" strokeLinecap="round" />
        : <path d="M60 40a45 45 0 0 1 80 0M140 40l4-22M140 40l-20-6" stroke="currentColor" strokeWidth="8" fill="none" strokeLinecap="round" />}
    </svg>
  )
}

function RobotMini() {
  return (
    <svg viewBox="0 0 60 60" aria-hidden="true"><rect x="6" y="6" width="48" height="48" rx="4" fill="var(--inset)" stroke="var(--tx)" strokeWidth="2" /><path d="M20 40L30 16L40 40Z" fill="var(--tx)" /></svg>
  )
}

export default function S30Opcheck() {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const answer = useOpcheckAnswer()
  const { status, results } = useOpcheckStatus(ros)
  const wheelData = useWheelSpeeds(ros)

  const stateName = state?.state ?? null
  const running = stateName === 'RUNNING_CHECK'
  const disabledAll = stale || state?.mode == null
  const estopHw = !!state?.estop_hw
  const estopUi = !!state?.estop_ui

  const [previewItem, setPreviewItem] = useState(null)
  const [startedItem, setStartedItem] = useState(null)
  const [selectErr, setSelectErr] = useState(null)
  const [held, setHeld] = useState(null)
  const initialStatusRef = useRef(status)
  useEffect(() => {
    if (status === initialStatusRef.current) return
    if (status?.item && status.result && status.result !== 'UNKNOWN') {
      setHeld({ item: status.item, result: status.result, detail: status.detail, next_screen: status.next_screen, basis: parseBasis(status.basis_json) })
    }
  }, [status])

  useEffect(() => {
    if (stateName === 'LIST') { setStartedItem(null) }
  }, [stateName])

  const activeItem = running ? (startedItem || status?.item || null) : null
  const shownItem = activeItem || previewItem
  const currentResult = (it) => results[it]?.result ?? 'UNKNOWN'

  // ESTOP wizard state (kept from the old implementation).
  const [sawPress, setSawPress] = useState(!!estopHw)
  const [sawRelease, setSawRelease] = useState(false)
  const [pressAnswered, setPressAnswered] = useState(false)
  const [releaseAnswered, setReleaseAnswered] = useState(false)
  const prevHwRef = useRef(!!estopHw)
  useEffect(() => {
    // Reset the wizard when leaving the ESTOP item so a rerun starts from step 1.
    if (!running || activeItem !== 'ESTOP') {
      setSawPress(false); setSawRelease(false); setPressAnswered(false); setReleaseAnswered(false)
    }
    const prev = prevHwRef.current
    if (estopHw && !prev) setSawPress(true)
    if (!estopHw && prev) setSawRelease((already) => already || sawPress)
    prevHwRef.current = !!estopHw
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [estopHw, running, activeItem])

  const motorHoldGate = running && activeItem === 'MOTOR' && !disabledAll && !estopHw && !estopUi
  const motorHold = useMotorHold(ros, motorHoldGate)
  const [motorDir, setMotorDir] = useState(null)
  function motorPress(dir) { setMotorDir(dir); motorHold.press(dir) }
  function motorRelease() { setMotorDir(null); motorHold.release() }

  async function startItem(item) {
    if (disabledAll) return
    setSelectErr(null)
    setHeld(null)
    setStartedItem(item)
    setPreviewItem(item)
    try {
      const res = await sendTrigger('ui.check_item', { item })
      if (!res?.accepted) {
        setStartedItem(null)
        setSelectErr(res?.reject_reason_key ? (REJECT_REASONS[res.reject_reason_key] ?? res.reject_reason_key) : null)
      }
    } catch {
      setStartedItem(null)
    }
  }

  function handleRow(item) {
    if (disabledAll) return
    if (running) {
      // 実行中は押すとその項目へ切り替わる（SM-3.1.2-117。結果は記録しない）。
      if (item !== activeItem) startItem(item)
      return
    }
    // LIST では押すだけでは始めない。手順を見て「確かめる」で始める。
    setHeld(null)
    setPreviewItem(item)
    setSelectErr(null)
  }

  async function handleBack() {
    try { await sendTrigger('ui.finish') } catch { /* 留まる */ }
  }
  async function handleAbort() {
    try { await sendTrigger('ui.abort') } catch { /* 留まる */ }
  }

  async function handleGotoCalib(item, verdict) {
    setCalibGuide({ from: item, result: verdict?.result })
    try {
      const res = await sendTrigger('ui.enter_mode', { mode: 'CALIB' })
      if (!res?.accepted) clearCalibGuide()
    } catch {
      clearCalibGuide()
    }
  }
  async function handleGotoRepair() {
    try { await sendTrigger('ui.stop') } catch { /* 留まる */ }
  }

  // Motor direction progress (done=... in detail).
  const doneDirs = status?.item === 'MOTOR' ? parseMotorDone(status.detail) : []

  // Steps text (numbers).
  const stepsText = shownItem ? S30_STEPS[shownItem] ?? [] : []
  const curStep = shownItem ? s30StepIndex(shownItem, {
    running: running && activeItem === shownItem,
    sawPress, sawRelease, answered: { press: pressAnswered, release: releaseAnswered },
  }) : 0

  // Counts by result mark.
  const counts = { OK: 0, WARN: 0, NG: 0, UNKNOWN: 0 }
  for (const it of S30_ITEM_ORDER) counts[resultKind(currentResult(it))] += 1

  // フックは無条件に呼ぶ（4 項目分）。使うのは実行中の項目の分だけ。
  const vEstop = useStickyVerdict(status, 'ESTOP')
  const vMotor = useStickyVerdict(status, 'MOTOR')
  const vImu = useStickyVerdict(status, 'IMU')
  const vLidar = useStickyVerdict(status, 'LIDAR')
  const stickyVerdict = activeItem === 'ESTOP' ? vEstop : activeItem === 'MOTOR' ? vMotor : activeItem === 'IMU' ? vImu : vLidar
  const liveVerdict = status?.item === activeItem && status?.result && status.result !== 'UNKNOWN'
    ? { result: status.result, detail: status.detail, next_screen: status.next_screen, basis: parseBasis(status.basis_json) }
    : null
  const activeVerdict = stickyVerdict ?? liveVerdict

  const leftList = (
    <>
      <div className="lhd">
        <ResultMark result="OK" small testId="s30-count-ok" /> <span>{counts.OK}</span>
        {'　'}
        <ResultMark result="NG" small testId="s30-count-ng" /> <span>{counts.NG}</span>
        {'　'}
        <ResultMark result="UNKNOWN" small testId="s30-count-unknown" /> <span>{counts.UNKNOWN}</span>
      </div>
      {S30_ITEM_ORDER.map((item) => (
        <button
          key={item}
          type="button"
          className={`lrow${shownItem === item ? ' on' : ''}`}
          data-testid={`s30-item-${item}`}
          onClick={() => handleRow(item)}
        >
          <span className="nm">
            {S30_ITEM_LABELS[item]}
            <small>{results[item]?.checked_at ?? ''}</small>
          </span>
          <ResultMark result={resultKind(currentResult(item))} testId={`s30-mark-${item}`} />
        </button>
      ))}
      {selectErr && <p className="note" data-testid="s30-select-err">{selectErr}</p>}
      {shownItem && (
        <ol className="csteps" data-testid="s30-steps">
          {stepsText.map((t, i) => (
            <li key={t} className={i < curStep ? 'done' : i === curStep ? 'cur' : ''}>
              <span className="n">{i + 1}</span><span>{t}</span>
            </li>
          ))}
        </ol>
      )}
    </>
  )

  const bodyContent = (() => {
    if (held && !activeItem) {
      return (
        <VerdictPanel item={held.item} verdict={held}
          onGotoCalib={() => handleGotoCalib(held.item, held)} onGotoRepair={handleGotoRepair}
          onBack={() => setHeld(null)} />
      )
    }
    if (!shownItem) return <p className="note">{S30_DETAIL_PLACEHOLDER}</p>
    if (!activeItem) {
      // LIST・プレビュー: 項目の手順の図を出す（機体は動かない。§3.12）。
      if (shownItem === 'MOTOR') {
        return (
          <div className="fig mot">
            <div className="arrows">
              <span /><MotorArrow dir="FORWARD" label={S30_MOTOR_FORWARD} done={false} disabled gate={false} active={false} onPress={() => {}} onRelease={() => {}} />
              <span />
              <MotorArrow dir="LEFT" label={S30_MOTOR_LEFT} done={false} disabled gate={false} active={false} onPress={() => {}} onRelease={() => {}} />
              <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}><RobotMini /></span>
              <MotorArrow dir="RIGHT" label={S30_MOTOR_RIGHT} done={false} disabled gate={false} active={false} onPress={() => {}} onRelease={() => {}} />
              <span /><MotorArrow dir="BACK" label={S30_MOTOR_BACK} done={false} disabled gate={false} active={false} onPress={() => {}} onRelease={() => {}} />
              <span />
            </div>
            <WheelSpeedView measured={wheelData.measured} command={wheelData.command} />
          </div>
        )
      }
      if (shownItem === 'ESTOP') {
        return (
          <div className="fig">
            <div className="cell"><EstopPict pushed={false} /><span className="cap"><ResultMark result="UNKNOWN" small testId="s30-estop-push-mark" />押す</span></div>
            <div className="cell"><EstopPict pushed={false} /><span className="cap"><ResultMark result="UNKNOWN" small testId="s30-estop-release-mark" />戻す</span></div>
          </div>
        )
      }
      if (shownItem === 'IMU') return <div className="fig"><ImuLive ros={ros} /></div>
      return <div className="fig"><LidarLive ros={ros} /></div>
    }
    // Running.
    if (activeItem === 'ESTOP') {
      if (activeVerdict) {
        return <VerdictPanel item="ESTOP" verdict={activeVerdict} onGotoCalib={() => handleGotoCalib('ESTOP', activeVerdict)} onGotoRepair={handleGotoRepair} />
      }
      let msg, asking = null
      if (!sawPress) msg = '非常停止ボタンを押してください'
      else if (!pressAnswered) { msg = '押されていることが画面に出ています。実物と一致していますか'; asking = 'press' }
      else if (!sawRelease) msg = '非常停止ボタンを解除してください'
      else if (!releaseAnswered) { msg = '解除されたことが画面に出ています。一致していますか'; asking = 'release' }
      else msg = S30_STATUS_WAITING
      return (
        <div data-testid="s30-estop-step">
          <div className="fig">
            <div className="cell"><EstopPict pushed={!!estopHw} /><span className="cap"><ResultMark result={sawPress ? 'OK' : 'UNKNOWN'} small testId="s30-estop-push-mark" />押す</span></div>
            <div className="cell"><EstopPict pushed={!!estopHw} /><span className="cap"><ResultMark result={sawRelease ? 'OK' : 'UNKNOWN'} small testId="s30-estop-release-mark" />戻す</span></div>
          </div>
          <p className="oneline" style={{ textAlign: 'center' }}>{msg}</p>
          {asking && (
            <div className="s30-answer-row">
              <button type="button" className="btn" data-testid="s30-estop-yes" onClick={() => { answer('ESTOP', true); if (asking === 'press') setPressAnswered(true); else setReleaseAnswered(true) }}>
                {WIN_RESUME_YES}
              </button>
              <button type="button" className="btn" data-testid="s30-estop-no" onClick={() => { answer('ESTOP', false); if (asking === 'press') setPressAnswered(true); else setReleaseAnswered(true) }}>
                {WIN_RESUME_NO}
              </button>
            </div>
          )}
          <p className="note">{estopHw ? S30_ESTOP_PRESSED : S30_ESTOP_RELEASED}</p>
        </div>
      )
    }
    if (activeItem === 'MOTOR') {
      if (activeVerdict) {
        return <VerdictPanel item="MOTOR" verdict={activeVerdict} onGotoCalib={() => handleGotoCalib('MOTOR', activeVerdict)} onGotoRepair={handleGotoRepair} />
      }
      return (
        <div>
          <p className="note">{S30_MOTOR_HOLD_NOTE}</p>
          <div className="fig mot">
            <div className="arrows">
              <span /><MotorArrow dir="FORWARD" label={S30_MOTOR_FORWARD} done={doneDirs.includes('FORWARD')} disabled={disabledAll} gate={motorHoldGate} active={motorDir === 'FORWARD'} onPress={motorPress} onRelease={motorRelease} />
              <span />
              <MotorArrow dir="LEFT" label={S30_MOTOR_LEFT} done={doneDirs.includes('LEFT')} disabled={disabledAll} gate={motorHoldGate} active={motorDir === 'LEFT'} onPress={motorPress} onRelease={motorRelease} />
              <span style={{ display: 'flex', alignItems: 'center', justifyContent: 'center' }}><RobotMini /></span>
              <MotorArrow dir="RIGHT" label={S30_MOTOR_RIGHT} done={doneDirs.includes('RIGHT')} disabled={disabledAll} gate={motorHoldGate} active={motorDir === 'RIGHT'} onPress={motorPress} onRelease={motorRelease} />
              <span /><MotorArrow dir="BACK" label={S30_MOTOR_BACK} done={doneDirs.includes('BACK')} disabled={disabledAll} gate={motorHoldGate} active={motorDir === 'BACK'} onPress={motorPress} onRelease={motorRelease} />
              <span />
            </div>
            <WheelSpeedView measured={wheelData.measured} command={wheelData.command} />
          </div>
          <div className="s30-motor-dirs" data-testid="s30-motor-dirs">
            {MOTOR_DIRS.map((dir) => (
              <span key={dir} data-testid={`s30-motor-dir-${dir}`}>{dir} {doneDirs.includes(dir) ? '済' : '未'}</span>
            ))}
          </div>
        </div>
      )
    }
    if (activeItem === 'IMU') {
      const verdict = activeVerdict
      if (verdict) return <VerdictPanel item="IMU" verdict={verdict} onGotoCalib={() => handleGotoCalib('IMU', verdict)} onGotoRepair={handleGotoRepair} />
      return (
        <div>
          <div className="row"><ResultMark result="UNKNOWN" testId="s30-running-mark" /><span>{S30_STATUS_WAITING}</span></div>
          <ImuLive ros={ros} />
        </div>
      )
    }
    // LIDAR
    const verdict = activeVerdict
    if (verdict) return <VerdictPanel item="LIDAR" verdict={verdict} onGotoCalib={() => handleGotoCalib('LIDAR', verdict)} onGotoRepair={handleGotoRepair} />
    return (
      <div>
        <div className="row"><ResultMark result="UNKNOWN" testId="s30-running-mark" /><span>{S30_STATUS_WAITING}</span></div>
        <LidarLive ros={ros} />
      </div>
    )
  })()

  const confirmDisabled = disabledAll || running || !previewItem
  const confirmReason = running ? '確かめています' : !previewItem ? '項目を選んでください' : null

  return (
    <div id="s30" className="screen">
      <StepsLayout
        opbar={(
          <HelpTip helpKey="s30-back">
            <button type="button" className="btn exit" data-testid="s30-back" disabled={disabledAll} onClick={handleBack}>
              {S30_BACK}
            </button>
          </HelpTip>
        )}
        list={leftList}
        body={(
          <>
            <div className="phd"><span className="pt">{shownItem ? S30_ITEM_LABELS[shownItem] : S30_STATUS_DETAIL_TITLE}</span></div>
            <div data-testid="s30-body">{bodyContent}</div>
          </>
        )}
        foot={(
          <>
            <HelpTip helpKey="s30-abort">
              <ReasonButton
                data-testid="s30-abort"
                disabled={!running || disabledAll}
                reason={!running ? '確かめている間だけ使えます' : null}
                onClick={handleAbort}
                className="primary"
              >
                {S30_ABORT2}
              </ReasonButton>
            </HelpTip>
            <span className="grow" />
            <HelpTip helpKey="s30-confirm">
              <ReasonButton
                data-testid="s30-confirm"
                disabled={confirmDisabled}
                reason={confirmReason}
                onClick={() => previewItem && startItem(previewItem)}
                className="primary"
              >
                {S30_CONFIRM}
              </ReasonButton>
            </HelpTip>
          </>
        )}
      />
    </div>
  )
}

function MotorArrow({ dir, label, done, disabled, gate, active, onPress, onRelease }) {
  const testId = `s30-motor-${dir.toLowerCase()}`
  return (
    <span style={{ position: 'relative', display: 'block' }}>
      <HoldButton direction={dir} label={label} active={active} disabled={disabled || !gate}
        onPress={onPress} onRelease={onRelease} testId={testId} />
      <span className="dm"><ResultMark result={done ? 'OK' : 'UNKNOWN'} small testId={`${testId}-mark`} /></span>
    </span>
  )
}

function ImuLive({ ros }) {
  const calib = useImuCalib(ros)
  return (
    <div data-testid="s30-imu-calib">
      <h4>{S30_IMU_CALIB_TITLE}</h4>
      {!calib && <p className="note" data-testid="s30-imu-calib-waiting">{S30_STATUS_WAITING}</p>}
      {calib && (
        <div className="row">
          {Object.keys(S30_IMU_CALIB_LABELS).map((k) => (
            <span key={k} data-testid={`s30-imu-${k}`}>
              {S30_IMU_CALIB_LABELS[k]} {calib[k]}/3
            </span>
          ))}
        </div>
      )}
    </div>
  )
}

function LidarLive({ ros }) {
  const scan = useScan(ros, TOPICS.SCAN)
  const { getTunableParams } = useTunableParams(ros)
  const [ranges, setRanges] = useState(null)
  useEffect(() => {
    let alive = true
    getTunableParams('lidar_filter', ['blind_angle_ranges'])
      .then((v) => { if (alive) setRanges(rangesFromFlat(v.blind_angle_ranges ?? [])) })
      .catch(() => { if (alive) setRanges(null) })
    return () => { alive = false }
  }, [getTunableParams])
  return (
    <div data-testid="s30-lidar-panel">
      <h4>{S30_LIDAR_VIEW_TITLE}</h4>
      {!scan && <p className="note" data-testid="s30-lidar-no-scan">{S30_LIDAR_NO_SCAN}</p>}
      {ranges === null && <p className="note" data-testid="s30-lidar-mask-unknown">{S30_LIDAR_MASK_UNKNOWN}</p>}
      <ScanMaskView scan={scan} ranges={ranges ?? []} />
    </div>
  )
}
