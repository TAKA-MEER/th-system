// screens/S30Opcheck.jsx — S-30, 始業点検 (SCREEN_NAMES.S30 in
// i18n/screens.js; WP-UI-08 / WP-MAINT-01). Spec-webui.md §3.12,
// DetailedDesign-maintenance.md §2.
//
// FSM (th_state/config/transitions.yaml T-OPC-01〜08):
//   LIST --ui.check_item{item}--> RUNNING_CHECK (effect: start_monitor{item}
//     -> /system/effect, opcheck_runner starts the item)
//   RUNNING_CHECK --evt.check_result(check_result_ok)--> LIST (record_result)
//   RUNNING_CHECK --evt.check_result(ng_and_calibrable)--> LIST (offer_calib)
//   RUNNING_CHECK --evt.check_result(ng_and_not_calibrable)--> REPAIR
//   REPAIR --ui.stop--> LIST
//   LIST --ui.enter_mode{mode:CALIB}--> CALIB (T-OPC-07; S-40 doesn't exist
//     yet, so this currently lands on S-01's screenless-mode escape button)
//   * --fault.recoverable--> LIST (abort_check)
//
// Item selection goes through /system/trigger's ui.check_item, exactly like
// every other screen's FSM-mediated action -- NOT /opcheck/run_item
// (RunCheck). Calling RunCheck directly would start opcheck_runner's
// item without moving the FSM to RUNNING_CHECK, desyncing the screen from
// th_state (see this packet's completion report, "本番の送信経路").
//
// The 4 items don't share one UI: ESTOP is a 4-step press/confirm/release/
// confirm wizard (§2.2), MOTOR is 4 hold-buttons + a live cmd-vs-measured
// graph (§2.3), IMU/LIDAR are display-only (/opcheck/status's result +
// detail, §2.4/§2.5 -- Spec-webui.md §3.12's "詳細: 選んだ項目のモニターと
// 自動判定"). LiDAR's raw-scan live view (§2.5 順1) was left as
// result-only display per the brief's explicit "無ければ判定表示だけでよい"
// escape hatch -- see completion report.
//
// Known node-side gap this screen works around rather than fixes (ROS
// changes are out of this packet's scope): opcheck_runner._monitor_tick
// republishes CheckStatus with result="UNKNOWN" every 100ms even the tick
// right after a final OK/WARN/NG, because none of T-OPC-02/03/06 send it a
// record_result/abort_check effect to close `_item`. Each item monitor below
// latches the first non-UNKNOWN status it sees for the run it's currently
// showing (see useStickyVerdict) so the verdict doesn't flicker back to the
// step wizard 100ms later.
import { useEffect, useRef, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useOpcheckStatus } from '../ros/useOpcheckStatus.js'
import { useOpcheckAnswer } from '../ros/useOpcheckAnswer.js'
import { useMotorHold } from '../ros/useMotorHold.js'
import { useWheelSpeeds } from '../ros/useWheelSpeeds.js'
import { useImuCalib } from '../ros/useImuCalib.js'
import { useScan } from '../ros/useScan.js'
import { useTunableParams } from '../ros/useTunableParams.js'
import { rangesFromFlat } from './calibCore.js'
import ScanMaskView from '../parts/ScanMaskView.jsx'
import HoldButton from '../parts/HoldButton.jsx'
import WheelSpeedView from '../parts/WheelSpeedView.jsx'
import { setCalibGuide, clearCalibGuide } from './calibGuide.js'
import { checkReasonLabel } from '../i18n/checks.js'
import { MOTOR_DIRS, parseMotorDone } from '../ros/motorDirs.js'
import { REJECT_REASONS } from '../i18n/reasons.js'
import { OP_LABELS } from '../i18n/states.js'
import {
  S30_ITEM_ORDER, S30_ITEM_LABELS, S30_LIST_TITLE, S30_OVERALL_TITLE,
  S30_RESULT_UNKNOWN, S30_RESULT_OK, S30_RESULT_WARN, S30_RESULT_NG, S30_RESULT_RUNNING,
  S30_RESULT_TONE, s30OverallLabel, S30_DETAIL_PLACEHOLDER, S30_ABORT, S30_ABORT_NOTE, S30_GOTO_CALIB,
  S30_ESTOP_STEP1, S30_ESTOP_STEP2, S30_ESTOP_STEP3, S30_ESTOP_STEP4,
  S30_MOTOR_FORWARD, S30_MOTOR_BACK, S30_MOTOR_LEFT, S30_MOTOR_RIGHT, S30_MOTOR_HOLD_NOTE,
  S30_STATUS_DETAIL_TITLE, S30_STATUS_WAITING,
  S30_IMU_CALIB_TITLE, S30_IMU_CALIB_LABELS, S30_LIDAR_VIEW_TITLE, S30_LIDAR_NO_SCAN,
  S30_LIDAR_MASK_UNKNOWN,
} from '../i18n/screens.js'
import { WIN_RESUME_YES, WIN_RESUME_NO } from '../i18n/states.js'

function resultLabel(result) {
  if (result === 'OK') return S30_RESULT_OK
  if (result === 'WARN') return S30_RESULT_WARN
  if (result === 'NG') return S30_RESULT_NG
  return S30_RESULT_UNKNOWN
}

// Latches the first non-UNKNOWN CheckStatus seen for `item` since this
// component instance mounted (i.e. since the current run of that item
// started -- callers conditionally render the per-item monitor so each run
// gets a fresh instance). See file header for why this must not read the
// cross-run module latch that ros/useOpcheckStatus.js also keeps.
function useStickyVerdict(status, item) {
  const [verdict, setVerdict] = useState(null)
  useEffect(() => {
    if (status?.item === item && status.result && status.result !== 'UNKNOWN') {
      setVerdict({ result: status.result, detail: status.detail, next_screen: status.next_screen })
    }
  }, [status, item])
  return verdict
}

function VerdictCard({ item, verdict, disabledAll, onGotoCalib }) {
  const showCalib = verdict.next_screen === 'imu_calib' || verdict.next_screen === 'lidar_calib'
  return (
    <div className="card">
      <div className="s30-verdict">
        <span className={`pill ${S30_RESULT_TONE[verdict.result] ?? ''}`}>{resultLabel(verdict.result)}</span>
        {verdict.detail && <span className="note">{checkReasonLabel(item, verdict.detail)}</span>}
      </div>
      {showCalib && (
        <button type="button" className="btn wide" disabled={disabledAll}
          data-testid="s30-goto-calib" onClick={() => onGotoCalib(item, verdict)}>
          {S30_GOTO_CALIB}
        </button>
      )}
    </div>
  )
}

function EstopMonitor({ status, estopHw, disabledAll, onAnswer, onGotoCalib }) {
  const verdict = useStickyVerdict(status, 'ESTOP')
  const [sawPress, setSawPress] = useState(!!estopHw)
  const [sawRelease, setSawRelease] = useState(false)
  const [pressAnswered, setPressAnswered] = useState(false)
  const [releaseAnswered, setReleaseAnswered] = useState(false)
  const prevHwRef = useRef(!!estopHw)

  useEffect(() => {
    const prev = prevHwRef.current
    if (estopHw && !prev) setSawPress(true)
    if (!estopHw && prev) {
      setSawRelease((already) => already || sawPress)
    }
    prevHwRef.current = !!estopHw
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [estopHw])

  if (verdict) return <VerdictCard item="ESTOP" verdict={verdict} disabledAll={disabledAll} onGotoCalib={onGotoCalib} />

  function handleAnswer(ok, which) {
    onAnswer('ESTOP', ok)
    if (which === 'press') setPressAnswered(true)
    else setReleaseAnswered(true)
  }

  let msg = S30_ESTOP_STEP1
  let asking = null
  if (!sawPress) {
    msg = S30_ESTOP_STEP1
  } else if (!pressAnswered) {
    msg = S30_ESTOP_STEP2
    asking = 'press'
  } else if (!sawRelease) {
    msg = S30_ESTOP_STEP3
  } else if (!releaseAnswered) {
    msg = S30_ESTOP_STEP4
    asking = 'release'
  } else {
    msg = S30_STATUS_WAITING
  }

  return (
    <div className="s30-step" data-testid="s30-estop-step">
      <div className="s30-step-msg">{msg}</div>
      {asking && (
        <div className="s30-answer-row">
          <button type="button" className="btn" disabled={disabledAll}
            data-testid="s30-estop-yes" onClick={() => handleAnswer(true, asking)}>
            {WIN_RESUME_YES}
          </button>
          <button type="button" className="btn" disabled={disabledAll}
            data-testid="s30-estop-no" onClick={() => handleAnswer(false, asking)}>
            {WIN_RESUME_NO}
          </button>
        </div>
      )}
    </div>
  )
}

const MOTOR_DIR_LABELS = {
  FORWARD: S30_MOTOR_FORWARD,
  BACK: S30_MOTOR_BACK,
  LEFT: S30_MOTOR_LEFT,
  RIGHT: S30_MOTOR_RIGHT,
}

// MOTOR の 4 方向の済み／未（brief-a13・SG-A13）。runner が UNKNOWN の detail
// 先頭に `done=...` で進捗を載せる（msg 不変）。spec §2.4 #2「未実施の方向がある間は
// 判定を確定しない」の途中経過表示。
function MotorDirProgress({ status }) {
  const done = status?.item === 'MOTOR' ? parseMotorDone(status.detail) : []
  return (
    <div className="s30-motor-dirs" data-testid="s30-motor-dirs">
      {MOTOR_DIRS.map((dir) => (
        <span key={dir} className={`pill ${done.includes(dir) ? 'ok' : ''}`}
          data-testid={`s30-motor-dir-${dir}`}>
          {MOTOR_DIR_LABELS[dir]} {done.includes(dir) ? '済' : '未'}
        </span>
      ))}
    </div>
  )
}

// SG-C13（Spec-checks.md §2.4 #3-3）: IMU の校正状態 sys/gyro/accel/mag のリアルタイム表示。
function ImuCalibPanel({ ros }) {
  const calib = useImuCalib(ros)
  return (
    <div data-testid="s30-imu-calib">
      <h4>{S30_IMU_CALIB_TITLE}</h4>
      {!calib && <p className="note" data-testid="s30-imu-calib-waiting">{S30_STATUS_WAITING}</p>}
      {calib && (
        <div className="row">
          {Object.keys(S30_IMU_CALIB_LABELS).map((k) => (
            <span key={k} className={`pill ${calib[k] >= 3 ? 'ok' : 'warn'}`}
              data-testid={`s30-imu-${k}`}>
              {S30_IMU_CALIB_LABELS[k]} {calib[k]}/3
            </span>
          ))}
        </div>
      )}
    </div>
  )
}

// SG-C13（Spec-checks.md §2.4 #4-1・#4-3）: 生スキャンのライブ表示に死角マスクを重ねる。
// マスクは機体の値（lidar_filter の blind_angle_ranges）を読む。画面に値を持たない。
// 死角除去前の生スキャン /scan を見る（マスクの内側の写り込みが見えるように。S-40 と同じ）。
function LidarMaskPanel({ ros }) {
  const scan = useScan(ros, '/scan')
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

function SimpleStatusMonitor({ item, ros, status, disabledAll, onGotoCalib }) {
  const verdict = useStickyVerdict(status, item)
  const live = status?.item === item ? status : null
  const shown = verdict ?? live

  return (
    <div className="card">
      <h3>{S30_STATUS_DETAIL_TITLE}</h3>
      {!shown && <p className="note">{S30_STATUS_WAITING}</p>}
      {shown && (
        <>
          <div className="s30-verdict">
            <span className={`pill ${S30_RESULT_TONE[shown.result] ?? ''}`}>
              {shown.result === 'UNKNOWN' ? S30_RESULT_RUNNING : resultLabel(shown.result)}
            </span>
          </div>
          {shown.detail && (
            <p className="note" data-testid={`s30-${item.toLowerCase()}-detail`}>
              {shown.result && shown.result !== 'UNKNOWN'
                ? checkReasonLabel(item, shown.detail)
                : shown.detail}
            </p>
          )}
        </>
      )}
      {item === 'IMU' && <ImuCalibPanel ros={ros} />}
      {item === 'LIDAR' && <LidarMaskPanel ros={ros} />}
      {verdict && (verdict.next_screen === 'imu_calib' || verdict.next_screen === 'lidar_calib') && (
        <button type="button" className="btn wide" disabled={disabledAll}
          data-testid="s30-goto-calib" onClick={() => onGotoCalib(item, verdict)}>
          {S30_GOTO_CALIB}
        </button>
      )}
    </div>
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

  const [selectedItem, setSelectedItem] = useState(null)
  const [selectErr, setSelectErr] = useState(null)

  // The item that is actually running server-side takes priority over the
  // locally-clicked one (status.item is authoritative once it arrives);
  // selectedItem only bridges the gap between "trigger accepted" and "first
  // CheckStatus for it". Reset once the FSM leaves RUNNING_CHECK.
  useEffect(() => {
    if (stateName === 'LIST') setSelectedItem(null)
  }, [stateName])

  // 実行中に別の項目へ切り替えた直後は、status.item がまだ前の項目のことがある
  // （abort_check → start_monitor の往復待ち）。押した項目を優先する（SM-3.1.2-117）。
  const activeItem = running ? (selectedItem || status?.item) : null

  // MOTOR's hold must stop the instant this screen is not in a state where
  // opcheck_runner's deadman is supposed to be listened to -- re-derived
  // continuously (useMotorHold re-reads `gate` on every tick), not just at
  // press time. opcheck_runner._sync_command doesn't check which item is
  // selected (see completion report "B"), so the UI-side gate is what keeps
  // a stray hold from driving the base once the MOTOR item isn't the one
  // actually running.
  const motorHoldGate = running && activeItem === 'MOTOR' && !disabledAll && !estopHw && !estopUi
  const motorHold = useMotorHold(ros, motorHoldGate)
  const [motorDir, setMotorDir] = useState(null)
  function motorPress(dir) { setMotorDir(dir); motorHold.press(dir) }
  function motorRelease() { setMotorDir(null); motorHold.release() }

  // LIST では項目を始める。RUNNING_CHECK では今の項目を中断して選んだ項目へ切り替える
  // （SM-3.1.2-116/-117。2026-10-04 まで実行中は一覧が押せず、判定が出るまで抜けられなかった）。
  async function handleSelect(item) {
    if (!(stateName === 'LIST' || running) || disabledAll) return
    if (running && item === activeItem) return
    const prevSelected = selectedItem
    setSelectErr(null)
    setSelectedItem(item)
    try {
      const res = await sendTrigger('ui.check_item', { item })
      if (!res?.accepted) {
        setSelectedItem(prevSelected)
        setSelectErr(res?.reject_reason_key ? (REJECT_REASONS[res.reject_reason_key] ?? res.reject_reason_key) : null)
      }
    } catch {
      setSelectedItem(prevSelected)
    }
  }

  // 実行中の項目をやめて一覧へ戻る（結果は記録しない。SM-3.1.2-116）。
  async function handleAbort() {
    try { await sendTrigger('ui.abort') } catch { /* rosbridge 一時失敗。留まる */ }
  }

  async function handleFinish() {
    try { await sendTrigger('ui.finish') } catch { /* rosbridge 一時失敗。留まる */ }
  }

  // S-40 への誘導元を受け渡してから ui.enter_mode{CALIB} を送る（T-OPC-07 に
  // 項目を運ぶ仕組みは無いので画面側の受け渡し。screens/calibGuide.js）。
  // 拒否・送信失敗では受け渡しを消す（S-01 など別経路で S-40 に入ったときに
  // 古い誘導が残らないようにする）。対応表に無い項目（ESTOP・MOTOR）は
  // setCalibGuide が置かずに捨てる。
  async function handleGotoCalib(item, verdict) {
    setCalibGuide({ from: item, result: verdict?.result })
    try {
      const res = await sendTrigger('ui.enter_mode', { mode: 'CALIB' })
      if (!res?.accepted) clearCalibGuide()
    } catch {
      clearCalibGuide() // rosbridge 一時失敗。受け渡しを残さない
    }
  }

  return (
    <div className="screen two-col" id="s30">
      <div className="left-col">
        <div className="top-actions sticky">
          <button type="button" className="btn sm" data-testid="s30-finish"
            disabled={disabledAll} onClick={handleFinish}>
            {OP_LABELS.finish}
          </button>
        </div>
        <div className="card">
          <h3>{S30_LIST_TITLE}</h3>
          {S30_ITEM_ORDER.map((item) => {
            // SG-B5（1b-8）: 判定と同時に FSM は LIST へ戻る（T-OPC-02/04）のに
            // 「校正へ」は RUNNING_CHECK の詳細カードにしか無かった。LIST の行にも
            // 直近の判定（results ラッチ。S-31 と同じ流儀）から出す。FSM 側は
            // spec（SM-3.1.2-087「校正へ導線を出す」・-092「LIST → CALIB」）
            // どおりなので変えない。T-OPC-07 は LIST 限定のため LIST のときだけ出す。
            const calibVerdict = (results[item]?.next_screen === 'imu_calib' || results[item]?.next_screen === 'lidar_calib')
              ? results[item] : null
            return (
              <div key={item} className="s30-row-wrap">
                <button
                  type="button"
                  className={`s30-row ${activeItem === item ? 'running' : ''}`}
                  disabled={disabledAll || !(stateName === 'LIST' || running)}
                  data-testid={`s30-item-${item}`}
                  onClick={() => handleSelect(item)}
                >
                  <span className="s30-row-label">{S30_ITEM_LABELS[item]}</span>
                  <span className={`pill ${S30_RESULT_TONE[results[item]?.result] ?? ''}`}>
                    {resultLabel(results[item]?.result)}
                  </span>
                </button>
                {calibVerdict && stateName === 'LIST' && (
                  <button type="button" className="btn wide" disabled={disabledAll}
                    data-testid={`s30-goto-calib-${item}`}
                    onClick={() => handleGotoCalib(item, calibVerdict)}>
                    {S30_GOTO_CALIB}
                  </button>
                )}
              </div>
            )
          })}
          {selectErr && <p className="note" data-testid="s30-select-err">{selectErr}</p>}
        </div>
        <div className="card">
          <h3>{S30_OVERALL_TITLE}</h3>
          <span className="pill" data-testid="s30-overall">{s30OverallLabel(results)}</span>
        </div>
      </div>

      <div>
        {!activeItem && (
          <div className="card"><p className="note">{S30_DETAIL_PLACEHOLDER}</p></div>
        )}
        {running && (
          <div className="top-actions">
            <button type="button" className="btn sm" data-testid="s30-abort"
              disabled={disabledAll} onClick={handleAbort}>
              {S30_ABORT}
            </button>
            <span className="xs mut">{S30_ABORT_NOTE}</span>
          </div>
        )}
        {activeItem === 'ESTOP' && (
          <EstopMonitor
            status={status} estopHw={estopHw} disabledAll={disabledAll}
            onAnswer={answer} onGotoCalib={handleGotoCalib}
          />
        )}
        {activeItem === 'MOTOR' && (() => {
          const verdict = status?.item === 'MOTOR' && status.result && status.result !== 'UNKNOWN'
            ? { result: status.result, detail: status.detail, next_screen: status.next_screen }
            : null
          if (verdict) {
            return <VerdictCard item="MOTOR" verdict={verdict} disabledAll={disabledAll} onGotoCalib={handleGotoCalib} />
          }
          return (
            <div className="card">
              <p className="note">{S30_MOTOR_HOLD_NOTE}</p>
              <MotorDirProgress status={status} />
              <div className="hold-grid">
                <HoldButton direction="FORWARD" label={S30_MOTOR_FORWARD}
                  active={motorDir === 'FORWARD'} disabled={disabledAll || !motorHoldGate}
                  onPress={motorPress} onRelease={motorRelease} testId="s30-motor-forward" />
                <HoldButton direction="BACK" label={S30_MOTOR_BACK}
                  active={motorDir === 'BACK'} disabled={disabledAll || !motorHoldGate}
                  onPress={motorPress} onRelease={motorRelease} testId="s30-motor-back" />
                <HoldButton direction="LEFT" label={S30_MOTOR_LEFT}
                  active={motorDir === 'LEFT'} disabled={disabledAll || !motorHoldGate}
                  onPress={motorPress} onRelease={motorRelease} testId="s30-motor-left" />
                <HoldButton direction="RIGHT" label={S30_MOTOR_RIGHT}
                  active={motorDir === 'RIGHT'} disabled={disabledAll || !motorHoldGate}
                  onPress={motorPress} onRelease={motorRelease} testId="s30-motor-right" />
              </div>
              <WheelSpeedView measured={wheelData.measured} command={wheelData.command} />
            </div>
          )
        })()}
        {(activeItem === 'IMU' || activeItem === 'LIDAR') && (
          <SimpleStatusMonitor item={activeItem} ros={ros} status={status} disabledAll={disabledAll} onGotoCalib={handleGotoCalib} />
        )}
      </div>
    </div>
  )
}
