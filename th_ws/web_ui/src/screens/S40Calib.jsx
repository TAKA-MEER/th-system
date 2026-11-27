// screens/S40Calib.jsx — S-40, 校正 (Spec-webui.md §3.14).
// 型「手順を進める」。タブは無い。左=項目一覧（前回の日付と誤差）、
// 右=項目名・段の印（「Step」の文字にしない）・「履歴」。
// 下端=左「やめる」／右=進むボタン（語は項目と段で決める。直進・旋回の
// 「走らせる」だけ塗る。IMU は「始める」）。
// 校正の途中でも、一覧の別の項目・「履歴」・「戻る」は押せる。押すと
// 実行中の校正をやめ（補正値を確定しない）、押したものへ移る
// （Spec-modes.md SM-3.1.2-145/-146）。機体側にやめる操作（ui.abort）を
// 先に送ってから移る（校正の再開の遷移は LIST 限定のため）。
//
// FSM (th_state/config/transitions.yaml T-CAL-01〜09):
//   LIST --ui.calib_item--> S1 (begin_wizard)
//   S1 --ui.calib_next--> S2 (run_measurement)
//   S2 --evt.calib_step_done--> S3 (build_preview)
//   S3 --ui.calib_next--> S4 (apply_and_verify)
//   S4 --evt.calib_step_done--> LIST (commit_calib)
//   S4 --evt.calib_verify_ng--> S2 (revert_calib)
//   * --ui.abort--> LIST (discard_calib)   * --fault.recoverable--> LIST
//   LIST --ui.enter_mode{mode:OPCHECK}--> OPCHECK (T-CAL-09)
//
// 送信経路（e2e/s40-calib.spec.js が本番の送信を縛る）:
//   項目の選択（LIST の「進む」）・次へ・中断 → /system/trigger の
//     ui.calib_item / ui.calib_next / ui.abort
//   実測値の入力 → /calib/submit。再走行 → /calib/start。
//   履歴のロールバック → /calib/rollback（LIST のときだけ）。
//   途中の項目切替・履歴は ui.abort の受理を待ってから移る（受理＝LIST）。
import { useEffect, useRef, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useCalibStatus } from '../ros/useCalibStatus.js'
import { useCalibService } from '../ros/useCalibService.js'
import { useScan } from '../ros/useScan.js'
import { takeCalibGuide } from './calibGuide.js'
import { s40CalibGuideText } from '../i18n/screens.js'
import BlindScanSelect from '../parts/BlindScanSelect.jsx'
import ArmedButton from '../parts/ArmedButton.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import ReasonButton from '../parts/ReasonButton.jsx'
import { StepsLayout } from '../shell/Layouts.jsx'
import { REJECT_REASONS } from '../i18n/reasons.js'
import * as T from '../i18n/calib.js'
import {
  CALIB_ITEM_ORDER, CALIB_STARTABLE, isWizardItem, stepNumber, activeItem, viewKind,
  canProceed, parseMeasured, rotationWarnings, imuProgress, runPercent, historyRows,
  lastCalibrated, lastVerification, previewText, blindLimits, blindReasonKey,
} from './calibCore.js'

function rejectText(res) {
  const key = res?.reject_reason_key
  return key ? (REJECT_REASONS[key] ?? key) : T.S40_SUBMIT_REJECTED
}

function Bar({ percent, testId }) {
  return (
    <div className="s40-bar" role="progressbar" aria-valuenow={percent} aria-valuemin={0}
      aria-valuemax={100} data-testid={testId}>
      <i style={{ width: `${percent}%` }} />
    </div>
  )
}

// 段ごとの図（Spec §3.14）。機体の後ろに走行の軌跡を描く。
function StraightFig() {
  return (
    <svg viewBox="0 0 520 200" role="img" aria-label="1 メートルの直線の図" style={{ height: '100%', width: 'auto' }}>
      <rect x="20" y="70" width="70" height="70" rx="4" fill="var(--inset)" stroke="var(--tx)" strokeWidth="3" />
      <path d="M40 120L80 105L40 90Z" fill="var(--tx)" />
      <path d="M100 105H470" stroke="currentColor" strokeWidth="4" strokeDasharray="14 10" />
      <path d="M470 80v50" stroke="currentColor" strokeWidth="5" />
      <text x="285" y="70" textAnchor="middle" fontSize="30" fontWeight="800" fill="currentColor">1 m</text>
    </svg>
  )
}

function RotationFig() {
  return (
    <svg viewBox="0 0 400 260" role="img" aria-label="半周の弧の図" style={{ height: '100%', width: 'auto' }}>
      <circle cx="200" cy="130" r="60" fill="var(--inset)" stroke="var(--tx)" strokeWidth="3" />
      <path d="M200 78 L188 100 L212 100 Z" fill="var(--tx)" />
      <path d="M110 130 A90 90 0 0 1 290 130" fill="none" stroke="currentColor" strokeWidth="6" strokeLinecap="round" strokeDasharray="12 8" />
      <path d="M290 130 l-8 -20 m8 20 l-22 -2 M110 130 l8 -20 m-8 20 l22 -2" stroke="currentColor" strokeWidth="6" strokeLinecap="round" fill="none" />
      <text x="200" y="245" textAnchor="middle" fontSize="26" fontWeight="800" fill="currentColor">半周</text>
    </svg>
  )
}

// IMU を機体から外して傾ける図（6 つの向き）。
function ImuFig() {
  const cube = (x, y, rot) => (
    <g transform={`translate(${x} ${y}) rotate(${rot})`} key={`${x}-${y}`}>
      <rect x="-26" y="-18" width="52" height="36" rx="4" fill="none" stroke="currentColor" strokeWidth="4" />
      <circle cx="-10" cy="-6" r="4" fill="currentColor;" />
    </g>
  )
  return (
    <svg viewBox="0 0 520 220" role="img" aria-label="IMU を外して傾ける図" style={{ height: '100%', width: 'auto' }}>
      {cube(60, 60, 0)}{cube(190, 60, 90)}{cube(320, 60, 180)}{cube(450, 60, 270)}
      {cube(125, 165, -25)}{cube(385, 165, 25)}
    </svg>
  )
}

export default function S40Calib() {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const calib = useCalibService()
  const { status } = useCalibStatus(ros)
  const rawScan = useScan(ros, '/scan')

  const fsmState = state?.state ?? null
  const disabledAll = stale || state?.mode == null
  const detail = status?.detail ?? {}

  const [selectedItem, setSelectedItem] = useState(null)
  const [err, setErr] = useState(null)
  const [measured, setMeasured] = useState('')
  const [submitMsg, setSubmitMsg] = useState(null)
  const [retryErr, setRetryErr] = useState(null)
  const [committed, setCommitted] = useState(false)
  const [rollbackMsg, setRollbackMsg] = useState(null)
  const [showHistory, setShowHistory] = useState(false)
  const [busy, setBusy] = useState(false)

  const item = activeItem(status, selectedItem, fsmState) ?? selectedItem ?? (showHistory ? 'LINEAR' : null) ?? 'LINEAR'
  const kind = viewKind({ fsmState, item, status })
  const step = stepNumber(fsmState)
  const inWizard = step > 0
  const proceed = canProceed({ fsmState, item, status, scan: rawScan })

  useEffect(() => {
    if (fsmState === 'LIST') { setSelectedItem(null); setShowHistory(false) }
    setMeasured('')
    setSubmitMsg(null)
    setRetryErr(null)
  }, [fsmState])

  const [guide, setGuide] = useState(null)
  const guideTakenRef = useRef(false)
  useEffect(() => {
    if (guideTakenRef.current) return
    guideTakenRef.current = true
    const g = takeCalibGuide()
    if (g) setGuide(g)
  }, [])
  useEffect(() => { if (inWizard) setGuide(null) }, [inWizard])
  // 誘導先の項目を選んだ状態で見せる（自動では始めない。開始は人が「進む」を押す）。
  useEffect(() => {
    if (guide?.target && !selectedItem && fsmState === 'LIST') setSelectedItem(guide.target)
  }, [guide, selectedItem, fsmState])
  const showGuide = guide && fsmState === 'LIST' && !showHistory

  useEffect(() => {
    if (status?.result === 'COMMITTED') setCommitted(true)
  }, [status])
  useEffect(() => { if (inWizard) setCommitted(false) }, [inWizard])

  async function sendAbort() {
    // 途中をやめる（補正値を確定しない）。受理＝LIST に戻った。
    try {
      const res = await sendTrigger('ui.abort', item ? { item } : {})
      return !!res?.accepted
    } catch {
      return false
    }
  }

  // LIST で「進む」を押す。S1 へ入る（ui.calib_item）。
  // 途中の切替の後半（abort 受理後）もここを通る：受理＝LIST のはずなので送る。
  async function startItem(it, force = false) {
    if ((!force && fsmState !== 'LIST') || disabledAll) return
    setErr(null)
    setSelectedItem(it)
    try {
      const res = await sendTrigger('ui.calib_item', { item: it })
      if (!res?.accepted) {
        setSelectedItem(null)
        setErr(rejectText(res))
      }
    } catch {
      setSelectedItem(null)
    }
  }

  // 途中の項目選び直し・履歴・戻る（SM-3.1.2-145/-146）。
  // 機体側にやめる操作（ui.abort）を先に送って受理を待ってから移る。
  async function switchWhileRunning(after) {
    if (busy) return
    setBusy(true)
    setErr(null)
    const ok = await sendAbort()
    setBusy(false)
    if (!ok) {
      setErr(T.S40_ROLLBACK_FAILED)
      return
    }
    await after()
  }

  function handleRow(it) {
    if (disabledAll) return
    if (inWizard) {
      switchWhileRunning(() => startItem(it, true))
      return
    }
    setShowHistory(false)
    setSelectedItem(it)
  }

  async function handleNext() {
    setErr(null)
    if (fsmState === 'LIST') {
      await startItem(selectedItem ?? item)
      return
    }
    try {
      const res = await sendTrigger('ui.calib_next', { item })
      if (!res?.accepted) setErr(rejectText(res))
    } catch { /* rosbridge 一時失敗。留まる */ }
  }

  async function handleAbort() {
    try { await sendTrigger('ui.abort', item ? { item } : {}) } catch { /* 留まる */ }
  }

  async function handleBack() {
    // 途中ならやめて（ui.abort）から抜ける（SM-3.1.2-146）。受理を待ってから ui.finish。
    if (inWizard) {
      await switchWhileRunning(async () => {
        try { await sendTrigger('ui.finish') } catch { /* 留まる */ }
      })
      return
    }
    try { await sendTrigger('ui.finish') } catch { /* 留まる */ }
  }

  async function handleToOpcheck() {
    setErr(null)
    try {
      const res = await sendTrigger('ui.enter_mode', { mode: 'OPCHECK' })
      if (!res?.accepted) setErr(rejectText(res))
    } catch { /* 留まる */ }
  }

  async function handleSubmit() {
    const value = parseMeasured(measured)
    if (value == null || !item) return
    setSubmitMsg(null)
    try {
      const res = await calib.submit(item, value)
      setSubmitMsg({
        ok: !!res?.success,
        before: previewText(res?.preview_before),
        after: previewText(res?.preview_after),
      })
    } catch {
      setSubmitMsg({ ok: false, before: '', after: '' })
    }
  }

  async function handleBlindSubmit(ranges) {
    setSubmitMsg(null)
    try {
      const res = await calib.submit('BLIND', 0, { ranges })
      setSubmitMsg({ ok: !!res?.success, before: '', after: '' })
    } catch {
      setSubmitMsg({ ok: false, before: '', after: '' })
    }
  }

  async function handleRetry() {
    setRetryErr(null)
    try {
      const res = await calib.start(item)
      if (!res?.started) setRetryErr(res?.message || T.S40_START_REJECTED)
    } catch {
      setRetryErr(T.S40_START_REJECTED)
    }
  }

  async function handleRollback(it, generation) {
    setRollbackMsg(null)
    try {
      const res = await calib.rollback(it, generation)
      setRollbackMsg(res?.success ? T.S40_ROLLBACK_DONE : T.S40_ROLLBACK_FAILED)
    } catch {
      setRollbackMsg(T.S40_ROLLBACK_FAILED)
    }
  }

  const measuredValue = parseMeasured(measured)

  // 段の印（P2。文字の「Step n/4」にしない。aria-current で現在段を示す）。
  const stepLabels = T.s40StepLabels(item)
  const segIndex = item === 'IMU' ? (step <= 1 ? 0 : step === 2 ? 1 : 2) : item === 'BLIND' ? (step <= 1 ? 0 : step === 2 ? 1 : 2) : step - 1
  // BLIND の S3・S4 は「確定」の段に寄せる（検証はその一部の合否で示す）。
  const segCur = item === 'BLIND' && step >= 3 ? 2 : Math.max(0, segIndex)
  const stepsRow = inWizard ? (
    <div className="steps" data-testid="s40-steps" role="list">
      {stepLabels.map((l, i) => (
        <span key={l} className={`st${i < segCur ? ' done' : i === segCur ? ' cur' : ''}`}
          role="listitem" aria-current={i === segCur ? 'step' : undefined}
          data-testid={`s40-step-${i + 1}`}>
          {l}
        </span>
      ))}
    </div>
  ) : (
    <div className="steps" data-testid="s40-steps" role="list">
      {stepLabels.map((l, i) => <span key={l} className="st" role="listitem" data-testid={`s40-step-${i + 1}`}>{l}</span>)}
    </div>
  )

  // 進むボタンの語（項目と段で決める。§3.14）。
  // LIST（開始前）: 直進・旋回=走らせる、それ以外=始める。
  const nextLabel = fsmState === 'LIST'
    ? (item === 'IMU' ? T.S40_NEXT_START : item === 'LINEAR' || item === 'ROTATION' ? T.S40_NEXT_RUN : T.S40_NEXT_START)
    : (kind === 'idle' ? T.S40_NEXT_START : T.s40NextLabel(item, step))
  // S1 の BLIND はスキャンの表示へ進む（走らない）。語は「走らせる」にしない。
  const nextLabelFinal = (item === 'BLIND' && step === 1) ? T.S40_GUIDE_NEXT_BLIND : nextLabel
  // 走らせるだけ塗る（機体が動くもの。D103）。S1・LIST の LINEAR/ROTATION が該当。
  const nextPaint = nextLabelFinal === T.S40_NEXT_RUN && (item === 'LINEAR' || item === 'ROTATION')
    && (step === 0 || step === 1)

  const leftList = (
    <>
      <div className="lhd">項目</div>
      {showGuide && (
        <p className="s40-guide" data-testid="s40-calib-guide">{s40CalibGuideText(guide)}</p>
      )}
      {CALIB_ITEM_ORDER.map((it) => {
        const last = lastCalibrated(detail, it)
        const verify = lastVerification(detail, it)
        const on = showHistory ? false : item === it
        return (
          <button
            key={it}
            type="button"
            className={`lrow${on ? ' on' : ''}${showGuide && guide.target === it ? ' guided' : ''}`}
            data-testid={`s40-item-${it}`}
            onClick={() => handleRow(it)}
          >
            <span className="nm">
              {T.S40_ITEM_LABELS[it]}
              <small>{`${T.S40_LAST_CALIBRATED}: ${last || T.S40_NEVER_CALIBRATED}`}{verify ? `　${T.S40_LAST_VERIFY}: ${verify}` : ''}</small>
            </span>
          </button>
        )
      })}
      {err && <p className="note" data-testid="s40-err">{err}</p>}
    </>
  )

  function renderMeasureInput(labelMap, submitLabel, testId) {
    return (
      <div className="row mt">
        <label className="grow">
          <span className="note">{labelMap[item]}</span>
          <input type="number" step="any" min="0" inputMode="decimal" className="s40-input"
            data-testid="s40-measured" value={measured} disabled={disabledAll}
            onChange={(e) => setMeasured(e.target.value)} />
        </label>
        <button type="button" className="btn" data-testid={testId}
          disabled={disabledAll || measuredValue == null} onClick={handleSubmit}>
          {submitLabel}
        </button>
      </div>
    )
  }

  function renderHistory() {
    return (
      <div data-testid="s40-history-view">
        <h3>{T.S40_HISTORY_TITLE}</h3>
        {CALIB_STARTABLE.map((it) => {
          const rows = historyRows(detail, it)
          return (
            <div key={it} className="s40-hist" data-testid={`s40-history-${it}`}>
              <h4>{T.S40_ITEM_LABELS[it]}</h4>
              {rows.length === 0 && <p className="note">{T.S40_HISTORY_EMPTY}</p>}
              {rows.map((r) => (
                <div key={r.generation} className="s40-row">
                  <div className="s40-row-main">
                    <span>{T.s40Generation(r.generation)}: {r.text}</span>
                    <span className="note">{r.calibratedAt}</span>
                    {r.verify && <span className="note">{`${T.S40_HISTORY_VERIFY}: ${r.verify}`}</span>}
                  </div>
                  <ArmedButton
                    idleLabel={T.S40_HISTORY_ROLLBACK}
                    armedLabel={T.S40_HISTORY_ROLLBACK_ARMED}
                    className="sm"
                    testId={`s40-rollback-${it}-${r.generation}`}
                    disabled={disabledAll || fsmState !== 'LIST'}
                    onConfirm={() => handleRollback(it, r.generation)}
                  />
                </div>
              ))}
            </div>
          )
        })}
        {rollbackMsg && <p className="note" data-testid="s40-rollback-msg">{rollbackMsg}</p>}
      </div>
    )
  }

  function renderBody() {
    if (showHistory) return renderHistory()
    if (kind === 'list') {
      return (
        <div data-testid="s40-placeholder">
          {committed && <p className="note" data-testid="s40-committed">{T.S40_COMMITTED}</p>}
          {committed && (
            <div className="row mt">
              <HelpTip helpKey="s40-opcheck">
                <button type="button" className="btn primary" data-testid="s40-to-opcheck" onClick={handleToOpcheck}>
                  {T.S40_TO_CHECK}
                </button>
              </HelpTip>
            </div>
          )}
          <p className="note">{T.S40_PLACEHOLDER}</p>
          {(item === 'LINEAR' || item === 'ROTATION') && (
            <div className="fig">{item === 'LINEAR' ? <StraightFig /> : <RotationFig />}</div>
          )}
          {item === 'IMU' && (
            <>
              <div className="fig"><ImuFig /></div>
              <p className="oneline">{T.S40_IMU_GUIDE}</p>
            </>
          )}
          {item === 'BLIND' && <p className="note">{T.s40GuideText('BLIND', detail)}</p>}
        </div>
      )
    }
    if (kind === 'guide') {
      return (
        <div data-testid="s40-view-guide">
          {item === 'LINEAR' && <div className="fig"><StraightFig /></div>}
          {item === 'ROTATION' && <div className="fig"><RotationFig /></div>}
          {item === 'IMU' && <div className="fig"><ImuFig /></div>}
          <p className="note" data-testid="s40-guide">
            {item === 'IMU' ? T.S40_IMU_GUIDE : T.s40GuideText(item, detail)}
          </p>
          {rotationWarnings(detail).map((w) => (
            <p key={w} className="s40-warn" data-testid="s40-warning">{T.S40_WARNINGS[w] ?? w}</p>
          ))}
        </div>
      )
    }
    const warnings = item === 'ROTATION' ? rotationWarnings(detail) : []
    const blindAfter = (() => {
      try { return JSON.parse(status?.preview_after || '{}') } catch { return {} }
    })()
    const blindInsane = status?.result === 'PREVIEW_INSANE'
    const blindNg = item === 'BLIND' && detail?.phase === 'RETRY_WAIT' && detail?.reason
    if (kind === 'blind_select' || kind === 'blind_preview') {
      return (
        <div data-testid={`s40-view-${kind}`}>
          {blindNg && (
            <p className="s40-warn" data-testid="s40-retry-title">
              {T.S40_RETRY_TITLE}（{T.s40ReasonLabel(detail.reason)}）
            </p>
          )}
          {warnings.map((w) => (
            <p key={w} className="s40-warn" data-testid="s40-warning">{T.S40_WARNINGS[w] ?? w}</p>
          ))}
          <BlindScanSelect scan={rawScan} registeredFlat={detail?.current?.blind_angle_ranges ?? []}
            limits={blindLimits(detail)} disabled={disabledAll}
            problemKey={blindInsane ? blindReasonKey(blindAfter.reason) || 'invalid_format' : null}
            submitLabel={kind === 'blind_preview' ? T.S40_BLIND_RESUBMIT : T.S40_BLIND_BUTTON_SUBMIT}
            onSubmit={handleBlindSubmit} />
          {submitMsg && !submitMsg.ok && !blindInsane && (
            <p className="note" data-testid="s40-submit-rejected">{T.S40_SUBMIT_REJECTED}</p>
          )}
          {kind === 'blind_preview' && (
            <div className="s40-preview" data-testid="s40-preview">
              <h4>{T.S40_PREVIEW_TITLE}</h4>
              <div data-testid="s40-preview-after">{T.s40BlindPreview(blindAfter.masked_points ?? 0, blindAfter.total_deg ?? 0)}</div>
            </div>
          )}
        </div>
      )
    }
    if (kind === 'running') {
      return (
        <div data-testid="s40-view-running">
          <p className="note">{T.S40_RUNNING}</p>
          <Bar percent={runPercent(detail)} testId="s40-run-bar" />
        </div>
      )
    }
    if (kind === 'retry') {
      return (
        <div data-testid="s40-view-retry">
          <p className="s40-warn" data-testid="s40-retry-title">
            {detail?.phase === 'RETRY_WAIT' && detail?.reason
              ? `${T.S40_RETRY_TITLE}（${T.s40ReasonLabel(detail.reason)}）`
              : T.S40_RETRY_TITLE}
          </p>
          <button type="button" className="btn grow" data-testid="s40-retry" onClick={handleRetry}>
            {T.S40_RETRY_BUTTON}
          </button>
          {retryErr && <p className="note" data-testid="s40-retry-err">{retryErr}</p>}
        </div>
      )
    }
    if (kind === 'measure' || kind === 'preview_ok') {
      return (
        <div data-testid={`s40-view-${kind}`}>
          {renderMeasureInput(T.S40_MEASURE_LABEL, T.S40_MEASURE_SUBMIT, 's40-submit')}
          {status?.result === 'PREVIEW_INSANE' && (
            <p className="s40-warn" data-testid="s40-insane">{T.S40_PREVIEW_INSANE}</p>
          )}
          {kind === 'preview_ok' && (
            <div className="s40-preview" data-testid="s40-preview">
              <h4>{T.S40_PREVIEW_TITLE}</h4>
              <div>{T.S40_PREVIEW_BEFORE}: <span data-testid="s40-preview-before">{previewText(status?.preview_before)}</span></div>
              <div>{T.S40_PREVIEW_AFTER}: <span data-testid="s40-preview-after">{previewText(status?.preview_after)}</span></div>
            </div>
          )}
          {submitMsg && !submitMsg.ok && (
            <p className="note" data-testid="s40-submit-rejected">{T.S40_SUBMIT_REJECTED}</p>
          )}
        </div>
      )
    }
    if (kind === 'verifying') {
      return (
        <div data-testid="s40-view-verifying">
          <p className="note" data-testid="s40-verifying">
            {item === 'BLIND' ? T.S40_BLIND_VERIFYING : T.S40_RUNNING_VERIFY}
          </p>
          {item !== 'BLIND' && <Bar percent={runPercent(detail)} testId="s40-run-bar" />}
        </div>
      )
    }
    if (kind === 'verify_input') {
      return (
        <div data-testid="s40-view-verify-input">
          {renderMeasureInput(T.S40_VERIFY_LABEL, T.S40_VERIFY_SUBMIT, 's40-verify-submit')}
          {submitMsg && !submitMsg.ok && (
            <p className="note" data-testid="s40-verify-rejected">{T.S40_SUBMIT_REJECTED}</p>
          )}
        </div>
      )
    }
    if (kind === 'imu_progress') {
      const prog = imuProgress(detail)
      return (
        <div data-testid="s40-view-imu_progress">
          <Bar percent={prog.percent} testId="s40-imu-bar" />
          <ul className="s40-imu-parts">
            {prog.parts.map((p) => (
              <li key={p.key} data-testid={`s40-imu-${p.key}`}>
                {T.S40_IMU_PARTS[p.key]}: {p.value}/3
              </li>
            ))}
          </ul>
        </div>
      )
    }
    if (kind === 'imu_done') {
      return (
        <div data-testid="s40-view-imu_done">
          <p className="note" data-testid="s40-imu-done">{T.S40_IMU_DONE}</p>
        </div>
      )
    }
    return <div>{T.S40_PLACEHOLDER}</div>
  }

  const findings = showHistory
  // 進むボタンが押せる段（S1・S3）。BLIND の S3 も次へ。
  const canNext = fsmState === 'LIST' ? !showHistory : proceed // showHistory は foot の「閉じる」に差し替える
  const nextDisabled = disabledAll || !canNext
  const nextBtn = findings ? (
    <HelpTip helpKey="s40-history-close">
      <button type="button" className="btn primary" data-testid="s40-history-close" onClick={() => setShowHistory(false)}>
        {T.S40_CLOSE}
      </button>
    </HelpTip>
  ) : (
    <HelpTip helpKey="s40-next">
      <ReasonButton
        data-testid="s40-next"
        disabled={nextDisabled}
        reason={nextDisabled ? (kind === 'running' || kind === 'verifying' ? '走っています' : canNext ? null : 'まだ進めません') : null}
        onClick={handleNext}
        className={nextPaint ? 'btn-adv' : 'primary'}
      >
        {nextLabelFinal}
      </ReasonButton>
    </HelpTip>
  )

  return (
    <div id="s40" className="screen">
      <StepsLayout
        opbar={(
          <HelpTip helpKey="s40-abort">
            <button type="button" className="btn exit" data-testid="s40-back" disabled={disabledAll} onClick={handleBack}>
              {T.S40_BACK}
            </button>
          </HelpTip>
        )}
        list={leftList}
        body={(
          <>
            <div className="phd">
              <span className="pt">{T.S40_ITEM_LABELS[item] ?? item}</span>
              {stepsRow}
              <HelpTip helpKey="s40-history">
                <button type="button" className="btn" data-testid="s40-history" onClick={() => {
                  if (inWizard) switchWhileRunning(async () => { setShowHistory(true) })
                  else setShowHistory(true)
                }}>
                  {T.S40_HISTORY_TITLE}
                </button>
              </HelpTip>
            </div>
            <div data-testid="s40-body">{renderBody()}</div>
          </>
        )}
        foot={(
          <>
            <HelpTip helpKey="s40-abort">
              <ReasonButton
                data-testid="s40-abort"
                disabled={!inWizard || disabledAll}
                reason={!inWizard ? '校正の途中だけ使えます' : null}
                onClick={handleAbort}
                className="primary"
              >
                {T.S40_QUIT}
              </ReasonButton>
            </HelpTip>
            <span className="grow" />
            {nextBtn}
          </>
        )}
      />
    </div>
  )
}
