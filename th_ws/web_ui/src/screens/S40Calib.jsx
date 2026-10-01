// screens/S40Calib.jsx — S-40, 校正 (SCREEN_NAMES.S40 in i18n/screens.js;
// WP-MAINT-03 / WP-UI-08). Spec-webui.md §3.14, DetailedDesign-maintenance.md §3.
//
// FSM (th_state/config/transitions.yaml T-CAL-01〜09):
//   LIST --ui.calib_item{item}--> S1 (begin_wizard)
//   S1 --ui.calib_next--> S2 (run_measurement: calib_runner が自律で走る/回る)
//   S2 --evt.calib_step_done--> S3 (build_preview)
//   S3 --ui.calib_next (guard preview_sane)--> S4 (apply_and_verify)
//   S4 --evt.calib_step_done--> LIST (commit_calib)
//   S4 --evt.calib_verify_ng--> S2 (revert_calib: 補正は適用前へ戻る)
//   * --ui.abort--> LIST (discard_calib)       * --fault.recoverable--> LIST (discard_calib)
//   LIST --ui.enter_mode{mode:OPCHECK}--> OPCHECK (T-CAL-09)
//
// BLIND（LiDAR 死角）は走らない。S2 でライブスキャン（生の /scan）上をなぞって角度帯を選び
// （parts/BlindScanSelect.jsx）、「確認」が /calib/submit{item:BLIND, arg_json:{ranges}} を送る。
// 機体が幅の上限（1 区間 30°・総幅 90°・8 区間）を検査し、超えれば PREVIEW_INSANE（S3 へ進めない）。
// S3 でプレビュー（消える点の数）→「次へ」で適用（3 ノードへ同時）と検証（S4。実測値の入力は無い）。
//
// 送信経路（e2e/s40-calib.spec.js が本番の送信を縛る。ボタンが繋がっていることを
// 実際に送ったリクエストで確かめる）:
//   項目の選択・次へ・中断  → /system/trigger の ui.calib_item / ui.calib_next / ui.abort
//   実測値の入力           → /calib/submit（SubmitCalib。FSM を経由しない calib_runner 直通）
//   再走行                 → /calib/start（StartCalib。検証 NG 後の S2 だけ）
//   履歴のロールバック      → /calib/rollback（RollbackCalib。LIST のときだけ）
// /calib/apply は使わない（適用は FSM の ui.calib_next が apply_and_verify で進める）。
//
// 状態の正本は機体側（FSM と calib_runner）。この画面は /system/state と
// /calib/status を表示しているだけで、「次へ」が押せるかも同じ条件で導く
// （screens/calibCore.js。S3 は PREVIEW_OK のときだけ）。
import { useEffect, useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useCalibStatus } from '../ros/useCalibStatus.js'
import { useCalibService } from '../ros/useCalibService.js'
import { useScan } from '../ros/useScan.js'
import BlindScanSelect from '../parts/BlindScanSelect.jsx'
import StepBar from '../parts/StepBar.jsx'
import { REJECT_REASONS } from '../i18n/reasons.js'
import * as T from '../i18n/calib.js'
import {
  CALIB_ITEM_ORDER, CALIB_STARTABLE, isWizardItem, stepNumber, activeItem, viewKind,
  canProceed, parseMeasured, rotationWarnings, imuProgress, runPercent, historyRows,
  lastCalibrated, previewText, blindLimits, blindReasonKey,
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

export default function S40Calib() {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const calib = useCalibService()
  const { status } = useCalibStatus(ros)
  // BLIND の選択は死角除去前の生スキャンを見る（マスクの内側が写っていないと選べない）。
  const rawScan = useScan(ros, '/scan')

  const fsmState = state?.state ?? null
  const disabledAll = stale || state?.mode == null
  const detail = status?.detail ?? {}

  const [tab, setTab] = useState('items')
  const [selectedItem, setSelectedItem] = useState(null)
  const [err, setErr] = useState(null)
  const [measured, setMeasured] = useState('')
  const [submitMsg, setSubmitMsg] = useState(null)
  const [retryErr, setRetryErr] = useState(null)
  const [committed, setCommitted] = useState(false)
  const [rollbackMsg, setRollbackMsg] = useState(null)

  const item = activeItem(status, selectedItem, fsmState)
  const kind = viewKind({ fsmState, item, status })
  const step = stepNumber(fsmState)
  const inWizard = step > 0
  const proceed = canProceed({ fsmState, item, status })

  // LIST に戻ったら入力欄・押した項目を畳む。ステップが変わったら入力と直前の応答も畳む。
  useEffect(() => {
    if (fsmState === 'LIST') setSelectedItem(null)
    setMeasured('')
    setSubmitMsg(null)
    setRetryErr(null)
  }, [fsmState])

  // 確定の通知は calib_runner が 1 回だけ result=COMMITTED で出す。次の校正を始めるまで残す。
  useEffect(() => {
    if (status?.result === 'COMMITTED') setCommitted(true)
  }, [status])
  useEffect(() => { if (inWizard) setCommitted(false) }, [inWizard])

  async function handleStart(it) {
    if (fsmState !== 'LIST' || disabledAll) return
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

  async function handleNext() {
    setErr(null)
    try {
      const res = await sendTrigger('ui.calib_next', { item })
      if (!res?.accepted) setErr(rejectText(res))
    } catch { /* rosbridge 一時失敗。留まる */ }
  }

  async function handleAbort() {
    try { await sendTrigger('ui.abort', item ? { item } : {}) } catch { /* 留まる */ }
  }

  async function handleFinish() {
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

  // BLIND: 選んだ角度帯を /calib/submit へ（measured は使わない。ranges が本体）。
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
  const unit = T.S40_UNIT[item] ?? ''

  // ── 右ペイン ─────────────────────────────────────────────
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

  function renderBody() {
    if (kind === 'list') {
      return (
        <div className="card" data-testid="s40-placeholder">
          {committed && <p className="note" data-testid="s40-committed">{T.S40_COMMITTED}</p>}
          <p className="note">{T.S40_PLACEHOLDER}</p>
          <button type="button" className="btn wide" disabled={disabledAll}
            data-testid="s40-to-opcheck" onClick={handleToOpcheck}>
            {T.S40_BACK_TO_OPCHECK}
          </button>
        </div>
      )
    }
    const warnings = item === 'ROTATION' ? rotationWarnings(detail) : []
    const blindAfter = (() => {
      try { return JSON.parse(status?.preview_after || '{}') } catch { return {} }
    })()
    const blindInsane = status?.result === 'PREVIEW_INSANE'
    const blindNg = item === 'BLIND' && detail?.phase === 'RETRY_WAIT' && detail?.reason
    return (
      <div className="card" data-testid={`s40-view-${kind}`}>
        <h3>{T.S40_ITEM_LABELS[item] ?? item}</h3>
        {isWizardItem(item) && <p className="s40-step-title" data-testid="s40-step-title">{T.s40StepTitle(step)}</p>}
        {kind === 'guide' && (
          <>
            <p className="note" data-testid="s40-guide">
              {item === 'IMU' ? T.S40_IMU_GUIDE : T.s40GuideText(item, detail)}
            </p>
            {warnings.map((w) => (
              <p key={w} className="s40-warn" data-testid="s40-warning">{T.S40_WARNINGS[w] ?? w}</p>
            ))}
            <button type="button" className="btn wide primary" disabled={disabledAll || !proceed}
              data-testid="s40-next" onClick={handleNext}>
              {item === 'BLIND' ? T.S40_GUIDE_NEXT_BLIND : T.S40_GUIDE_NEXT}
            </button>
          </>
        )}
        {(kind === 'blind_select' || kind === 'blind_preview') && (
          <>
            {blindNg && (
              <p className="s40-warn" data-testid="s40-retry-title">
                {T.S40_RETRY_TITLE}（{T.s40ReasonLabel(detail.reason)}）
              </p>
            )}
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
            {kind === 'blind_preview' && (
              <button type="button" className="btn wide primary" disabled={disabledAll || !proceed}
                data-testid="s40-next" onClick={handleNext}>
                {T.S40_BLIND_NEXT}
              </button>
            )}
          </>
        )}
        {kind === 'running' && (
          <>
            <p className="note">{T.S40_RUNNING}</p>
            <Bar percent={runPercent(detail)} testId="s40-run-bar" />
          </>
        )}
        {kind === 'retry' && (
          <>
            <p className="s40-warn" data-testid="s40-retry-title">
              {detail?.phase === 'RETRY_WAIT' && detail?.reason
                ? `${T.S40_RETRY_TITLE}（${T.s40ReasonLabel(detail.reason)}）`
                : T.S40_RETRY_TITLE}
            </p>
            <button type="button" className="btn wide primary" disabled={disabledAll}
              data-testid="s40-retry" onClick={handleRetry}>
              {T.S40_RETRY_BUTTON}
            </button>
            {retryErr && <p className="note" data-testid="s40-retry-err">{retryErr}</p>}
          </>
        )}
        {(kind === 'measure' || kind === 'preview_ok') && (
          <>
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
            <button type="button" className="btn wide primary" disabled={disabledAll || !proceed}
              data-testid="s40-next" onClick={handleNext}>
              {T.S40_PREVIEW_NEXT}
            </button>
          </>
        )}
        {kind === 'verifying' && (
          <>
            <p className="note" data-testid="s40-verifying">
              {item === 'BLIND' ? T.S40_BLIND_VERIFYING : T.S40_RUNNING_VERIFY}
            </p>
            {item !== 'BLIND' && <Bar percent={runPercent(detail)} testId="s40-run-bar" />}
          </>
        )}
        {kind === 'verify_input' && (
          <>
            {renderMeasureInput(T.S40_VERIFY_LABEL, T.S40_VERIFY_SUBMIT, 's40-verify-submit')}
            {submitMsg && !submitMsg.ok && (
              <p className="note" data-testid="s40-verify-rejected">{T.S40_SUBMIT_REJECTED}</p>
            )}
          </>
        )}
        {kind === 'imu_progress' && (() => {
          const prog = imuProgress(detail)
          return (
            <>
              <Bar percent={prog.percent} testId="s40-imu-bar" />
              <ul className="s40-imu-parts">
                {prog.parts.map((p) => (
                  <li key={p.key} data-testid={`s40-imu-${p.key}`}>
                    {T.S40_IMU_PARTS[p.key]}: {p.value}/3
                  </li>
                ))}
              </ul>
            </>
          )
        })()}
        {kind === 'imu_done' && (
          <>
            <p className="note" data-testid="s40-imu-done">{T.S40_IMU_DONE}</p>
            <button type="button" className="btn wide primary" disabled={disabledAll || !proceed}
              data-testid="s40-next" onClick={handleNext}>
              {T.S40_IMU_COMMIT}
            </button>
          </>
        )}
      </div>
    )
  }

  const stepViews = T.S40_STEP_LABELS.map((label, i) => ({
    id: `s${i + 1}`, label, done: step > i + 1,
  }))

  return (
    <div className="screen two-col" id="s40">
      {isWizardItem(item) && inWizard && (
        <div className="stepbar-cell">
          <StepBar steps={stepViews} currentIndex={step - 1} testId="s40" />
        </div>
      )}
      <div className="left-col">
        <div className="top-actions sticky">
          <button type="button" className="btn sm danger" data-testid="s40-abort"
            disabled={disabledAll || !inWizard} onClick={handleAbort}>
            {T.S40_ABORT}
          </button>
          <button type="button" className="btn sm" data-testid="s40-finish"
            disabled={disabledAll || inWizard} onClick={handleFinish}>
            {T.S40_FINISH}
          </button>
        </div>
        <div className="card">
          <div className="row">
            <button type="button" className={`btn sm ${tab === 'items' ? 'on' : ''}`}
              data-testid="s40-tab-items" onClick={() => setTab('items')}>
              {T.S40_TAB_ITEMS}
            </button>
            <button type="button" className={`btn sm ${tab === 'history' ? 'on' : ''}`}
              data-testid="s40-tab-history" onClick={() => setTab('history')}>
              {T.S40_TAB_HISTORY}
            </button>
          </div>
          {tab === 'items' && (
            <>
              {CALIB_ITEM_ORDER.map((it) => {
                const startable = CALIB_STARTABLE.includes(it)
                const last = lastCalibrated(detail, it)
                return (
                  <div key={it} className={`s40-row ${item === it ? 'running' : ''}`}
                    data-testid={`s40-item-${it}`}>
                    <div className="s40-row-main">
                      <span className="s40-row-label">{T.S40_ITEM_LABELS[it]}</span>
                      <span className="note">
                        {`${T.S40_LAST_CALIBRATED}: ${last || T.S40_NEVER_CALIBRATED}`}
                      </span>
                      {it === 'ROTATION' && (
                        <span className="s40-warn" data-testid="s40-rotation-first">{T.S40_ROTATION_FIRST}</span>
                      )}
                    </div>
                    <button type="button" className="btn sm" data-testid={`s40-start-${it}`}
                      disabled={disabledAll || fsmState !== 'LIST' || !startable}
                      onClick={() => handleStart(it)}>
                      {T.S40_START}
                    </button>
                  </div>
                )
              })}
              {err && <p className="note" data-testid="s40-err">{err}</p>}
            </>
          )}
          {tab === 'history' && (
            <>
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
                        </div>
                        <button type="button" className="btn sm"
                          data-testid={`s40-rollback-${it}-${r.generation}`}
                          disabled={disabledAll || fsmState !== 'LIST'}
                          onClick={() => handleRollback(it, r.generation)}>
                          {T.S40_HISTORY_ROLLBACK}
                        </button>
                      </div>
                    ))}
                  </div>
                )
              })}
              {rollbackMsg && <p className="note" data-testid="s40-rollback-msg">{rollbackMsg}</p>}
            </>
          )}
        </div>
      </div>
      <div>{renderBody()}</div>
    </div>
  )
}
