// screens/S01Main.jsx — S-01, the main menu screen (SCREEN_NAMES.S01 in
// i18n/screens.js; DetailedDesign-wp1.md WP-UI-02).
//
// Two independent things live here: mode selection (menuItems() drives
// which of the 10 buttons are enabled -- screens/mainMenuItems.js) and the
// shutdown flow (SHUTDOWN_TITLE in i18n/screens.js; DetailedDesign-webui.md
// §8.3 / DetailedDesign-state.md §12.5's 5 steps). Both use the shared W-4
// confirm window (shell/confirmWindow.js) for anything beyond a single
// button press, since M-3 forbids expanding the unsaved list into the card
// itself.
//
// Known limitation (DetailedDesign-wp1.md WP-UI-02 §11, and this packet's
// own out-of-scope column): no functional node sets SystemState.unsaved
// yet, so /shutdown/prepare's list is expected to be empty in practice, and
// the per-item save/discard buttons below only mark an item as locally
// resolved -- there is no per-item save/discard service to call yet (each
// item type would need a different one: /map_session/save, /route/save,
// ...). This screen wires the 5-step *path* through; real persistence is
// future work.
import { useState, useEffect } from 'react'
import { createPortal } from 'react-dom'
import { useSystemState } from '../ros/useSystemState.js'
import { useAutoCheckStatus } from '../ros/useAutoCheckStatus.js'
import { useLinkStatus } from '../ros/useLinkStatus.js'
import { linkDownKeys } from '../ros/linkStatusState.js'
import { autoCheckWarns } from '../ros/autoCheckState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useStdTrigger } from '../ros/useStdTrigger.js'
import { SERVICES } from '../ros/topics.js'
import { useConfirmWindow } from '../shell/confirmWindow.js'
import ArmedButton from '../parts/ArmedButton.jsx'
import { menuItems, MENU_GROUPS, needsFinishEscape } from './mainMenuItems.js'
import attributes from '../generated/attributes.json'
import modeEntry from '../generated/mode_entry.json'
import { modeLabel } from '../i18n/modes.js'
import { reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import {
  GROUP_MOVE_TITLE, GROUP_FIELD_TITLE, GROUP_MAINT_TITLE, S01_SETTINGS,
  S01_FINISH_ESCAPE,
  S01_NET_TITLE, S01_NET_OK, S01_NET_CHECKING, S01_NET_WARN,
  S01_NET_SUPPRESSED, S01_NET_OPEN, S01_NET_LINK_DOWN, S00_ITEMS,
  WIN_REASON_TITLE, WIN_REASON_OK,
  SHUTDOWN_TITLE, SHUTDOWN_UNSAVED_LABEL, SHUTDOWN_NONE, SHUTDOWN_BUTTON, SHUTDOWN_HINT,
  SHUTDOWN_WIN_TITLE, SHUTDOWN_WIN_INTRO, SHUTDOWN_WIN_NONE, SHUTDOWN_SAVE,
  SHUTDOWN_DISCARD_IDLE, SHUTDOWN_DISCARD_ARMED, SHUTDOWN_CANCEL, SHUTDOWN_CONFIRM,
  SHUTDOWN_LOADING, SHUTDOWN_STOPPING, SHUTDOWN_BLOCKED_HINT,
  SHUTDOWN_DONE_TITLE, SHUTDOWN_DONE_BODY, SHUTDOWN_RESOLVED,
  SHUTDOWN_LOAD_ERROR, unsavedCountLabel, unsavedLabel,
} from '../i18n/screens.js'

const GROUP_TITLES = { move: GROUP_MOVE_TITLE, field: GROUP_FIELD_TITLE, maint: GROUP_MAINT_TITLE }

function parseUnsaved(message) {
  try {
    const list = JSON.parse(message || '[]')
    return Array.isArray(list) ? list : []
  } catch {
    return []
  }
}

export default function S01Main({ onEnter, onOpenSettings, onOpenConnect }) {
  const { state, stale, ros } = useSystemState()
  const { auto } = useAutoCheckStatus(ros)
  const { link } = useLinkStatus(ros)
  // S-00 の機器別の行と同じ値（Spec-webui.md §3.2「S-00 と同じ内容を要約表示」）。
  const linkDown = linkDownKeys(link)
  const sendTrigger = useTrigger()
  const shutdownPrepare = useStdTrigger(SERVICES.SHUTDOWN_PREPARE)
  const shutdownExecute = useStdTrigger(SERVICES.SHUTDOWN_EXECUTE)
  const confirmWindow = useConfirmWindow()

  // null | { kind: 'reason', reasonKey } | { kind: 'shutdown' }
  const [activeWindow, setActiveWindow] = useState(null)

  const [shutdownPhase, setShutdownPhase] = useState('idle') // idle|loading|ready|executing|stopping|done
  const [unsavedItems, setUnsavedItems] = useState([])
  const [resolvedItems, setResolvedItems] = useState(new Set())
  const [shutdownError, setShutdownError] = useState(null) // reject_reason_key, or a raw string for load failures

  // 1b-6 SG-B6: /shutdown/execute が通っただけでは完了にしない。実際に止まった
  // （接続が切れて stale になった）のを確かめてから「電源を切って構いません」を出す。
  useEffect(() => {
    if (shutdownPhase === 'stopping' && stale) setShutdownPhase('done')
  }, [shutdownPhase, stale])

  const mode = state?.mode ?? null
  // M-1 (nothing pressable while starting up) + the general fail-safe rule
  // that a stale link disables every operation (DetailedDesign-wp1.md
  // WP-UI-01 §6.2). menuItems() itself already returns all-disabled for
  // mode INIT/unknown; this additionally covers "we have a last-known mode
  // but the link just went stale".
  const disabledAll = stale || mode == null || mode === 'INIT'

  const items = menuItems(state, modeEntry, attributes)
  const liveUnsaved = state?.unsaved ?? []

  function closeWindow() {
    setActiveWindow(null)
    confirmWindow.close()
  }

  async function handleMenuClick(item) {
    if (disabledAll) return
    if (!item.enabled) {
      setActiveWindow({ kind: 'reason', reasonKey: item.reasonKey })
      confirmWindow.open()
      return
    }
    try {
      const res = await sendTrigger('ui.enter_mode', { mode: item.mode })
      if (!res?.accepted) {
        setActiveWindow({ kind: 'reason', reasonKey: res?.reject_reason_key ?? null })
        confirmWindow.open()
        return
      }
      // accepted: hand off to main.jsx, which owns the screen FSM — the
      // destination for this mode comes from the MODE_TO_SCREEN map there
      // (DetailedDesign-wp3.md WP-TRANSIT-01 §11 c1: screen transitions are
      // a map so later packets add one row per screen). If no screen is
      // mapped yet (e.g. a mode whose screen is a later packet), it's a
      // no-op here; /system/state will reflect the mode change anyway.
      if (onEnter) onEnter(item.mode)
    } catch {
      setActiveWindow({ kind: 'reason', reasonKey: null })
      confirmWindow.open()
    }
  }

  async function openShutdown() {
    if (disabledAll) return
    setActiveWindow({ kind: 'shutdown' })
    confirmWindow.open()
    setShutdownPhase('loading')
    setShutdownError(null)
    try {
      const res = await shutdownPrepare()
      setUnsavedItems(parseUnsaved(res?.message))
      setResolvedItems(new Set())
      setShutdownPhase('ready')
    } catch {
      setUnsavedItems([])
      setResolvedItems(new Set())
      setShutdownError(SHUTDOWN_LOAD_ERROR)
      setShutdownPhase('ready')
    }
  }

  function resolveItem(key) {
    setResolvedItems((prev) => new Set(prev).add(key))
  }

  const allResolved = unsavedItems.length === 0 || unsavedItems.every((k) => resolvedItems.has(k))

  async function confirmShutdown() {
    if (!allResolved || shutdownPhase !== 'ready') return
    setShutdownPhase('executing')
    setShutdownError(null)
    try {
      const res = await shutdownExecute()
      if (res?.success) {
        // 完了は stale（実際に止まったこと）を見てから。ここでは進捗表示にして
        // ウィンドウを閉じる（応答を待たずに完了を出さない。SG-B6）。
        setShutdownPhase('stopping')
        closeWindow()
      } else {
        setShutdownError(res?.message || 'unsaved_remains')
        setShutdownPhase('ready')
      }
    } catch {
      setShutdownError(SHUTDOWN_LOAD_ERROR)
      setShutdownPhase('ready')
    }
  }

  return (
    <div className="screen" id="s01">
      {/* ネットワーク接続確認の要約（Spec-webui.md §3.2・Spec-ops.md §2.6）。
          S-00 と同じ警告を出す。警告だけでボタン類の活性は変えない（C-r4）。
          タップで S-00 相当の詳細へ。 */}
      <div className="card" data-testid="s01-net-summary">
        <h3>{S01_NET_TITLE}</h3>
        {linkDown.length > 0 && (
          <div className="row mb">
            <span className="pill ng" data-testid="s01-net-link-down">
              {S01_NET_LINK_DOWN}
              {linkDown.map((k) => S00_ITEMS.find((it) => it.key === k)?.label ?? k).join('、')}
            </span>
          </div>
        )}
        {!auto && <div className="note">{S01_NET_CHECKING}</div>}
        {auto?.suppressed && <div className="note">{S01_NET_SUPPRESSED}</div>}
        {auto && !auto.suppressed && autoCheckWarns(auto) && (
          <div className="row mb">
            <span className="pill ng" data-testid="s01-net-warn">{S01_NET_WARN}</span>
          </div>
        )}
        {auto && !auto.suppressed && !autoCheckWarns(auto) && auto.overall === 'OK' && linkDown.length === 0 && (
          <div className="note">{S01_NET_OK}</div>
        )}
        {auto && !auto.suppressed && auto.overall !== 'OK' && !autoCheckWarns(auto) && (
          <div className="note">{S01_NET_CHECKING}</div>
        )}
        {onOpenConnect && (
          <button
            type="button"
            className="btn wide mt"
            onClick={onOpenConnect}
            data-testid="s01-net-open"
          >
            {S01_NET_OPEN}
          </button>
        )}
      </div>
      {MENU_GROUPS.map((group) => (
        <div className="card" key={group.key}>
          <h3>{GROUP_TITLES[group.key]}</h3>
          <div className="btnrow n2">
            {group.modes.map((m) => {
              const item = items.find((it) => it.mode === m)
              return (
                <button
                  key={m}
                  type="button"
                  className={`btn ${item.enabled ? '' : 'dis'}`}
                  disabled={disabledAll}
                  onClick={() => handleMenuClick(item)}
                  data-testid={`s01-mode-${m}`}
                >
                  {modeLabel(m)}
                </button>
              )
            })}
          </div>
          {/* WS-9X: 「設定」は FSM のモードではないので menuItems() も
              .btnrow（モード選択ボタンのグリッド）にも入れない。S-50 を開く
              だけ（onOpenSettings）。stale / INIT でも設定は読めるように
              disabledAll では切らない。 */}
          {group.key === 'maint' && onOpenSettings && (
            <button
              type="button"
              className="btn wide mt"
              onClick={onOpenSettings}
              data-testid="s01-open-settings"
            >
              {S01_SETTINGS}
            </button>
          )}
        </div>
      ))}

      {needsFinishEscape(mode) && (
        <div className="card">
          <button
            type="button"
            className="btn danger wide"
            disabled={disabledAll}
            onClick={async () => {
              try {
                const res = await sendTrigger('ui.finish')
                if (!res?.accepted) {
                  setActiveWindow({ kind: 'reason', reasonKey: res?.reject_reason_key ?? null })
                  confirmWindow.open()
                  return
                }
                if (onEnter) onEnter('IDLE')
              } catch {
                setActiveWindow({ kind: 'reason', reasonKey: null })
                confirmWindow.open()
              }
            }}
            data-testid="s01-finish-escape"
          >
            {S01_FINISH_ESCAPE}
          </button>
        </div>
      )}

      <div className="card">
        <h3>{SHUTDOWN_TITLE}</h3>
        {shutdownPhase === 'done' ? (
          <div className="note">
            <div className="b">{SHUTDOWN_DONE_TITLE}</div>
            <div>{SHUTDOWN_DONE_BODY}</div>
          </div>
        ) : shutdownPhase === 'stopping' ? (
          <div className="note" data-testid="shutdown-stopping">
            <div className="b">{SHUTDOWN_STOPPING}</div>
          </div>
        ) : (
          <>
            <div className="row mb">
              <span className="grow sm">{SHUTDOWN_UNSAVED_LABEL}</span>
              <span className={`pill ${liveUnsaved.length ? 'ng' : 'ok'}`}>
                {liveUnsaved.length ? unsavedCountLabel(liveUnsaved.length) : SHUTDOWN_NONE}
              </span>
            </div>
            <button type="button" className="btn danger wide" disabled={disabledAll} onClick={openShutdown}>
              {SHUTDOWN_BUTTON}
            </button>
            <div className="hint mt">{SHUTDOWN_HINT}</div>
          </>
        )}
      </div>

      {activeWindow?.kind === 'reason' && confirmWindow.isOpen && confirmWindow.mountNode && createPortal(
        <>
          <header>{WIN_REASON_TITLE}</header>
          <div className="bodyw">
            <p>{reasonLabel(activeWindow.reasonKey) ?? UNKNOWN_REASON_LABEL}</p>
          </div>
          <footer>
            <button type="button" className="btn primary" onClick={closeWindow}>{WIN_REASON_OK}</button>
          </footer>
        </>,
        confirmWindow.mountNode,
      )}

      {activeWindow?.kind === 'shutdown' && confirmWindow.isOpen && confirmWindow.mountNode && createPortal(
        <>
          <header>{SHUTDOWN_WIN_TITLE}</header>
          <div className="bodyw">
            {shutdownPhase === 'loading' && <p className="mut sm">{SHUTDOWN_LOADING}</p>}
            {shutdownPhase !== 'loading' && (
              <>
                <div className="sm mb">{SHUTDOWN_WIN_INTRO}</div>
                {unsavedItems.length === 0 ? (
                  <div className="note">{SHUTDOWN_WIN_NONE}</div>
                ) : (
                  <table className="lst">
                    <tbody>
                      {unsavedItems.map((key) => (
                        <tr key={key}>
                          <td>{unsavedLabel(key)}</td>
                          <td className="r">
                            {resolvedItems.has(key) ? (
                              <span className="pill ok">{SHUTDOWN_RESOLVED}</span>
                            ) : (
                              <>
                                <button type="button" className="btn sm" onClick={() => resolveItem(key)}>
                                  {SHUTDOWN_SAVE}
                                </button>
                                {' '}
                                <ArmedButton
                                  className="sm"
                                  idleLabel={SHUTDOWN_DISCARD_IDLE}
                                  armedLabel={SHUTDOWN_DISCARD_ARMED}
                                  onConfirm={() => resolveItem(key)}
                                />
                              </>
                            )}
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                )}
                {shutdownError && <div className="note mt">{reasonLabel(shutdownError) ?? shutdownError}</div>}
              </>
            )}
          </div>
          <footer>
            <button type="button" className="btn" onClick={closeWindow}>{SHUTDOWN_CANCEL}</button>
            {!allResolved && shutdownPhase === 'ready' && (
              <span className="hint" data-testid="shutdown-blocked-reason">
                {SHUTDOWN_BLOCKED_HINT(unsavedItems.length - resolvedItems.size)}
              </span>
            )}
            <button
              type="button"
              className="btn danger"
              disabled={shutdownPhase !== 'ready' || !allResolved}
              onClick={confirmShutdown}
            >
              {SHUTDOWN_CONFIRM}
            </button>
          </footer>
        </>,
        confirmWindow.mountNode,
      )}
    </div>
  )
}
