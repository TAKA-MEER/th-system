// shell/AppShell.jsx — the shell: fixed header, scrolling body, W-1..W-6,
// and the estop release bar (DetailedDesign-webui.md §1/§2).
// Screens (WP-UI-02+) are rendered as `children`; this packet builds no
// screen content (DetailedDesign-wp1.md WP-UI-01 §1).
import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { SystemStateProvider, useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useActiveScreenPublisher } from '../ros/useActiveScreenPublisher.js'
import { TOPICS, MSG_TYPES } from '../ros/topics.js'
import attributes from '../generated/attributes.json'
import Header from './Header.jsx'
import Windows from './Windows.jsx'
import { isW1Active } from './limits.js'
import { ConfirmWindowContext } from './confirmWindow.js'
import { ESTOP_RELEASE_NOTE, ESTOP_RELEASE_BUTTON } from '../i18n/states.js'
import './theme.css'

// DetailedDesign-wp1.md WP-UI-01 §3.1: /safety/estop_ui is republished at
// 2Hz while held, not sent once and latched client-side.
const ESTOP_UI_HZ = 2

function EstopReleaseBar({ show, onRelease }) {
  return (
    <div id="release" className={show ? 'show' : ''}>
      <div className="rt">{ESTOP_RELEASE_NOTE}</div>
      <button type="button" id="releaseBtn" onClick={onRelease}>{ESTOP_RELEASE_BUTTON}</button>
    </div>
  )
}

function AppShellInner({ screenName, screenId, children }) {
  const { ros, state, fault, stale } = useSystemState()
  const sendTrigger = useTrigger()
  // N-15: /ui/active_screen (DetailedDesign-wp1.md WP-UI-01 §3.1). Only
  // publishes once a screen has actually announced its screen_id -- see
  // main.jsx's Screens().
  useActiveScreenPublisher(ros, screenId)

  const [devMode] = useState(
    () => new URLSearchParams(window.location.search).get('dev') === '1')
  // This client's own intent to hold the UI estop latch up. Deliberately
  // local, not derived from state.estop_ui: a freshly loaded/reloaded page
  // must not start repeating "true" just because some other client is
  // holding it (DetailedDesign-safety.md §6.3 only specifies press/release
  // semantics for a single operator client).
  const [uiEngaged, setUiEngaged] = useState(false)
  const [estopDismissed, setEstopDismissed] = useState(false)

  // W-4 (confirm window) host state, exposed to screens via
  // shell/confirmWindow.js's context. See that file and shell/Windows.jsx
  // for why this is a portal-mount-node handshake rather than rendered
  // content passed down as a prop.
  const [confirmOpen, setConfirmOpen] = useState(false)
  const [confirmMount, setConfirmMount] = useState(null)

  const estopTopicRef = useRef(null)
  const publishTimerRef = useRef(null)

  const mode = state?.mode ?? null
  const stateName = state?.state ?? null
  const estopUi = !!state?.estop_ui
  const estopHw = !!state?.estop_hw
  const zone = state?.zone && state.zone !== 'NA' ? state.zone : null
  const w1Active = isW1Active(mode, stateName, !!fault?.active)
  // C-2 (WP-CARRY-01 §4/§7): server-side reject reason for a UI estop press
  // rejected during CARRY (C-06r), surfaced to W-2 -- see Windows.jsx.
  const lastRejectReason = state?.last_reject_reason ?? ''

  // A higher-priority window (W-1/W-2) always wins; a confirm window a
  // screen opened before the fault/estop landed must not linger underneath
  // it (DetailedDesign-webui.md §2.1 stacking order applies to W-4 too).
  useEffect(() => {
    if (w1Active || mode === 'CARRY') setConfirmOpen(false)
  }, [w1Active, mode])

  const confirmWindowApi = useMemo(() => ({
    isOpen: confirmOpen,
    mountNode: confirmMount,
    open: () => setConfirmOpen(true),
    close: () => setConfirmOpen(false),
  }), [confirmOpen, confirmMount])

  useEffect(() => {
    if (!ros) { estopTopicRef.current = null; return }
    const ROSLIB = window.ROSLIB
    if (!ROSLIB) return
    estopTopicRef.current = new ROSLIB.Topic({
      ros, name: TOPICS.ESTOP_UI, messageType: MSG_TYPES.BOOL,
    })
    return () => { estopTopicRef.current = null }
  }, [ros])

  const publishEstopUi = useCallback((value) => {
    estopTopicRef.current?.publish(new window.ROSLIB.Message({ data: value }))
  }, [])

  // DetailedDesign-safety.md §6.3: "press" must be re-affirmed continuously
  // (2Hz) so a dead UI doesn't leave a false impression of being held;
  // "release" is sent once, explicitly, and only on the release-bar press.
  useEffect(() => {
    if (!uiEngaged) return
    publishEstopUi(true)
    publishTimerRef.current = setInterval(() => publishEstopUi(true), 1000 / ESTOP_UI_HZ)
    return () => clearInterval(publishTimerRef.current)
  }, [uiEngaged, publishEstopUi])

  const handleEstopClick = useCallback(() => setUiEngaged(true), [])

  const handleRelease = useCallback(() => {
    setUiEngaged(false)
    publishEstopUi(false)
  }, [publishEstopUi])

  return (
    <div id="app">
      <Header
        screenName={screenName}
        mode={mode}
        stale={stale}
        zone={zone}
        fault={fault}
        devMode={devMode}
        estopEngaged={uiEngaged || mode === 'ESTOP'}
        onEstopClick={handleEstopClick}
        faultBadgeVisible={w1Active && estopDismissed}
        onFaultBadgeClick={() => setEstopDismissed(false)}
      />
      {/* Screens (WP-UI-02+) read mode/state/stale themselves via
          useSystemState() -- see ros/useSystemState.js -- rather than
          having them threaded through here as props. They reach W-4
          (the confirm window) via useConfirmWindow() -- see
          shell/confirmWindow.js. */}
      <ConfirmWindowContext.Provider value={confirmWindowApi}>
        <main id="body">
          {children}
        </main>
      </ConfirmWindowContext.Provider>
      <Windows
        mode={mode}
        stateName={stateName}
        estopUi={estopUi}
        estopHw={estopHw}
        fault={fault}
        attributes={attributes}
        onTrigger={sendTrigger}
        estopDismissed={estopDismissed}
        setEstopDismissed={setEstopDismissed}
        confirmOpen={confirmOpen}
        onConfirmMount={setConfirmMount}
        lastRejectReason={lastRejectReason}
      />
      <EstopReleaseBar show={uiEngaged || mode === 'ESTOP'} onRelease={handleRelease} />
    </div>
  )
}

export default function AppShell({ screenName, screenId, url, children }) {
  return (
    <SystemStateProvider url={url}>
      <AppShellInner screenName={screenName} screenId={screenId}>{children}</AppShellInner>
    </SystemStateProvider>
  )
}
