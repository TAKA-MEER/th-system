// screens/S01Main.jsx — S-01, the main menu screen (SCREEN_NAMES.S01 in
// i18n/screens.js; DetailedDesign-wp1.md WP-UI-02).
//
// Spec-webui §3.2: 一番上は 5 タイル（試験場 ▸・走行 ▸・点検と校正 ▸・自律測定・
// 設定）と下の段の「運用の終了」「接続確認」。下の階層は「戻る」で上へ。
// タイルの大きさの順位は mainMenuItems.js の TILE_SIZES の 1 か所だけが持つ。
// ライン誘導・電子リードの入口は消した（2026-11-27 D7）。
//
// Two independent things live here: mode selection (menuItems() drives which
// of the 9 buttons are enabled -- screens/mainMenuItems.js) and the
// shutdown flow (SHUTDOWN_TITLE in i18n/screens.js; DetailedDesign-webui.md
// §8.3 / DetailedDesign-state.md §12.5's 5 steps). Both use the shared
// WindowShell (shell/WindowShell.jsx) via the W-4 confirm window portal.
//
// 1b-6 (SG-B6): W-4 の「保存」「破棄」は項目ごとの本物の操作を呼ぶ。
//   venue_map … 保存: /map_session/open (slot VENUE, mode save)
//               破棄: /slam_control/discard_map
//   route     … 保存: ui.save ／ 破棄: ui.discard（T-IDLE-01/02）
//   calib・map_patch … 機体側に画面から呼べる操作が無い（報告参照）。
//               ボタンを出さず理由を出し、「停止する」は押せないままにする。
// 「処理済み」は操作の応答ではなく、/shutdown/prepare を呼び直して一覧から
// 消えたことで決める（機体側が正本。応答だけで済んだことにしない）。
import { useState, useEffect } from 'react'
import { createPortal } from 'react-dom'
import { useSystemState } from '../ros/useSystemState.js'
import { useRouteCatalog } from '../ros/useRouteCatalog.js'
import { useAutoCheckStatus } from '../ros/useAutoCheckStatus.js'
import { useLinkStatus } from '../ros/useLinkStatus.js'
import { linkDownKeys } from '../ros/linkStatusState.js'
import { autoCheckWarns } from '../ros/autoCheckState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useStdTrigger } from '../ros/useStdTrigger.js'
import { useOnsiteService } from '../ros/useOnsiteService.js'
import { SERVICES } from '../ros/topics.js'
import { useConfirmWindow } from '../shell/confirmWindow.js'
import WindowShell from '../shell/WindowShell.jsx'
import ArmedButton from '../parts/ArmedButton.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import { menuItems, TOP_TILES, GROUP_TILES, needsFinishEscape } from './mainMenuItems.js'
import attributes from '../generated/attributes.json'
import modeEntry from '../generated/mode_entry.json'
import { modeLabel } from '../i18n/modes.js'
import { reasonLabel, UNKNOWN_REASON_LABEL } from '../i18n/reasons.js'
import {
  IconField, IconDrive, IconMaint, IconMeasure, IconGear, IconDay, IconPrep,
  IconFollow, IconManual, IconReplay, IconTeach, IconCheck, IconCalib, IconBack,
} from '../parts/icons.jsx'
import {
  S01_TILE_FIELD, S01_TILE_DRIVE, S01_TILE_MAINT, S01_TILE_AUTO, S01_TILE_SETTINGS,
  S01_BACK, S01_CONNECT, S01_AUTO_TITLE, S01_AUTO_NO, S01_AUTO_YES,
  S01_FINISH_ESCAPE,
  S01_NET_TITLE, S01_NET_OK, S01_NET_CHECKING, S01_NET_WARN,
  S01_NET_SUPPRESSED, S01_NET_OPEN, S01_NET_LINK_DOWN, S00_ITEMS,
  WIN_REASON_TITLE, WIN_REASON_OK,
  SHUTDOWN_TITLE, SHUTDOWN_UNSAVED_LABEL, SHUTDOWN_NONE, SHUTDOWN_BUTTON, SHUTDOWN_HINT,
  SHUTDOWN_WIN_TITLE, SHUTDOWN_WIN_INTRO, SHUTDOWN_WIN_NONE, SHUTDOWN_SAVE,
  SHUTDOWN_DISCARD_IDLE, SHUTDOWN_DISCARD_ARMED, SHUTDOWN_CANCEL, SHUTDOWN_CONFIRM,
  SHUTDOWN_LOADING, SHUTDOWN_STOPPING, SHUTDOWN_BLOCKED_HINT,
  SHUTDOWN_DONE_TITLE, SHUTDOWN_DONE_BODY, SHUTDOWN_RESOLVED,
  SHUTDOWN_LOAD_ERROR, SHUTDOWN_ITEM_FAILED, SHUTDOWN_NO_ACTION, unsavedCountLabel, unsavedLabel,
} from '../i18n/screens.js'

const TILE_ICONS = {
  field: <IconField />, drive: <IconDrive />, maint: <IconMaint />,
  measure: <IconMeasure />, gear: <IconGear />, day: <IconDay />, prep: <IconPrep />,
  follow: <IconFollow />, manual: <IconManual />, replay: <IconReplay />,
  teach: <IconTeach />, check: <IconCheck />, calib: <IconCalib />,
}

const GROUP_NAMES = { field: S01_TILE_FIELD, drive: S01_TILE_DRIVE, maint: S01_TILE_MAINT }

const TILE_HELP = {
  field: 's01-field', drive: 's01-drive', maint: 's01-maint',
  auto: 's01-auto', settings: 's01-settings',
}

const MODE_HELP = {
  FOLLOW: 's01-mode-follow', MANUAL: 's01-mode-manual',
  TEACH_FOLLOW: 's01-mode-teach-follow', TEACH_MANUAL: 's01-mode-teach-manual',
  REPLAY: 's01-mode-replay', PREP: 's01-mode-prep', AT_HOME: 's01-mode-at-home',
  OPCHECK: 's01-mode-opcheck', CALIB: 's01-mode-calib',
}

// グリッドの置き場所（モックアップ LAYOUT の既定）。
const TOP_AREA = {
  landscape: {
    cols: '1.35fr 1fr 1fr', rows: '1fr 1fr .42fr',
    areas: { field: '1/1/4/2', drive: '1/2/3/3', maint: '1/3/3/4', auto: '3/2/4/3', settings: '3/3/4/4' },
  },
  portrait: {
    cols: '1fr 1fr', rows: '1.25fr 1fr .42fr',
    areas: { field: '1/1/2/3', drive: '2/1/3/2', maint: '2/2/3/3', auto: '3/1/4/2', settings: '3/2/4/3' },
  },
}

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
  const routes = useRouteCatalog(ros)
  const { auto } = useAutoCheckStatus(ros)
  const { link } = useLinkStatus(ros)
  // S-00 の機器別の行と同じ値（Spec-webui.md §3.2「S-00 と同じ内容を要約表示」）。
  const linkDown = linkDownKeys(link)
  const sendTrigger = useTrigger()
  const shutdownPrepare = useStdTrigger(SERVICES.SHUTDOWN_PREPARE)
  const shutdownExecute = useStdTrigger(SERVICES.SHUTDOWN_EXECUTE)
  const discardMap = useStdTrigger('/slam_control/discard_map')
  const onsite = useOnsiteService()
  const confirmWindow = useConfirmWindow()

  const [lvl, setLvl] = useState('top')
  // null | { kind: 'reason', reasonKey } | { kind: 'shutdown' } | { kind: 'auto' }
  const [activeWindow, setActiveWindow] = useState(null)

  const [shutdownPhase, setShutdownPhase] = useState('idle') // idle|loading|ready|executing|stopping|done
  const [unsavedItems, setUnsavedItems] = useState([])
  const [resolvedItems, setResolvedItems] = useState(new Set())
  const [shutdownError, setShutdownError] = useState(null) // reject_reason_key, or a raw string for load failures
  const [busyKey, setBusyKey] = useState(null)       // 保存／破棄の応答待ちの項目
  const [itemErrors, setItemErrors] = useState({})   // 項目 → 失敗の理由（文字列）

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

  const items = menuItems(state, modeEntry, attributes, {
    // 1b-11 SG-B13: 経路の本数は /route/catalog。
    // TEST_MODE の未 seed（useRouteCatalog 初期値 []）は「0 本」として扱い、
    // 教示再生を止める（空の一覧で押せる方が誤り。サーバ側も同じく拒否する）。
    routeCount: routes.length,
  })
  const liveUnsaved = state?.unsaved ?? []

  function closeWindow() {
    setActiveWindow(null)
    confirmWindow.close()
  }

  function openReason(reasonKey) {
    setActiveWindow({ kind: 'reason', reasonKey })
    confirmWindow.open()
  }

  async function enterMode(targetMode) {
    try {
      const res = await sendTrigger('ui.enter_mode', { mode: targetMode })
      if (!res?.accepted) {
        openReason(res?.reject_reason_key ?? null)
        return
      }
      // accepted: hand off to main.jsx, which owns the screen FSM — the
      // destination for this mode comes from the MODE_TO_SCREEN map there.
      // If no screen is mapped yet (e.g. a mode whose screen is a later
      // packet), it's a no-op here; /system/state will reflect the mode
      // change anyway.
      if (onEnter) onEnter(targetMode)
    } catch {
      openReason(null)
    }
  }

  async function handleMenuClick(item) {
    if (disabledAll) return
    if (!item.enabled) {
      openReason(item.reasonKey)
      return
    }
    await enterMode(item.mode)
  }

  function handleTile(tile) {
    if (disabledAll) return
    if (tile.kind === 'group') {
      setLvl(tile.key)
    } else if (tile.kind === 'settings') {
      if (onOpenSettings) onOpenSettings()
    } else if (tile.kind === 'auto') {
      setActiveWindow({ kind: 'auto' })
      confirmWindow.open()
    } else if (tile.kind === 'mode') {
      const item = items.find((it) => it.mode === tile.mode)
      if (item) handleMenuClick(item)
    }
  }

  async function handleFinishEscape() {
    if (disabledAll) return
    try {
      const res = await sendTrigger('ui.finish')
      if (!res?.accepted) {
        openReason(res?.reject_reason_key ?? null)
        return
      }
      if (onEnter) onEnter('IDLE')
    } catch {
      openReason(null)
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
      setItemErrors({})
      setShutdownPhase('ready')
    } catch {
      setUnsavedItems([])
      setResolvedItems(new Set())
      setShutdownError(SHUTDOWN_LOAD_ERROR)
      setShutdownPhase('ready')
    }
  }

  // 項目ごとの本物の操作（無いものは null）。上のファイル冒頭コメント参照。
  const ITEM_ACTIONS = {
    venue_map: {
      save: () => onsite.saveVenueMap(),
      discard: () => discardMap(),
    },
    route: {
      save: () => sendTrigger('ui.save'),
      discard: () => sendTrigger('ui.discard'),
    },
  }

  const sleep = (ms) => new Promise((r) => setTimeout(r, ms))

  // 操作を呼び、/shutdown/prepare を呼び直して一覧から消えたら処理済みにする。
  async function runItem(key, kind) {
    const action = ITEM_ACTIONS[key]?.[kind]
    if (!action || busyKey) return
    setBusyKey(key)
    setItemErrors((prev) => ({ ...prev, [key]: null }))
    try {
      const res = await action()
      if (res && (res.success === false || res.accepted === false)) {
        setItemErrors((prev) => ({ ...prev, [key]: res.message || reasonLabel(res.reject_reason_key) || SHUTDOWN_ITEM_FAILED }))
        return
      }
      // 機体側が反映するまで少し待ちながら確かめる（教示経路の確定は非同期）。
      for (let i = 0; i < 8; i += 1) {
        const p = await shutdownPrepare()
        const remaining = parseUnsaved(p?.message)
        if (!remaining.includes(key)) {
          setResolvedItems((prev) => new Set(prev).add(key))
          return
        }
        await sleep(400)
      }
      setItemErrors((prev) => ({ ...prev, [key]: SHUTDOWN_ITEM_FAILED }))
    } catch {
      setItemErrors((prev) => ({ ...prev, [key]: SHUTDOWN_ITEM_FAILED }))
    } finally {
      setBusyKey(null)
    }
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

  function renderTile(tile, area, orientation) {
    const isGroup = tile.kind === 'group'
    const isMode = tile.kind === 'mode'
    let name = ''
    let item = null
    if (isGroup) name = GROUP_NAMES[tile.key]
    else if (tile.kind === 'auto') name = S01_TILE_AUTO
    else if (tile.kind === 'settings') name = S01_TILE_SETTINGS
    else if (isMode) {
      item = items.find((it) => it.mode === tile.mode)
      name = modeLabel(tile.mode)
    }
    // 「設定」は FSM を動かさないので disabledAll に縛らない（昔から）。
    const disabled = tile.kind === 'settings' ? false : (disabledAll || (item ? !item.enabled : false))
    const testId = isMode ? `s01-mode-${tile.mode}`
      : tile.kind === 'auto' ? 's01-open-auto'
      : tile.kind === 'settings' ? 's01-open-settings'
      : `s01-group-${tile.key}`
    const helpKey = isMode ? MODE_HELP[tile.mode] : TILE_HELP[tile.key]
    const reason = disabled && item && !disabledAll ? (reasonLabel(item.reasonKey) ?? null) : null
    return (
      <HelpTip key={tile.key} helpKey={helpKey}>
        <button
          type="button"
          className={`tile ${tile.size} ${disabled ? 'dis' : ''}`}
          style={area ? { gridArea: area } : undefined}
          disabled={tile.kind === 'settings' ? undefined : (disabledAll ? disabledAll : undefined)}
          onClick={() => handleTile(tile)}
          data-testid={testId}
          data-tile-size={tile.size}
          data-reason={reason ?? ''}
        >
          <span className="tile-ic" aria-hidden="true">{TILE_ICONS[tile.icon]}</span>
          <span className="nm">{name}{(isGroup ? ' ▸' : '')}</span>
        </button>
      </HelpTip>
    )
  }

  const orientation = typeof window !== 'undefined' && window.innerWidth < window.innerHeight
    ? 'portrait' : 'landscape'
  const topLayout = TOP_AREA[orientation]
  const topTiles = lvl === 'top'
    ? TOP_TILES.map((t) => renderTile(t, topLayout.areas[t.key], orientation))
    : GROUP_TILES[lvl].map((t) => renderTile(t, null, orientation))

  const groupName = GROUP_NAMES[lvl]

  return (
    <div className="screen" id="s01">
      <div className="opbar" data-pos="opbar">
          {lvl !== 'top' && (
            <HelpTip helpKey="s01-back">
              <button
                type="button"
                className="btn exit"
                onClick={() => setLvl('top')}
                data-testid="s01-back"
              >
                <IconBack />
                <span>{S01_BACK}</span>
              </button>
            </HelpTip>
          )}
          <span className="grow" />
          {onOpenConnect && (
            <HelpTip helpKey="s01-connect">
              <button
                type="button"
                className="btn exit"
                onClick={onOpenConnect}
                data-testid="s01-net-open"
              >
                <span>{S01_CONNECT}</span>
              </button>
            </HelpTip>
          )}
        </div>
      <div className="s01-main">
        <div className="mcol">
          {lvl !== 'top' && <div className="s01-banner">{groupName}</div>}
          <div
            className="tiles"
            data-testid="s01-tiles"
            data-lvl={lvl}
            style={lvl === 'top' ? {
              gridTemplateColumns: topLayout.cols,
              gridTemplateRows: topLayout.rows,
            } : undefined}
          >
            {topTiles}
          </div>
        </div>
        <div className="side">
        {/* ネットワーク接続確認の要約（Spec-webui.md §3.2・Spec-ops.md §2.6）。
            S-00 と同じ警告を出す。警告だけでボタン類の活性は変えない（C-r4）。 */}
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
        </div>

        {needsFinishEscape(mode) && (
          <div className="card">
            <button
              type="button"
              className="btn danger wide"
              disabled={disabledAll}
              onClick={handleFinishEscape}
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
              <HelpTip helpKey="s01-shutdown">
              <button type="button" className="btn danger wide" disabled={disabledAll} onClick={openShutdown} data-testid="s01-shutdown">
                {SHUTDOWN_BUTTON}
              </button>
              </HelpTip>
              <div className="hint mt">{SHUTDOWN_HINT}</div>
            </>
          )}
        </div>
        </div>
      </div>

      {activeWindow?.kind === 'reason' && confirmWindow.isOpen && confirmWindow.mountNode && createPortal(
        <WindowShell className="confirm" testId="s01-reason-window" title={WIN_REASON_TITLE}>
          <p>{reasonLabel(activeWindow.reasonKey) ?? UNKNOWN_REASON_LABEL}</p>
          <div className="mt">
            <button type="button" className="btn primary" onClick={closeWindow}>{WIN_REASON_OK}</button>
          </div>
        </WindowShell>,
        confirmWindow.mountNode,
      )}

      {activeWindow?.kind === 'auto' && confirmWindow.isOpen && confirmWindow.mountNode && createPortal(
        <WindowShell className="confirm" testId="s01-auto-window" title={S01_AUTO_TITLE}
          footer={<>
            <button type="button" className="btn" onClick={closeWindow} data-testid="s01-auto-no">{S01_AUTO_NO}</button>
            <button type="button" className="btn primary" onClick={() => { closeWindow(); enterMode('AUTO_MEASURE') }} data-testid="s01-auto-yes">{S01_AUTO_YES}</button>
          </>}>
          <p data-testid="s01-auto-body">{modeLabel('AUTO_MEASURE')}</p>
        </WindowShell>,
        confirmWindow.mountNode,
      )}

      {activeWindow?.kind === 'shutdown' && confirmWindow.isOpen && confirmWindow.mountNode && createPortal(
        <WindowShell className="confirm" testId="s01-shutdown-window" title={SHUTDOWN_WIN_TITLE}
          footer={<>
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
          </>}>
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
                          ) : !ITEM_ACTIONS[key] ? (
                            <span className="xs mut" data-testid={`shutdown-noaction-${key}`}>
                              {SHUTDOWN_NO_ACTION}
                            </span>
                          ) : (
                            <>
                              <button
                                type="button"
                                className="btn sm"
                                disabled={busyKey != null}
                                onClick={() => runItem(key, 'save')}
                                data-testid={`shutdown-save-${key}`}
                              >
                                {SHUTDOWN_SAVE}
                              </button>
                              {' '}
                              <ArmedButton
                                className="sm"
                                idleLabel={SHUTDOWN_DISCARD_IDLE}
                                armedLabel={SHUTDOWN_DISCARD_ARMED}
                                onConfirm={() => runItem(key, 'discard')}
                              />
                              {itemErrors[key] && (
                                <div className="xs tone-ng" data-testid={`shutdown-error-${key}`}>
                                  {itemErrors[key]}
                                </div>
                              )}
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
        </WindowShell>,
        confirmWindow.mountNode,
      )}
    </div>
  )
}
