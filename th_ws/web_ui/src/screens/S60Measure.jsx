// screens/S60Measure.jsx — S-60, 自律測定 (AUTO_MEASURE の画面。Spec-webui.md §3.16)。
//
// 行き先選び・設定は置かない。置くのは「測定機能の画面へ」「自律測定を終了」「手動」
// と状態（WAIT=指示待ち／PAUSE=一時停止）の表示だけ（DetailedDesign-measure.md §6.2）。
// 「走行」は置かない（戻すのは測定機能の画面の meas.start）。
// 「自律測定を終了」は毎回 W-4「測定機構の腕が格納位置にあることを確かめましたか」
// を挟み、「メインメニューへ」のときだけ ui.finish を送る（SM-3.1.1-18）。
import { useEffect, useState } from 'react'
import { createPortal } from 'react-dom'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useParamsOverrides } from '../ros/useParamsOverrides.js'
import { useJogPanel } from '../shell/jogPanel.js'
import { useConfirmWindow } from '../shell/confirmWindow.js'
import { stateLabel } from '../i18n/states.js'
import {
  S60_OPEN_MEASURE, S60_FINISH, S60_MANUAL,
  S60_ARM_TOGGLE_TITLE, S60_ARM_QUESTION, S60_ARM_CANCEL, S60_ARM_OK,
} from '../i18n/screens.js'

export default function S60Measure() {
  const { state, stale, ros } = useSystemState()
  const sendTrigger = useTrigger()
  const jogPanel = useJogPanel()
  const confirmWindow = useConfirmWindow()

  const mode = state?.mode ?? null
  const stateName = state?.state ?? null
  const disabledAll = stale || mode == null

  const [confirmArm, setConfirmArm] = useState(false)

  // 「測定機能の画面へ」の移り先は registry の measure_ui_port（given 8091）で、
  // camera_ui_url と同じ経路（/params/get の effective）で読む（ビルド時生成物の
  // 値は読まない。S-50 では変えない）。読めないときは 8091。
  const { getParams } = useParamsOverrides(ros)
  const [measurePort, setMeasurePort] = useState(null)
  useEffect(() => {
    let alive = true
    getParams(['measure_ui_port'])
      .then((payload) => {
        if (alive) setMeasurePort(payload?.effective?.measure_ui_port ?? null)
      })
      .catch(() => { if (alive) setMeasurePort(null) })
    return () => { alive = false }
  }, [getParams])

  function openFkScreen() {
    if (disabledAll) return
    const port = (typeof measurePort === 'number' && Number.isFinite(measurePort))
      ? measurePort : 8091
    window.location.assign(`http://${window.location.hostname}:${port}/`)
  }

  // 自律測定を終了する前は必ず腕の格納の確認（W-4）を挟む。やめる は何も送らない。
  function confirmEnd() {
    setConfirmArm(false)
    confirmWindow.close()
    sendTrigger('ui.finish')
  }

  return (
    <div className="screen" id="s60">
      <div className="card">
        <h3 data-testid="s60-state">{stateLabel(stateName)}</h3>
        <div className="note">測定機能の指示で動いています。走行は測定機能の画面から行います。</div>
        <div className="btnrow n2">
          <button
            type="button"
            className="btn"
            data-testid="s60-open-measure"
            disabled={disabledAll}
            onClick={openFkScreen}
          >
            {S60_OPEN_MEASURE}
          </button>
          <button
            type="button"
            className="btn"
            data-testid="s60-manual"
            disabled={disabledAll}
            onClick={() => jogPanel.open()}
          >
            {S60_MANUAL}
          </button>
        </div>
        <button
          type="button"
          className="btn danger wide mt"
          data-testid="s60-finish"
          disabled={disabledAll}
          onClick={() => { setConfirmArm(true); confirmWindow.open() }}
        >
          {S60_FINISH}
        </button>
      </div>

      {confirmArm && confirmWindow.isOpen && confirmWindow.mountNode && createPortal(
        <>
          <header>{S60_ARM_TOGGLE_TITLE}</header>
          <div className="bodyw">
            <p data-testid="s60-arm-question">{S60_ARM_QUESTION}</p>
          </div>
          <footer>
            <button
              type="button"
              className="btn"
              data-testid="s60-arm-cancel"
              onClick={() => { setConfirmArm(false); confirmWindow.close() }}
            >
              {S60_ARM_CANCEL}
            </button>
            <button
              type="button"
              className="btn primary"
              data-testid="s60-arm-ok"
              onClick={confirmEnd}
            >
              {S60_ARM_OK}
            </button>
          </footer>
        </>,
        confirmWindow.mountNode,
      )}
    </div>
  )
}
