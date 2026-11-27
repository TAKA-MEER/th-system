// screens/S11Manual.jsx — S-11, 手動走行 (Spec-webui.md §3.4・§3.5)。
//
// 型「走らせる」（RunLayout）。主表示＝左に計器、右に機体のまわりの図。
// 左上＝終了＋自動ブレーキの切り替え（ロック付き）。右列 ① は空けて位置を保ち、
// ②「停止する」だけ（幅いっぱい。走行の契機はスティックだけ）、④ 速度のスライダ＋
// スティック＋速度の段。タブは使わない。古い操作カードは使わない。
import { useState } from 'react'
import { useSystemState } from '../ros/useSystemState.js'
import { useTrigger } from '../ros/useTrigger.js'
import { useLimiterStatus } from '../ros/useLimiterStatus.js'
import { useScan } from '../ros/useScan.js'
import { useWheelSpeeds } from '../ros/useWheelSpeeds.js'
import { useOdomTwist } from '../ros/useOdomTwist.js'
import { useTunableParams } from '../ros/useTunableParams.js'
import { useEffect } from 'react'
import { canToggleAutoBrake, approachDistance, approachIsAround, showApproachWarning } from '../shell/autoBrake.js'
import { RunLayout } from '../shell/Layouts.jsx'
import AutoBrakeLockToggle from '../parts/AutoBrakeLockToggle.jsx'
import StopRunToggle from '../parts/StopRunToggle.jsx'
import HelpTip from '../parts/HelpTip.jsx'
import SpeedGauge from '../parts/SpeedGauge.jsx'
import SurroundView from '../parts/SurroundView.jsx'
import JogConsole from '../parts/JogConsole.jsx'
import { guideFor, brakeCutOf } from '../parts/surroundGeom.js'
import { rangesFromFlat } from './calibCore.js'
import attributes from '../generated/attributes.json'
import { S11_APPROACH_WARN, S11_APPROACH_WARN_AROUND } from '../i18n/screens.js'
import { OP_LABELS } from '../i18n/states.js'
import './manualRun.css'

// 自動ブレーキの表示（T1-5）。正本は /system/state.auto_brake（機体側）。
function useAutoBrake(state) {
  const autoBrakeDefault = attributes?.MANUAL?.auto_brake_default === 'on'
  const [local, setLocal] = useState(null)
  if (state?.auto_brake != null) return [!!state.auto_brake, setLocal]
  const value = local ?? autoBrakeDefault
  return [value, setLocal]
}

// 死角（lidar_filter の blind_angle_ranges。画面に値を持たない）。
function useBlindRanges(ros) {
  const { getTunableParams } = useTunableParams(ros)
  const [ranges, setRanges] = useState([])
  useEffect(() => {
    let alive = true
    getTunableParams('lidar_filter', ['blind_angle_ranges'])
      .then((v) => { if (alive) setRanges(rangesFromFlat(v.blind_angle_ranges ?? [])) })
      .catch(() => { if (alive) setRanges([]) })
    return () => { alive = false }
  }, [getTunableParams])
  return ranges
}

export default function S11Manual({ onFinish }) {
  const { ros, state, stale } = useSystemState()
  const sendTrigger = useTrigger()
  const limiter = useLimiterStatus(ros)
  const scan = useScan(ros)
  const wheels = useWheelSpeeds(ros)
  const twist = useOdomTwist(ros)
  const blindRanges = useBlindRanges(ros)
  const disabledAll = stale || state?.mode == null

  const [brake] = useAutoBrake(state)
  const stateName = state?.state ?? null
  const canToggle = !disabledAll && canToggleAutoBrake(state, attributes, stale)

  const measured = wheels?.measured ?? []
  const command = wheels?.command ?? []
  const latestM = measured[measured.length - 1] ?? null
  const v = latestM ? (latestM.left + latestM.right) / 2 : null
  const guide = guideFor(v, twist?.wz)
  const brakeActive = brakeCutOf(limiter) > 0

  // 近い点の距離（機体の外形から。LimiterStatus の approach_clearance_m など）。
  const nearDist = approachDistance(limiter)
  const nearText = (showApproachWarning(limiter, brake) || brakeCutOf(limiter) > 0) && nearDist != null && Number.isFinite(nearDist)
    ? (approachIsAround(limiter) ? S11_APPROACH_WARN_AROUND(nearDist) : S11_APPROACH_WARN(nearDist))
    : null

  async function handleFinish() {
    try {
      const res = await sendTrigger('ui.finish')
      if (res?.accepted && onFinish) onFinish()
    } catch {
      // rosbridge の一時的な失敗。留まる（安全側）
    }
  }

  return (
    <div id="s11" className="screen">
      <RunLayout
        opbar={(
          <>
            <HelpTip helpKey="finish">
              <button
                type="button"
                className="btn exit"
                data-testid="s11-finish"
                disabled={disabledAll}
                onClick={handleFinish}
              >
                <span>{OP_LABELS.finish}</span>
              </button>
            </HelpTip>
            <HelpTip helpKey="brake">
              <AutoBrakeLockToggle brake={brake} canToggle={canToggle} prefix="s11" mode={state?.mode} stateName={stateName} />
            </HelpTip>
          </>
        )}
        main={(
          <div className="s11-main">
            <HelpTip helpKey="s11-gauge"><SpeedGauge measured={measured} command={command} twist={twist} limiter={limiter} /></HelpTip>
            <div className="surround-wrap">
              <HelpTip helpKey="s11-surround">
              <SurroundView
                scan={scan}
                blindRanges={blindRanges}
                guide={guide}
                brakeActive={brakeActive}
                nearText={nearText}
              />
              </HelpTip>
            </div>
          </div>
        )}
        where={<div aria-hidden="true" />}
        toggle={(
          <StopRunToggle
            testId="s11-stop-run"
            runLabel={null}
            stopEnabled={!disabledAll}
            onStop={() => sendTrigger('ui.stop')}
          />
        )}
        ops={<div aria-hidden="true" />}
        jog={(
          <JogConsole ros={ros} disabled={stale} keyboard />
        )}
      />
    </div>
  )
}
