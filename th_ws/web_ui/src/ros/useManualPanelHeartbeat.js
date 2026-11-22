// ros/useManualPanelHeartbeat.js — W-6（手動操作パネル）を開いている間の心拍（A-15）。
//
// 開いている間 5 Hz で /ui/manual_panel に client_id を送り、閉じたら空文字を 1 回送って
// 止める。機体（state_manager）はこの知らせが新しい間だけ、スティックに触れていなくても
// 自動ブレーキの切替（場内の OFF）を受け付け、途絶・閉じる・状態の変化で ON に戻す
// （Spec-webui §3.5）。**ジョグのリース（/ui/jog_lease。useJogLease.js）とは別**:
// リースは機体で ui.jog.hold になり自律の一時停止を起こすので、開いただけでは送らない。
//
// TEST_MODE（Playwright）は useJogLease.js と同じ流儀で、送信を window.__thManualPanelPublishes
// に {data} で記録する（実際には送らない）。
import { useEffect } from 'react'
import { TOPICS, MSG_TYPES } from './topics'
import { getClientId } from './clientId'

const TEST_MODE = typeof window !== 'undefined' && window.__thTestState !== undefined
export const MANUAL_PANEL_HEARTBEAT_MS = 200

export function useManualPanelHeartbeat(ros, open) {
  useEffect(() => {
    if (!open) return undefined
    const ROSLIB = typeof window !== 'undefined' ? window.ROSLIB : undefined
    const topic = (!TEST_MODE && ros && ROSLIB)
      ? new ROSLIB.Topic({ ros, name: TOPICS.MANUAL_PANEL, messageType: MSG_TYPES.STRING })
      : null
    const send = (data) => {
      if (TEST_MODE) {
        window.__thManualPanelPublishes = window.__thManualPanelPublishes ?? []
        window.__thManualPanelPublishes.push({ data })
      }
      topic?.publish(new ROSLIB.Message({ data }))
    }
    const clientId = getClientId()
    send(clientId)
    const timer = setInterval(() => send(clientId), MANUAL_PANEL_HEARTBEAT_MS)
    return () => {
      clearInterval(timer)
      send('')   // 閉じた（機体はすぐ ON に戻す。取りこぼしても心拍の途絶で戻る）
    }
  }, [ros, open])
}
