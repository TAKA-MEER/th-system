// shell/useSaveConfirm.js — W-4 保存確認の寿命を持つ hook (1b-7, SG-B3 / A-16).
//
// /system/effect の購読は ros/useSystemEffect.js、振り分けと開く条件は
// shell/effectDispatch.js（dispatchEffect／saveAskItems）。ここは
// 「ask が来て未保存（教示の記録・試験場内地図）があれば W-4 を立て、答えたら降ろす」だけを
// する。表示は shell/Windows.jsx（シェル所有の W-4）。
//
// W-3（useGuideBanner.js）と違い、モード変化での自動クローズはしない。
// ask が届いた時点で FSM は既に IDLE で、W-4 への答え（保存／破棄）が次の操作だからである。
// ESTOP／CARRY・IDLE 離脱でのクローズは AppShell 側。
//
// A-16: 試験場内地図が未保存のとき、「保存」は /map_session/open（slot VENUE・save。
// 運用の終了の一覧と同じ経路。IDLE から通る）、「破棄」は /slam_control/discard_map。
// 失敗したら W-4 を閉じずに理由を出す（保存できていないのに閉じて安心させない）。
// 教示の記録は従来どおり ui.save／ui.discard（T-IDLE-01/02）。
import { useCallback, useEffect, useRef, useState } from 'react'
import { useSystemEffect } from '../ros/useSystemEffect.js'
import { useRouteStatus } from '../ros/useRouteStatus.js'
import { useOnsiteService } from '../ros/useOnsiteService.js'
import { useStdTrigger } from '../ros/useStdTrigger.js'
import { dispatchEffect, saveAskItems } from './effectDispatch.js'
import { SAVE_ASK_FAILED } from '../i18n/states.js'

export function useSaveConfirm(ros, unsaved, sendTrigger) {
  const effect = useSystemEffect(ros)
  const routeStatus = useRouteStatus(ros)
  const onsite = useOnsiteService()
  const discardMap = useStdTrigger('/slam_control/discard_map')
  const [ask, setAsk] = useState(null)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState(null)

  // effect 到着時の判定に使う最新値。state 変数だと effect 到着 effect（deps [effect]）の
  // クロージャが古くなるため ref で持つ。
  const statusRef = useRef(null)
  statusRef.current = routeStatus
  const unsavedRef = useRef([])
  unsavedRef.current = unsaved ?? []

  useEffect(() => {
    if (!effect) return
    const decided = dispatchEffect(effect)
    if (decided.kind !== 'ask_save' && decided.kind !== 'ask_save_if_unsaved') return
    const items = saveAskItems(decided, statusRef.current, unsavedRef.current)
    if (items.route || items.venueMap) {
      setError(null)
      setAsk({ kind: decided.kind, routeId: decided.routeId, ...items })
    }
  }, [effect])

  const closeSaveAsk = useCallback(() => { setAsk(null); setError(null) }, [])

  const run = useCallback(async (kind) => {
    if (!ask || busy) return
    setBusy(true)
    setError(null)
    try {
      if (ask.venueMap) {
        const res = await (kind === 'save' ? onsite.saveVenueMap() : discardMap())
        if (!res || res.success === false) {
          setError(res?.message || SAVE_ASK_FAILED)
          return
        }
      }
      if (ask.route) {
        sendTrigger(kind === 'save' ? 'ui.save' : 'ui.discard').catch(() => {})
      }
      setAsk(null)
    } catch {
      setError(SAVE_ASK_FAILED)
    } finally {
      setBusy(false)
    }
  }, [ask, busy, onsite, discardMap, sendTrigger])

  return {
    saveAsk: ask,
    saveBusy: busy,
    saveError: error,
    closeSaveAsk,
    saveYes: useCallback(() => run('save'), [run]),
    saveNo: useCallback(() => run('discard'), [run]),
  }
}
