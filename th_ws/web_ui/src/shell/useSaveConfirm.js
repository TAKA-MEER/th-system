// shell/useSaveConfirm.js — W-4 保存確認の寿命を持つ hook (1b-7, SG-B3).
//
// /system/effect の購読は ros/useSystemEffect.js、振り分けと開く条件は
// shell/effectDispatch.js（dispatchEffect／shouldOpenSaveAsk）。ここは
// 「ask が来て未保存の教示記録があれば W-4 を立て、答えたら降ろす」だけを
// する。表示は shell/Windows.jsx（シェル所有の W-4）。
//
// W-3（useGuideBanner.js）と違い、モード変化での自動クローズはしない。
// ask が届いた時点で FSM は既に IDLE で、W-4 への答え（ui.save／ui.discard）
// が次の遷移だからである。ESTOP／CARRY・IDLE 離脱でのクローズは AppShell 側。
import { useEffect, useRef, useState } from 'react'
import { useSystemEffect } from '../ros/useSystemEffect.js'
import { useRouteStatus } from '../ros/useRouteStatus.js'
import { dispatchEffect, shouldOpenSaveAsk } from './effectDispatch.js'

export function useSaveConfirm(ros) {
  const effect = useSystemEffect(ros)
  const routeStatus = useRouteStatus(ros)
  const [ask, setAsk] = useState(null)

  // effect 到着時の判定に使う最新の /route/status。state 変数だと
  // effect 到着 effect（deps [effect]）のクロージャが古くなるため ref で持つ。
  const statusRef = useRef(null)
  statusRef.current = routeStatus

  useEffect(() => {
    if (!effect) return
    const decided = dispatchEffect(effect)
    if (decided.kind !== 'ask_save' && decided.kind !== 'ask_save_if_unsaved') return
    if (shouldOpenSaveAsk(decided, statusRef.current)) {
      setAsk({ kind: decided.kind, routeId: decided.routeId })
    }
  }, [effect])

  return { saveAsk: ask, closeSaveAsk: () => setAsk(null) }
}
