// shell/useGuideBanner.js — W-3 案内の帯の寿命を持つ hook (1b-6, SG-B2).
//
// /system/effect の購読は ros/useSystemEffect.js (1 箇所)、振り分けは
// shell/effectDispatch.js。ここは「guide が来たら帯を立て、閉じる条件で
// 降ろす」だけをする。表示は shell/GuideBanner.jsx。
import { useEffect, useState } from 'react'
import { useSystemEffect } from '../ros/useSystemEffect.js'
import { dispatchEffect, shouldGuideClose } from './effectDispatch.js'

export function useGuideBanner(ros, mode, estopHw) {
  const effect = useSystemEffect(ros)
  const [guide, setGuide] = useState(null)

  // effect が届くたびに振り分け、guide だけ帯を立てる。mode は「届いたときの
  // モード」として記録する (deps に mode を入れると、モード変化で再実行され
  // modeAtOpen が付け替わって自動クローズが効かなくなるため入れない)。
  useEffect(() => {
    if (!effect) return
    const decided = dispatchEffect(effect)
    if (decided.kind === 'guide') setGuide({ key: decided.key, modeAtOpen: mode })
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [effect])

  // 自動クローズ: モード変化、またはキーごとの解消条件。
  useEffect(() => {
    if (guide && shouldGuideClose(guide, { mode, estopHw })) setGuide(null)
  }, [guide, mode, estopHw])

  return { guideKey: guide?.key ?? null, closeGuide: () => setGuide(null) }
}
