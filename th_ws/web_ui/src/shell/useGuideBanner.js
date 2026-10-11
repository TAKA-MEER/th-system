// shell/useGuideBanner.js — W-3 案内の帯の寿命を持つ hook (1b-6, SG-B2).
//
// /system/effect の購読は ros/useSystemEffect.js (1 箇所)、振り分けは
// shell/effectDispatch.js。ここは「guide が来たら帯を立て、閉じる条件で
// 降ろす」だけをする。表示は shell/GuideBanner.jsx。
import { useEffect, useState } from 'react'
import { useSystemEffect } from '../ros/useSystemEffect.js'
import { dispatchEffect, shouldGuideClose, initEstopHeldGuideKey } from './effectDispatch.js'

export function useGuideBanner(ros, mode, sinceMs, estopHw) {
  const effect = useSystemEffect(ros)
  const [guide, setGuide] = useState(null)

  // effect が届くたびに振り分け、guide だけ帯を立てる。mode は「届いたときの
  // モード」として記録する (deps に mode を入れると、モード変化で再実行され
  // modeAtOpen が付け替わって自動クローズが効かなくなるため入れない)。
  // seenPressed は届いた時点で押されていれば真 (遅れて届いた guide 用)。
  useEffect(() => {
    if (!effect) return
    const decided = dispatchEffect(effect)
    if (decided.kind === 'guide') {
      setGuide({
        key: decided.key,
        modeAtOpen: mode,
        stampMs: decided.stampMs,
        seenPressed: estopHw,
      })
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [effect])

  // 自動クローズ: guide より後に入ったモードへの変化、またはキーごとの解消条件。
  // estop の「押されているのを見た」ラッチもここで進める。
  useEffect(() => {
    if (!guide) return
    const latched = (estopHw && !guide.seenPressed) ? { ...guide, seenPressed: true } : guide
    if (latched !== guide) setGuide(latched)
    if (shouldGuideClose(latched, { mode, sinceMs, estopHw })) setGuide(null)
  }, [guide, mode, sinceMs, estopHw])

  // 起動時の物理非常停止の案内は状態から導く (一回きりの effect を取りこぼす
  // 画面でも出す。SM-3.1.2-003)。閉じても、離されて押し直されるまでは出し直さない。
  const heldKey = initEstopHeldGuideKey({ mode, estopHw })
  const [heldDismissed, setHeldDismissed] = useState(false)
  useEffect(() => {
    if (!heldKey) setHeldDismissed(false)
  }, [heldKey])

  const effectKey = guide?.key ?? null
  const guideKey = effectKey ?? (heldDismissed ? null : heldKey)
  const closeGuide = () => {
    setGuide(null)
    if (heldKey) setHeldDismissed(true)
  }
  return { guideKey, closeGuide }
}
