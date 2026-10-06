// shell/GuideBanner.jsx — W-3 案内の帯 (1b-6, SG-B2).
//
// Spec-webui.md §4.0: 本文の下に帯として差し込む 1 行。置き場 (#winGuide、
// theme.css で帯の見た目) は shell/Windows.jsx が持ち、中身だけがこの部品。
// 文言は i18n/guides.js。閉じるボタンあり (W6_CLOSE「閉じる」を流用)。
import { guideLabel } from '../i18n/guides.js'
import { W6_CLOSE } from '../i18n/screens.js'

export default function GuideBanner({ guideKey, onClose }) {
  const text = guideLabel(guideKey)
  if (!text) return null
  return (
    <>
      <span className="g-tx" data-testid="w3-guide" role="status">{text}</span>
      <button type="button" className="btn sm" data-testid="w3-guide-close" onClick={onClose}>
        {W6_CLOSE}
      </button>
    </>
  )
}
