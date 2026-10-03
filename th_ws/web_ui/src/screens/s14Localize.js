// screens/s14Localize.js — S-14 初期姿勢の確度表示の純ロジック（W-01 P3）。
// /route/status（RouteStatus）の localize_quality を画面表示へ写す。
// 数値は出さず「高い／低い」の 2 値＋目安文（DetailedDesign-transit-localize-options.md §4）。
//
// localize_quality の取りうる値（ROS 側 P2 が足す。未マージの間は undefined）:
//   'high'       成立 → 「高い」
//   'low_margin' 成立したが似た場所あり → 「低い」＋警告（READY の再生は押せるまま）
//   'failed'     不成立（LOCALIZE に留まる）→ 「見つかりません」＋案内
//   'searching'  探索中 → 「探しています」（60 秒「長引いている」注記は出さない）
//   'unknown'／''／undefined 旧経路・未受信 → 確度欄を出さない
//
// S14Replay.jsx が使う。unit（test/unit/s14-localize-quality.test.js）と
// e2e（e2e/s14-localize.spec.js）の両方で縛る。
import {
  S14_QUALITY_TITLE,
  S14_QUALITY_HIGH,
  S14_QUALITY_LOW,
  S14_QUALITY_LOW_HINT,
  S14_QUALITY_FAILED,
  S14_QUALITY_FAILED_HINT,
  S14_QUALITY_SEARCHING,
} from '../i18n/screens.js'

// quality → { visible, level }。level は 'high' / 'low' / 'failed' /
// 'searching' のいずれか。visible が false なら確度欄を出さない。
export function localizeQualityView(quality) {
  switch (quality) {
    case 'high':
      return { visible: true, level: 'high' }
    case 'low_margin':
      return { visible: true, level: 'low' }
    case 'failed':
      return { visible: true, level: 'failed' }
    case 'searching':
      return { visible: true, level: 'searching' }
    default:
      return { visible: false, level: null }
  }
}

// level → 確度欄の本文。null（出さない）もありうる。
export function localizeQualityText(level) {
  switch (level) {
    case 'high':
      return `${S14_QUALITY_TITLE}：${S14_QUALITY_HIGH}`
    case 'low':
      return `${S14_QUALITY_TITLE}：${S14_QUALITY_LOW}。${S14_QUALITY_LOW_HINT}`
    case 'failed':
      return `${S14_QUALITY_TITLE}：${S14_QUALITY_FAILED}。${S14_QUALITY_FAILED_HINT}`
    case 'searching':
      return `${S14_QUALITY_TITLE}：${S14_QUALITY_SEARCHING}`
    default:
      return null
  }
}
