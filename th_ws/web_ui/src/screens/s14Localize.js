// screens/s14Localize.js — S-14 初期姿勢の確度表示の純ロジック（W-01 P3）。
// /route/status（RouteStatus）の localize_quality を画面表示へ写す。
// 数値は出さず「高い／低い」の 2 値＋目安文（DetailedDesign-transit-localize-options.md §4）。
//
// localize_quality の取りうる値（ROS 側 P2 が足す。未マージの間は undefined）:
//   'high'       成立 → 「高い」
//   'low_margin' 成立したが似た場所あり → 「低い」＋警告（READY の再生は押せるまま）
//   'failed'     不成立（LOCALIZE に留まる）→ 「見つかりません」＋案内
//   'off_route'  推定は成立したが経路から離れすぎ（A-24）→ 「経路から離れています」
//                ＋経路の近くへ置き直す案内（推定の失敗と区別する）
//   'searching'  探索中 → 「探しています」（60 秒「長引いている」注記は出さない）
//   'unknown'／''／undefined 旧経路・未受信 → 確度欄を出さない
//
// S14Replay.jsx が使う。unit（test/unit/s14-localize-quality.test.js,
// test/unit/s14-replay-ready.test.js）と e2e（e2e/s14-localize.spec.js,
// e2e/s14-ready.spec.js）の両方で縛る。
import {
  S14_QUALITY_TITLE,
  S14_QUALITY_HIGH,
  S14_QUALITY_LOW,
  S14_QUALITY_LOW_HINT,
  S14_QUALITY_FAILED,
  S14_QUALITY_FAILED_HINT,
  S14_QUALITY_OFF_ROUTE,
  S14_QUALITY_OFF_ROUTE_HINT,
  S14_QUALITY_SEARCHING,
} from '../i18n/screens.js'

// quality → { visible, level }。level は 'high' / 'low' / 'failed' /
// 'off_route' / 'searching' のいずれか。visible が false なら確度欄を出さない。
export function localizeQualityView(quality) {
  switch (quality) {
    case 'high':
      return { visible: true, level: 'high' }
    case 'low_margin':
      return { visible: true, level: 'low' }
    case 'failed':
      return { visible: true, level: 'failed' }
    case 'off_route':
      return { visible: true, level: 'off_route' }
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
    case 'off_route':
      return `${S14_QUALITY_TITLE}：${S14_QUALITY_OFF_ROUTE}。${S14_QUALITY_OFF_ROUTE_HINT}`
    case 'searching':
      return `${S14_QUALITY_TITLE}：${S14_QUALITY_SEARCHING}`
    default:
      return null
  }
}

// A-23 ②: 「再生」を押せるか。位置が決まる前（LOCALIZE）は押せず、理由を返す。
// READY は押せる（localize_quality='unknown'＝地図の画像が無い経路も、
// 始点決め打ちで READY に進むので押せる）。PAUSE からの再開（一時停止の続き）も
// ここでは止めない。RUN の「再生」も止めない（Spec-webui.md §3.7）。
export function replayRunGate(stateName, quality) {
  if (stateName !== 'LOCALIZE') return { enabled: true, reason: null }
  const { level } = localizeQualityView(quality)
  if (level === 'failed') return { enabled: false, reason: S14_QUALITY_FAILED }
  if (level === 'off_route') return { enabled: false, reason: S14_QUALITY_OFF_ROUTE }
  // searching・high（reload 待ち）・未受信・unknown（読み直し中）は「探しています」。
  return { enabled: false, reason: S14_QUALITY_SEARCHING }
}

// A-22: 「地図を読み込んでいます」を地図の上に出すか。経路を選ぶと LOCALIZE に入り、
// その間に地図を読み直す。既存の信号だけで判定する（新しいトピックは足さない）。
//   - localize_quality === 'searching' … 探索＋地図の読み直し中（replay_runner）
//   - /route/map_view がまだ届いていない … 地図そのものがまだ無い
// failed／off_route は地図の読み込みが終わった（または失敗した）ので出さない。
export function mapLoadingVisible(stateName, quality, hasMap) {
  if (stateName !== 'LOCALIZE') return false
  if (quality === 'failed' || quality === 'off_route') return false
  return quality === 'searching' || !hasMap
}
