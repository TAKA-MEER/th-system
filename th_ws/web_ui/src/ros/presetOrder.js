// ros/presetOrder.js — S-50 速度プリセット3段階の順序規則（SG-C14）。
// Spec-webui.md §3.15「S-50 の項目の定義」: 低速 ≦ 中速 ≦ 高速を保つ。
// 各段の設定範囲は「下の段の値以上、上の段の値以下」（低速の下限と高速の
// 上限は上限そのもの＝100% が天井）。違反は保存不可で理由を表示する。
//
// 機体側（params_audit の /params/set）にも同じ規則があり、そちらが正本。
// ここは「押す前に分かる」ための写し（文言は i18n/screens.js）。

export const PRESET_ORDER_NAMES = ['speed_preset_low', 'speed_preset_mid', 'speed_preset_high']
export const PRESET_MIN = 0.05
export const PRESET_MAX = 1.0

// 違反のキー（無ければ ''）。i18n/screens.js の S50_PRESET_ORDER_REASONS が文言を持つ。
export function presetOrderError(values) {
  const [low, mid, high] = PRESET_ORDER_NAMES.map((n) => values?.[n])
  if (![low, mid, high].every((v) => typeof v === 'number' && Number.isFinite(v))) return ''
  if (low < PRESET_MIN || mid < PRESET_MIN || high < PRESET_MIN) return 'below_min'
  if (high > PRESET_MAX) return 'over_ceiling'
  if (low > mid) return 'low_gt_mid'
  if (mid > high) return 'mid_gt_high'
  return ''
}
