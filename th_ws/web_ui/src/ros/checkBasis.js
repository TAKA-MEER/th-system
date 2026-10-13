// ros/checkBasis.js — CheckStatus.basis_json（判定の根拠）の解釈。純関数（React 非依存）。
//
// 機体（opcheck_runner）は最終判定のときだけ、判定に使った測定値と基準の組を
// JSON 配列で載せる:
//   [{"key":"coverage_gap_deg","value":7.5,"threshold":20.0,"unit":"deg","cmp":"lt","ok":true}, ...]
// ラベル（日本語）は機体に持たせず i18n/screens.js が持つ。古い機体（フィールド無し）・
// 判定中（空文字）・壊れた JSON は空配列 = 「根拠なし」として扱い、画面は従来どおりにする。
const CMPS = ['lt', 'le', 'ge']

export function parseBasis(json) {
  if (typeof json !== 'string' || json === '') return []
  let arr
  try { arr = JSON.parse(json) } catch { return [] }
  if (!Array.isArray(arr)) return []
  const out = []
  for (const b of arr) {
    if (!b || typeof b.key !== 'string' || b.key === '') continue
    out.push({
      key: b.key,
      value: typeof b.value === 'number' && Number.isFinite(b.value) ? b.value : null,
      threshold: typeof b.threshold === 'number' && Number.isFinite(b.threshold) ? b.threshold : null,
      unit: typeof b.unit === 'string' ? b.unit : '',
      cmp: CMPS.includes(b.cmp) ? b.cmp : 'le',
      // 基準を満たしたかは機体が決める。false だけを「満たさない」として扱う
      ok: b.ok !== false,
    })
  }
  return out
}
