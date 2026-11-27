// parts/ResultMark.jsx — 結果の印（Spec-webui §3.3.1 の P1）。
//
// 結果は語ではなく形で分ける: ✓＝正常（緑）/ !＝要確認（黄の縁）/
// ×＝だめ（黄の地）/ —＝まだ（灰）。結果に赤は使わない（赤は非常停止だけ）。
// 色だけに頼らないよう、形（記号）自体で区別する。
const MARKS = {
  OK: { glyph: '✓', label: '正常' },
  WARN: { glyph: '!', label: '要確認' },
  NG: { glyph: '×', label: 'だめ' },
  UNKNOWN: { glyph: '—', label: 'まだ' },
}

export default function ResultMark({ result, testId, small = false }) {
  const m = MARKS[result] ?? MARKS.UNKNOWN
  return (
    <span
      className={`rm rm-${(result ?? 'UNKNOWN').toLowerCase()}${small ? ' sm' : ''}`}
      data-testid={testId}
      data-result={result ?? 'UNKNOWN'}
      role="img"
      aria-label={m.label}
    >
      <span aria-hidden="true">{m.glyph}</span>
    </span>
  )
}
