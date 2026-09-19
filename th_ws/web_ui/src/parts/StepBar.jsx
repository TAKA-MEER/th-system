// parts/StepBar.jsx — 手順インジケータ（brief-onsite-ux UX-1。S-20 / S-21 共用）。
//
// 画面本文の一番上（タブ列より上）に横一列で置き、「いま何すべきか」を文字を
// 読まずに分からせる。丸番号 → ラベル → 細い横線 → 次の丸、と続く。
//
//   done（完了）: 丸を塗りつぶし、中に ✓（インライン SVG。絵文字は使わない）
//   current（現在）: 太い枠＋外側リングで他より大きく、ラベルも太字
//   （現在以外の未着手）: 淡色（opacity を下げる）
//
// 現在の段は aria-current="step"、全体は role="list"、各段は role="listitem"。
// 狭い画面ではラベルを畳んで丸だけにする（CSS 側。Spec-webui.md §1.4 と同じ考え方）。
import { Fragment } from 'react'

function CheckIcon() {
  return (
    <svg viewBox="0 0 16 16" className="stepbar-check" aria-hidden="true">
      <path d="M3 8.5 L6.5 12 L13 4.5" fill="none" stroke="currentColor"
        strokeWidth="2.4" strokeLinecap="round" strokeLinejoin="round" />
    </svg>
  )
}

export default function StepBar({ steps, currentIndex, testId }) {
  return (
    <ol className="stepbar" role="list" data-testid={`${testId}-stepper`}>
      {steps.map((step, i) => {
        const done = !!step.done
        const cur = i === currentIndex
        const cls = `stepbar-step${done ? ' done' : ''}${cur ? ' cur' : ''}`
        return (
          <Fragment key={step.id}>
            {i > 0 && <li className="stepbar-link" aria-hidden="true" role="presentation" />}
            <li
              role="listitem"
              className={cls}
              data-testid={`step-${i + 1}`}
              aria-current={cur ? 'step' : undefined}
            >
              <span className="stepbar-num" aria-hidden="true">
                {done ? <CheckIcon /> : i + 1}
              </span>
              <span className="stepbar-label">{step.label}</span>
            </li>
          </Fragment>
        )
      })}
    </ol>
  )
}