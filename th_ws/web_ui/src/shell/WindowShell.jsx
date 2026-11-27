// shell/WindowShell.jsx — 窓（W-1/W-2/W-4/W-5）の共通の出し方。
// Spec-webui §4: 本文の中央、ヘッダ以外を幕（#overlay .backdrop）で暗くする。
// どの画面・向きでも同じ。窓の題は「何が起きたか」＋対処 1 行は呼び出し側が渡す。
// W-5（経路が塞がれた）は画面側の S-21 が次段で同じ部品を使う（今は帯）。
// 使い方と各 spec の節は src/shell/README.md。
export default function WindowShell({ className = '', testId, title, children, footer = null }) {
  return (
    <div className={`win show ${className}`} data-testid={testId}>
      {title != null && <header>{title}</header>}
      <div className="bodyw">{children}</div>
      {footer != null && <footer>{footer}</footer>}
    </div>
  )
}
