// shell/confirmWindow.js — context for screens to open/close W-4 (the
// generic confirm window: DetailedDesign-webui.md §6, row W-4).
//
// The window itself must render as a sibling of #hdr (shell/Windows.jsx),
// not as a descendant of #body, so it shares the same stacking context as
// the header (z-index 100 must always win -- U-1) and the same overlay
// backdrop as W-1/W-2. But its *content* (the shutdown flow's per-item
// save/discard state, a rejection reason, ...) is owned by whichever screen
// opened it, and that content must stay live across re-renders (a screen's
// local state changing while the window is open has to show up inside it).
//
// A React portal is how a component renders into a DOM node it doesn't
// itself parent: shell/AppShell.jsx owns the mount node (a plain DOM ref,
// exposed as `mountNode` here) and shell/Windows.jsx renders the `.win`
// chrome around it; the screen portals its own JSX (header/body/footer)
// into that node with ReactDOM.createPortal. This avoids the alternative of
// threading rendered JSX through parent state, which would go stale between
// the screen's re-renders and AppShell's.
import { createContext, useContext } from 'react'

export const ConfirmWindowContext = createContext(null)

export function useConfirmWindow() {
  const ctx = useContext(ConfirmWindowContext)
  if (!ctx) throw new Error('useConfirmWindow() must be used inside <AppShell>')
  return ctx
}
