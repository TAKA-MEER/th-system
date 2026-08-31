// shell/jogPanel.js — context for screens to open/close W-6, the floating
// manual-operation panel (DetailedDesign-webui.md §6.3, §6 W-6 / Spec-webui.md
// §4.0 W-6).
//
// W-6 is opened by a screen's "手動" button on screens that can't keep the
// virtual stick on screen (S-14/S-15/S-16/S-20/S-21), and is closed by that
// button's sibling "閉じる", by another drive operation, by leaving the
// screen, or by an estop/fault (see Windows.jsx). The panel itself lives in
// the shell (Windows.jsx, a sibling of #hdr) so it floats above the body as
// `position:absolute` without moving the body's layout (U3-5).
//
// Like shell/confirmWindow.js this is a bare context; the shell owns the
// open/close state and provides it through context so a screen can open the
// panel without the shell knowing anything about that screen.
import { createContext, useContext } from 'react'

export const JogPanelContext = createContext(null)

export function useJogPanel() {
  const ctx = useContext(JogPanelContext)
  if (!ctx) throw new Error('useJogPanel() must be used inside <AppShell>')
  return ctx
}
