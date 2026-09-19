// C-2 (DetailedDesign-wp1.md WP-CARRY-01 §4.3, §7): while in CARRY, the UI
// E-Stop button must not be hidden, must look like it won't work, and W-2
// must say the drive is already cut.
//
// This does NOT go through /system/trigger. The UI estop's own safety path
// is /safety/estop_ui, published unconditionally by AppShell.jsx while the
// button is held (DetailedDesign-safety.md §6.2: th_state must never sit on
// that path -- it must stay able to cut drive even if the FSM hangs).
// state_manager separately subscribes to that same topic and, in CARRY,
// rejects the resulting ui.estop.press event via C-06r
// (test_state_core.py::test_estop_in_carry_rejected), republishing
// reject_reason_key = estop_disabled_in_carry through
// SystemState.last_reject_reason (DetailedDesign-state.md :764). Windows.jsx
// only reads that field back out -- it never calls a service to get it.
//
// This spec has no rosbridge backend (helpers.js -- U-3), so the click
// itself is a no-op wire-wise; setTestState() after it stands in for
// state_manager's side of that round-trip.
//
// Button visibility/clickability in CARRY is already covered by
// estop-clickable.spec.js; W-2's lack of a dismiss control while estop_hw
// stays pressed is already covered by w2-cannot-close.spec.js. This file
// only adds what C-2 needs on top of those.
import { test, expect } from '@playwright/test'
import { gotoWithState, setTestState } from './helpers.js'

test('W-2 says the drive is already cut, before the UI estop is ever pressed', async ({ page }) => {
  await gotoWithState(page, { mode: 'CARRY', estop_hw: true })
  await expect(page.locator('.win.carry.show')).toContainText('駆動は既に切れています')
})

test('pressing the UI estop in CARRY surfaces estop_disabled_in_carry via SystemState.last_reject_reason', async ({ page }) => {
  await gotoWithState(page, { mode: 'CARRY', estop_hw: true })

  await page.locator('#estopBtn').click()
  // Stand-in for state_manager's C-06r rejection reaching this client
  // through /system/state (see file header).
  await setTestState(page, { last_reject_reason: 'estop_disabled_in_carry' })

  await expect(page.locator('.win.carry.show')).toContainText('手押し中は非常停止を操作できません')
})

test('the estop button does not visually claim success (no "engaged" look) while in CARRY', async ({ page }) => {
  await gotoWithState(page, { mode: 'CARRY', estop_hw: true })

  const estopBtn = page.locator('#estopBtn')
  await estopBtn.click()

  // .engaged is the class a *genuinely accepted* press gets (theme.css
  // #estopBtn.engaged). Getting it here would visually claim the press
  // worked when C-06r actually rejected it -- worse than a plainly inert
  // button (the packet's own rationale for C-2). CARRY instead gets its
  // own disabled-in-carry look (theme.css), and the button stays enabled
  // and unhidden -- the safety-path publish to /safety/estop_ui must not
  // be blocked by the UI (§6.2).
  await expect(estopBtn).not.toHaveClass(/engaged/)
  await expect(estopBtn).toHaveClass(/disabled-in-carry/)
  await expect(estopBtn).toBeEnabled()
})
