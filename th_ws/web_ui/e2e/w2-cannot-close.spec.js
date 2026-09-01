// C-1 (DetailedDesign-wp1.md WP-CARRY-01 §4.3, §7): W-2 (手押しウィンドウ) is
// not closable until the physical E-Stop button is released. Unlike W-1/W-4,
// Windows.jsx renders no dismiss/resume button in the carry window at all
// while estop_hw stays true (see its `carryHwReleased` gate) -- and #overlay
// wires no generic backdrop-click / Escape dismissal for any window (only
// per-window buttons close things). This test pins that absence down so a
// future "add a generic close affordance" change can't silently reopen the
// gap this invariant exists to prevent (a test operator must recognize the
// drive is cut, C-1's rationale).
import { test, expect } from '@playwright/test'
import { gotoWithState, setTestState } from './helpers.js'

test('W-2 has no dismiss control and survives backdrop-click/Escape while hw estop is held', async ({ page }) => {
  await gotoWithState(page, { mode: 'CARRY', estop_hw: true })

  const carryWin = page.locator('.win.carry')
  await expect(carryWin).toHaveClass(/show/)

  // No button at all inside W-2 while still pressed (no resume/dismiss --
  // Windows.jsx only renders those once carryHwReleased is true).
  await expect(carryWin.locator('button')).toHaveCount(0)

  // A click on the modal backdrop must not close it (no dismiss handler is
  // wired to #overlay .backdrop for any window). force:true because the
  // centered win.carry itself visually covers the backdrop at its default
  // click point -- that's exactly why this needs a direct backdrop click,
  // not a proxy for "click outside the window".
  await page.locator('#overlay .backdrop').click({ force: true })
  await expect(carryWin).toHaveClass(/show/)

  // Escape must not close it either.
  await page.keyboard.press('Escape')
  await expect(carryWin).toHaveClass(/show/)

  // Sanity check: the absence of buttons above is really gated on hw
  // release, not a rendering bug -- once released, the resume/dismiss
  // controls do appear.
  await setTestState(page, { estop_hw: false })
  await expect(carryWin.locator('button')).not.toHaveCount(0)
})
