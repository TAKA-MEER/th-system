// DetailedDesign-webui.md §2.1 (SD-3): with W-1 open, the estop *release*
// bar must still be clickable -- otherwise pressing the estop leaves you
// unable to release it (W-1's own hint says "解除は画面下端の解除バーから").
// This guards the #release z-index:70 sitting above #overlay's backdrop
// (z-index:50). Companion to estop-clickable.spec.js (the engage button).
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('estop release bar is clickable while W-1 (fault window) is open', async ({ page }) => {
  await gotoWithState(page, { mode: 'ESTOP', estop_ui: true, estop_hw: true })

  // W-1 must actually be open (and its backdrop shown) for this to mean
  // anything.
  await expect(page.locator('.win.fault.show')).toBeVisible()
  await expect(page.locator('#overlay.has-modal .backdrop')).toBeVisible()

  const releaseBar = page.locator('#release')
  await expect(releaseBar).toBeVisible()

  const releaseBtn = page.locator('#releaseBtn')
  await expect(releaseBtn).toBeVisible()
  // Playwright's click() fails if another element is on top at the hit point
  // (actionability check) -- exactly the failure mode this test guards
  // (the .backdrop used to swallow the tap).
  await releaseBtn.click()
})
