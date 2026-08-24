// DetailedDesign-webui.md §9-1 (U-1): with W-1 open, the estop button must
// still be clickable. This is the whole point of the header's z-index-100
// layer (theme.css #hdr) sitting above the #overlay windows (z-index 50).
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('estop button is clickable while W-1 (fault window) is open', async ({ page }) => {
  await gotoWithState(page, { mode: 'ESTOP', estop_ui: true, estop_hw: true })

  // W-1 (the fault/estop window) must actually be open for this test to
  // mean anything.
  await expect(page.locator('.win.fault.show')).toBeVisible()

  const estopBtn = page.locator('#estopBtn')
  await expect(estopBtn).toBeVisible()
  // Playwright's click() fails if another element is on top at that point
  // (actionability check) -- exactly the failure mode this test guards.
  await estopBtn.click()
})

test('estop button is clickable while W-2 (carry window) is open', async ({ page }) => {
  await gotoWithState(page, { mode: 'CARRY', estop_hw: true })

  await expect(page.locator('.win.carry.show')).toBeVisible()

  const estopBtn = page.locator('#estopBtn')
  await expect(estopBtn).toBeVisible()
  await estopBtn.click()
})
