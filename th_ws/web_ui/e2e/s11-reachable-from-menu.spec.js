// DetailedDesign-wp3.md WP-TRANSIT-01 §7: S-01's 手動走行 (mode MANUAL) button,
// once th_state accepts ui.enter_mode, must land on S-11 instead of staying on
// S-01 (this packet's whole point: "今回の目的そのもの").
//
// There is no th_state in the e2e environment, so the acceptance is stubbed
// via window.__thTestTrigger['ui.enter_mode'] = { accepted: true } — the same
// TEST_MODE pattern useStdTrigger.js uses for services (helpers.js stubTrigger,
// ros/useTrigger.js's test hook). This drives the *accepted* branch of
// S01Main.handleMenuClick -> main.jsx's MODE_TO_SCREEN['MANUAL'] = 'S11'.
import { test, expect } from '@playwright/test'
import { gotoScreen, stubTrigger } from './helpers.js'

test('S-01 手動走行 accepted by th_state -> S-11 renders', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true, reject_reason_key: '' } })
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  const manual = page.getByRole('button', { name: '手動走行' })
  await expect(manual).toBeEnabled()
  await manual.click()

  // The header shows the S-11 screen name (SCREEN_NAMES.S11 = '手動操作').
  await expect(page.locator('#s11')).toBeVisible()
})

test('S-01 手動走行 rejected by th_state stays on S-01 with a reason window', async ({ page }) => {
  // Do not stub: useTrigger defaults non-stubbed triggers to a denial, so
  // this exercises the reject branch without a reason key (U-11: the UI
  // shows why even for a denied-but-visible button).
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  const manual = page.getByRole('button', { name: '手動走行' })
  await manual.click()

  // Rejected: no S-11, reason window shown on S-01.
  await expect(page.locator('#s11')).toHaveCount(0)
  await expect(page.locator('.win.confirm.show')).toBeVisible()
})
