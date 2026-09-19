// DetailedDesign-wp1.md WP-UI-02 §6.2: evt.link_ok folds into
// SystemState.mode leaving 'INIT' (DetailedDesign-state.md §12.1 step 7).
// Until then S-00 must stay put and must not even show a "advance" button
// (not just disable one) -- DetailedDesign-webui.md §3.1 "進む | 必須3者が
// 揃うまでメインメニューへのボタンは非活性".
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState } from './helpers.js'

test('no advance button while mode is INIT', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'INIT' })
  await expect(page.locator('#screenName')).toHaveText('接続確認')
  await expect(page.locator('[data-testid="s00-advance"]')).toHaveCount(0)
})

test('advance button appears once mode leaves INIT, and moves to S-01', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'INIT' })
  await expect(page.locator('[data-testid="s00-advance"]')).toHaveCount(0)

  await setTestState(page, { mode: 'IDLE' })
  const advance = page.locator('[data-testid="s00-advance"]')
  await expect(advance).toBeVisible()

  await advance.click()
  await expect(page.locator('#screenName')).toHaveText('メインメニュー')
})

test('no advance button while the link is unknown (mode never arrived)', async ({ page }) => {
  // Distinct from the INIT case: nothing seeded at all except an empty
  // patch, exercising the "mode is null" fail-safe branch of the same
  // ready check (menuItems()/S00Connect's `ready` both treat null like INIT).
  await gotoScreen(page, 'S00', {})
  await setTestState(page, { mode: null })
  await expect(page.locator('[data-testid="s00-advance"]')).toHaveCount(0)
})
