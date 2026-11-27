// DetailedDesign-wp1.md WP-UI-02 §6.2: evt.link_ok folds into
// SystemState.mode leaving 'INIT' (DetailedDesign-state.md §12.1 step 7).
// Until then S-00 must stay put. Spec-webui §3.1 (2026-11-27): 右列の
// 「メインメニューへ」は必須 3 者が揃うまで押せない（押すと理由）。
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState } from './helpers.js'

test('mode is INIT: advance is shown disabled and shows a reason on click', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'INIT' })
  await expect(page.locator('#screenName')).toHaveText('接続確認')
  const advance = page.locator('[data-testid="s00-advance"]')
  await expect(advance).toBeVisible()
  await expect(advance).toHaveClass(/dis/)
  await advance.click({ force: true })
  await expect(page.locator('[data-testid="s00-advance-reason"]')).toContainText('確認中')
  await expect(page.locator('#screenName')).toHaveText('接続確認')
})

test('advance appears enabled once mode leaves INIT, and moves to S-01', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'INIT' })
  await expect(page.locator('[data-testid="s00-advance"]')).toHaveClass(/dis/)

  await setTestState(page, { mode: 'IDLE' })
  const advance = page.locator('[data-testid="s00-advance"]')
  await expect(advance).not.toHaveClass(/dis/)

  await advance.click()
  await expect(page.locator('#screenName')).toHaveText('メインメニュー')
})

test('mode unknown (mode never arrived): advance is disabled with a reason', async ({ page }) => {
  // Distinct from the INIT case: nothing seeded at all except an empty
  // patch, exercising the "mode is null" fail-safe branch of the same
  // ready check.
  await gotoScreen(page, 'S00', {})
  await setTestState(page, { mode: null })
  const advance = page.locator('[data-testid="s00-advance"]')
  await expect(advance).toBeVisible()
  await expect(advance).toHaveClass(/dis/)
  await expect(page.locator('#screenName')).toHaveText('接続確認')
})
