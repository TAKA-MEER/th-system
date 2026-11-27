// S-13 はタブを使わない（Spec-webui §3.3）。入口は主表示の中央のカード。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('S-13 has no tabs; entry card is in the main view', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL' })
  await page.locator('#s13').waitFor()
  await expect(page.locator('#s13 [role=tab]')).toHaveCount(0)
  await expect(page.locator('#s13 .s13-entry')).toBeVisible()
  await expect(page.getByTestId('s13-map-map')).toBeVisible()
})
