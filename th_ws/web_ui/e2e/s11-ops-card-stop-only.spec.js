// S-11 は「停止する」だけの停止│走行トグル（runLabel=null）。走行の契機は
// スティックだけ（Spec-webui.md §3.4）。古い操作カードは使わない。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('S-11 offers 停止する only (no run side)', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE', tracker_enabled: true })
  await page.locator('#s11').waitFor()
  const toggle = page.locator('[data-testid="s11-stop-run"]')
  await expect(toggle).toBeVisible()
  await expect(toggle.getByRole('button', { name: '停止する' })).toHaveCount(1)
  await expect(toggle.getByRole('button', { name: '走行する' })).toHaveCount(0)
  await expect(page.locator('#s11 .card.ops')).toHaveCount(0)
  await expect(page.locator('#s11 [role=tab]')).toHaveCount(0)
})
