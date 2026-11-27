// e2e/w0a-help.spec.js — ヘルプ表示（§3.3.6）の縛り。
// ヘッダには重ねない。ヘッダと停止は1回で効く（この部品はクリックを止めない）。
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('ヘルプはマウスホバーで出て、ヘッダに重ならない', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE', state: 'NONE' })
  const finishBtn = page.getByTestId('harness-finish')
  await expect(finishBtn).toBeVisible()
  await finishBtn.hover()
  await page.waitForTimeout(900)
  const help = page.getByTestId('help-finish')
  await expect(help).toBeVisible()
  const helpBox = await help.boundingBox()
  const hdrBox = await page.locator('#hdr').boundingBox()
  expect(helpBox.y).toBeGreaterThanOrEqual(hdrBox.y + hdrBox.height - 1)
})

test('ヘルプが出ていてもボタンのクリックは吸われない（1回で効く）', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE', state: 'NONE' })
  const finishBtn = page.getByTestId('harness-finish')
  let clicked = false
  await finishBtn.evaluate((el) => { el.addEventListener('click', () => { window.__helpClicked = true }) })
  await finishBtn.hover()
  await page.waitForTimeout(900)
  await expect(page.getByTestId('help-finish')).toBeVisible()
  await finishBtn.click()
  clicked = await page.evaluate(() => window.__helpClicked === true)
  expect(clicked).toBe(true)
  await expect(page.locator('#estopBtn')).toBeVisible()
})
