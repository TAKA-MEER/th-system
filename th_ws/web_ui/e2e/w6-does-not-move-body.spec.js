// DetailedDesign-wp3.md WP-UI-03 §9-9 / U3-5: opening/closing W-6 must not
// move the main display a single pixel. The panel is `position:absolute`
// (theme.css #jogWin), so its appear/disappear must leave the body layout
// untouched -- a regression to a re-flowing band would shrink the map column.
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

const rect = (page) => page.locator('#body').evaluate((el) => {
  const r = el.getBoundingClientRect()
  return { x: r.x, y: r.y, width: r.width, height: r.height }
})

test('opening and closing W-6 does not move the main display', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'IDLE' })
  await page.locator('#body').waitFor()

  const before = await rect(page)
  await page.click('[data-testid="open-jog"]')
  await page.locator('#jogWin.show').waitFor()
  await page.waitForTimeout(50) // allow any transition to settle
  expect(await rect(page), "W-6 open must not move the body").toEqual(before)

  // Close it again (the "閉じる" button) and re-verify.
  await page.click('#jogWin .jw-hd .btn')
  // 閉じた W-6 は `display:none`（theme.css `#jogWin.show` だけが block）。
  // 既定の waitFor() は「visible になるまで」なので永久に待つ。hidden で待つ。
  await page.locator('#jogWin').waitFor({ state: 'hidden' })
  await page.waitForTimeout(50)
  expect(await rect(page), "W-6 close must not move the body").toEqual(before)
})
