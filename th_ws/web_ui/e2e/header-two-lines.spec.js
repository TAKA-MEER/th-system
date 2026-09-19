// DetailedDesign-webui.md §9-4 / §2.5 (U-10): the header never wraps past
// two rows (row1 / row2), across the widths the shell is meant to run at.
// A wrap adds roughly a full row's height (>=30px); a generous 230px cap
// catches that while tolerating the estop button's own §2.2 minimum height
// (72px) inflating row2.
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

const SIZES = [
  { width: 420, height: 840 },   // narrowest supported portrait (mockup default)
  { width: 768, height: 1024 },  // tablet portrait, the §2.4 minimum short edge
  { width: 1024, height: 768 },  // tablet landscape
  { width: 1280, height: 800 },
  { width: 1920, height: 1200 },
]

const HEADER_HEIGHT_BUDGET_PX = 230

for (const size of SIZES) {
  test(`header stays within 2 rows at ${size.width}x${size.height}`, async ({ page }) => {
    await page.setViewportSize(size)
    await gotoWithState(page, { mode: 'FOLLOW', state: 'RUN' })
    const box = await page.locator('#hdr').boundingBox()
    expect(box.height).toBeLessThanOrEqual(HEADER_HEIGHT_BUDGET_PX)
  })
}

test('header stays within 2 rows with dev pill + zone pill shown', async ({ page }) => {
  await page.setViewportSize({ width: 768, height: 1024 })
  await page.addInitScript(() => { window.__thTestState = { mode: 'PREP', state: 'MAPPING', zone: 'IN' } })
  await page.goto('/?dev=1')
  const box = await page.locator('#hdr').boundingBox()
  expect(box.height).toBeLessThanOrEqual(HEADER_HEIGHT_BUDGET_PX)
})
