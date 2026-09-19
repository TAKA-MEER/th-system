// DetailedDesign-webui.md §9-3 / §2.2 (U-9): the estop button's height must
// be greater than every other button on screen, not just larger in area.
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('estop button is taller than every other button on screen (portrait)', async ({ page }) => {
  await page.setViewportSize({ width: 420, height: 840 })
  // ESTOP with both latches still held: shows the fault window's "hide"
  // footer button plus the release bar's button alongside the header.
  await gotoWithState(page, { mode: 'ESTOP', estop_ui: true, estop_hw: true })
  await expect(page.locator('.win.fault.show')).toBeVisible()

  const heights = await page.locator('button:visible').evaluateAll(
    (els) => els.map((el) => ({
      height: el.getBoundingClientRect().height,
      id: el.id || el.className,
    })),
  )
  expect(heights.length).toBeGreaterThan(1)

  const estopHeight = heights.find((h) => h.id.includes('estopBtn'))?.height
  expect(estopHeight).toBeGreaterThan(0)

  for (const h of heights) {
    if (h.id.includes('estopBtn')) continue
    expect(h.height, `button "${h.id}" (${h.height}px) must not be taller than #estopBtn (${estopHeight}px)`)
      .toBeLessThan(estopHeight)
  }
})

test('estop button meets its minimum size (>=72px tall, >=160px wide)', async ({ page }) => {
  await gotoWithState(page, {})
  const box = await page.locator('#estopBtn').boundingBox()
  expect(box.height).toBeGreaterThanOrEqual(72)
  expect(box.width).toBeGreaterThanOrEqual(160)
})
