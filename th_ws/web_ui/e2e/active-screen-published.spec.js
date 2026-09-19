// N-15 / DetailedDesign-wp1.md WP-UI-01 §3.1: /ui/active_screen must be
// published at 2Hz while an operator screen is mounted. There is no
// rosbridge backend in this e2e environment (see helpers.js), so
// ros/useActiveScreenPublisher.js records each publish attempt on
// window.__thActiveScreenPublishes instead of opening a real
// ROSLIB.Topic -- same shape as ros/useStdTrigger.js's
// window.__thTestServices stub.
//
// DetailedDesign-names.md §4.1's derive_limits() takes the minimum speed
// limit across interacting terminals, and treats zero of them as a link-down
// fail-safe (speed_limit=stop). ?view=audience must never be counted as an
// interacting terminal or it would pollute that minimum -- see helpers.js's
// gotoScreen() vs the raw goto('/?view=audience') this file uses.
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('publishes /ui/active_screen at roughly 2Hz on S-00', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'IDLE' })
  await page.waitForFunction(() => (window.__thActiveScreenPublishes ?? []).length > 0)

  await page.waitForTimeout(1100)
  const publishes = await page.evaluate(() => window.__thActiveScreenPublishes)

  // Immediate publish on mount + a tick every 500ms; allow slack for CI
  // timer jitter but this must not be "sent once and forgotten".
  expect(publishes.length).toBeGreaterThanOrEqual(2)
  for (const msg of publishes) {
    expect(msg.screen_id).toBe('S-00')
  }
})

test('does not publish on ?view=audience', async ({ page }) => {
  await page.goto('/?view=audience')
  // Give it the same window a real publisher would need for >= 2 ticks.
  await page.waitForTimeout(1200)

  const publishes = await page.evaluate(() => window.__thActiveScreenPublishes ?? [])
  expect(publishes).toEqual([])
})

test('client_id is stable across a reload', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'IDLE' })
  await page.waitForFunction(() => (window.__thActiveScreenPublishes ?? []).length > 0)
  const firstId = await page.evaluate(() => window.__thActiveScreenPublishes[0].client_id)
  expect(firstId).toBeTruthy()

  // Simulates the tablet reloading the page (new __thTestState/init scripts,
  // same browsing context -- so localStorage, which client_id persists
  // through, survives).
  await gotoScreen(page, 'S00', { mode: 'IDLE' })
  await page.waitForFunction(() => (window.__thActiveScreenPublishes ?? []).length > 0)
  const secondId = await page.evaluate(() => window.__thActiveScreenPublishes[0].client_id)

  expect(secondId).toBe(firstId)
})
