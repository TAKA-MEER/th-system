// DetailedDesign-webui.md §9-5 (U-3): zero requests to any host other than
// the page's own origin. The robot's AP has no upstream internet, so any
// CDN / Google Fonts / icon-font request would simply hang on the tablet.
// Must run against the production build (`npm run build && preview`), not
// `npm run dev` -- see playwright.config.js's webServer.
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('no requests go to a host other than the page origin', async ({ page }) => {
  const external = []
  page.on('request', (req) => {
    const url = new URL(req.url())
    if (url.protocol === 'data:' || url.protocol === 'blob:') return
    if (url.origin !== 'http://localhost:4173') external.push(req.url())
  })

  await gotoWithState(page, { mode: 'IDLE' })
  // Let any async/lazy requests (fonts, late images) have a chance to fire.
  await page.waitForTimeout(1000)

  expect(external, `unexpected external requests: ${JSON.stringify(external)}`).toEqual([])
})

test('no requests go to a host other than the page origin (rosbridge-less, no test state)', async ({ page }) => {
  // Same check without the test-state hook, so the real (failing, since
  // there is no backend) rosbridge connection attempt is exercised too --
  // it must still target the page's own host, never an external one.
  const external = []
  page.on('request', (req) => {
    const url = new URL(req.url())
    if (url.protocol === 'data:' || url.protocol === 'blob:') return
    if (url.origin !== 'http://localhost:4173') external.push(req.url())
  })

  await page.goto('/')
  await page.waitForTimeout(1000)

  expect(external, `unexpected external requests: ${JSON.stringify(external)}`).toEqual([])
})
