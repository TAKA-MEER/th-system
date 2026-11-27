// e2e/w0a-reason-motion.spec.js — 押せない理由（§3.3.3）と reduced motion（§3.3.4/§3.3.5）。
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('押せないボタンを押すと、すぐ上に「押せません　〈理由〉」が出てヘッダに重ならない', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE', state: 'NONE' })
  const btn = page.getByTestId('harness-reason-btn')
  await expect(btn).toBeVisible()
  // 押せない側は aria-disabled の見た目（Playwright は「無効」とみて通常 click できない）。
  await btn.dispatchEvent('click')
  const bubble = page.getByTestId('harness-reason')
  await expect(bubble).toBeVisible()
  await expect(bubble).toHaveText('押せません　場外では手動は使えません')
  const bBox = await bubble.boundingBox()
  const btnBox = await btn.boundingBox()
  const hdrBox = await page.locator('#hdr').boundingBox()
  // 吹き出しはボタンの上側（ボタンの下端より上）
  expect(bBox.y + bBox.height).toBeLessThanOrEqual(btnBox.y + 1)
  // ヘッダと重ならない（吹き出しの上端がヘッダの下端より下）
  expect(bBox.y).toBeGreaterThanOrEqual(hdrBox.y + hdrBox.height - 1)
})

test('prefers-reduced-motion: reduce では明滅と枠の点滅が止まる', async ({ page }) => {
  await page.emulateMedia({ reducedMotion: 'reduce' })
  await gotoWithState(page, { mode: 'FOLLOW', state: 'RUN', auto_brake: false })
  const animOf = (sel) => page.locator(sel).evaluate((el) => getComputedStyle(el).animationName)
  await expect(page.getByTestId('autonomous-frame')).toBeVisible()
  expect(await animOf('[data-testid="autonomous-frame"]')).toBe('none')
  expect(await animOf('#autoBrakeOff')).toBe('none')
  expect(await animOf('[data-testid="harness-stoprun-run"]')).toBe('none')
})

test('prefers-reduced-motion: no-preference では明滅が効く', async ({ page }) => {
  await page.emulateMedia({ reducedMotion: 'no-preference' })
  await gotoWithState(page, { mode: 'FOLLOW', state: 'RUN', auto_brake: false })
  const animOf = (sel) => page.locator(sel).evaluate((el) => getComputedStyle(el).animationName)
  expect(await animOf('[data-testid="autonomous-frame"]')).not.toBe('none')
  expect(await animOf('#autoBrakeOff')).not.toBe('none')
  expect(await animOf('[data-testid="harness-stoprun-run"]')).not.toBe('none')
})
