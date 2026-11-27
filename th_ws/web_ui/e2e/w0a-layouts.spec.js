// e2e/w0a-layouts.spec.js — レイアウト骨組み・人検出（§3.3・§3.5）。
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('レイアウト部品は画面内で data-pos づきに並ぶ（DOM が状態で動かない）', async ({ page }) => {
  // OperationCard の格子位置は状態で動かない（状態が変わっても同じセル）。
  await gotoWithState(page, { mode: 'FOLLOW', state: 'RUN' })
  const stop1 = await page.locator('.op-stop').boundingBox()
  await page.evaluate(() => window.__thSetTestState({ state: 'PAUSE' }))
  await page.waitForTimeout(300)
  const stop2 = await page.locator('.op-stop').boundingBox()
  expect(stop1.x).toBe(stop2.x)
  expect(stop1.y).toBe(stop2.y)
})

test('人検出トグルは入のときだけ明るく点く', async ({ page }) => {
  // ハーネスは enabled=false で表示するだけ（実機の状態が正本）。
  // 入（on）は .on が付き、切（off）が選ばれている方にだけ .on が付く。
  await gotoWithState(page, { mode: 'IDLE', state: 'NONE' })
  await expect(page.getByTestId('harness-tracker-on')).not.toHaveClass(/ on$/)
  await expect(page.getByTestId('harness-tracker-off')).toHaveClass(/on/)
})
