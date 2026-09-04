// WS-9R (2026-09-04): 止まっている理由を必ず画面に出す。
//
// 実機フィードバック「謎の一時停止が発生する。画面をスクロールしたりすると
// 復帰する」。原因は在席確認（derive_limits）が speed_limit=stop に倒していた
// ことだったが、**フォルトでも一時停止でもないので画面に何も出ていなかった**。
// 操作者に理由が分からないのが「謎」の正体。
//
// 帯は本文を押し下げてはいけない（操作者の指の下でボタンが動く）。
// e2e/header-geometry-stable.spec.js が本文の開始位置を見張っている。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('在席が確認できないと理由が画面に出る', async ({ page }) => {
  await gotoScreen(page, 'S01', {
    mode: 'IDLE', zone: 'NA', speed_limit: 'stop',
  })
  const banner = page.getByTestId('stop-banner')
  await expect(banner).toBeVisible()
  await expect(banner).toContainText('在席が確認できない')
})

test('通常走行中は帯を出さない', async ({ page }) => {
  await gotoScreen(page, 'S01', {
    mode: 'IDLE', zone: 'OUT', speed_limit: 'v_max',
  })
  await expect(page.getByTestId('stop-banner')).toHaveCount(0)
})

test('非常停止中は W-1 の窓が説明するので帯を二重に出さない', async ({ page }) => {
  await gotoScreen(page, 'S01', {
    mode: 'ESTOP', zone: 'NA', speed_limit: 'stop',
  })
  await expect(page.getByTestId('stop-banner')).toHaveCount(0)
})

test('帯は本文を押し下げない（指の下でボタンが動かない）', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', zone: 'OUT', speed_limit: 'v_max' })
  const before = await page.evaluate(
    () => document.getElementById('body').getBoundingClientRect().top)

  await page.evaluate(() => {
    window.__thSetTestState?.({ mode: 'IDLE', zone: 'NA', speed_limit: 'stop' })
  })
  await expect(page.getByTestId('stop-banner')).toBeVisible()

  const after = await page.evaluate(
    () => document.getElementById('body').getBoundingClientRect().top)
  expect(after).toBe(before)
})
