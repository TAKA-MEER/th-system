// brief-onsite-fix A: S-01 の「終了して待機に戻る」ボタン（data-testid="s01-finish-escape"）。
// 画面の無いモード（OPCHECK 等）で S-01 が出ているときだけボタンが見え、
// ui.finish を送信することを確かめる。IDLE のときはボタンが無い。
import { test, expect } from '@playwright/test'
import { gotoScreen, stubTrigger } from './helpers.js'

test('OPCHECK: finish-escape ボタンが見え、押すと ui.finish を送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.finish': { accepted: true } })
  await gotoScreen(page, 'S01', { mode: 'OPCHECK', state: 'LIST' })

  const btn = page.getByTestId('s01-finish-escape')
  await expect(btn).toBeVisible()
  await expect(btn).toBeEnabled()

  await btn.click()

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.some((c) => c.trigger === 'ui.finish')).toBeTruthy()
})

test('IDLE: finish-escape ボタンは存在しない', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  const btn = page.getByTestId('s01-finish-escape')
  await expect(btn).toHaveCount(0)
})

test('INIT: finish-escape ボタンは存在しない', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'INIT' })

  const btn = page.getByTestId('s01-finish-escape')
  await expect(btn).toHaveCount(0)
})
