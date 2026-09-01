// P5 / demo-teach-replay: S-14 の操作カード「再生」ボタン（ラベル「再生」）が
// 実際に ui.run を送ること。REPLAY の run_label を operationCard の runLabel で
// 差し替え、onTrigger が空関数でないことを検証する（S-11 の ui.stop と同じ狙い）。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('S-14 再生 button is labeled 再生 and sends ui.run', async ({ page }) => {
  await gotoScreen(page, 'S14', { mode: 'REPLAY', state: 'READY' })
  await page.locator('#s14').waitFor()

  // ラベルが「再生」であること（runLabel 差し替え）。
  await expect(page.locator('.opsgrid .op-run')).toHaveText('再生')

  await page.locator('.opsgrid .op-run').click()

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger), '再生ボタンが ui.run を送っていない')
    .toContain('ui.run')
})
