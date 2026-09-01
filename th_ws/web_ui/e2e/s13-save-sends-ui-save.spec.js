// P5 / demo-teach-replay: S-13 の操作カード「保存」ボタンが実際に ui.save を
// 送ること（operationCardLayout('TEACH_MANUAL') の has_record から save スロット
// を出し、onTrigger が空関数でないことを検証。S-11 の ui.stop と同じ狙い）。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('S-13 保存 button sends ui.save', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'REC' })
  await page.locator('#s13').waitFor()

  await page.locator('.opsgrid .op-save').click()

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger), '保存ボタンが ui.save を送っていない')
    .toContain('ui.save')
})
