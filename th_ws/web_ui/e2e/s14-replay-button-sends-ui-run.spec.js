// S-14 の停止│走行トグルの「再生する」が実際に ui.run を送ること。
// StopRunToggle の run 側が onRun で ui.run に配線されていることを検証する
// （S-11 の ui.stop と同じ狙い。READY で位置が決まって押せる）。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('S-14 再生する button is labeled 再生する and sends ui.run', async ({ page }) => {
  await gotoScreen(page, 'S14', { mode: 'REPLAY', state: 'READY' })
  await page.locator('#s14').waitFor()

  // ラベルが「再生する」であること。
  await expect(page.getByTestId('stop-run-toggle-run')).toContainText('再生する')

  await page.getByTestId('stop-run-toggle-run').click()

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger), '再生ボタンが ui.run を送っていない')
    .toContain('ui.run')
})
