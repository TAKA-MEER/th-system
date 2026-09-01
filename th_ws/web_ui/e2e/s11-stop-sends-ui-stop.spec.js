// DetailedDesign-wp3.md WP-TRANSIT-01 §4.2 / T1-4: S-11 の操作カードは
// 「停止」1 つだけ。**その 1 つが実際に ui.stop を送ること**を確かめる。
//
// なぜこのテストが要るか: 初版は OperationCard に `onTrigger={() => {}}` を
// 渡しており、**この画面で唯一のボタンが無反応**だった（2026-09-01）。
// 「ボタンが 1 つだけ在る」ことは s11-ops-card-stop-only.spec.js が見ていたが、
// 「押すと何が起きるか」は誰も見ていなかった。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('the only button on S-11 actually sends ui.stop', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'RUN' })
  await page.locator('#s11').waitFor()

  await page.click('.opsgrid .op-stop')

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger), '停止ボタンが ui.stop を送っていない')
    .toContain('ui.stop')
})
