// P5 / demo-teach-replay: S-13 の「記録開始」ボタンが、この画面で唯一のボタン
// に近い操作として、**実際に ui.route_select { new:true, id } を送ること**を
// 確かめる。過去にこの UI で「ボタンが在るのに押しても無反応」（onClick 空）を
// 出したため、trigger が本当に送られることを検証する（S11 の ui.stop と同じ狙い）。
//
// th_state は無いのでロード時に TEACH_MANUAL/ROUTE_SEL を直接注入し、記録開始の
// 後の状態遷移（REC 表示切替）は扱わない -- 検証するのは「押すと trigger が
// window.__thTriggerCalls に記録される」ことだけ（useTrigger のテストフック）。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('S-13 記録開始 sends ui.route_select with new:true and a generated id', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL' })
  await page.locator('#s13').waitFor()

  // 走行タブが初期表示 → 教示タブへ切替。
  await page.getByRole('tab', { name: '教示' }).click()

  // 経路名を入力して「記録開始」。
  await page.locator('[data-testid="s13-route-name"]').fill('デモ経路')
  await page.locator('[data-testid="s13-record-start"]').click()

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  const sel = calls.find((c) => c.trigger === 'ui.route_select')
  expect(sel, '記録開始が ui.route_select を送っていない').toBeTruthy()
  // useTrigger の TEST_MODE は argJson を JSON 文字列ではなく生オブジェクトで記録する。
  const arg = sel.argJson
  expect(arg.new, '新規教示 (new:true) が送られていない').toBe(true)
  expect(arg.id, '経路 id が生成されていない').toBeTruthy()
  expect(arg.id, '経路 id は route_ で始まる').toMatch(/^route_/)
})
