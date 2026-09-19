// P5 / demo-teach-replay: S-13 の 走行/教示 タブがモックアップと同じ「タブ」の
// 見た目になること。theme.css へ .tabs/.tab/.tab.on を移植する前は、切替は
// 動くのに CSS が無く、ただのボタン列に見えていた（移植漏れ）。
// ここでは「選択中タブに下線（border-bottom）の色がつく」ことだけを固定する。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('S-13 tabs render as tabs: the active tab gets a coloured underline', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL' })
  await page.locator('#s13').waitFor()

  const drive = page.getByRole('tab', { name: '走行' })
  const teach = page.getByRole('tab', { name: '教示' })

  const underline = (loc) =>
    loc.evaluate((el) => getComputedStyle(el).borderBottomColor)
  const isColoured = (c) =>
    c && c !== 'transparent' && !/rgba\(\s*0,\s*0,\s*0,\s*0\s*\)/.test(c)

  // 初期表示は 走行 タブ。
  expect(isColoured(await underline(drive)), '選択中タブに下線の色がついていない').toBe(true)
  expect(isColoured(await underline(teach)), '非選択タブに下線の色がついている').toBe(false)

  await teach.click()
  expect(isColoured(await underline(teach)), '切替後の選択タブに下線の色がついていない').toBe(true)
  expect(isColoured(await underline(drive)), '切替後の非選択タブに下線の色がついている').toBe(false)
})
