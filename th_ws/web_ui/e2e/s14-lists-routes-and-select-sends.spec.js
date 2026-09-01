// P5 / demo-teach-replay: S-14 が /route/catalog（RouteList.routes）を一覧し、
// 経路を選択して「この経路で進む」が実際に ui.route_select {id, reverse:false} を
// 送ること。catalog は e2e に th_state が無いので helpers の
// gotoScreenWithRouteCatalog で window.__thTestRouteCatalog に seed する。
import { test, expect } from '@playwright/test'
import { gotoScreenWithRouteCatalog } from './helpers.js'

const ROUTES = [
  { id: 'route_r1', name: '経路A', length_m: 3.14, point_count: 12, start_yaw: 0.1, recorded_at: 0 },
  { id: 'route_r2', name: '経路B', length_m: 5.55, point_count: 21, start_yaw: 0.2, recorded_at: 0 },
]

test('S-14 lists routes and selecting + proceed sends ui.route_select', async ({ page }) => {
  await gotoScreenWithRouteCatalog(page, 'S14', { mode: 'REPLAY', state: 'ROUTE_SEL' }, ROUTES)
  await page.locator('#s14').waitFor()

  // 2 行表示。
  await expect(page.getByTestId('s14-route-row')).toHaveCount(2)

  // 1 行選択 → 「この経路で進む」。
  await page.getByTestId('s14-select-route_r2').click()
  await page.getByTestId('s14-proceed').click()

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  const sel = calls.find((c) => c.trigger === 'ui.route_select')
  expect(sel, 'この経路で進むが ui.route_select を送っていない').toBeTruthy()
  // useTrigger の TEST_MODE は argJson を JSON 文字列ではなく生オブジェクトで記録する。
  const arg = sel.argJson
  expect(arg.id, '選択した経路 id が送られていない').toBe('route_r2')
  expect(arg.reverse, '順再生 (reverse:false) が送られていない').toBe(false)
})

test('S-14 shows empty message and disables proceed when no routes', async ({ page }) => {
  await gotoScreenWithRouteCatalog(page, 'S14', { mode: 'REPLAY', state: 'ROUTE_SEL' }, [])
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('s14-empty')).toBeVisible()
  await expect(page.getByTestId('s14-proceed')).toBeDisabled()
})
