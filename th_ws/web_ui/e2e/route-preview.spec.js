// WS-3 / demo-teach-replay: S-14 の経路プレビューが /route/status の target_index
// を実際に描画へ届けること（純-pursuit 目標点マーカー）。経路や status は e2e に
// th_state が無いので helpers の gotoScreenWithRoutePreview で
// window.__thTestRouteCatalog / __thTestRoutePreview / __thTestRouteStatus に seed する。
//
// RoutePreview は TEST_MODE で描いた内容を window.__thRoutePreviewDrawn に記録し、
// canvas の data-target-index にも反映する。target_index=-1（未走行）のときは
// マーカーを描かないため、ここでは「target_index=5 なら canvas が 5 を描く」ことと
// 「target_index=-1 なら -1（マーカーなし）」の両方を検証する。S-14 側が targetIndex を
// -1 に固定すると 1 本目が赤くなる（変異 3）。
import { test, expect } from '@playwright/test'
import { gotoScreenWithRoutePreview } from './helpers.js'

const ROUTES = [
  { id: 'route_r1', name: '経路A', length_m: 3.14, point_count: 12 },
]

// 8 点の経路（index 5 を目標点にする）。
const PREVIEW = Array.from({ length: 8 }, (_, i) => ({ x: i, y: Math.sin(i) }))

// 空経路では「経路がありません」を出し、進むボタンを無効にする。

test('S-14 preview draws the pure-pursuit targetIndex from /route/status', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 } },
  )
  await page.locator('#s14').waitFor()

  const canvas = page.locator('[data-testid="route-preview"]')
  await expect(canvas).toBeVisible()
  await expect(canvas).toHaveAttribute('data-target-index', '5')

  const drawn = await page.evaluate(() => window.__thRoutePreviewDrawn)
  expect(drawn.targetIndex, '描画した targetIndex が -1 でない').toBe(5)
})

test('S-14 preview omits the target marker when target_index is -1', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S14',
    { mode: 'REPLAY', state: 'ROUTE_SEL' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: -1 } },
  )
  await page.locator('#s14').waitFor()

  const canvas = page.locator('[data-testid="route-preview"]')
  await expect(canvas).toBeVisible()
  await expect(canvas).toHaveAttribute('data-target-index', '-1')
})

test('S-14 preview shows the empty placeholder when there is no preview', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S14',
    { mode: 'REPLAY', state: 'READY' },
    { routes: ROUTES, preview: [], status: { target_index: -1 } },
  )
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('route-preview-empty')).toBeVisible()
})