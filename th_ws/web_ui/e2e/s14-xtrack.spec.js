// S-14 経路からのずれ（Spec-webui.md §3.7）: 走行中・一時停止中に
// /route/status の cross_track_m / cross_track_max_m が小数 2 桁で出ること。
// 未受信なら '--'、走行前（READY）ならカード自体が出ないこと。
import { test, expect } from '@playwright/test'
import { gotoScreenWithRoutePreview } from './helpers.js'

const ROUTES = [{ id: 'r1', name: '経路1', length_m: 5, point_count: 10 }]

test('S-14 shows cross-track now and max while RUN', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    {
      routes: ROUTES, preview: [], status: { cross_track_m: 0.1234, cross_track_max_m: 0.4567 },
    })
  await page.locator('#s14').waitFor()
  await expect(page.locator('[data-testid="s14-xtrack"]'))
    .toHaveText('いま 0.12 m・最大 0.46 m')
})

test('S-14 keeps showing max while PAUSE', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'PAUSE' },
    {
      routes: ROUTES, preview: [], status: { cross_track_m: 0, cross_track_max_m: 0.45 },
    })
  await page.locator('#s14').waitFor()
  await expect(page.locator('[data-testid="s14-xtrack"]'))
    .toHaveText('いま 0.00 m・最大 0.45 m')
})

test('S-14 shows placeholder when status has no cross-track yet', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: [], status: {} })
  await page.locator('#s14').waitFor()
  await expect(page.locator('[data-testid="s14-xtrack"]')).toHaveText('--')
})

test('S-14 hides cross-track card before RUN (READY)', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'READY' },
    {
      routes: ROUTES, preview: [], status: { cross_track_m: 0, cross_track_max_m: 0 },
    })
  await page.locator('#s14').waitFor()
  await expect(page.locator('[data-testid="s14-xtrack"]')).toHaveCount(0)
})
