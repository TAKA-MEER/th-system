// 1b-11（SG-B13・SG-B14・SG-B19・SG-C6・SG-C9 の S-14 分）の e2e。
//  - S-01: 教示再生は経路 0 本で理由付きで押せない／電子リードはデバイス未接続で押せない
//  - S-14: 地図更新の切り替えは出さない（使えない間。§3.3 (c)）／「保存」も出さない
//  - S-14: 始点・終点の印（OnsiteMap の経路の印）
import { test, expect } from '@playwright/test'
import {
  gotoScreen, gotoScreenWithRouteCatalog, gotoScreenWithRoutePreview, setTestState,
} from './helpers.js'

const ROUTES = [{ id: 'r1', name: '経路1', length_m: 5, point_count: 10 }]
const PREVIEW = [
  { x: 0, y: 0, yaw: 0 }, { x: 1, y: 0, yaw: 0 }, { x: 2, y: 0.5, yaw: 0 },
]

test('S-01: 経路 0 本だと教示再生は dis 表示で、押すと no_route_recorded の理由が出る', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true, leash_present: true })
  const replay = page.getByRole('button', { name: '教示再生' })
  await expect(replay).toHaveClass(/\bdis\b/)
  await replay.click()
  await expect(page.locator('.win.confirm.show')).toBeVisible()
  await expect(page.locator('.win.confirm.show')).toContainText('教示済みの経路がありません')
})

test('S-01: 経路があれば教示再生は押せる', async ({ page }) => {
  await gotoScreenWithRouteCatalog(page, 'S01',
    { mode: 'IDLE', tracker_enabled: true, leash_present: true }, ROUTES)
  const replay = page.getByRole('button', { name: '教示再生' })
  await expect(replay).not.toHaveClass(/\bdis\b/)
})

test('S-01: リードデバイス未接続だと電子リードは dis 表示', async ({ page }) => {
  await gotoScreenWithRouteCatalog(page, 'S01',
    { mode: 'IDLE', tracker_enabled: true, leash_present: false }, ROUTES)
  await expect(page.getByRole('button', { name: '電子リード' })).toHaveClass(/\bdis\b/)
  await setTestState(page, { leash_present: true })
  await expect(page.getByRole('button', { name: '電子リード' })).not.toHaveClass(/\bdis\b/)
})

test('S-01: 教示は保管場所から開始する案内が出る', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await expect(page.locator('body')).toContainText('保管場所から開始してください')
})

test('S-14: 地図の更新の切り替えは出さず、「保存」も出ない（使えない間。§3.3 (c)）', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'PAUSE' },
    { routes: ROUTES, preview: PREVIEW, status: {} })
  await page.locator('#s14').waitFor()
  await expect(page.getByTestId('s14-map-update-toggle')).toHaveCount(0)
  await expect(page.locator('#s14')).not.toContainText('地図の書き足しは現在使えません')
  await expect(page.locator('.opsgrid .op-save')).toHaveCount(0)
})

test('S-14: 経路の始点・終点の印を描く（OnsiteMap の経路の印）', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'READY' },
    { routes: ROUTES, preview: PREVIEW, status: {} })
  await page.locator('#s14').waitFor()
  await expect(page.getByTestId('s14-map-route-start')).toBeVisible()
  await expect(page.getByTestId('s14-map-route-start-label')).toContainText('始点')
  await expect(page.getByTestId('s14-map-route-end-label')).toContainText('終点')
  await expect(page.getByTestId('s14-map-dest')).toBeVisible()
})

test('S-14: 探索中の確度の印は「探しています」で、原因を断定する待ち時間は出さない', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'LOCALIZE' },
    { routes: ROUTES, preview: PREVIEW, status: { localize_quality: 'searching' } })
  await page.locator('#s14').waitFor()
  await expect(page.getByTestId('s14-localize-quality')).toContainText('探しています')
  await expect(page.getByTestId('s14-localize-wait')).toHaveCount(0)

  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'LOCALIZE' },
    { routes: ROUTES, preview: PREVIEW, status: { localize_quality: 'failed' } })
  await page.locator('#s14').waitFor()
  await expect(page.getByTestId('s14-localize-quality')).toContainText('見つかりません')
  await expect(page.getByTestId('s14-localize-wait')).toHaveCount(0)
})
