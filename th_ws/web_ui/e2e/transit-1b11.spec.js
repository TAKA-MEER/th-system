// 1b-11（SG-B13・SG-B14・SG-B19・SG-C6・SG-C9 の S-14 分）の e2e。
//  - S-01: 教示再生は経路 0 本で理由付きで押せない／電子リードはデバイス未接続で押せない
//  - S-14: 地図更新トグル（既定 OFF）→ 送信、ON のときだけ操作カードに「保存」
//  - S-14: 始点・終点の印／後方の死角の表示／待ち時間の表示
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

test('S-14: 地図の更新は既定 OFF で「保存」が無く、押すと map_update を送り、ON で「保存」が出る', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'PAUSE' },
    { routes: ROUTES, preview: PREVIEW, status: {} })
  await page.locator('#s14').waitFor()
  const toggle = page.getByTestId('s14-map-update-toggle')
  await expect(toggle).toHaveAttribute('aria-pressed', 'false')
  await expect(page.locator('.opsgrid .op-save')).toHaveCount(0)

  await toggle.click()
  const flags = await page.evaluate(() => window.__thSetFlagCalls ?? [])
  expect(flags.some((c) => c.flag === 'map_update' && c.value === true),
    'トグルが /system/set_flag map_update を送っていない').toBe(true)

  // 機体が受理して state.map_update が真になった想定
  await setTestState(page, { map_update: true })
  await expect(toggle).toHaveAttribute('aria-pressed', 'true')
  const save = page.locator('.opsgrid .op-save')
  await expect(save).toHaveCount(1)
  await save.click()
  const sent = (await page.evaluate(() => window.__thTriggerCalls ?? [])).map((c) => c.trigger)
  expect(sent, '保存ボタンが ui.save を送っていない').toContain('ui.save')

  await setTestState(page, { map_update: false })
  await expect(page.locator('.opsgrid .op-save')).toHaveCount(0)
})

test('S-14: 地図更新が機体に拒否されたら理由が出て、表示は変わらない', async ({ page }) => {
  // addInitScript の引数に関数は渡せないためスクリプト内に直接書く（goto の前）。
  await page.addInitScript(() => {
    window.__thTestSetFlag = {
      map_update: () => ({ accepted: false, reject_reason_key: 'not_allowed' }),
    }
  })
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'PAUSE' },
    { routes: ROUTES, preview: PREVIEW, status: {} })
  await page.locator('#s14').waitFor()

  await page.getByTestId('s14-map-update-toggle').click()
  await expect(page.getByTestId('s14-map-update-error')).toContainText('今の状態ではこの操作はできません')
  await expect(page.getByTestId('s14-map-update-toggle')).toHaveAttribute('aria-pressed', 'false')
})

test('S-14: 経路の始点・終点の印を描く', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'READY' },
    { routes: ROUTES, preview: PREVIEW, status: {} })
  await page.locator('#s14').waitFor()
  await expect.poll(async () => page.evaluate(() => window.__thRoutePreviewEndpointsDrawn))
    .toEqual({ start: true, end: true })
})

test('S-14: 後方は死角の表示が出る', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'READY' },
    { routes: ROUTES, preview: PREVIEW, status: {} })
  await page.locator('#s14').waitFor()
  await expect(page.locator('#s14')).toContainText('後方は LiDAR の死角')
})

test('S-14: 探索中は待ち時間を原因を断定しない文言で出す・failed では汎用の置き直し案内を出さない', async ({ page }) => {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'LOCALIZE' },
    { routes: ROUTES, preview: PREVIEW, status: { localize_quality: 'searching' } })
  await page.locator('#s14').waitFor()
  const wait = page.getByTestId('s14-localize-wait')
  await expect(wait).toContainText('30〜45秒')
  await expect(wait).not.toContainText('セッション')

  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'LOCALIZE' },
    { routes: ROUTES, preview: PREVIEW, status: { localize_quality: 'failed' } })
  await page.locator('#s14').waitFor()
  await expect(page.getByTestId('s14-localize-wait')).toHaveCount(0)
})
