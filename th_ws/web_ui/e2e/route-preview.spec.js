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
import { gotoScreenWithRoutePreview, gotoScreenWithRouteRobot } from './helpers.js'

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

// ---------------------------------------------------------------- WS-6.4: robot-centred view ----

const POSE = { x: 0, y: 0, yaw: 0 }
// 3〜5m の有限レンジの最小形 LaserScan（7.5m 以内なので描かれる）。
const SCAN = {
  angle_min: 0,
  angle_increment: 0.1,
  ranges: [3, 4, 5, 0.5, 6],
}

test('S-14 scans on-canvas: the robot-centred view draws scan points (#5)', async ({ page }) => {
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 }, pose: POSE, scan: SCAN },
  )
  await page.locator('#s14').waitFor()

  const canvas = page.locator('[data-testid="route-preview"]')
  await expect(canvas).toBeVisible()

  await expect.poll(async () => page.evaluate(() => window.__thRoutePreviewScanDrawn))
    .toBeGreaterThan(0)
})

test('S-14 does not rescale when the robot moves (fixed zoom, no flicker #4)', async ({ page }) => {
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 }, pose: POSE, scan: SCAN },
  )
  await page.locator('#s14').waitFor()

  // Capture the fixed scale at pose A, then move the robot 1m and confirm the
  // scale is unchanged (the centred transform must not re-fit to route data).
  const s1 = await page.evaluate(() => window.__thRoutePreviewFit?.scale)
  expect(s1).toBeGreaterThan(0)

  await page.evaluate(() => window.__thSetTestOdomPose({ x: 1, y: -0.5, yaw: 0.3 }))
  await expect.poll(async () => page.evaluate(() => window.__thRoutePreviewFit?.scale))
    .toBe(s1)
})

// ---------------------------------------------------------------- WS-8A: range_max clip ----

test('S-14 draws scan points far past the old 7.5m clip (range_max honoured)', async ({ page }) => {
  // Open-venue walls at 8–12m used to be dropped by the hard 7.5m clip; the
  // clip now honours scanData.range_max, so far points are drawn.
  const farScan = {
    angle_min: 0,
    angle_increment: 0.1,
    ranges: [12, 8, 9], // all > 7.5 but <= range_max
    range_max: 15,
  }
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 }, pose: POSE, scan: farScan },
  )
  await page.locator('#s14').waitFor()

  // All three far returns must survive the clip.
  await expect.poll(async () => page.evaluate(() => window.__thRoutePreviewScanDrawn)).toBe(3)
})

test('S-14 drops points beyond scanData.range_max but keeps nearer ones', async ({ page }) => {
  // r=1 stays, r=20 (beyond range_max 15) is dropped -> exactly 1 drawn.
  const mixed = {
    angle_min: 0,
    angle_increment: 0.1,
    ranges: [1, 20],
    range_max: 15,
  }
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 }, pose: POSE, scan: mixed },
  )
  await page.locator('#s14').waitFor()

  await expect.poll(async () => page.evaluate(() => window.__thRoutePreviewScanDrawn)).toBe(1)
})