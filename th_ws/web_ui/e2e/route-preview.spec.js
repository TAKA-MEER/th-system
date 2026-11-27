// S-14 の主表示（OnsiteMap）の地図：経路・始点・終点・機体・スキャンと地図ラスタ。
// 経路や status は e2e に th_state が無いので helpers の
// gotoScreenWithRoutePreview / gotoScreenWithRouteRobot で
// window.__thTestRouteCatalog / __thTestRoutePreview / __thTestRouteStatus に
// seed する（useScan の __thTestScan も併せて seed する。helpers 参照）。
import { test, expect } from '@playwright/test'
import { gotoScreenWithRoutePreview, gotoScreenWithRouteRobot } from './helpers.js'

const ROUTES = [
  { id: 'route_r1', name: '経路A', length_m: 3.14, point_count: 12 },
]

// 8 点の経路。
const PREVIEW = Array.from({ length: 8 }, (_, i) => ({ x: i, y: Math.sin(i) }))

test('S-14 の地図に経路が描かれる（空なら描かれない）', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 } },
  )
  await page.locator('#s14').waitFor()

  const map = page.getByTestId('s14-map')
  await expect(map).toBeVisible()
  await expect(page.getByTestId('s14-map-path')).toBeVisible()
  // 経路の終点に行き先の輪が出る。
  await expect(page.getByTestId('s14-map-dest')).toBeVisible()
  // 始点の印と始点・終点の名前が出る。
  await expect(page.getByTestId('s14-map-route-start')).toBeVisible()
  await expect(page.getByTestId('s14-map-route-start-label')).toContainText('始点')
  await expect(page.getByTestId('s14-map-route-end-label')).toContainText('終点')
})

test('S-14 の地図に経路が無ければ経路・行き先の輪は描かれない', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S14',
    { mode: 'REPLAY', state: 'READY' },
    { routes: ROUTES, preview: [], status: { target_index: -1 } },
  )
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('s14-map')).toBeVisible()
  await expect(page.getByTestId('s14-map-path')).toHaveCount(0)
  await expect(page.getByTestId('s14-map-route-start')).toHaveCount(0)
})

// ---------------------------------------------------------------- 地図の道具 ----

test('S-14 の地図に地図の端の道具（入れない所・凡例）が出て、凡例は開いて閉じられる', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: {} },
  )
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('s14-map-tools')).toBeVisible()
  await expect(page.getByTestId('s14-map-costmap-toggle')).toBeVisible()
  await expect(page.getByTestId('s14-map-legend')).toHaveCount(0)
  await page.getByTestId('s14-map-legend-toggle').click()
  await expect(page.getByTestId('s14-map-legend')).toBeVisible()
  await page.getByTestId('s14-map-legend-close').click()
  await expect(page.getByTestId('s14-map-legend')).toHaveCount(0)
})

// ---------------------------------------------------------------- スキャン・機体 ----

const POSE = { x: 0, y: 0, yaw: 0 }
// 3〜5m の有限レンジの最小形 LaserScan（useScan の /scan_filtered 相当）。
const SCAN = {
  angle_min: 0,
  angle_increment: 0.1,
  ranges: [3, 4, 5, 0.5, 6],
}

test('S-14 の地図に LiDAR の点が描かれる（機体のまわりの点）', async ({ page }) => {
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 }, pose: POSE, scan: SCAN },
  )
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('s14-map')).toBeVisible()
  const scan = page.getByTestId('s14-map-scan')
  await expect(scan).toBeVisible()
  await expect(scan.locator('circle')).toHaveCount(5)
})

test('S-14 の地図に機体の印が出る（位置と向きの三角＋大きさの四角）', async ({ page }) => {
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 }, pose: POSE, scan: SCAN },
  )
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('s14-map-robot')).toBeVisible()
  await expect(page.getByTestId('s14-map-robot-footprint')).toBeVisible()
})

test('S-14 の地図は機体が動いても機体を描き続ける（位置の追従）', async ({ page }) => {
  const routePose = { x: 1, y: 0.5, yaw: 0, frame: 'map' }
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 }, pose: POSE, routePose, scan: SCAN },
  )
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('s14-map-robot')).toBeVisible()
  await page.evaluate(() => window.__thSetTestRoutePose({ x: 2, y: 1, yaw: 0.3, frame: 'map' }))
  await expect(page.getByTestId('s14-map-robot')).toBeVisible()
})

// ---------------------------------------------------------------- /map ライヤ ----

// 最小 OccupancyGrid: 4x4, res 0.5, origin (-1,-1) → world x/y in [-1,1]。
const ROUTE_MAP = {
  info: {
    resolution: 0.5,
    width: 4,
    height: 4,
    origin: { position: { x: -1, y: -1 }, orientation: { x: 0, y: 0, w: 1, z: 0 } },
  },
  data: [0, 100, -1, 0, 100, 0, 0, -1, -1, 0, 100, 0, 0, -1, 100, 0],
}

test('S-14 の地図に地図ラスタ（/route/map_view）が描かれる', async ({ page }) => {
  // 地図（[-1,1]）の中に収まる経路で測る（外側は OnsiteMap の表示範囲外）。
  const previewInMap = Array.from({ length: 8 }, (_, i) => ({ x: -0.8 + i * 0.2, y: Math.sin(i) * 0.3 }))
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: previewInMap, status: { target_index: 5 }, pose: POSE, scan: SCAN, map: ROUTE_MAP },
  )
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('s14-map-raster')).toBeVisible()
  // 経路・スキャンも従来どおり描かれる（レイヤは共存）。
  await expect(page.getByTestId('s14-map-path')).toBeVisible()
  await expect(page.getByTestId('s14-map-scan')).toBeVisible()
})

test('S-14 は地図が無くても経路とスキャンで動く（地図無しの運用）', async ({ page }) => {
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 5 }, pose: POSE, scan: SCAN },
  )
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('s14-map-raster')).toHaveCount(0)
  await expect(page.getByTestId('s14-map-path')).toBeVisible()
  await expect(page.getByTestId('s14-map-scan')).toBeVisible()
})

test('S-14 は map フレームの /route/robot_pose を機体の位置に使う', async ({ page }) => {
  // routePose を map 座標 (0.5, 0.5) に seed。正しければ機体の印が描かれる
  // （odom 固定だと経路・地図とずれて出ない）。
  const routePose = { x: 0.5, y: 0.5, yaw: 0, frame: 'map' }
  await gotoScreenWithRouteRobot(
    page, 'S14',
    { mode: 'REPLAY', state: 'RUN' },
    {
      routes: ROUTES, preview: PREVIEW, status: { target_index: 5 },
      pose: POSE, routePose, scan: SCAN, map: ROUTE_MAP,
    },
  )
  await page.locator('#s14').waitFor()

  await expect(page.getByTestId('s14-map-robot')).toBeVisible()
  await expect(page.getByTestId('s14-map-raster')).toBeVisible()
})
