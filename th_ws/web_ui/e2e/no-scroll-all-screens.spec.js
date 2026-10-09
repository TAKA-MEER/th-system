// e2e/no-scroll-all-screens.spec.js — 全画面・全タブが「スクロール無し」で収まること。
//
// 2026-10-10: 横長（論理 1280x720）で S-01 / S-13 / S-14 / S-21 / S-50 などが本文の高さを
// 超えて、画面内でスクロールしないと操作できなかった。論理キャンバスは端末に依らず
// 横長 1280x720 / 縦長 960x1280 の 2 つだけ（shell/stageMetrics.js）なので、この 2 つの
// 大きさで測れば全端末分を縛れる（実寸は拡大されるだけで折り返しは変わらない）。
//
// 縛るもの: #body とその中のスクロール領域（タブの中身 .tabpane など）が溢れないこと。
// 縛らないもの: 件数が決まっていない一覧（.lst-scroll ＝経路・ピンの一覧）の枠内スクロール。
// 各画面の「既定の状態」（ピン 6 件・経路 7 件・人検出の候補 4 件など現場で出る程度の
// データを入れた状態）で測る。データを増やして溢れるのは一覧の枠内に限る、という約束。
import { test, expect } from '@playwright/test'

const pin = (id, name, kind, x, y, yawDeg = 90) => {
  const r = (yawDeg * Math.PI) / 180
  return {
    id, name, kind, registered_at: '',
    pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(r / 2), w: Math.cos(r / 2) } },
  }
}
const PINS = [
  pin('p1', '盤 A-1', 'PANEL', 86, 88, 92), pin('p2', '盤 A-2', 'PANEL', 176, 88, 91),
  pin('p3', '盤 B-1', 'PANEL', 100, 40), pin('p4', '盤 B-2', 'PANEL', 120, 40),
  pin('p5', '盤 C-1', 'PANEL', 140, 40), pin('p6', '待機場所', 'HOME', 70, 190, 274),
]
const ONSITE = {
  __thTestOnsitePins: PINS,
  __thTestPersonTargets: {
    candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }, { x: 2, y: 1 }, { x: -2, y: 1 }],
    selected_index: 0, confidence: 0.87, is_lost: false, lost_reason: '',
  },
  __thTestOdomPose: { x: 0, y: 0, yaw: 0 },
  __thTestHomeDeclared: true,
}
const ROUTES = Array.from({ length: 3 }, (_, i) => ({
  id: `route_r${i}`, name: `経路${'ABC'[i]}`, length_m: 3.14 + i, point_count: 12 + i, start_yaw: 0.1, recorded_at: 0,
}))
const PREVIEW = Array.from({ length: 40 }, (_, i) => ({ x: i * 0.2, y: Math.sin(i / 5) }))
const DETAIL = {
  commanded: 1.0, current: { wheel_radius_scale: 1.0 }, progress: 0.4, warnings: ['linear_not_calibrated'],
  last_calibrated: { LINEAR: '2026-10-01T10:00:00', ROTATION: '', IMU: '' },
  history: { LINEAR: [{ generation: 1, values: { wheel_radius_scale: 0.98 }, calibrated_at: '2026-09-30T09:00:00' }], ROTATION: [], IMU: [] },
}
const calib = (o = {}) => ({
  item: 'LINEAR', step: 'S3', result: 'WAIT_MEASURED', preview_before: '{"a":1}', preview_after: '{"a":2}', detail: DETAIL, ...o,
})
const SCAN = {
  angle_min: -3.14, angle_increment: 0.0175, range_min: 0.1, range_max: 12,
  ranges: Array.from({ length: 360 }, (_, i) => 2 + Math.sin(i / 20)),
}

const CASES = [
  ['S-00', 'S00', { mode: 'INIT' }, {}],
  ['S-01', 'S01', { mode: 'IDLE', tracker_enabled: true, unsaved: ['venue_map', 'route', 'calib', 'map_patch'] }, {}],
  ['S-11', 'S11', { mode: 'MANUAL', state: 'PAUSE', tracker_enabled: true }, {}],
  ['S-13 選択', 'S13', { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL', tracker_enabled: true }, { __thTestRouteCatalog: ROUTES }],
  ['S-13 記録', 'S13', { mode: 'TEACH_MANUAL', state: 'RECORD', tracker_enabled: true }, { __thTestRoutePreview: PREVIEW }],
  ['S-14 経路選択', 'S14', { mode: 'REPLAY', state: 'ROUTE_SEL', tracker_enabled: true }, { __thTestRouteCatalog: ROUTES, __thTestRoutePreview: PREVIEW }],
  ['S-14 再生中', 'S14', { mode: 'REPLAY', state: 'RUN', tracker_enabled: true },
    { __thTestRouteCatalog: ROUTES, __thTestRoutePreview: PREVIEW, __thTestRouteStatus: { target_index: 5, cross_track_m: 0.1, cross_track_max_m: 0.3 } }],
  ['S-20 登録', 'S20', { mode: 'PREP', state: 'REGISTER', tracker_enabled: true }, ONSITE],
  ['S-20 地図', 'S20', { mode: 'PREP', state: 'MAPPING', tracker_enabled: true }, ONSITE],
  ['S-21 待機', 'S21', { mode: 'AT_HOME', tracker_enabled: true }, ONSITE],
  ['S-21 移動', 'S21', { mode: 'PANEL_NAV', state: 'MOVING', tracker_enabled: true }, ONSITE],
  ['S-21 ピン無し', 'S21', { mode: 'AT_HOME', tracker_enabled: true }, {}],
  ['S-30 一覧', 'S30', { mode: 'OPCHECK', state: 'LIST' }, {}],
  ['S-30 モーター', 'S30', { mode: 'OPCHECK', state: 'RUNNING_CHECK' },
    { __thTestOpcheckStatus: { item: 'MOTOR', result: 'UNKNOWN', detail: 'done= hold=NONE', next_screen: '' } }],
  ['S-30 LiDAR', 'S30', { mode: 'OPCHECK', state: 'RUNNING_CHECK' },
    { __thTestOpcheckStatus: { item: 'LIDAR', result: 'UNKNOWN', detail: '', next_screen: '' }, __thTestScan: SCAN }],
  ['S-31', 'S31', { mode: 'OPCHECK', state: 'REPAIR' }, {}],
  ['S-40 一覧', 'S40', { mode: 'CALIB', state: 'LIST' }, { __thTestCalibStatus: calib({ result: 'IDLE', item: '' }) }],
  ['S-40 実測', 'S40', { mode: 'CALIB', state: 'S3' }, { __thTestCalibStatus: calib() }],
  ['S-40 死角', 'S40', { mode: 'CALIB', state: 'S3' },
    { __thTestCalibStatus: calib({ item: 'BLIND' }), __thTestScan: SCAN }],
]

const SIZES = [
  { name: '横長', width: 1280, height: 720 },
  { name: '縦長', width: 960, height: 1280 },
]

// #body 内で、中身が枠を超えている縦スクロール領域（一覧 .lst-scroll は除く）を返す。
async function overflowing(page) {
  return page.evaluate(() => {
    const out = []
    for (const e of [document.querySelector('#body'), ...document.querySelectorAll('#body *')]) {
      if (!e || e.classList.contains('lst-scroll')) continue
      const oy = getComputedStyle(e).overflowY
      if ((oy === 'auto' || oy === 'scroll') && e.scrollHeight > e.clientHeight + 1) {
        out.push(`${e.tagName.toLowerCase()}.${String(e.className)} +${e.scrollHeight - e.clientHeight}px`)
      }
    }
    return out
  })
}

// 画面の中の role=tab を 1 つずつ押して、そのたびに測る（タブごとに中身が違うため）。
async function checkAllTabs(page, where) {
  expect(await overflowing(page), `${where} (top)`).toEqual([])
  const labels = (await page.locator('#body [role=tab]').allTextContents()).map((t) => t.trim())
  for (const label of new Set(labels)) {
    await page.locator('#body [role=tab]', { hasText: label }).first().click()
    expect(await overflowing(page), `${where} > ${label}`).toEqual([])
  }
}

for (const size of SIZES) {
  for (const [name, screen, state, seeds] of CASES) {
    test(`${name} は ${size.name} (${size.width}x${size.height}) でスクロール無しに収まる`, async ({ page }) => {
      await page.setViewportSize({ width: size.width, height: size.height })
      await page.addInitScript(({ s, scr, seeds: sd }) => {
        window.__thTestState = s
        window.__thTestScreen = scr
        for (const k of Object.keys(sd)) window[k] = sd[k]
      }, { s: state, scr: screen, seeds })
      await page.goto('/')
      await page.locator('#body').waitFor()
      await checkAllTabs(page, `${name} ${size.name}`)
    })
  }

  test(`S-50 設定の全タブ・全区画は ${size.name} でスクロール無しに収まる`, async ({ page }) => {
    await page.setViewportSize({ width: size.width, height: size.height })
    await page.addInitScript(() => {
      window.__thTestState = { mode: 'IDLE', tracker_enabled: true }
      window.__thTestScreen = 'S01'
    })
    await page.goto('/')
    await page.getByTestId('s01-open-settings').click()
    await expect(page.locator('#s50')).toBeVisible()
    for (const sec of ['follow', 'lidar', 'slam', 'preset']) {
      await page.getByTestId(`s50-sec-${sec}`).click()
      expect(await overflowing(page), `S-50 一般 > ${sec} ${size.name}`).toEqual([])
    }
    for (const tab of ['display', 'dev']) {
      await page.getByTestId(`s50-tab-${tab}`).click()
      expect(await overflowing(page), `S-50 ${tab} ${size.name}`).toEqual([])
    }
  })
}
