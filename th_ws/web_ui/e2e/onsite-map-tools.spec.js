// W0b: 地図の端の道具（「入れない所」「凡例」）と BLOCKED の黄の円の e2e。
// 画面の中身の作り直しは次の段なので、地図の部品だけを S-21 のいまの画面で確かめる
// （S-20 の回すボタンは試験準備だけ。回転の座標変換は unit で縛る）。
import { test, expect } from '@playwright/test'
import {
  gotoScreenWithOnsite, unlockOnsiteVenueMap, setTestState, unlockOnsiteMap,
} from './helpers.js'

function pin(id, name, kind, x, y, yawDeg = 90) {
  const rad = (yawDeg * Math.PI) / 180
  return {
    id, name, kind, registered_at: '',
    pose: {
      position: { x, y, z: 0 },
      orientation: { x: 0, y: 0, z: Math.sin(rad / 2), w: Math.cos(rad / 2) },
    },
  }
}

const PINS = [
  pin('p1', '盤 A-1', 'PANEL', 86, 88, 92),
  pin('p2', '盤 A-2', 'PANEL', 176, 88, 91),
  pin('p3', '待機場所', 'HOME', 70, 190, 274),
]

const TARGETS = {
  candidates: [{ x: 1.2, y: 0.4 }],
  selected_index: 0,
  confidence: 0.87,
  is_lost: false,
  lost_reason: '',
}

const ROUTE_MAP = {
  info: {
    resolution: 0.5,
    width: 4,
    height: 4,
    origin: { position: { x: -1, y: -1 }, orientation: { x: 0, y: 0, w: 1, z: 0 } },
  },
  data: [0, 100, -1, 0, 100, 0, 0, -1, -1, 0, 100, 0, 0, -1, 100, 0],
}

// コストマップ（Nav2 グローバル）。99（内接）・100（致死）のセルがある。
const COSTMAP = {
  info: {
    resolution: 0.5,
    width: 4,
    height: 4,
    origin: { position: { x: -1, y: -1 }, orientation: { x: 0, y: 0, w: 1, z: 0 } },
  },
  data: [99, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
}

const PATH = [{ x: -0.5, y: -0.5 }, { x: 0.5, y: 0.5 }, { x: 1, y: 1 }]

async function goto21(page, state = { mode: 'IDLE', state: 'NONE' }, extra = {}) {
  await gotoScreenWithOnsite(page, 'S21', state, {
    pins: PINS,
    targets: TARGETS,
    pose: { x: 0, y: 0, yaw: 0 },
    routeMap: ROUTE_MAP,
    openVenueMap: { success: true, message: '' },
    costmap: COSTMAP,
    plannedPath: PATH,
    ...extra,
  })
  await unlockOnsiteVenueMap(page)
}

test('地図の道具: 「入れない所」でコストマップの描画が消える', async ({ page }) => {
  await goto21(page)
  const cm = page.locator('[data-testid="s21-map-costmap"]')
  await expect(cm).toBeVisible()
  const btn = page.locator('[data-testid="s21-map-costmap-toggle"]')
  await expect(btn).toHaveAttribute('aria-label', '入れない所')
  await expect(btn).toHaveAttribute('aria-pressed', 'true')
  await btn.click()
  await expect(cm).toHaveCount(0)
  await expect(btn).toHaveAttribute('aria-pressed', 'false')
  // localStorage に残る（全地図で共通）
  expect(await page.evaluate(() => window.localStorage.getItem('th.costmapVisible'))).toBe('0')
  await btn.click()
  await expect(cm).toBeVisible()
})

test('地図の道具: 「凡例」が開閉する', async ({ page }) => {
  await goto21(page)
  const toggle = page.locator('[data-testid="s21-map-legend-toggle"]')
  const legend = page.locator('[data-testid="s21-map-legend"]')
  await expect(legend).toHaveCount(0)
  await toggle.click()
  await expect(legend).toBeVisible()
  await expect(toggle).toHaveAttribute('aria-expanded', 'true')
  // 11 項目（2 列）。地図の見方が legend で示される。
  await expect(legend.locator('li')).toHaveCount(11)
  for (const word of ['通れる床', '入れない所', '壁', 'まだ見ていない所', '経路', '機体', '待機場所', '配電盤']) {
    await expect(legend).toContainText(word)
  }
  await page.locator('[data-testid="s21-map-legend-close"]').click()
  await expect(legend).toHaveCount(0)
  await toggle.click()
  await expect(legend).toBeVisible()
  await toggle.click()
  await expect(legend).toHaveCount(0)
})

test('BLOCKED の間だけ、塞いでいる所に注意の黄の円が出る', async ({ page }) => {
  // 経路の先の最初の通れない点＝コストマップの 99 セル（world -1+0*0.5 付近）に
  // 経路が乗るよう、PATH の先端が 99 セルの近傍を通る。
  const blockedPath = [{ x: -0.75, y: -0.75 }, { x: 0.5, y: 0.5 }]
  // 先に地図を開いてから BLOCKED に遷移する（BLOCKED 中は地図の門が閉じるため）。
  await goto21(page, { mode: 'IDLE', state: 'NONE' }, { plannedPath: blockedPath })
  await expect(page.locator('[data-testid="s21-map-blocked"]')).toHaveCount(0)
  await setTestState(page, { mode: 'PANEL_NAV', state: 'BLOCKED' })
  const circle = page.locator('[data-testid="s21-map-blocked"]')
  await expect(circle).toBeVisible()
  // 注意の黄の円（点線の縁）であること
  await expect(circle).toHaveAttribute('stroke-dasharray', /./)
  await expect(circle).toHaveAttribute('fill', '#ffd100')
  await setTestState(page, { mode: 'PANEL_NAV', state: 'NAV' })
  await expect(circle).toHaveCount(0)
})

test('試験準備（S-20）: 「左へ回す」「右へ回す」で地図の向きが 15° ずつ変わる', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', { mode: 'PREP', state: 'MAPPING', tracker_enabled: true },
    {
      pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 },
      routeMap: ROUTE_MAP, mappingActive: true,
    },
  )
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)
  const left = page.locator('[data-testid="s20-map-rotate-left"]')
  const right = page.locator('[data-testid="s20-map-rotate-right"]')
  await expect(left).toHaveAttribute('aria-label', '左へ回す')
  await expect(right).toHaveAttribute('aria-label', '右へ回す')
  const content = page.locator('[data-testid="s20-map"] > g')
  const tf = () => content.evaluate((el) => el.getAttribute('transform') ?? '')
  await expect.poll(tf).toBe('')
  await right.click()
  await expect.poll(tf).toContain('rotate(15 ')
  await left.click()
  await expect.poll(tf).toBe('')
  await left.click()
  await expect.poll(tf).toContain('rotate(345 ')
  // 試験（S-21）には回すボタンが出ない（地図の向きを変えられるのは試験準備だけ）。
  await goto21(page)
  await expect(page.locator('[data-testid="s21-map-rotate-left"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s21-map-rotate-right"]')).toHaveCount(0)
})
