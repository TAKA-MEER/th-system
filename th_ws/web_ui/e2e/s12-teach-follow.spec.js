// S-12 教示（追従）（型「走らせる」）。
// FSM: ROUTE_SEL --ui.route_select--> REC、REC --ui.stop--> PAUSE
//   PAUSE --ui.run--> REC、REC/PAUSE --ui.save--> SAVED
import { test, expect } from '@playwright/test'

const TARGETS = {
  candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
  selected_index: 0, confidence: 0.87, is_lost: false, lost_reason: '',
}
const ROUTES = [
  { id: 'route_a', name: '廊下の往復', length_m: 12.3, point_count: 40, start_yaw: 0, recorded_at: 0 },
]

async function openS12(page, state, { routes = ROUTES, status } = {}) {
  await page.addInitScript(({ s, r, st }) => {
    window.__thTestState = s
    window.__thTestScreen = 'S12'
    window.__thTestPersonTargets = {
      candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
      selected_index: 0, confidence: 0.87, is_lost: false, lost_reason: '',
    }
    window.__thTestRouteCatalog = r
    if (st) window.__thTestRouteStatus = st
  }, { s: state, r: routes, st: status ?? null })
  await page.goto('/')
  await page.locator('#s12').waitFor()
}

async function triggers(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

test('S-12: ROUTE_SEL では入口のカードが出て新規で記録を始める', async ({ page }) => {
  await openS12(page, { mode: 'TEACH_FOLLOW', state: 'ROUTE_SEL', tracker_enabled: true })
  await expect(page.getByTestId('s12-route-name')).toBeVisible()
  await expect(page.getByTestId('s12-route-row')).toHaveCount(1)
  await page.locator('[data-testid="s12-new-start"] button').click()
  const calls = await triggers(page)
  expect(calls.map((c) => c.trigger), '新規開始が ui.route_select を送っていない').toContain('ui.route_select')
})

test('S-12: REC で停止、PAUSE で再開、REC/PAUSE で保存を送る', async ({ page }) => {
  await openS12(page, { mode: 'TEACH_FOLLOW', state: 'REC', tracker_enabled: true })
  await page.locator('[data-testid="s12-stop-run-stop"]').click()
  let calls = await triggers(page)
  expect(calls.map((c) => c.trigger), 'REC で停止が ui.stop を送っていない').toContain('ui.stop')

  await page.evaluate(() => window.__thSetTestState({ mode: 'TEACH_FOLLOW', state: 'PAUSE' }))
  await page.locator('[data-testid="s12-stop-run-run"]').click()
  calls = await triggers(page)
  expect(calls.map((c) => c.trigger), 'PAUSE で再開が ui.run を送っていない').toContain('ui.run')

  await page.locator('[data-testid="s12-save"]').click()
  calls = await triggers(page)
  expect(calls.map((c) => c.trigger), '保存が ui.save を送っていない').toContain('ui.save')
})

test('S-12: 記録の量が出る', async ({ page }) => {
  await openS12(page, { mode: 'TEACH_FOLLOW', state: 'REC', tracker_enabled: true }, {
    status: { recorded_m: 12.4, elapsed_sec: 92, points: 100, start_yaw: 0.5, saved: false },
  })
  // 距離と時間を 1 行で。点数・開始時の向きは出さない。
  await expect(page.getByTestId('s12-record-amount-value')).toHaveText('12.4 m　1:32')
  await expect(page.getByTestId('s12-record-amount')).not.toContainText('点')
})

test('S-12: 地図は小さく出て押すと大小を入れ替える', async ({ page }) => {
  await openS12(page, { mode: 'TEACH_FOLLOW', state: 'REC', tracker_enabled: true })
  await expect(page.getByTestId('s12-map-map')).toBeVisible()
  const m = page.locator('.s12-map')
  await expect(m).toHaveClass(/small/)
  await page.locator('[data-testid="s12-map-swap"]').click()
  await expect(m).toHaveClass(/large/)
})

test('S-12: レーダーのタップが ui.select_target を送る', async ({ page }) => {
  await openS12(page, { mode: 'TEACH_FOLLOW', state: 'REC', tracker_enabled: true })
  // 小さな地図がレーダーの左下を覆うので、そこの候補（1）は押せない。
  // 上の候補（0）を押して確認する。
  await page.locator('[data-testid="radar-cand-0"]').click()
  const calls = await triggers(page)
  expect(calls.map((c) => c.trigger), 'レーダーのタップが ui.select_target を送っていない').toContain('ui.select_target')
})

test('S-12: 状態を変えても右列の位置が動かない', async ({ page }) => {
  await openS12(page, { mode: 'TEACH_FOLLOW', state: 'ROUTE_SEL', tracker_enabled: true })
  const box = async () => page.locator('#s12 .g-drv-row').boundingBox()
  const before = await box()
  await page.evaluate(() => window.__thSetTestState({ mode: 'TEACH_FOLLOW', state: 'REC' }))
  const after = await box()
  expect(Math.abs(before.x - after.x), 'main の左が動いた').toBeLessThan(1)
  expect(Math.abs(before.width - after.width), 'main の幅が動いた').toBeLessThan(1)
  for (const pos of ['where', 'toggle', 'ops', 'jog']) {
    await expect(page.locator(`#s12 [data-pos="${pos}"]`)).toBeVisible()
  }
})

test('S-12: 終了が ui.finish を送る', async ({ page }) => {
  await openS12(page, { mode: 'TEACH_FOLLOW', state: 'REC', tracker_enabled: true })
  await page.locator('[data-testid="s12-finish"]').click()
  const calls = await triggers(page)
  expect(calls.map((c) => c.trigger), '終了が ui.finish を送っていない').toContain('ui.finish')
})

// ---- 差し戻し（見た目）: 小地図・向く│保存・レーダー ----
test('S-12: レーダーは区画いっぱい。小さい地図は左下の枠つき（約 300×200）で道具を出さない', async ({ page }) => {
  await openS12(page, { mode: 'TEACH_FOLLOW', state: 'REC', tracker_enabled: true })
  const main = await page.locator('[data-pos="main"]').boundingBox()
  const radar = await page.getByTestId('radar').boundingBox()
  expect(radar.width, 'レーダーが区画いっぱいでない').toBeGreaterThanOrEqual(main.width - 2)
  const mini = await page.locator('.s12-map.small').boundingBox()
  expect(mini.width).toBeGreaterThan(280)
  expect(mini.width).toBeLessThanOrEqual(310)
  expect(mini.height).toBeGreaterThan(180)
  expect(mini.height).toBeLessThanOrEqual(210)
  expect(mini.x - main.x, '小地図が左に寄っていない').toBeLessThan(30)
  expect((main.y + main.height) - (mini.y + mini.height), '小地図が下に寄っていない').toBeLessThan(30)
  const border = await page.locator('.s12-map.small').evaluate((e) => getComputedStyle(e).borderTopWidth)
  expect(border, '小地図に枠が無い').toBe('1px')
  await expect(page.getByTestId('s12-map-map-tools')).toBeHidden()
  await page.locator('[data-testid="s12-map-swap"]').click()
  await expect(page.getByTestId('s12-map-map-tools')).toBeVisible()
})

test('S-12: 向くと保存は横に 2 つ並んで右列の幅いっぱい', async ({ page }) => {
  await openS12(page, { mode: 'TEACH_FOLLOW', state: 'PAUSE', tracker_enabled: true })
  const face = await page.getByTestId('s12-face').boundingBox()
  const save = await page.getByTestId('s12-save').boundingBox()
  const tog = await page.getByTestId('s12-stop-run').boundingBox()
  expect(Math.abs(face.y - save.y), '向くと保存が横に並んでいない').toBeLessThanOrEqual(2)
  expect(face.x + face.width, '向くが保存の左でない').toBeLessThanOrEqual(save.x + 1)
  expect(face.x, '左端が揃っていない').toBeGreaterThanOrEqual(tog.x - 1)
  expect(save.x + save.width, '右端が幅いっぱいでない').toBeGreaterThanOrEqual(tog.x + tog.width - 4)
  expect(face.y, '向くが停止│走行の下でない').toBeGreaterThanOrEqual(tog.y + tog.height - 1)
})
