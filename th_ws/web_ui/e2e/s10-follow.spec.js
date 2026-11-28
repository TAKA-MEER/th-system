// S-10 追従走行（型「走らせる」）。機体が受け付ける操作だけをボタンにする。
// FSM: SELECT --ui.select_target--> SELECT、SELECT --ui.confirm--> CONFIRM
//   CONFIRM --ui.run--> RUN、RUN --ui.stop--> PAUSE、PAUSE --ui.run--> RUN
import { test, expect } from '@playwright/test'

const TARGETS = {
  candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
  selected_index: -1, confidence: 0.87, is_lost: false, lost_reason: '',
}

async function openS10(page, state) {
  await page.addInitScript(({ s }) => {
    window.__thTestState = s
    window.__thTestScreen = 'S10'
    window.__thTestPersonTargets = {
      candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
      selected_index: -1, confidence: 0.87, is_lost: false, lost_reason: '',
    }
  }, { s: state })
  await page.goto('/')
  await page.locator('#s10').waitFor()
}

async function triggers(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

test('S-10: レーダーのタップが ui.select_target を送る', async ({ page }) => {
  await openS10(page, { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true })
  await page.locator('[data-testid="radar-cand-1"]').click()
  const sel = (await triggers(page)).find((c) => c.trigger === 'ui.select_target')
  expect(sel, 'レーダーのタップが ui.select_target を送っていない').toBeTruthy()
  expect(sel.argJson, '候補 1 の index で送られていない').toEqual({ index: 1 })
})

test('S-10: 向くが ui.confirm を、走行するが ui.run を、停止するが ui.stop を送る', async ({ page }) => {
  await openS10(page, { mode: 'FOLLOW', state: 'CONFIRM', tracker_enabled: true })
  await page.evaluate(() => window.__thSetTestPersonTargets({
    candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
    selected_index: 0, confidence: 0.87, is_lost: false, lost_reason: '',
  }))
  await page.locator('[data-testid="s10-stop-run-run"]').click()
  let calls = await triggers(page)
  expect(calls.map((c) => c.trigger), 'CONFIRM で走行するが ui.run を送っていない').toContain('ui.run')

  await page.evaluate(() => window.__thSetTestState({ mode: 'FOLLOW', state: 'RUN' }))
  await page.locator('[data-testid="s10-stop-run-stop"]').click()
  calls = await triggers(page)
  expect(calls.map((c) => c.trigger), 'RUN で停止するが ui.stop を送っていない').toContain('ui.stop')

  // SELECT に戻して向くを押す
  await page.evaluate(() => window.__thSetTestState({ mode: 'FOLLOW', state: 'SELECT' }))
  await page.locator('[data-testid="s10-face"]').click()
  calls = await triggers(page)
  expect(calls.map((c) => c.trigger), '向くが ui.confirm を送っていない').toContain('ui.confirm')
})

test('S-10: 対象を選んでいないときは走行できず理由が出る', async ({ page }) => {
  await openS10(page, { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true })
  await expect(page.getByTestId('s10-no-target')).toBeVisible()
  await page.locator('[data-testid="s10-face"]').click({ force: true })
  await expect(page.getByTestId('s10-face-reason')).toBeVisible()
})

test('S-10: 選んだ対象までの距離が出る', async ({ page }) => {
  await openS10(page, { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true })
  await page.evaluate(() => window.__thSetTestPersonTargets({
    candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
    selected_index: 0, confidence: 0.87, is_lost: false, lost_reason: '',
  }))
  await expect(page.getByTestId('s10-distance-value')).toHaveText('1.3 m')
})

test('S-10: 状態を変えても右列の位置が動かない', async ({ page }) => {
  await openS10(page, { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true })
  const box = async () => page.locator('#s10 .content.g-drv').boundingBox()
  const before = await box()
  await page.evaluate(() => window.__thSetTestPersonTargets({
    candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
    selected_index: 0, confidence: 0.87, is_lost: false, lost_reason: '',
  }))
  await page.evaluate(() => window.__thSetTestState({ mode: 'FOLLOW', state: 'RUN' }))
  const after = await box()
  expect(Math.abs(before.x - after.x), 'main の左が動いた').toBeLessThan(1)
  expect(Math.abs(before.width - after.width), 'main の幅が動いた').toBeLessThan(1)
  // ①②③④ のデータ位置が残っている
  for (const pos of ['where', 'toggle', 'ops', 'jog']) {
    await expect(page.locator(`#s10 [data-pos="${pos}"]`)).toBeVisible()
  }
  // 停止│走行は押せる側だけ塗る（走行中は停止が黒）
  const stopBg = await page.locator('[data-testid="s10-stop-run-stop"]').evaluate((el) => getComputedStyle(el).backgroundColor)
  const runBg = await page.locator('[data-testid="s10-stop-run-run"]').evaluate((el) => getComputedStyle(el).backgroundColor)
  expect(stopBg, 'RUN で停止が塗っていない').not.toBe('rgba(0, 0, 0, 0)')
  expect(stopBg, 'RUN で停止が塗っていない').not.toBe('transparent')
  expect(runBg).not.toBe(stopBg)
})

test('S-10: 終了が ui.finish を送る', async ({ page }) => {
  await openS10(page, { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true })
  await page.locator('[data-testid="s10-finish"]').click()
  const calls = await triggers(page)
  expect(calls.map((c) => c.trigger), '終了が ui.finish を送っていない').toContain('ui.finish')
})

// ---- 差し戻し（見た目）: レーダーを主表示いっぱいに・色・「向く」 ----
const TWO = {
  candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
  confidence: 0.87, is_lost: false, lost_reason: '',
}

test('S-10: レーダーは主表示（右列を除く本文）いっぱいで、円は短い辺いっぱい', async ({ page }) => {
  await openS10(page, { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true })
  const main = await page.locator('[data-pos="main"]').boundingBox()
  const radar = await page.getByTestId('radar').boundingBox()
  expect(radar.width, 'レーダーの幅が主表示の幅いっぱいでない').toBeGreaterThanOrEqual(main.width - 2)
  expect(radar.height, 'レーダーの高さが主表示の高さいっぱいでない').toBeGreaterThanOrEqual(main.height - 2)
  const vh = page.viewportSize().height
  expect(main.y + main.height, '主表示の下に大きな空きがある').toBeGreaterThan(vh - 40)
})

test('S-10: 縦長でもレーダーは主表示の幅いっぱい', async ({ page }) => {
  await page.setViewportSize({ width: 960, height: 1280 })
  await openS10(page, { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true })
  const main = await page.locator('[data-pos="main"]').boundingBox()
  const radar = await page.getByTestId('radar').boundingBox()
  expect(radar.width).toBeGreaterThanOrEqual(main.width - 2)
  expect(main.width, '縦長で主表示が幅いっぱいでない').toBeGreaterThan(960 * 0.85)
})

test('S-10: レーダーの色。候補は灰の丸＋番号、選んだ人は反転、機体は緑、青を使わない', async ({ page }) => {
  await openS10(page, { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true })
  await page.evaluate((t) => window.__thSetTestPersonTargets({ ...t, selected_index: 0 }), TWO)
  const fill = (id) => page.locator(`[data-testid="${id}"] circle`).first().evaluate((e) => getComputedStyle(e).fill)
  const grey = await fill('radar-cand-1')
  const sel = await fill('radar-cand-0')
  expect(grey, '候補の丸が中立の灰でない').toBe('rgb(128, 128, 128)')
  expect(sel, '選んだ人が反転（濃い地）でない').toBe('rgb(26, 26, 26)')
  await expect(page.locator('[data-testid="radar-cand-0"] .rv-num')).toHaveText('1')
  await expect(page.locator('[data-testid="radar-cand-1"] .rv-num')).toHaveText('2')
  const heading = await page.locator('[data-testid="radar-heading"] polygon').evaluate((e) => getComputedStyle(e).fill)
  expect(heading, '機体の三角が進むの色（緑）でない').toMatch(/^rgb\(\d+, \d+, \d+\)$/)
  const [r, g, b] = heading.match(/\d+/g).map(Number)
  expect(g, '機体の三角が緑でない').toBeGreaterThan(r)
  expect(g).toBeGreaterThan(b)
  const html = await page.getByTestId('radar').evaluate((e) => e.outerHTML.toLowerCase())
  expect(html, '青（旧: #90caf9・#245089）が残っている').not.toMatch(/90caf9|245089/)
})

test('S-10: 向くは停止│走行の下に幅いっぱい。押せるときだけ薄い緑', async ({ page }) => {
  await openS10(page, { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true })
  const face = page.getByTestId('s10-face')
  const toggle = page.getByTestId('s10-stop-run')
  const bg = () => face.evaluate((e) => getComputedStyle(e).backgroundColor)
  const off = await bg()
  await page.evaluate((t) => window.__thSetTestPersonTargets({ ...t, selected_index: 0 }), TWO)
  await expect(face).not.toHaveClass(/\bdis\b/)
  const on = await bg()
  expect(on, '押せるときの「向く」が薄い緑でない').not.toBe(off)
  const [r, g, b] = on.match(/\d+/g).map(Number)
  expect(g, '押せるときの「向く」が緑みでない').toBeGreaterThan(r)
  expect(g).toBeGreaterThan(b)
  const fb = await face.boundingBox()
  const tb = await toggle.boundingBox()
  expect(fb.y, '「向く」が停止│走行の下でない').toBeGreaterThanOrEqual(tb.y + tb.height - 1)
  expect(Math.abs(fb.width - tb.width), '「向く」が幅いっぱいでない').toBeLessThanOrEqual(4)
})
