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
  const box = async () => page.locator('#s10 .g-drv-row').boundingBox()
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
