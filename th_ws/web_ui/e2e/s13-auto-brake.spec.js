// 1b-15 の残り・1b-7 S-13: S-13 の自動ブレーキ切替（ロックの2回押し）・OFF 中の接近警告。
//
// S-11（s11-auto-brake.spec.js）と同じ見た目・同じ経路（ros/useSetFlag.js の
// 本番の経路。TEST_MODE の記録先 window.__thSetFlagCalls）。
// 「ボタンがある」ではなく「押すと機体へ要求が送られる」ことを見る。
import { test, expect } from '@playwright/test'
import { gotoScreenWithLimiter } from './helpers.js'

const LIMITER_PASS = {
  alive: true, action: 'PASS', nearest_obstacle_m: 5.0, source_class: 'MANUAL',
  applied_limit_mps: 1.0, approach_warning: false,
}

async function stubAccepted(page) {
  await page.addInitScript(() => {
    window.__thTestSetFlag = { auto_brake: () => ({ accepted: true, reject_reason_key: '' }) }
  })
}

async function flagCalls(page) {
  return page.evaluate(() => window.__thSetFlagCalls ?? [])
}

// S-13 の切替は走行タブ（初期表示）にある。教示タブを開かずに押せる。
test('OFF -> ON sends the request without a confirmation', async ({ page }) => {
  await stubAccepted(page)
  await gotoScreenWithLimiter(page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL', zone: 'OUT', auto_brake: false }, LIMITER_PASS)
  await page.locator('#s13').waitFor()
  await expect(page.getByTestId('s13-auto-brake')).toHaveText('OFF')

  await page.getByTestId('s13-auto-brake-on').click()
  const calls = await flagCalls(page)
  expect(calls.map((c) => [c.flag, c.value])).toEqual([['auto_brake', true]])
})

test('ON -> OFF: ロック2回＋「切」で送る', async ({ page }) => {
  await stubAccepted(page)
  await gotoScreenWithLimiter(page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'REC', zone: 'OUT', auto_brake: true }, LIMITER_PASS)
  await page.locator('#s13').waitFor()
  // REC では教示タブが初期表示。切替は走行タブにある。
  await page.getByRole('tab', { name: '走行' }).click()
  await expect(page.getByTestId('s13-auto-brake')).toHaveText('ON')

  await page.getByTestId('s13-auto-brake-toggle').click()

  await expect(page.getByTestId('s13-auto-brake-toggle')).toContainText('もう一度押すと外す')
  expect(await flagCalls(page), '1回目では要求を送ってはいけない').toEqual([])
  await page.getByTestId('s13-auto-brake-toggle').click()
  await expect(page.getByTestId('s13-auto-brake-off')).toBeVisible()
  expect(await flagCalls(page), 'ロックが外れただけでは要求を送ってはいけない').toEqual([])
  await page.getByTestId('s13-auto-brake-off').click()
  const calls = await flagCalls(page)
  expect(calls.map((c) => [c.flag, c.value])).toEqual([['auto_brake', false]])
})

test('ON -> OFF: 黄のまま押さなければ戻る', async ({ page }) => {
  await stubAccepted(page)
  await gotoScreenWithLimiter(page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'REC', zone: 'OUT', auto_brake: true }, LIMITER_PASS)
  await page.locator('#s13').waitFor()
  await page.getByRole('tab', { name: '走行' }).click()
  await page.getByTestId('s13-auto-brake-toggle').click()
  await expect(page.getByTestId('s13-auto-brake-toggle')).toContainText('もう一度押すと外す')
  await page.waitForTimeout(3200)
  await expect(page.getByTestId('s13-auto-brake-toggle')).not.toContainText('もう一度押すと外す')
  expect(await flagCalls(page)).toEqual([])
  await expect(page.getByTestId('s13-auto-brake')).toHaveText('ON')
})

test('a request rejected by the vehicle shows the reason and the display stays', async ({ page }) => {
  await page.addInitScript(() => {
    window.__thTestSetFlag = {
      auto_brake: () => ({ accepted: false, reject_reason_key: 'auto_brake_locked' }),
    }
  })
  await gotoScreenWithLimiter(page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL', zone: 'OUT', auto_brake: false }, LIMITER_PASS)
  await page.locator('#s13').waitFor()
  await page.getByTestId('s13-auto-brake-on').click()
  await expect(page.getByTestId('s13-auto-brake-error')).toContainText('切り替えられません')
  await expect(page.getByTestId('s13-auto-brake')).toHaveText('OFF')
})

test('approach warning appears while OFF when the limiter reports it', async ({ page }) => {
  await gotoScreenWithLimiter(page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'REC', zone: 'OUT', auto_brake: false },
    { ...LIMITER_PASS, nearest_obstacle_m: 30, approach_clearance_m: 0.5,
      approach_warning: true })
  await page.locator('#s13').waitFor()
  await page.getByRole('tab', { name: '走行' }).click()
  await expect(page.getByTestId('s13-approach-warning')).toBeVisible()
  await expect(page.getByTestId('s13-auto-brake-off-hint')).toBeVisible()
  // OFF 中は action が PASS のままでも「障害物なし」と出さない。
  await expect(page.locator('#s13')).not.toContainText('障害物なし')
  await expect(page.locator('#s13')).toContainText('機体から前方 50 cm に障害物')
})

test('no approach warning while ON even if the signal were set', async ({ page }) => {
  await gotoScreenWithLimiter(page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'REC', zone: 'OUT', auto_brake: true },
    { ...LIMITER_PASS, approach_warning: true })
  await page.locator('#s13').waitFor()
  await page.getByRole('tab', { name: '走行' }).click()
  await expect(page.getByTestId('s13-approach-warning')).toHaveCount(0)
  await expect(page.getByTestId('s13-auto-brake-off-hint')).toHaveCount(0)
})
