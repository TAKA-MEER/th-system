// 1b-15 SG-B10: S-11 の自動ブレーキ切替・確認（W-4）・OFF 中の接近警告。
//
// 「ボタンがある」ではなく「押すと機体へ要求が送られる」ことを見る
// （s11-stop-sends-ui-stop.spec.js と同じ理由。DetailedDesign-wp3.md §11 c6）。
// 送信は ros/useSetFlag.js の本番の経路（TEST_MODE の記録先 window.__thSetFlagCalls）。
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

test('OFF -> ON sends the request without a confirmation', async ({ page }) => {
  await stubAccepted(page)
  await gotoScreenWithLimiter(page, 'S11',
    { mode: 'MANUAL', state: 'PAUSE', zone: 'OUT', auto_brake: false }, LIMITER_PASS)
  await page.locator('#s11').waitFor()
  await expect(page.getByTestId('s11-auto-brake')).toHaveText('OFF')

  await page.getByTestId('s11-auto-brake-toggle').click()

  await expect(page.getByTestId('s11-auto-brake-confirm')).toHaveCount(0)
  const calls = await flagCalls(page)
  expect(calls.map((c) => [c.flag, c.value])).toEqual([['auto_brake', true]])
})

test('ON -> OFF asks first; "yes" sends the request, nothing before', async ({ page }) => {
  await stubAccepted(page)
  await gotoScreenWithLimiter(page, 'S11',
    { mode: 'MANUAL', state: 'PAUSE', zone: 'IN', auto_brake: true }, LIMITER_PASS)
  await page.locator('#s11').waitFor()
  await expect(page.getByTestId('s11-auto-brake')).toHaveText('ON')

  await page.getByTestId('s11-auto-brake-toggle').click()

  await expect(page.getByTestId('s11-auto-brake-confirm')).toBeVisible()
  expect(await flagCalls(page), '確認の前に要求を送ってはいけない').toEqual([])

  await page.getByTestId('s11-auto-brake-confirm-yes').click()
  const calls = await flagCalls(page)
  expect(calls.map((c) => [c.flag, c.value])).toEqual([['auto_brake', false]])
  await expect(page.getByTestId('s11-auto-brake-confirm')).toHaveCount(0)
})

test('ON -> OFF: "no" closes the window and sends nothing', async ({ page }) => {
  await stubAccepted(page)
  await gotoScreenWithLimiter(page, 'S11',
    { mode: 'MANUAL', state: 'PAUSE', zone: 'IN', auto_brake: true }, LIMITER_PASS)
  await page.locator('#s11').waitFor()
  await page.getByTestId('s11-auto-brake-toggle').click()
  await expect(page.getByTestId('s11-auto-brake-confirm')).toBeVisible()

  await page.getByTestId('s11-auto-brake-confirm-no').click()

  await expect(page.getByTestId('s11-auto-brake-confirm')).toHaveCount(0)
  expect(await flagCalls(page)).toEqual([])
  await expect(page.getByTestId('s11-auto-brake')).toHaveText('ON')
})

test('a request rejected by the vehicle shows the reason and the display stays', async ({ page }) => {
  await page.addInitScript(() => {
    window.__thTestSetFlag = {
      auto_brake: () => ({ accepted: false, reject_reason_key: 'auto_brake_locked' }),
    }
  })
  await gotoScreenWithLimiter(page, 'S11',
    { mode: 'MANUAL', state: 'PAUSE', zone: 'OUT', auto_brake: false }, LIMITER_PASS)
  await page.locator('#s11').waitFor()
  await page.getByTestId('s11-auto-brake-toggle').click()
  await expect(page.getByTestId('s11-auto-brake-error')).toContainText('切り替えられません')
  await expect(page.getByTestId('s11-auto-brake')).toHaveText('OFF')
})

// 注: window.__thSetTestLimiterStatus は AppShell と S-11 の両方が入れるため、後から
// 差し替えても S-11 側に届かない（試験の道具の制約）。種で出し分ける。
test('approach warning appears while OFF when the limiter reports it', async ({ page }) => {
  await gotoScreenWithLimiter(page, 'S11',
    { mode: 'MANUAL', state: 'RUN', zone: 'OUT', auto_brake: false },
    { ...LIMITER_PASS, nearest_obstacle_m: 0.5, approach_warning: true })
  await page.locator('#s11').waitFor()
  await expect(page.getByTestId('s11-approach-warning')).toBeVisible()
  await expect(page.getByTestId('s11-auto-brake-off-hint')).toBeVisible()
  // OFF 中は action が PASS のままでも「障害物なし」と出さない。
  await expect(page.locator('#s11')).not.toContainText('障害物なし')
  await expect(page.locator('#s11')).toContainText('0.5')
})

test('no approach warning while OFF when the limiter does not report it', async ({ page }) => {
  await gotoScreenWithLimiter(page, 'S11',
    { mode: 'MANUAL', state: 'RUN', zone: 'OUT', auto_brake: false }, LIMITER_PASS)
  await page.locator('#s11').waitFor()
  await expect(page.getByTestId('s11-approach-warning')).toHaveCount(0)
  await expect(page.getByTestId('s11-auto-brake-off-hint')).toBeVisible()
})

test('no approach warning while ON even if the signal were set', async ({ page }) => {
  await gotoScreenWithLimiter(page, 'S11',
    { mode: 'MANUAL', state: 'RUN', zone: 'IN', auto_brake: true },
    { ...LIMITER_PASS, approach_warning: true })
  await page.locator('#s11').waitFor()
  await expect(page.getByTestId('s11-approach-warning')).toHaveCount(0)
  await expect(page.getByTestId('s11-auto-brake-off-hint')).toHaveCount(0)
})
