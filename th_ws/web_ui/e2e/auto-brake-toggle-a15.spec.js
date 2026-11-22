// A-15: W-6（ジョグ介入）と自律系の画面の自動ブレーキ切替。
//
// 「ボタンがある」ではなく「押すと機体へ要求が送られる」ことを見る（本番の送信経路
// ros/useSetFlag.js の TEST_MODE 記録 window.__thSetFlagCalls）。
//   - W-6: 開いている間はスティックに触れていなくても押せる。心拍を送る。ON→OFF は W-4 で確認。
//   - S-14 / S-21: 開発モードで項目 auto_brake を選んでいるときだけ切替が出る。
import { test, expect } from '@playwright/test'
import { gotoScreen, gotoScreenWithDevMode, setTestDevMode } from './helpers.js'

const ITEMS = ['link', 'lidar_fault', 'scan_stop', 'battery', 'opcheck', 'auto_brake']
const flags = (on) => Object.fromEntries(ITEMS.map((k) => [k, on.includes(k)]))
const dev = (on) => ({
  dev_mode: true, ignore: flags(on), effective: flags(on),
  estop_hw_known: false, estop_hw_pressed: false,
})
const DEV_AB = dev(['auto_brake'])
const DEV_OTHER = dev(['link', 'scan_stop'])   // auto_brake は選んでいない

async function accepted(page) {
  await page.addInitScript(() => {
    window.__thTestSetFlag = { auto_brake: () => ({ accepted: true, reject_reason_key: '' }) }
  })
}
const calls = (page) => page.evaluate(() => window.__thSetFlagCalls ?? [])
const sent = async (page) => (await calls(page)).map((c) => [c.flag, c.value])

async function openW6(page, state) {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreen(page, 'S21', { mode: 'PANEL_NAV', state: 'NAV', zone: 'IN', ...state })
  await page.locator('#s21').waitFor()
  await page.locator('#s21 .op-manual').click()
  await expect(page.locator('#jogWin.show')).toBeVisible()
}

test('W-6: ON -> OFF は確認（W-4）を挟み、「OFF にする」で初めて要求が送られる', async ({ page }) => {
  await accepted(page)
  await openW6(page, { jog_active: true, auto_brake: true })
  const w6 = page.locator('#jogWin.show')
  await w6.getByTestId('w6-auto-brake-toggle').click()
  await expect(page.getByTestId('w6-auto-brake-confirm')).toBeVisible()
  expect(await sent(page), '確認の前に要求を送ってはいけない').toEqual([])

  await page.getByTestId('w6-auto-brake-confirm-yes').click()
  expect(await sent(page)).toEqual([['auto_brake', false]])
  await expect(page.getByTestId('w6-auto-brake-confirm')).toHaveCount(0)
})

test('W-6: 確認で「やめる」なら何も送らない', async ({ page }) => {
  await accepted(page)
  await openW6(page, { jog_active: true, auto_brake: true })
  await page.locator('#jogWin.show').getByTestId('w6-auto-brake-toggle').click()
  await page.getByTestId('w6-auto-brake-confirm-no').click()
  await expect(page.getByTestId('w6-auto-brake-confirm')).toHaveCount(0)
  expect(await sent(page)).toEqual([])
})

test('W-6: OFF -> ON は確認なしで送られる', async ({ page }) => {
  await accepted(page)
  await openW6(page, { jog_active: true, auto_brake: false })
  await page.locator('#jogWin.show').getByTestId('w6-auto-brake-toggle').click()
  await expect(page.getByTestId('w6-auto-brake-confirm')).toHaveCount(0)
  expect(await sent(page)).toEqual([['auto_brake', true]])
})

test('W-6: 機体に拒否されたら理由を出す', async ({ page }) => {
  await page.addInitScript(() => {
    window.__thTestSetFlag = { auto_brake: () => ({ accepted: false, reject_reason_key: 'auto_brake_locked' }) }
  })
  await openW6(page, { jog_active: true, auto_brake: false })
  await page.locator('#jogWin.show').getByTestId('w6-auto-brake-toggle').click()
  await expect(page.getByTestId('w6-auto-brake-error')).toContainText('切り替えられません')
  await expect(page.locator('#jogWin').getByTestId('w6-auto-brake')).toHaveAttribute('data-state', 'off')
})

test('S-21: 開発モードの項目 auto_brake を選んでいるときだけ切替カードが出る', async ({ page }) => {
  await accepted(page)
  await gotoScreenWithDevMode(page, 'S21',
    { mode: 'PANEL_NAV', state: 'NAV', zone: 'IN', auto_brake: true }, DEV_OTHER)
  await page.locator('#s21').waitFor()
  await expect(page.getByTestId('s21-auto-brake-card'), '項目を選んでいないのに出ている').toHaveCount(0)

  await setTestDevMode(page, DEV_AB)
  await expect(page.getByTestId('s21-auto-brake-card')).toBeVisible()
  await page.getByTestId('s21-auto-brake-toggle').click()
  await page.getByTestId('s21-auto-brake-confirm-yes').click()
  expect(await sent(page)).toEqual([['auto_brake', false]])

  await setTestDevMode(page, DEV_OTHER)
  await expect(page.getByTestId('s21-auto-brake-card')).toHaveCount(0)
})

test('S-14: 開発モードの項目を選んでいないと切替は出ない／選ぶと出て要求が送られる', async ({ page }) => {
  await accepted(page)
  await gotoScreenWithDevMode(page, 'S14',
    { mode: 'REPLAY', state: 'PAUSE', zone: 'IN', auto_brake: true }, DEV_OTHER)
  await page.locator('#s14').waitFor()
  await expect(page.getByTestId('s14-auto-brake-card')).toHaveCount(0)

  await setTestDevMode(page, DEV_AB)
  await expect(page.getByTestId('s14-auto-brake-card')).toBeVisible()
  await page.getByTestId('s14-auto-brake-toggle').click()
  await page.getByTestId('s14-auto-brake-confirm-yes').click()
  expect(await sent(page)).toEqual([['auto_brake', false]])
})

test('S-14: 開発モードの種が無い（未受信）なら出ない', async ({ page }) => {
  await gotoScreen(page, 'S14', { mode: 'REPLAY', state: 'PAUSE', zone: 'IN', auto_brake: true })
  await page.locator('#s14').waitFor()
  await expect(page.getByTestId('s14-auto-brake-card')).toHaveCount(0)
})

test('W-6: 開くと /ui/manual_panel の心拍が出て、閉じると空文字 1 回で止まる（ジョグのリースは出さない）', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreen(page, 'S21', { mode: 'PANEL_NAV', state: 'NAV', zone: 'IN', jog_active: false, auto_brake: true })
  await page.locator('#s21').waitFor()
  const beats = () => page.evaluate(() => (window.__thManualPanelPublishes ?? []).map((m) => m.data))
  await page.waitForTimeout(500)
  expect(await beats(), '開く前に送ってはいけない').toEqual([])

  await page.locator('#s21 .op-manual').click()
  await expect(page.locator('#jogWin.show')).toBeVisible()
  await expect.poll(async () => (await beats()).filter((d) => d !== '').length).toBeGreaterThanOrEqual(3)
  // スティックには触れていないので、ジョグのリース・速度指令は出ていない（開いただけで PAUSE に落とさない）
  expect(await page.evaluate(() => (window.__thJogPublishes ?? []).length)).toBe(0)

  await page.locator('#jogWin .jw-hd button').click()
  await expect(page.locator('#jogWin')).toBeHidden()
  await expect.poll(async () => (await beats()).at(-1)).toBe('')
  const n = (await beats()).length
  await page.waitForTimeout(700)
  expect((await beats()).length, '閉じたあとも送り続けている').toBe(n)
})

test('W-6: スティックに触れていなくても（jog_active=false の自律系）切替を押せて要求が送られる', async ({ page }) => {
  await accepted(page)
  await openW6(page, { jog_active: false, auto_brake: true })
  const t = page.locator('#jogWin.show').getByTestId('w6-auto-brake-toggle')
  await expect(t).toBeEnabled()
  await t.click()
  await page.getByTestId('w6-auto-brake-confirm-yes').click()
  expect(await sent(page)).toEqual([['auto_brake', false]])
})
