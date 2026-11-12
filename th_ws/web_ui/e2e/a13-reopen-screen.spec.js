// A-13 (2026-11-12): 端末を開き直すと S-00（接続確認）に戻り、そこで再開しても
// 機体が動かなかった。機体が INIT 以外なら、開き直した画面はそのモードの画面に
// し、決まった画面の id を ui.screen（C-14 set_screen）で機体へ送る。
// 送信はソースの文字列ではなく、実際に画面をマウントして
// window.__thTriggerCalls に積まれた内容で確かめる。
import { test, expect } from '@playwright/test'
import { gotoReopen } from './helpers.js'

const uiScreenCalls = (page) => page.evaluate(
  () => (window.__thTriggerCalls ?? []).filter((c) => c.trigger === 'ui.screen'))

test('A-13: 盤前移動の途中で開き直すと S-21 を開き、ui.screen を S-21 で送る', async ({ page }) => {
  await gotoReopen(page, { mode: 'PANEL_NAV', state: 'PAUSE', pause_reason: 'presence_lost' })

  await expect(page.locator('#s21')).toBeVisible()
  await expect(page.locator('#s00')).toHaveCount(0)
  // W-1（再開しますか）は開き直した画面の上に出る。
  await expect(page.locator('.win.fault.show')).toBeVisible()
  await expect(page.locator('.win.fault.show')).toContainText('操作端末が離れたため一時停止しました')

  await page.waitForFunction(
    () => (window.__thTriggerCalls ?? []).some((c) => c.trigger === 'ui.screen'))
  const calls = await uiScreenCalls(page)
  expect(calls.length).toBeGreaterThan(0)
  for (const c of calls) expect(c.argJson).toEqual({ screen_id: 'S-21' })

  // requester は端末 ID（th_state は requester を client_id として在席に入れる）。
  const clientId = await page.evaluate(() => window.localStorage.getItem('th_ui_client_id'))
  expect(clientId).toBeTruthy()
  expect(calls[0].requester).toBe(clientId)
})

test('A-13: IDLE で開き直すと S-01 を開き、ui.screen を S-01 で送る', async ({ page }) => {
  await gotoReopen(page, { mode: 'IDLE', state: 'NONE' })

  await expect(page.locator('#s01')).toBeVisible()
  await expect(page.locator('#s00')).toHaveCount(0)

  await page.waitForFunction(
    () => (window.__thTriggerCalls ?? []).some((c) => c.trigger === 'ui.screen'))
  const ids = (await uiScreenCalls(page)).map((c) => c.argJson.screen_id)
  expect(ids).toContain('S-01')
  expect(ids).not.toContain('S-00')
})

test('A-13: mode が INIT のときは従来どおり S-00 を開く', async ({ page }) => {
  await gotoReopen(page, { mode: 'INIT', state: 'CHECK' })

  await expect(page.locator('#s00')).toBeVisible()
  await expect(page.locator('#s21')).toHaveCount(0)
})
