// e2e/w1-presence-pause.spec.js — 1b-1 SG-A12。
//
// 走行中に操作端末が離れると state_manager が PAUSE＋pause_reason=presence_lost
// に落とす（C-16）。フォルトでは無いので faultActive は立たず、戻ってきた端末は
// 一回きりの open_window effect も受け取れない。W-1 は /system/state の
// pause_reason から出す（状態由来なので再読み込み後も出る）。
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('SG-A12: PAUSE＋presence_lost で W-1 に再開確認が出る（フォルト無し）', async ({ page }) => {
  await gotoWithState(page, { mode: 'FOLLOW', state: 'PAUSE', pause_reason: 'presence_lost' })
  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  await expect(win).toContainText('操作端末が離れたため一時停止しました')
  // フォルトが無いので最初から「再開しますか」の形（FOLLOW は yes_no）。
  await expect(win.getByRole('button', { name: 'はい' })).toBeVisible()
  await expect(win.getByRole('button', { name: 'いいえ' })).toBeVisible()
  await win.getByRole('button', { name: 'はい' }).click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_yes')
})

test('SG-A12: 再読み込み後も W-1 が出る（状態由来・effect に頼らない）', async ({ page }) => {
  await gotoWithState(page, { mode: 'FOLLOW', state: 'PAUSE', pause_reason: 'presence_lost' })
  await expect(page.locator('.win.fault.show')).toBeVisible()
  await page.reload()
  await expect(page.locator('.win.fault.show')).toBeVisible()
  await expect(page.locator('.win.fault.show')).toContainText('操作端末が離れたため一時停止しました')
})

test('SG-A12: PREP/PAUSE でも「はい／いいえ」が出る（attributes の PREP は yes_no）', async ({ page }) => {
  await gotoWithState(page, { mode: 'PREP', state: 'PAUSE', pause_reason: 'presence_lost' })
  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  await expect(win.getByRole('button', { name: 'はい' })).toBeVisible()
  await expect(win.getByRole('button', { name: 'いいえ' })).toBeVisible()
})

test('SG-A12: MANUAL/PAUSE は「確認」1 択（ack_only）', async ({ page }) => {
  await gotoWithState(page, { mode: 'MANUAL', state: 'PAUSE', pause_reason: 'presence_lost' })
  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  await expect(win.getByRole('button', { name: '確認' })).toBeVisible()
  await win.getByRole('button', { name: '確認' }).click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_ack')
})

test('SG-A12: 理由の無い PAUSE（自主停止・フォルト無し）では窓を出さない（従来どおり）', async ({ page }) => {
  await gotoWithState(page, { mode: 'FOLLOW', state: 'PAUSE', pause_reason: '' })
  await expect(page.locator('.win.fault.show')).toHaveCount(0)
})
