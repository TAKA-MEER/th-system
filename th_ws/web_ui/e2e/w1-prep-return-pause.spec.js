// e2e/w1-prep-return-pause.spec.js — 1b-1。
//
// PREP/RETURN（試験準備の自動帰還）がジョグ・非常停止からの「戻る」で
// PREP/PAUSE に落ちたとき、出る手段は W-1 の「はい」（ui.resume_yes→RETURN）／
// 「いいえ」（ui.resume_no→MAPPING）だけ。PREP の「走行」は inert なので、
// W-1 が出ないと詰む。pause_reason は jog や空になる。
import { test, expect } from '@playwright/test'
import { gotoScreen, gotoWithState, setTestState } from './helpers.js'

const triggers = async (page) =>
  (await page.evaluate(() => window.__thTriggerCalls ?? [])).map((c) => c.trigger)

test('PREP/RETURN→ジョグで PAUSE（W-1 は出ない）→ ジョグを離すと「はい／いいえ」が出て、はいで ui.resume_yes', async ({ page }) => {
  await gotoWithState(page, { mode: 'PREP', state: 'RETURN', jog_active: false, pause_reason: '' })
  await expect(page.locator('.win.fault.show')).toHaveCount(0)

  await setTestState(page, { state: 'PAUSE', jog_active: true, pause_reason: 'jog' })
  await expect(page.locator('.win.fault.show')).toHaveCount(0)

  await setTestState(page, { jog_active: false })
  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  await expect(win).toContainText('自動帰還を一時停止しています')
  await expect(win.getByRole('button', { name: 'はい' })).toBeVisible()
  await expect(win.getByRole('button', { name: 'いいえ' })).toBeVisible()
  await win.getByRole('button', { name: 'はい' }).click()
  expect(await triggers(page)).toContain('ui.resume_yes')
})

test('pause_reason が空の PREP/PAUSE（非常停止から戻った状態）でも W-1 が出て、いいえで ui.resume_no', async ({ page }) => {
  await gotoWithState(page, { mode: 'PREP', state: 'PAUSE', jog_active: false, pause_reason: '' })
  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  await expect(win.getByRole('button', { name: 'はい' })).toBeVisible()
  await win.getByRole('button', { name: 'いいえ' }).click()
  expect(await triggers(page)).toContain('ui.resume_no')
})

test('PREP/RETURN で停止ボタンが押せて ui.stop を送る（SM-3.1.2-050a の入口）', async ({ page }) => {
  await gotoScreen(page, 'S20', { mode: 'PREP', state: 'RETURN' })
  await page.locator('#s20').waitFor()
  const stop = page.locator('#s20 .op-stop')
  await expect(stop).toBeVisible()
  await expect(stop).toBeEnabled()
  await stop.click()
  expect(await triggers(page)).toContain('ui.stop')
})

test('停止起点の PREP/PAUSE（理由なし）でも W-1 が出て、はいで ui.resume_yes', async ({ page }) => {
  // T-PREP-18 は effect を持たないので pause_reason は空のまま。非常停止から
  // 戻った場合と同じ表示になり、「はい」で RETURN（経路の残りから帰還）。
  await gotoWithState(page, { mode: 'PREP', state: 'PAUSE', jog_active: false, pause_reason: '' })
  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  await expect(win).toContainText('自動帰還を一時停止しています')
  await win.getByRole('button', { name: 'はい' }).click()
  expect(await triggers(page)).toContain('ui.resume_yes')
})
