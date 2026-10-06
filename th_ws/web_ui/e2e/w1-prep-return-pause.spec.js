// e2e/w1-prep-return-pause.spec.js — 1b-1。
//
// PREP/RETURN（試験準備の自動帰還）がジョグ・非常停止からの「戻る」で
// PREP/PAUSE に落ちたとき、出る手段は W-1 の「はい」（ui.resume_yes→RETURN）／
// 「いいえ」（ui.resume_no→MAPPING）だけ。PREP の「走行」は inert なので、
// W-1 が出ないと詰む。pause_reason は jog や空になる。
import { test, expect } from '@playwright/test'
import { gotoWithState, setTestState } from './helpers.js'

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
