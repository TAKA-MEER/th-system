// e2e/w1-ack-closes-latch.spec.js — 実機 2026-10-13 の再現。
//
// MANUAL/RUN で LiDAR を抜くと MANUAL/PAUSE (pause_reason=fault)。解消後の W-1 は
// 「確認」1 択になる。「確認」(ui.resume_ack) は走行を再開せずモードだけ戻す
// (Spec-webui §4.1) ので、機体の応答は MANUAL/PAUSE のまま pause_reason が '' に
// 戻るだけ。WS-9Z のラッチ (faultPauseSeen) が PAUSE を抜けるまで保持されていたため、
// 窓が閉じず「確認」を押すたびに開き直した。
import { test, expect } from '@playwright/test'
import { gotoWithState, setTestFault, setTestState } from './helpers.js'

const FT = { fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' }
const win = (page) => page.locator('.win.fault.show')
const triggers = (page) => page.evaluate(() => (window.__thTriggerCalls ?? []).map((c) => c.trigger))

test('(a) MANUAL: 確認を押して機体が pause_reason を戻したら W-1 が閉じ、開き直さない', async ({ page }) => {
  await gotoWithState(page, { mode: 'MANUAL', state: 'RUN', pause_reason: '' })
  await expect(win(page)).toHaveCount(0)
  // 回復フォルト: 機体は PAUSE に入り pause_reason=fault。
  await setTestState(page, { state: 'PAUSE', pause_reason: 'fault' })
  await setTestFault(page, { active: true, ...FT })
  await expect(win(page)).toBeVisible()
  await setTestFault(page, { active: false, ...FT })
  await expect(win(page)).toBeVisible()
  const ack = win(page).getByRole('button', { name: '確認' })
  await expect(ack).toBeVisible()
  await ack.click()
  expect(await triggers(page)).toContain('ui.resume_ack')
  // 機体の応答: MANUAL/PAUSE のまま pause_reason が '' に戻る。
  await setTestState(page, { state: 'PAUSE', pause_reason: '' })
  await expect(win(page)).toHaveCount(0)
  await page.waitForTimeout(3000)
  await expect(win(page)).toHaveCount(0)
})

test('(b) フォルトが表示前に消えても（WS-9Z）窓が出る', async ({ page }) => {
  await gotoWithState(page, { mode: 'MANUAL', state: 'RUN', pause_reason: '' })
  await setTestState(page, { state: 'PAUSE', pause_reason: 'fault' })
  await setTestFault(page, { active: true, ...FT })
  await setTestFault(page, { active: false, ...FT })
  await expect(win(page)).toBeVisible()
  await expect(win(page).getByRole('button', { name: '確認' })).toBeVisible()
})

test('(b2) pause_reason が fault になる前に fault が立って消えても、窓は早く下りない', async ({ page }) => {
  await gotoWithState(page, { mode: 'MANUAL', state: 'RUN', pause_reason: '' })
  // PAUSE が pause_reason 空のまま先に届き、fault が立つ → 解消 → 遅れて 'fault'。
  await setTestState(page, { state: 'PAUSE', pause_reason: '' })
  await setTestFault(page, { active: true, ...FT })
  await setTestFault(page, { active: false, ...FT })
  await setTestState(page, { pause_reason: 'fault' })
  await expect(win(page)).toBeVisible()
})

test('(c) run_state の無いモード（AT_HOME）は従来どおり: 確認後も PAUSE なら窓は残る', async ({ page }) => {
  await gotoWithState(page, { mode: 'AT_HOME', state: 'PAUSE', jog_active: false, pause_reason: 'fault' })
  await setTestFault(page, { active: true, ...FT })
  await setTestFault(page, { active: false, ...FT })
  await expect(win(page)).toBeVisible()
  // 保険経路（run_state 無し＋ジョグでない PAUSE）が救うので、pause_reason が戻っても残る。
  await setTestState(page, { pause_reason: '' })
  await page.waitForTimeout(500)
  await expect(win(page)).toBeVisible()
  await setTestState(page, { state: 'IDLE_H' })
  await expect(win(page)).toHaveCount(0)
})

test('(d) 走行モード（PANEL_NAV）: 「はい」で機体が pause_reason を戻すと閉じる', async ({ page }) => {
  await gotoWithState(page, { mode: 'PANEL_NAV', state: 'NAV', pause_reason: '' })
  await setTestState(page, { state: 'PAUSE', pause_reason: 'fault' })
  await setTestFault(page, { active: true, ...FT })
  await setTestFault(page, { active: false, ...FT })
  await expect(win(page)).toBeVisible()
  await win(page).getByRole('button', { name: 'はい' }).click()
  expect(await triggers(page)).toContain('ui.resume_yes')
  await setTestState(page, { state: 'PAUSE', pause_reason: '' })
  await expect(win(page)).toHaveCount(0)
})

test('(d2) 走行モード（PANEL_NAV）: 「いいえ」でも同様に閉じる', async ({ page }) => {
  await gotoWithState(page, { mode: 'PANEL_NAV', state: 'NAV', pause_reason: '' })
  await setTestState(page, { state: 'PAUSE', pause_reason: 'fault' })
  await setTestFault(page, { active: true, ...FT })
  await setTestFault(page, { active: false, ...FT })
  await win(page).getByRole('button', { name: 'いいえ' }).click()
  expect(await triggers(page)).toContain('ui.resume_no')
  await setTestState(page, { state: 'PAUSE', pause_reason: '' })
  await expect(win(page)).toHaveCount(0)
})

test('(e) 古い機体（pause_reason が無い）は従来どおり PAUSE を抜けるまで保持', async ({ page }) => {
  await gotoWithState(page, { mode: 'MANUAL', state: 'RUN' })
  await setTestState(page, { state: 'PAUSE', pause_reason: undefined })
  await setTestFault(page, { active: true, ...FT })
  await setTestFault(page, { active: false, ...FT })
  await expect(win(page)).toBeVisible()
  await page.waitForTimeout(500)
  await expect(win(page)).toBeVisible()
  await setTestState(page, { state: 'RUN' })
  await expect(win(page)).toHaveCount(0)
})
