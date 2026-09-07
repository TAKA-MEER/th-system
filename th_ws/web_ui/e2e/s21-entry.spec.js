// brief-UI-S21-entry: S-01 メインメニューから S-21 試験を開く実導線。
// S-21 は FSM のモードではないので、S-50 設定と同じ「S-01 のサブ画面」方式
// （main.jsx の onsiteTestOpen）で開く。?__thTestScreen は使わない
// （gotoScreen('S01', ...) の '__thTestScreen=S01' で S-01 そのものを開き、
// 「試験」ボタン → resolveScreen(onsiteTestOpen) → S-21 を検証する）。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

const IDLE = { mode: 'IDLE', state: 'NONE' }

test('S-01 の「試験」ボタンで S-21 を実際の導線で開く', async ({ page }) => {
  await gotoScreen(page, 'S01', IDLE)
  await page.locator('#s01').waitFor()
  await expect(page.locator('[data-testid="s01-open-onsite-test"]')).toHaveText('試験（当日）')
  await expect(page.locator('#s21')).toBeHidden()

  await page.locator('[data-testid="s01-open-onsite-test"]').click()
  await expect(page.locator('#s21')).toBeVisible()
  await expect(page.locator('[data-testid="s21-finish"]')).toBeVisible()
  // IDLE のまま開くので「行き先を選んでください」がそのまま出る。
  await expect(page.locator('[data-testid="s21-state"]')).toHaveText('行き先を選んでください')
})

test('S-21 の「終了」は ui.finish を送り、S-01 に戻る（onsiteTestOpen を閉じる）', async ({ page }) => {
  await gotoScreen(page, 'S01', IDLE)
  await page.locator('#s01').waitFor()
  await page.locator('[data-testid="s01-open-onsite-test"]').click()
  await expect(page.locator('#s21')).toBeVisible()

  await page.locator('[data-testid="s21-finish"]').click()
  await expect(page.locator('#s01')).toBeVisible()
  expect(
    (await page.evaluate(() => window.__thTriggerCalls ?? []))
      .filter((c) => c.trigger === 'ui.finish'),
    '終了が ui.finish を送っていない',
  ).toHaveLength(1)
})

test('「試験」は IDLE のときだけ押せる（INIT では非活性）', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'INIT', state: 'NONE' })
  await page.locator('#s01').waitFor()
  await expect(page.locator('[data-testid="s01-open-onsite-test"]')).toBeDisabled()
})

test('設定(S-50)を開いてから「試験」を押すと、設定が閉じて S-21 が出る（相互排他）', async ({ page }) => {
  await gotoScreen(page, 'S01', IDLE)
  await page.locator('#s01').waitFor()

  // 設定を開く → S-50。
  await page.locator('[data-testid="s01-open-settings"]').click()
  await expect(page.locator('#s50')).toBeVisible()
  // 戻る → S-01。
  await page.locator('[data-testid="s50-back"]').click()
  await expect(page.locator('#s01')).toBeVisible()

  // そのまま「試験」→ S-21（S-50 は出ない）。
  await page.locator('[data-testid="s01-open-onsite-test"]').click()
  await expect(page.locator('#s21')).toBeVisible()
  await expect(page.locator('#s50')).toBeHidden()
})