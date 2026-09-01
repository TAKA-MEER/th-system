// 2026-09-01 (Spec-modes.md §3.1.1 SM-3.1.1-11): UI 非常停止ボタン起因の ESTOP
// （重大フォルト無し・押下前が復帰可能なモード）は、解除後に W-1 が
// 「元のモードに戻る／メインメニューへ」の 2 択に変わる。
//
// rosbridge バックエンドは無い（helpers.js / U-3）ので、ボタンのクリックは
// window.__thTriggerCalls に積まれることだけを検証する（FSM 側の遷移は
// test_state_core.py::test_ui_estop_resume_to_prev_mode がカバー）。
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('解除済み・押下前 MANUAL の ESTOP は 2 択を出し、戻るが ui.resume_yes を送る', async ({ page }) => {
  // estop_ui/hw とも false = 解除済み。prev_mode=MANUAL = 復帰可能。
  await gotoWithState(page, { mode: 'ESTOP', prev_mode: 'MANUAL', estop_ui: false, estop_hw: false })

  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  const back = page.getByRole('button', { name: '元のモードに戻る' })
  const menu = page.getByRole('button', { name: 'メインメニューへ' })
  await expect(back).toBeVisible()
  await expect(menu).toBeVisible()

  await back.click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_yes')
})

test('メインメニューへ は ui.resume_no を送る', async ({ page }) => {
  await gotoWithState(page, { mode: 'ESTOP', prev_mode: 'TEACH_MANUAL', estop_ui: false, estop_hw: false })
  await page.getByRole('button', { name: 'メインメニューへ' }).click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_no')
})

test('重大フォルト起因の ESTOP は「戻る」を出さず「確認」だけ', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'MANUAL', estop_ui: false, estop_hw: false,
  })
  await page.evaluate(() => window.__thSetTestFault({ active: true, severity: 'CRITICAL', fault_type: 'LOCALIZATION_LOST' }))

  await expect(page.getByRole('button', { name: '元のモードに戻る' })).toHaveCount(0)
  await expect(page.locator('#overlay').getByRole('button', { name: '確認' })).toBeVisible()
})

test('押下前が IDLE（メニューで押した）の ESTOP は 2 択を出さない', async ({ page }) => {
  await gotoWithState(page, { mode: 'ESTOP', prev_mode: 'IDLE', estop_ui: false, estop_hw: false })
  await expect(page.getByRole('button', { name: '元のモードに戻る' })).toHaveCount(0)
})
