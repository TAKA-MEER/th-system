// 2026-09-01 (Spec-modes.md §3.1.1 SM-3.1.1-11): UI 非常停止ボタン起因の ESTOP
// （estop_from_ui=true・重大フォルト無し・押下前が復帰可能なモード）は、解除後に
// W-1 が「元のモードに戻る／メインメニューへ」の 2 択に変わる。
// fault 起因の ESTOP（estop_from_ui=false。DRIVE_RUNAWAY 等）は「安全のため停止しました」
// ＋「確認」だけ（#1 / #3 の実機フィードバック対応）。
//
// rosbridge バックエンドは無い（helpers.js / U-3）ので、ボタンのクリックは
// window.__thTriggerCalls に積まれることだけを検証する（FSM 側の遷移は
// test_state_core.py::test_ui_estop_resume_to_prev_mode がカバー）。
import { test, expect } from '@playwright/test'
import { gotoWithState } from './helpers.js'

test('UI 起因・解除済み・押下前 MANUAL の ESTOP は 2 択を出し、戻るが ui.resume_yes を送る', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'MANUAL', estop_from_ui: true, estop_ui: false, estop_hw: false,
  })

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
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'TEACH_MANUAL', estop_from_ui: true, estop_ui: false, estop_hw: false,
  })
  await page.getByRole('button', { name: 'メインメニューへ' }).click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_no')
})

test('fault 起因の ESTOP（estop_from_ui=false）は「戻る」を出さず「確認」＋システム文言', async ({ page }) => {
  // fault は 200ms で消えるので active:false・severity 空でも estop_from_ui=false のまま。
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'MANUAL', estop_from_ui: false, estop_ui: false, estop_hw: false,
  })
  await page.evaluate(() => window.__thSetTestFault({ active: false, severity: '', fault_type: 'DRIVE_RUNAWAY' }))

  await expect(page.locator('.win.fault.show')).toContainText('安全のため停止しました')
  await expect(page.getByRole('button', { name: '元のモードに戻る' })).toHaveCount(0)
  const ack = page.locator('#overlay').getByRole('button', { name: '確認' })
  await expect(ack).toBeVisible()
  await ack.click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_ack')
})

test('重大フォルトが継続中の ESTOP は確認ボタンも出さない', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'MANUAL', estop_from_ui: false, estop_ui: false, estop_hw: false,
  })
  await page.evaluate(() => window.__thSetTestFault({ active: true, severity: 'CRITICAL', fault_type: 'DRIVE_RUNAWAY' }))
  await expect(page.locator('#overlay').getByRole('button', { name: '確認' })).toHaveCount(0)
})

test('押下前が IDLE（メニューで押した）の UI-ESTOP は 2 択を出さない', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'IDLE', estop_from_ui: true, estop_ui: false, estop_hw: false,
  })
  await expect(page.getByRole('button', { name: '元のモードに戻る' })).toHaveCount(0)
})
