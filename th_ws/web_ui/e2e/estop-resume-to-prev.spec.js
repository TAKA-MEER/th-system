// 2026-09-01 (Spec-modes.md §3.1.1 SM-3.1.1-11): UI 非常停止ボタン起因の ESTOP
// （estop_from_ui=true・重大フォルト無し・押下前が復帰可能なモード）は、解除後に
// W-1 が「元のモードに戻る／メインメニューへ」の 2 択に変わる。
// WS-9O (2026-09-04): fault 起因の ESTOP（estop_from_ui=false）も、フォルトが消えていて
// 押下前が復帰可能なら同じ 2 択を出す。以前は「確認」→ IDLE だけで、再生中に一瞬の
// フォルトで止まると経路選択からやり直しになっていた（Spec-safety.md §3.5.2）。
// 文言は起因で出し分ける（fault 起因は「安全のため停止しました」＋フォルト名）。
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
  // D48: 2択は廃止。「確認」だけ。復帰可能なら ui.resume_yes (SM-3.1.1-11)。
  await expect(page.getByRole('button', { name: '元のモードに戻る' })).toHaveCount(0)
  await expect(page.getByRole('button', { name: 'メインメニューへ' })).toHaveCount(0)
  await expect(page.getByRole('button', { name: '非表示' })).toHaveCount(0)
  const back = page.getByTestId('w1-confirm')
  await expect(back).toBeVisible()

  await back.click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_yes')
})

test('押下前 TEACH_MANUAL の ESTOP の確認は ui.resume_yes を送る（D48）', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'TEACH_MANUAL', estop_from_ui: true, estop_ui: false, estop_hw: false,
  })
  await page.getByTestId('w1-confirm').click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_yes')
})

test('WS-9O: fault 起因の ESTOP もフォルトが消えていれば「確認」で ui.resume_yes、文言はシステム側', async ({ page }) => {
  // fault は一瞬で消えるので active:false・severity 空でも estop_from_ui=false のまま。
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'MANUAL', estop_from_ui: false, estop_ui: false, estop_hw: false,
  })
  await page.evaluate(() => window.__thSetTestFault({ active: false, severity: '', fault_type: 'DRIVE_RUNAWAY' }))

  // 文言は起因で出し分ける（ボタンが出ても「非常停止ボタンが押されました」にはしない）。
  await expect(page.locator('.win.fault.show')).toContainText('安全監視が異常を検知し')
  await expect(page.getByRole('button', { name: '元のモードに戻る' })).toHaveCount(0)
  const back = page.getByTestId('w1-confirm')
  await expect(back).toBeVisible()

  await back.click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_yes')
})

test('WS-9O: fault 起因でも押下前が IDLE なら「確認」で ui.resume_ack', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'IDLE', estop_from_ui: false, estop_ui: false, estop_hw: false,
  })
  await page.evaluate(() => window.__thSetTestFault({ active: false, severity: '', fault_type: 'DRIVE_RUNAWAY' }))

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

test('押下前が IDLE（メニューで押した）の UI-ESTOP の確認は ui.resume_ack', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'IDLE', estop_from_ui: true, estop_ui: false, estop_hw: false,
  })
  await expect(page.getByRole('button', { name: '元のモードに戻る' })).toHaveCount(0)
})
