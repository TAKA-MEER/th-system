// DetailedDesign-state.md §12.5 / DetailedDesign-webui.md §8.3: the 5-step
// shutdown path -- (1) card shows the unsaved count and a single button,
// (2) pressing it opens W-4 with the unsaved list, (3) each item gets
// 保存/破棄, (4) "停止する" stays disabled until every item is resolved,
// (5) after /shutdown/execute succeeds the screen (not the window) shows
// completion. /shutdown/prepare and /shutdown/execute are stubbed via
// ros/useStdTrigger.js's test hook (no rosbridge backend in this spec, same
// as every other shell e2e test -- U-3).
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState, stubServices } from './helpers.js'

test('shutdown card shows the live unsaved count from SystemState.unsaved', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', unsaved: ['route', 'calib'] })
  const card = page.locator('.card', { hasText: '運用の終了' })
  await expect(card).toContainText('2 件')
})

test('"制御系を停止する" is pressable even with unsaved data present (M-4)', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', unsaved: ['route'] })
  const shutdownBtn = page.getByRole('button', { name: '制御系を停止する' })
  await expect(shutdownBtn).toBeEnabled()
})

test('5-step flow: prepare -> per-item resolve -> gated confirm -> execute -> completion shown on screen', async ({ page }) => {
  await stubServices(page, {
    '/shutdown/prepare': { success: true, message: JSON.stringify(['route']) },
    '/shutdown/execute': { success: true, message: '' },
  })
  await gotoScreen(page, 'S01', { mode: 'IDLE', unsaved: ['route'] })

  // Step 1: the card.
  await page.getByRole('button', { name: '制御系を停止する' }).click()

  // Step 2: W-4 opens with the unsaved list (from /shutdown/prepare).
  const win = page.locator('.win.confirm.show')
  await expect(win).toBeVisible()
  await expect(win).toContainText('教示経路')

  // Step 4 (checked before step 3): "停止する" starts disabled, unsaved remains.
  const confirmBtn = win.getByRole('button', { name: '停止する' })
  await expect(confirmBtn).toBeDisabled()

  // Step 3: resolve the one item ("保存").
  await win.getByRole('button', { name: '保存', exact: true }).click()
  await expect(win).toContainText('処理済み')
  await expect(confirmBtn).toBeEnabled()

  // Step 4: press "停止する" -> /shutdown/execute (stubbed success).
  await confirmBtn.click()

  // Step 5: completion shows on the screen, not inside the window; the
  // window itself closes.
  await expect(win).toHaveCount(0)
  const card = page.locator('.card', { hasText: '運用の終了' })
  await expect(card).toContainText('停止が完了しました')
  await expect(card).toContainText('電源を切って構いません')
})

test('discard uses the two-stage armed button (§3.2.1 step 3)', async ({ page }) => {
  await stubServices(page, {
    '/shutdown/prepare': { success: true, message: JSON.stringify(['calib']) },
  })
  await gotoScreen(page, 'S01', { mode: 'IDLE', unsaved: ['calib'] })
  await page.getByRole('button', { name: '制御系を停止する' }).click()

  const win = page.locator('.win.confirm.show')
  await expect(win).toContainText('校正の補正値')

  const discard = win.getByRole('button', { name: '破棄', exact: true })
  await expect(discard).toBeVisible()
  // First press arms it (label swaps); the item must not resolve yet.
  await discard.click()
  await expect(win.getByRole('button', { name: '本当に破棄' })).toBeVisible()
  await expect(win).not.toContainText('処理済み')
  // Second press confirms.
  await win.getByRole('button', { name: '本当に破棄' }).click()
  await expect(win).toContainText('処理済み')
})

test('server-side rejection (unsaved_remains) is shown via i18n/reasons.js, not raised as a dialog', async ({ page }) => {
  await stubServices(page, {
    '/shutdown/prepare': { success: true, message: '[]' },
    '/shutdown/execute': { success: false, message: 'unsaved_remains' },
  })
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await page.getByRole('button', { name: '制御系を停止する' }).click()

  const win = page.locator('.win.confirm.show')
  await expect(win).toContainText('未保存のデータはありません')
  await win.getByRole('button', { name: '停止する' }).click()
  await expect(win).toContainText('未保存のデータが残っています')
})
