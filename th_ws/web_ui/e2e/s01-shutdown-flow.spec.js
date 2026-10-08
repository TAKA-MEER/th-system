// DetailedDesign-state.md §12.5 / DetailedDesign-webui.md §8.3: the 5-step
// shutdown path -- (1) card shows the unsaved count and a single button,
// (2) pressing it opens W-4 with the unsaved list, (3) each item gets
// 保存/破棄, (4) "停止する" stays disabled until every item is resolved,
// (5) after /shutdown/execute succeeds the screen (not the window) shows
// completion. /shutdown/prepare and /shutdown/execute are stubbed via
// ros/useStdTrigger.js's test hook (no rosbridge backend in this spec, same
// as every other shell e2e test -- U-3).
import { test, expect } from '@playwright/test'
import {
  gotoScreen, setTestStale, stubServices, stubTrigger, stdTriggerCalls, onsiteServiceCalls,
} from './helpers.js'

// 機体側が反映したふり: 以後の /shutdown/prepare は残りの一覧を返す。
async function prepareNowReturns(page, list) {
  await page.evaluate((l) => {
    window.__thTestServices['/shutdown/prepare'] = { success: true, message: JSON.stringify(l) }
  }, list)
}

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
  await stubTrigger(page, { 'ui.save': { accepted: true, reject_reason_key: null } })
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

  // Step 3: 「保存」は ui.save を送り、/shutdown/prepare を呼び直して一覧から消えたら
  // 処理済みになる（応答だけでは済みにしない）。
  await prepareNowReturns(page, [])
  await win.getByRole('button', { name: '保存', exact: true }).click()
  await expect(win).toContainText('処理済み')
  const sent = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(sent.some((c) => c.trigger === 'ui.save')).toBe(true)
  await expect(confirmBtn).toBeEnabled()

  // Step 4: press "停止する" -> /shutdown/execute (stubbed success).
  await confirmBtn.click()

  // Step 5 (1b-6 SG-B6): execute の成功だけでは完了を出さない。進捗表示になり、
  // 実際に止まった（接続が切れた）のを確かめてから完了を出す。
  await expect(win).toHaveCount(0)
  const card = page.locator('.card', { hasText: '運用の終了' })
  await expect(card).toContainText('停止しています')
  await expect(card).not.toContainText('停止が完了しました')

  // 接続が切れたら完了表示。
  await setTestStale(page, true)
  await expect(card).toContainText('停止が完了しました')
  await expect(card).toContainText('電源を切って構いません')
})

test('execute が通っても接続が切れなければ完了を出さない（応答待ちの変異の標的）', async ({ page }) => {
  await stubServices(page, {
    '/shutdown/prepare': { success: true, message: '[]' },
    '/shutdown/execute': { success: true, message: '' },
  })
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await page.getByRole('button', { name: '制御系を停止する' }).click()

  const win = page.locator('.win.confirm.show')
  await win.getByRole('button', { name: '停止する' }).click()
  await expect(win).toHaveCount(0)
  const card = page.locator('.card', { hasText: '運用の終了' })
  await expect(card).toContainText('停止しています')
  await expect(card).not.toContainText('停止が完了しました')
})

test('未保存が残っている間は「停止する」が押せず理由が出る', async ({ page }) => {
  await stubServices(page, {
    '/shutdown/prepare': { success: true, message: JSON.stringify(['route', 'calib']) },
  })
  await gotoScreen(page, 'S01', { mode: 'IDLE', unsaved: ['route', 'calib'] })
  await page.getByRole('button', { name: '制御系を停止する' }).click()

  const win = page.locator('.win.confirm.show')
  const confirmBtn = win.getByRole('button', { name: '停止する' })
  await expect(confirmBtn).toBeDisabled()
  await expect(win.getByTestId('shutdown-blocked-reason')).toContainText('残り 2 件')
})

test('discard uses the two-stage armed button (§3.2.1 step 3) and calls the real discard', async ({ page }) => {
  await stubServices(page, {
    '/shutdown/prepare': { success: true, message: JSON.stringify(['venue_map']) },
    '/slam_control/discard_map': { success: true, message: '' },
  })
  await gotoScreen(page, 'S01', { mode: 'IDLE', unsaved: ['venue_map'] })
  await page.getByRole('button', { name: '制御系を停止する' }).click()

  const win = page.locator('.win.confirm.show')
  await expect(win).toContainText('試験場内地図')

  const discard = win.getByRole('button', { name: '破棄', exact: true })
  await expect(discard).toBeVisible()
  // First press arms it (label swaps); nothing is called and the item must not resolve yet.
  await discard.click()
  await expect(win.getByRole('button', { name: '本当に破棄' })).toBeVisible()
  await expect(win).not.toContainText('処理済み')
  expect((await stdTriggerCalls(page)).some((c) => c.service === '/slam_control/discard_map')).toBe(false)
  // Second press confirms: calls discard_map, then re-asks prepare.
  await prepareNowReturns(page, [])
  await win.getByRole('button', { name: '本当に破棄' }).click()
  await expect(win).toContainText('処理済み')
  expect((await stdTriggerCalls(page)).some((c) => c.service === '/slam_control/discard_map')).toBe(true)
})

test('venue_map の「保存」は /map_session/open (VENUE, save) を呼ぶ', async ({ page }) => {
  await stubServices(page, {
    '/shutdown/prepare': { success: true, message: JSON.stringify(['venue_map']) },
  })
  await page.addInitScript(() => { window.__thTestSaveVenueMap = { success: true, message: '' } })
  await gotoScreen(page, 'S01', { mode: 'IDLE', unsaved: ['venue_map'] })
  await page.getByRole('button', { name: '制御系を停止する' }).click()
  const win = page.locator('.win.confirm.show')
  await prepareNowReturns(page, [])
  await win.getByRole('button', { name: '保存', exact: true }).click()
  await expect(win).toContainText('処理済み')
  const calls = await onsiteServiceCalls(page)
  const c = calls.find((x) => x.service === '/map_session/open')
  expect(c.request.slot).toBe('VENUE')
  expect(c.request.mode).toBe('save')
})

test('操作が通っても機体側の一覧から消えなければ処理済みにしない', async ({ page }) => {
  await stubTrigger(page, { 'ui.save': { accepted: true, reject_reason_key: null } })
  await stubServices(page, {
    '/shutdown/prepare': { success: true, message: JSON.stringify(['route']) },
  })
  await gotoScreen(page, 'S01', { mode: 'IDLE', unsaved: ['route'] })
  await page.getByRole('button', { name: '制御系を停止する' }).click()
  const win = page.locator('.win.confirm.show')
  await win.getByRole('button', { name: '保存', exact: true }).click()
  await expect(win.getByTestId('shutdown-error-route')).toBeVisible({ timeout: 10000 })
  await expect(win).not.toContainText('処理済み')
  await expect(win.getByRole('button', { name: '停止する' })).toBeDisabled()
})

test('画面から扱えない項目（校正の補正値）は保存・破棄を出さず、停止も押せない', async ({ page }) => {
  await stubServices(page, {
    '/shutdown/prepare': { success: true, message: JSON.stringify(['calib']) },
  })
  await gotoScreen(page, 'S01', { mode: 'IDLE', unsaved: ['calib'] })
  await page.getByRole('button', { name: '制御系を停止する' }).click()
  const win = page.locator('.win.confirm.show')
  await expect(win.getByTestId('shutdown-noaction-calib')).toBeVisible()
  await expect(win.getByRole('button', { name: '保存', exact: true })).toHaveCount(0)
  await expect(win.getByRole('button', { name: '停止する' })).toBeDisabled()
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
