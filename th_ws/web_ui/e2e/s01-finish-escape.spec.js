// brief-onsite-fix A: S-01 の「終了して待機に戻る」ボタン（data-testid="s01-finish-escape"）。
// 画面の無いモード（FOLLOW 等）で S-01 が出ているときだけボタンが見え、
// ui.finish を送信することを確かめる。IDLE のときはボタンが無い。
//
// WP-UI-08: 以前は OPCHECK をこの「画面の無いモード」の例に使っていたが、
// S-30 始業点検ができたことで OPCHECK は MODE_TO_SCREEN に載り、
// resolveScreen が S-30 を返すようになった（gotoScreen の第2引数はどの画面を
// 開くかの決め手ではない。読むのは state.mode/state.state だけ -- DRIVE_S11
// の合成画面を除き testScreen は screenRouting.js で使われない）。
// WP-MAINT-03: CALIB も S-40 ができて MODE_TO_SCREEN に載ったので、今は FOLLOW を例に使う
// （画面の無いモードは FOLLOW / LEASH / LINE 等）。
import { test, expect } from '@playwright/test'
import { gotoScreen, stubTrigger } from './helpers.js'

test('FOLLOW: finish-escape ボタンが見え、押すと ui.finish を送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.finish': { accepted: true } })
  await gotoScreen(page, 'S01', { mode: 'FOLLOW', state: 'RUN' })

  const btn = page.getByTestId('s01-finish-escape')
  await expect(btn).toBeVisible()
  await expect(btn).toBeEnabled()

  await btn.click()

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.some((c) => c.trigger === 'ui.finish')).toBeTruthy()
})

test('IDLE: finish-escape ボタンは存在しない', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  const btn = page.getByTestId('s01-finish-escape')
  await expect(btn).toHaveCount(0)
})

test('INIT: finish-escape ボタンは存在しない', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'INIT' })

  const btn = page.getByTestId('s01-finish-escape')
  await expect(btn).toHaveCount(0)
})
