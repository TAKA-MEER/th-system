// T-6 (2026-10-10) SM-3.1.2-003 / CL-B-6: 物理非常停止が押されたまま起動したときの案内。
// 一回きりの /system/effect に頼らず、/system/state (INIT かつ estop_hw) から導いて
// 疎通確認 (S-00) に出る。effect は一切流さない (画面を開く前に出た effect を
// 取りこぼした実機の状況の再現)。本番の hook (useGuideBanner) を通す。
// 変異: 状態からの導出を外す／INIT の条件を外す／閉じた後の出し直しを外すと赤くなること。
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState } from './helpers.js'

test('effect が無くても S-00 に案内が出て、IDLE へ進む導線が無い', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'INIT', state: 'CHECK', estop_hw: true })
  await expect(page.getByTestId('w3-guide'))
    .toContainText('起動時に非常停止ボタンが押されたままです。戻してください')
  await expect(page.getByTestId('s00-advance')).toHaveCount(0)
})

test('ボタンを戻すと案内が消える', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'INIT', state: 'CHECK', estop_hw: true })
  await expect(page.getByTestId('w3-guide')).toBeVisible()
  await setTestState(page, { estop_hw: false })
  await expect(page.getByTestId('w3-guide')).toHaveCount(0)
})

test('押されていない INIT では案内が出ない', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'INIT', state: 'CHECK', estop_hw: false })
  await expect(page.getByTestId('w3-guide')).toHaveCount(0)
})

test('閉じても押し直すと出し直す', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'INIT', state: 'CHECK', estop_hw: true })
  await page.getByTestId('w3-guide-close').click()
  await expect(page.getByTestId('w3-guide')).toHaveCount(0)
  await setTestState(page, { estop_hw: false })
  await setTestState(page, { estop_hw: true })
  await expect(page.getByTestId('w3-guide')).toBeVisible()
})

test('INIT 以外 (IDLE) で押下されても起動時の案内は出ない', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', state: 'NONE', estop_hw: true })
  await expect(page.getByTestId('w3-guide')).toHaveCount(0)
})
