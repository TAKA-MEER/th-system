// S-01 メインメニューから S-21 試験を開く実導線。
//
// 2026-09-08: S-21 は AT_HOME モード（Spec-modes.md §2.3）で開くようになった。
// 以前は S-21 に対応するモードが無く、S-50 設定と同じ「S-01 のサブ画面」方式
// （main.jsx の onsiteTestOpen というローカルフラグ）で出していた。AT_HOME が
// できたので、「試験（当日）」は他のモード選択ボタンと同じ ui.enter_mode になり、
// 画面は /system/state から導出される。
//
// ?__thTestScreen は S-01 そのものを開くためだけに使い、そこから先はボタンを押す。
import { test, expect } from '@playwright/test'
import { gotoScreen, stubTrigger, setTestState } from './helpers.js'

const IDLE = { mode: 'IDLE', state: 'NONE', tracker_enabled: true }

test('S-01 の「試験（当日）」で ui.enter_mode{mode:AT_HOME} を送り、S-21 が出る', async ({ page }) => {
  // 受理されたら FSM が AT_HOME になる。TEST_MODE には FSM が無いので、
  // 受理応答を返しつつテスト側で /system/state を AT_HOME に進める。
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await gotoScreen(page, 'S01', IDLE)
  await page.locator('#s01').waitFor()

  const btn = page.locator('[data-testid="s01-mode-AT_HOME"]')
  await expect(btn).toHaveText('試験（当日）')
  await expect(page.locator('#s21')).toBeHidden()

  await btn.click()

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  const enter = calls.filter((c) => c.trigger === 'ui.enter_mode')
  expect(enter, '「試験（当日）」が ui.enter_mode を送っていない').toHaveLength(1)
  expect(enter[0].argJson).toEqual({ mode: 'AT_HOME' })

  // FSM が AT_HOME になったら画面が切り替わる（画面は mode から導出する）。
  await setTestState(page, { mode: 'AT_HOME', state: 'IDLE_H' })
  await expect(page.locator('#s21')).toBeVisible()
  await expect(page.locator('[data-testid="s21-finish"]')).toBeVisible()
})

test('S-21 の「終了」は ui.finish を送り、IDLE に戻ると S-01 が出る', async ({ page }) => {
  await stubTrigger(page, { 'ui.finish': { accepted: true } })
  await gotoScreen(page, 'S01', { mode: 'AT_HOME', state: 'IDLE_H' })
  await expect(page.locator('#s21')).toBeVisible()

  await page.locator('[data-testid="s21-finish"]').click()
  expect(
    (await page.evaluate(() => window.__thTriggerCalls ?? []))
      .filter((c) => c.trigger === 'ui.finish'),
    '終了が ui.finish を送っていない',
  ).toHaveLength(1)

  // 画面はローカルに戻さない。FSM が IDLE になったのを見て戻る。
  await setTestState(page, { mode: 'IDLE', state: 'NONE' })
  await expect(page.locator('#s01')).toBeVisible()
})

test('「試験（当日）」は INIT では押せない（M-1: 起動中は何も押せない）', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'INIT', state: 'NONE' })
  await page.locator('#s01').waitFor()
  await expect(page.locator('[data-testid="s01-mode-AT_HOME"]')).toBeDisabled()
})

test('AT_HOME は設定(S-50)より優先される（動作系モードが最優先）', async ({ page }) => {
  await gotoScreen(page, 'S01', IDLE)
  await page.locator('#s01').waitFor()

  // 設定を開く → S-50。
  await page.locator('[data-testid="s01-open-settings"]').click()
  await expect(page.locator('#s50')).toBeVisible()

  // 設定を開いたまま FSM が AT_HOME になったら S-21 が出る（S-50 は引っ込む）。
  await setTestState(page, { mode: 'AT_HOME', state: 'IDLE_H' })
  await expect(page.locator('#s21')).toBeVisible()
  await expect(page.locator('#s50')).toBeHidden()
})
