// e2e/w1-fault-pause-latch.spec.js — WS-9Z (2026-09-09).
//
// isW1Active(mode, stateName, faultActive) only shows W-1 while faultActive
// is still true. safety_monitor's RECOVERABLE faults (ESP32_DISCONNECTED /
// LIDAR_LOST -- known WiFi/serial jitter) typically clear within ~100ms;
// state_manager's C-03 leaves mode/state parked in PAUSE regardless, and
// only ui.resume_* (this window's own buttons) gets it out
// (guard: fault_cleared). If the fault has already cleared by the time the
// WebUI renders, isW1Active never returns true and the window never opens.
//
// For modes with no run_state fallback (AT_HOME / AT_PANEL / OPCHECK /
// CALIB -- see th_state/config/attributes.yaml), that window is the *only*
// way out. Reproduced live on th_robot (2026-09-09): AT_HOME/PAUSE stuck
// with no window ever shown; `ros2 service call /system/trigger ...
// ui.resume_ack` (sent directly, bypassing the UI) was accepted immediately
// (fault_cleared guard passed) and released it.
//
// AppShell.jsx now latches "a fault was seen while PAUSE" (faultPauseSeen)
// until state leaves PAUSE, and falls back to "PAUSE + no run_state +
// !jog_active can only be fault-caused" for the case where the page loads
// (or reloads) after the fault has already come and gone.
import { test, expect } from '@playwright/test'
import { gotoWithState, setTestFault, setTestState } from './helpers.js'

test('WS-9Z: フォルトが表示前に消えても、run_state の無いモードでは窓が開いたまま残る', async ({ page }) => {
  await gotoWithState(page, { mode: 'AT_HOME', state: 'PAUSE', jog_active: false })
  // fault が「アクティブ→解消」の順で 2 回届く（実機で確認した ~100ms のパターン）。
  await setTestFault(page, { active: true, fault_type: 'ESP32_DISCONNECTED', severity: 'RECOVERABLE' })
  await setTestFault(page, { active: false, fault_type: 'ESP32_DISCONNECTED', severity: 'RECOVERABLE' })

  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  // fault は既に解消しているので「確認」（ack_only。attributes.yaml の AT_HOME）が出る。
  const ack = win.getByRole('button', { name: '確認' })
  await expect(ack).toBeVisible()
  await ack.click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_ack')
})

test('WS-9Z: 保険経路 — 読み込み時点で既に PAUSE（fault は一度も見ていない）でも窓を出す', async ({ page }) => {
  // fault を一度も active にしないまま最初から AT_HOME/PAUSE で開く
  // （ページの再読込・再訪問で、フォルトの立ち上がりを一度も観測できない状況を模す）。
  await gotoWithState(page, { mode: 'AT_HOME', state: 'PAUSE', jog_active: false })
  await expect(page.locator('.win.fault.show')).toBeVisible()
})

test('WS-9Z: AT_PANEL も同じ保険経路の対象', async ({ page }) => {
  await gotoWithState(page, { mode: 'AT_PANEL', state: 'PAUSE', jog_active: false })
  await expect(page.locator('.win.fault.show')).toBeVisible()
})

test('WS-9Z: ジョグ保持中の PAUSE（jog_active:true）では窓を出さない（フォルトではない）', async ({ page }) => {
  await gotoWithState(page, { mode: 'AT_HOME', state: 'PAUSE', jog_active: true })
  await expect(page.locator('.win.fault.show')).toHaveCount(0)
})

test('WS-9Z: run_state のあるモード（PANEL_NAV）の自主停止 PAUSE では窓を出さない', async ({ page }) => {
  // ui.stop による自主停止（フォルト無し）は「走行」ボタンで戻る設計であり、
  // 保険経路（run_state 無し限定）の対象外。誤って W-1 を出すと二重の UI になる。
  await gotoWithState(page, { mode: 'PANEL_NAV', state: 'PAUSE', jog_active: false })
  await expect(page.locator('.win.fault.show')).toHaveCount(0)
})

test('WS-9Z: PANEL_NAV でもフォルトが実際に見えれば窓を出し（走行ボタンとは独立に）解消後は確認できる', async ({ page }) => {
  await gotoWithState(page, { mode: 'PANEL_NAV', state: 'PAUSE', jog_active: false })
  await setTestFault(page, { active: true, fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' })
  await setTestFault(page, { active: false, fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' })
  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  // PANEL_NAV は yes_no（attributes.yaml）。
  await expect(win.getByRole('button', { name: 'いいえ' })).toBeVisible()
  await expect(win.getByRole('button', { name: 'はい' })).toBeVisible()
})

test('WS-9Z: PAUSE を抜けたらラッチは解除される（次に別モードで PAUSE に入っても誤って残らない）', async ({ page }) => {
  await gotoWithState(page, { mode: 'AT_HOME', state: 'PAUSE', jog_active: false })
  await expect(page.locator('.win.fault.show')).toBeVisible()
  await page.evaluate(() => window.__thSetTestState({ state: 'IDLE_H' }))
  await expect(page.locator('.win.fault.show')).toHaveCount(0)
  // 別モードの自主停止 PAUSE（run_state あり）に移っても、古いラッチのせいで
  // 誤って窓が出ないこと。
  await page.evaluate(() => window.__thSetTestState({ mode: 'PANEL_NAV', state: 'PAUSE' }))
  await expect(page.locator('.win.fault.show')).toHaveCount(0)
})
