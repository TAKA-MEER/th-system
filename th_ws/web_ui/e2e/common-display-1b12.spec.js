// 1b-12 共通の表示（SG-C2・C3・C9 の W-6 分・C10・C11）。
// 状態は e2e の縫い目（window.__thSetTestState / __thSetTestFault）で与えるが、
// 描画は本番の AppShell / Header / Windows / OperationCard をそのまま通る。
import { test, expect } from '@playwright/test'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { gotoWithState, gotoScreen, setTestState, setTestFault } from './helpers.js'

const attributes = JSON.parse(
  readFileSync(fileURLToPath(new URL('../src/generated/attributes.json', import.meta.url)), 'utf8'),
)

const BG = async (loc) => loc.evaluate((el) => getComputedStyle(el).backgroundColor)
const BG_TRANSPARENT = 'rgba(0, 0, 0, 0)'
const BG_CURRENT = 'rgb(44, 74, 116)' // --ctrl-on（青の塗り）
const BG_CONFIRM = 'rgb(23, 54, 29)' // 緑の塗り（保存）

// ── SG-C10: 非常停止を解除バー ────────────────────────────────────────────

test('SG-C10: 重大フォルトで入った ESTOP では解除バーが出ず、W-1 に「確認」が出る', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'IDLE', estop_from_ui: false, estop_ui: false, estop_hw: false,
  })
  await setTestFault(page, { active: true, fault_type: 'DRIVE_RUNAWAY', severity: 'CRITICAL' })
  await expect(page.locator('.win.fault.show')).toBeVisible()
  await expect(page.locator('#release')).toBeHidden()

  // 原因が解消すると W-1 が「確認」になる。押しても何も起きない解除バーは相変わらず出ない。
  await setTestFault(page, { active: false })
  await expect(page.locator('.win.fault.show footer').getByRole('button', { name: '確認' })).toBeVisible()
  await expect(page.locator('#release')).toBeHidden()
})

test('SG-C10: UI 非常停止の ESTOP では解除バーが出る（再読込・別端末の押下でも）', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'MANUAL', estop_from_ui: true, estop_ui: true, estop_hw: false,
  })
  await expect(page.locator('#release')).toBeVisible()
  await expect(page.locator('#releaseBtn')).toHaveText('非常停止を解除')
})

test('SG-C10: 重大フォルトの ESTOP に UI 非常停止が重なれば解除バーが出る', async ({ page }) => {
  await gotoWithState(page, {
    mode: 'ESTOP', prev_mode: 'IDLE', estop_from_ui: false, estop_ui: false, estop_hw: false,
  })
  await setTestFault(page, { active: true, fault_type: 'MUX_DEAD', severity: 'CRITICAL' })
  await expect(page.locator('#release')).toBeHidden()
  await page.locator('#estopBtn').click()
  await expect(page.locator('#release')).toBeVisible()
})

test('SG-C10: UI 非常停止ボタンを押すと解除バーが出て、解除で消える', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE' })
  await expect(page.locator('#release')).toBeHidden()
  await page.locator('#estopBtn').click()
  await expect(page.locator('#release')).toBeVisible()
  await page.locator('#releaseBtn').click()
  await expect(page.locator('#release')).toBeHidden()
})

// ── SG-C3: ヘッダの色 ─────────────────────────────────────────────────────

test('SG-C3: ヘッダの色がモード・開発モードで変わる', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE' })
  const hdr = page.locator('#hdr')
  const bgOf = async () => hdr.evaluate((el) => getComputedStyle(el).backgroundImage)

  await expect(hdr).toHaveAttribute('data-tone', 'normal')
  const normal = await bgOf()

  await setTestState(page, { mode: 'ESTOP', estop_ui: true })
  await expect(hdr).toHaveAttribute('data-tone', 'estop')
  const estop = await bgOf()

  await setTestState(page, { mode: 'CARRY', estop_ui: false, estop_hw: true })
  await expect(hdr).toHaveAttribute('data-tone', 'carry')
  const carry = await bgOf()

  expect(new Set([normal, estop, carry]).size, '3 つの色が互いに違うこと').toBe(3)
})

test('SG-C3: 開発モード中はヘッダの色が変わる（非常停止の赤は開発モードに負けない）', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE' })
  const hdr = page.locator('#hdr')
  const bgOf = () => hdr.evaluate((el) => getComputedStyle(el).backgroundImage)
  const normal = await bgOf()

  // ?dev=1 で開き直す（初期化スクリプトの状態は引き継がれる）。
  await page.goto('/?dev=1')
  await expect(page.locator('#devPill')).toBeVisible()
  await expect(hdr).toHaveAttribute('data-tone', 'dev')
  expect(await bgOf()).not.toBe(normal)

  await setTestState(page, { mode: 'ESTOP', estop_ui: true })
  await expect(hdr).toHaveAttribute('data-tone', 'estop')
})

// ── SG-C2: 操作カードの色（U-17） ─────────────────────────────────────────

async function openHarness(page, st) {
  await gotoWithState(page, st)
  await expect(page.locator('[data-testid="opcard-harness"] .opsgrid')).toBeVisible()
}
const op = (page, cls) => page.locator(`[data-testid="opcard-harness"] .opsgrid .${cls}`)

test('SG-C2: 走行中は走行だけが青塗り、停止・確認・手動は枠のみ、保存は緑塗り', async ({ page }) => {
  await openHarness(page, { mode: 'REPLAY', state: attributes.REPLAY.run_state })
  expect(await BG(op(page, 'op-run'))).toBe(BG_CURRENT)
  expect(await BG(op(page, 'op-stop'))).toBe(BG_TRANSPARENT)
  expect(await BG(op(page, 'op-check'))).toBe(BG_TRANSPARENT)
  expect(await BG(op(page, 'op-manual'))).toBe(BG_TRANSPARENT)
  expect(await BG(op(page, 'op-save'))).toBe(BG_CONFIRM)
})

test('SG-C2: 停止中は停止だけが青塗り（走行は青くない）', async ({ page }) => {
  await openHarness(page, { mode: 'REPLAY', state: attributes.REPLAY.resume_state })
  expect(await BG(op(page, 'op-stop'))).toBe(BG_CURRENT)
  expect(await BG(op(page, 'op-run'))).toBe(BG_TRANSPARENT)
  expect(await BG(op(page, 'op-save'))).toBe(BG_CONFIRM)
})

test('SG-C2: 確認中は確認だけが青塗り（青が 2 つ並ばない）', async ({ page }) => {
  await openHarness(page, { mode: 'REPLAY', state: attributes.REPLAY.initial_state })
  expect(await BG(op(page, 'op-check'))).toBe(BG_CURRENT)
  expect(await BG(op(page, 'op-run'))).toBe(BG_TRANSPARENT)
  expect(await BG(op(page, 'op-stop'))).toBe(BG_TRANSPARENT)
  await expect(page.locator('[data-testid="opcard-harness"] .opsgrid .btn.on')).toHaveCount(1)
})

test('SG-C2: ボタンの幅は置く数で変わらない（停止だけの S-11 でも格子の 3 等分の 1 マス）', async ({ page }) => {
  const share = async (stopSel, gridSel) => {
    const w = (await page.locator(stopSel).boundingBox()).width
    const g = (await page.locator(gridSel).boundingBox()).width
    return w / g
  }
  await openHarness(page, { mode: 'REPLAY', state: 'PAUSE' })
  const withMany = await share('[data-testid="opcard-harness"] .op-stop', '[data-testid="opcard-harness"] .opsgrid')

  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  const only = await share('#s11 .opsgrid .op-stop', '#s11 .opsgrid')
  expect(only, '停止だけでも行いっぱいに伸びない').toBeLessThan(0.4)
  expect(Math.abs(only - withMany), '置く数で幅の割合が変わらない').toBeLessThan(0.03)
})

// ── SG-C11: W-6 は操作カードの停止・確認・走行・保存で閉じる ──────────────

for (const [cls, label] of [['op-stop', '停止'], ['op-check', '確認'], ['op-run', '走行'], ['op-save', '保存']]) {
  test(`SG-C11: W-6 は操作カードの「${label}」で閉じる`, async ({ page }) => {
    await openHarness(page, { mode: 'REPLAY', state: 'PAUSE' })
    await page.locator('[data-testid="harness-open-jog"]').click()
    await expect(page.locator('#jogWin.show')).toBeVisible()
    await op(page, cls).click()
    await expect(page.locator('#jogWin')).toBeHidden()
  })
}

test('SG-C11: 「手動」は W-6 を閉じない（手動に移っただけ）', async ({ page }) => {
  await openHarness(page, { mode: 'REPLAY', state: 'PAUSE' })
  await page.locator('[data-testid="harness-open-jog"]').click()
  await expect(page.locator('#jogWin.show')).toBeVisible()
  await op(page, 'op-manual').click()
  await expect(page.locator('#jogWin.show')).toBeVisible()
})

test('SG-C11: 本番の S-21 でも停止で W-6 が閉じ、ui.stop が送られる', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreen(page, 'S21', { mode: 'PANEL_NAV', state: 'NAV' })
  await page.locator('#s21').waitFor()
  await page.locator('#s21 .op-manual').click()
  await expect(page.locator('#jogWin.show')).toBeVisible()
  await page.locator('#s21 .op-stop').click()
  await expect(page.locator('#jogWin')).toBeHidden()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.stop')
})

// ── SG-C9（W-6 分）: 死角と自動ブレーキの状態 ─────────────────────────────

test('SG-C9: W-6 に「後方は死角があります」と自動ブレーキの状態が出る', async ({ page }) => {
  await openHarness(page, { mode: 'REPLAY', state: 'PAUSE', jog_active: true, auto_brake: true })
  await page.locator('[data-testid="harness-open-jog"]').click()
  const w6 = page.locator('#jogWin.show')
  await expect(w6.getByTestId('w6-rear-blind')).toHaveText('後方は死角があります')
  await expect(w6.getByTestId('w6-auto-brake')).toHaveText('自動ブレーキ：ON')

  // 機体の /system/state.auto_brake に従う（ジョグ中に OFF になれば OFF と出す）
  await setTestState(page, { auto_brake: false })
  await expect(w6.getByTestId('w6-auto-brake')).toHaveText('自動ブレーキ：OFF')
  await expect(w6.getByTestId('w6-auto-brake')).toHaveAttribute('data-state', 'off')

  // 未受信は ON とも OFF とも言わない
  await setTestState(page, { auto_brake: null })
  await expect(w6.getByTestId('w6-auto-brake')).toHaveText('自動ブレーキ：不明')
})
