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
// Spec-webui.md §3.3.1（JIS配色）: 明るい地が既定。--ctrl-on は選んだ状態の塗り。
const BG_CURRENT = 'rgb(91, 102, 117)' // --ctrl-on（今の状態）
const BG_CONFIRM = 'rgb(23, 54, 29)' // 緑の塗り（保存）
const BG_NEXT_STOP = 'rgb(161, 30, 30)' // 赤の塗り（次に押す停止）
const BG_FACE = 'rgb(230, 234, 240)' // --ctrl（面＋枠。透明ではない）
const BG_NEXT_RUN = 'rgb(213, 219, 228)' // --ctrl-hi（次に押す走行。白い太枠）

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

test('SG-C3: ヘッダの地色はモードで変わらない。状態は状態札で読む', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE', state: 'NONE' })
  const hdr = page.locator('#hdr')
  const bgOf = async () => hdr.evaluate((el) => getComputedStyle(el).backgroundColor)
  const normal = await bgOf()

  // 緑「進む」状態の札が出る。地色は変わらない。
  await setTestState(page, { mode: 'FOLLOW', state: 'RUN' })
  await expect(page.locator('#modePill')).toHaveAttribute('data-tone', 'go')
  expect(await bgOf()).toBe(normal)

  // 非常停止の札は赤
  await setTestState(page, { mode: 'ESTOP', state: 'PAUSE', estop_ui: true })
  await expect(page.locator('#modePill')).toHaveAttribute('data-tone', 'estop')
  expect(await bgOf()).toBe(normal)

  // 一時停止は白灰黒の札
  await setTestState(page, { mode: 'FOLLOW', state: 'PAUSE' })
  await expect(page.locator('#modePill')).toHaveAttribute('data-tone', 'neutral')
  expect(await bgOf()).toBe(normal)
})

test('SG-C3: 開発モード中はヘッダの色が変わり、開発札で読む（非常停止の赤は開発モードに負けない）', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE', state: 'NONE' })
  const hdr = page.locator('#hdr')
  const bgOf = () => hdr.evaluate((el) => getComputedStyle(el).backgroundColor)
  const normal = await bgOf()

  // ?dev=1 で開き直す（初期化スクリプトの状態は引き継がれる）。
  await page.goto('/?dev=1')
  await expect(page.locator('#devPill')).toBeVisible()
  expect(await bgOf()).not.toBe(normal)

  await setTestState(page, { mode: 'ESTOP', state: 'PAUSE', estop_ui: true })
  await expect(page.locator('#modePill')).toHaveAttribute('data-tone', 'estop')
})

// ── SG-C2: 操作カードの色（U-17） ─────────────────────────────────────────

async function openHarness(page, st) {
  await gotoWithState(page, st)
  await expect(page.locator('[data-testid="opcard-harness"] .opsgrid')).toBeVisible()
}
const op = (page, cls) => page.locator(`[data-testid="opcard-harness"] .opsgrid .${cls}`)

test('SG-C2: 走行中は走行だけが今の色、停止は赤、確認・手動は面＋枠、保存は緑', async ({ page }) => {
  await openHarness(page, { mode: 'REPLAY', state: attributes.REPLAY.run_state })
  expect(await BG(op(page, 'op-run'))).toBe(BG_CURRENT)
  expect(await BG(op(page, 'op-stop'))).toBe(BG_NEXT_STOP)
  expect(await BG(op(page, 'op-check'))).toBe(BG_FACE)
  expect(await BG(op(page, 'op-manual'))).toBe(BG_FACE)
  expect(await BG(op(page, 'op-save'))).toBe(BG_CONFIRM)
})

test('SG-C2: 停止中は停止だけが今の色（走行は違う）', async ({ page }) => {
  await openHarness(page, { mode: 'REPLAY', state: attributes.REPLAY.resume_state })
  expect(await BG(op(page, 'op-stop'))).toBe(BG_CURRENT)
  expect(await BG(op(page, 'op-run'))).toBe(BG_NEXT_RUN)
  expect(await BG(op(page, 'op-save'))).toBe(BG_CONFIRM)
})

test('SG-C2: 確認中は確認だけが今の色（今の色が 2 つ並ばない）', async ({ page }) => {
  await openHarness(page, { mode: 'REPLAY', state: attributes.REPLAY.initial_state })
  expect(await BG(op(page, 'op-check'))).toBe(BG_CURRENT)
  expect(await BG(op(page, 'op-run'))).toBe(BG_FACE)
  expect(await BG(op(page, 'op-stop'))).toBe(BG_NEXT_STOP)
  await expect(page.locator('[data-testid="opcard-harness"] .opsgrid .btn.on')).toHaveCount(1)
})

test('SG-C2: S-11 の「停止する」は新しい停止│走行トグルで右列いっぱい（古い操作カードは使わない）', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  const toggle = page.locator('[data-testid="s11-stop-run"]')
  await expect(toggle).toBeVisible()
  const w = (await toggle.boundingBox()).width
  const col = (await page.locator('#s11 [data-pos="toggle"]').boundingBox()).width
  expect(w / col, '停止だけでも右列いっぱいに伸びる').toBeGreaterThan(0.9)
  await expect(page.locator('#s11 .card.ops')).toHaveCount(0)
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
