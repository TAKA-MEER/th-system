// 8-g P5a-ui: S-01 自律測定の入口・S-60・腕の格納の確認（DetailedDesign-measure.md §6.2）。
import { test, expect } from '@playwright/test'
import {
  gotoScreenWithMeasure, gotoScreenWithParams, setTestState, setTestMeasureStatus, setTestFault, stubTrigger,
} from './helpers.js'

async function triggers(page) {
  return (await page.evaluate(() => window.__thTriggerCalls ?? []))
}

function autoGotos(calls) {
  return calls.filter((c) => c.trigger === 'ui.goto' && c.argJson?.kind === 'AUTO_MEASURE')
}

// ── (a) link_ok=false は W-4 を出さず理由を出す（何も送らない）──
test('a: link_ok false で「自律測定」→ W-4 は出ず理由が出る。ui.goto を送らない', async ({ page }) => {
  await stubTrigger(page, { 'ui.goto': { accepted: true } })
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'IDLE', tracker_enabled: true },
    { link_ok: false, mode: 'IDLE', state: 'NONE' })

  const btn = page.locator('[data-testid="s01-mode-AUTO_MEASURE"]')
  await expect(btn).toBeEnabled()
  await btn.click()

  await expect(page.locator('.win.confirm.show')).toHaveCount(0)
  await expect(page.locator('[data-testid="s01-measure-reason"]')).toContainText('測定機能と通信できません')
  expect(autoGotos(await triggers(page)), '拒否したのに ui.goto を送っている').toHaveLength(0)
})

test('a: /measure/status 未受信でも W-4 は出さず理由を出す', async ({ page }) => {
  await stubTrigger(page, { 'ui.goto': { accepted: true } })
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'IDLE', tracker_enabled: true }, null)

  await page.locator('[data-testid="s01-mode-AUTO_MEASURE"]').click()

  await expect(page.locator('.win.confirm.show')).toHaveCount(0)
  await expect(page.locator('[data-testid="s01-measure-reason"]')).toContainText('測定機能')
  expect(autoGotos(await triggers(page))).toHaveLength(0)
})

// ── (b) link_ok=true → W-4「自律動作を許可しますか」 ──
test('b: 許可する で ui.goto{kind:AUTO_MEASURE} をちょうど1回送る。許可しないは何も送らない', async ({ page }) => {
  await stubTrigger(page, { 'ui.goto': { accepted: true } })
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'IDLE', tracker_enabled: true },
    { link_ok: true, mode: 'IDLE', state: 'NONE' })

  await page.locator('[data-testid="s01-mode-AUTO_MEASURE"]').click()
  await expect(page.locator('.win.confirm.show')).toContainText('自律動作を許可しますか')

  // 許可しない → 何も送らない
  await page.locator('[data-testid="s01-permit-no"]').click()
  expect(await triggers(page), '許可しないのに送信した').toHaveLength(
    (await triggers(page)).filter((c) => c.trigger === 'ui.screen').length)
  expect(autoGotos(await triggers(page))).toHaveLength(0)

  // もう一度押して許可する
  await page.locator('[data-testid="s01-mode-AUTO_MEASURE"]').click()
  await expect(page.locator('.win.confirm.show')).toContainText('自律動作を許可しますか')
  await page.locator('[data-testid="s01-permit-yes"]').click()

  const gotos = autoGotos(await triggers(page))
  expect(gotos).toHaveLength(1)
  expect(gotos[0].argJson).toEqual({ kind: 'AUTO_MEASURE' })
})

// ── (c) AUTO_MEASURE/WAIT で S-60 が出て screen_id が S-60 ──
test('c: AUTO_MEASURE/WAIT で S-60 が出て /ui/active_screen の screen_id が S-60。「走行」ボタンが無い', async ({ page }) => {
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'AUTO_MEASURE', state: 'WAIT', tracker_enabled: true },
    { link_ok: true, mode: 'AUTO_MEASURE', state: 'WAIT' })

  await expect(page.locator('[data-testid="s60-state"]')).toContainText('指示待ち')

  await page.waitForFunction(() =>
    (window.__thActiveScreenPublishes ?? []).some((m) => m.screen_id === 'S-60'))

  await expect(page.getByRole('button', { name: /^走行$/ })).toHaveCount(0)
  await expect(page.locator('[data-testid="s60-open-measure"]')).toBeVisible()
  await expect(page.locator('[data-testid="s60-finish"]')).toBeVisible()
  await expect(page.locator('[data-testid="s60-manual"]')).toBeVisible()
})

// ── (d) 「自律測定を終了」→ W-4 腕の確認 → やめるは送らない／メインメニューへは ui.finish 1 回 ──
test('d: 終了は腕の格納の確認を挟む。やめるは何も送らず、メインメニューへで ui.finish を1回送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.finish': { accepted: true } })
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'AUTO_MEASURE', state: 'WAIT', tracker_enabled: true },
    { link_ok: true, mode: 'AUTO_MEASURE', state: 'WAIT' })

  const finish = page.locator('[data-testid="s60-finish"]')
  await expect(finish).toBeVisible()

  await finish.click()
  await expect(page.locator('.win.confirm.show')).toContainText('測定機構の腕が格納位置にあることを確かめましたか')
  await expect(page.locator('[data-testid="s60-arm-question"]')).toBeVisible()

  // やめる → 何も送らない
  await page.locator('[data-testid="s60-arm-cancel"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.finish')).toHaveLength(0)

  // もう一度押してメインメニューへ
  await finish.click()
  await expect(page.locator('.win.confirm.show')).toContainText('腕が格納位置にあること')
  await page.locator('[data-testid="s60-arm-ok"]').click()

  const sent = (await triggers(page)).filter((c) => c.trigger === 'ui.finish')
  expect(sent).toHaveLength(1)
})

// ── (e) 「測定機能の画面へ」は同じタブで measure_ui_port の URL へ ──
test('e: 「測定機能の画面へ」は /params/get の measure_ui_port へ移る（8091 以外）', async ({ page }) => {
  await gotoScreenWithParams(page, 'S01',
    { mode: 'AUTO_MEASURE', state: 'WAIT', tracker_enabled: true },
    { get: { effective: { measure_ui_port: 18091 }, pending: {} } })

  const btn = page.locator('[data-testid="s60-open-measure"]')
  await expect(btn).toBeVisible()
  await expect(btn).toBeEnabled()

  const [req] = await Promise.all([
    page.waitForEvent('request', (r) => r.url().includes(':18091')),
    btn.click(),
  ])
  expect(req.url()).toBe(`http://localhost:18091/`)
})

// ── (f) ESTOP（prev_mode AUTO_MEASURE）の W-1 は腕の確認を挟む ──
test('f: ESTOP prev_mode AUTO_MEASURE の W-1「メインメニューへ」は W-4 を挟む。やめるは送らず、抜けるは ui.resume_no 1 回', async ({ page }) => {
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'ESTOP', prev_mode: 'AUTO_MEASURE', estop_ui: false, estop_hw: false, estop_from_ui: true },
    { link_ok: true, mode: 'ESTOP', state: 'NONE' })

  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  const menu = page.getByRole('button', { name: 'メインメニューへ' })
  await expect(menu).toBeVisible()

  await menu.click()
  await expect(page.locator('[data-testid="w4-arm-ask"]')).toBeVisible()
  await expect(page.locator('[data-testid="w4-arm-question"]')).toContainText('格納位置')

  // やめる → 何も送らない
  await page.locator('[data-testid="w4-arm-cancel"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.resume_no')).toHaveLength(0)

  // もう一度押してメインメニューへ
  await menu.click()
  await expect(page.locator('[data-testid="w4-arm-ask"]')).toBeVisible()
  await page.locator('[data-testid="w4-arm-ok"]').click()

  const sent = (await triggers(page)).filter((c) => c.trigger === 'ui.resume_no')
  expect(sent).toHaveLength(1)
})

test('f: prev_mode が MANUAL なら W-4 を出さずすぐ送る', async ({ page }) => {
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'ESTOP', prev_mode: 'MANUAL', estop_ui: false, estop_hw: false, estop_from_ui: true },
    { link_ok: true, mode: 'ESTOP', state: 'NONE' })

  await page.getByRole('button', { name: 'メインメニューへ' }).click()

  await expect(page.locator('[data-testid="w4-arm-ask"]')).toHaveCount(0)
  const sent = (await triggers(page)).filter((c) => c.trigger === 'ui.resume_no')
  expect(sent).toHaveLength(1)
})

// ── (g) CARRY（prev_mode AUTO_MEASURE）の W-2 も腕の確認を挟む ──
test('g: CARRY prev_mode AUTO_MEASURE の W-2 は W-4 を挟む。やめるは送らず、抜けるは ui.finish 1 回', async ({ page }) => {
  await stubTrigger(page, { 'ui.finish': { accepted: true } })
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'CARRY', prev_mode: 'AUTO_MEASURE', estop_hw: false, estop_ui: false },
    { link_ok: true, mode: 'CARRY', state: 'NONE' })

  const carry = page.locator('.win.carry')
  await expect(carry).toBeVisible()

  // W-2 の「メインメニューへ」相当は ui.finish を送るボタン（再開しない）。
  const dismiss = carry.locator('[data-testid="w2-c"]')
  await expect(dismiss).toBeVisible()
  await dismiss.click()

  await expect(page.locator('[data-testid="w4-arm-ask"]')).toBeVisible()
  await page.locator('[data-testid="w4-arm-cancel"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.finish')).toHaveLength(0)

  await dismiss.click()
  await expect(page.locator('[data-testid="w4-arm-ask"]')).toBeVisible()
  await page.locator('[data-testid="w4-arm-ok"]').click()

  const sent = (await triggers(page)).filter((c) => c.trigger === 'ui.finish')
  expect(sent).toHaveLength(1)
})

test('g: CARRY prev_mode MANUAL なら W-4 を出さずすぐ送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.finish': { accepted: true } })
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'CARRY', prev_mode: 'MANUAL', estop_hw: false, estop_ui: false },
    { link_ok: true, mode: 'CARRY', state: 'NONE' })

  const dismiss = page.locator('.win.carry').locator('[data-testid="w2-c"]')
  await expect(dismiss).toBeVisible()
  await dismiss.click()

  await expect(page.locator('[data-testid="w4-arm-ask"]')).toHaveCount(0)
  const sent = (await triggers(page)).filter((c) => c.trigger === 'ui.finish')
  expect(sent).toHaveLength(1)
})

// ── (h) AUTO_MEASURE の回復フォルト PAUSE の W-1「確認」は W-4 を出さず送信する ──
// PAUSE に留まり IDLE へ抜けない（C-05 の resume_state）ため SM-3.1.1-18 の対象外。
test('h: AUTO_MEASURE/PAUSE の W-1「確認」は腕の確認を出さず ui.resume_ack をすぐ送る', async ({ page }) => {
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'AUTO_MEASURE', state: 'PAUSE', tracker_enabled: true, auto_brake: true },
    { link_ok: true, mode: 'AUTO_MEASURE', state: 'PAUSE' })
  await setTestFault(page, { active: true, fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' })

  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  await expect(win).toContainText('確認')

  // フォルトが消えて W-1 が「再開しますか」になると「確認」ボタンが出る。
  await setTestFault(page, { active: false, severity: '', fault_type: 'LIDAR_LOST' })
  const ack = win.getByRole('button', { name: '確認', exact: true })
  await expect(ack).toBeVisible()
  await ack.click()

  await expect(page.locator('[data-testid="w4-arm-ask"]')).toHaveCount(0)
  const sent = (await triggers(page)).filter((c) => c.trigger === 'ui.resume_ack')
  expect(sent).toHaveLength(1)
})

// ── (i) AUTO_MEASURE は run_state=NAV なので meas.stop の PAUSE では W-1 を出さない ──
test('i: AUTO_MEASURE/PAUSE（フォルト無し・pause_reason 空）で W-1 が出ない。run_state が無いモードと ESTOP では従来どおり W-1 が出る', async ({ page }) => {
  await gotoScreenWithMeasure(page, 'S60',
    { mode: 'AUTO_MEASURE', state: 'PAUSE', tracker_enabled: true, auto_brake: true, pause_reason: '' },
    { link_ok: true, mode: 'AUTO_MEASURE', state: 'PAUSE' })
  await expect(page.locator('.win.fault.show')).toHaveCount(0)

  // 対照: run_state が null の AT_PANEL の PAUSE では W-1 が出る。
  await gotoScreenWithMeasure(page, 'S21',
    { mode: 'AT_PANEL', state: 'PAUSE', tracker_enabled: true, auto_brake: true, pause_reason: '' },
    { link_ok: true, mode: 'AT_PANEL', state: 'PAUSE' })
  await expect(page.locator('.win.fault.show')).toBeVisible()

  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'ESTOP', state: 'NONE', tracker_enabled: true, auto_brake: true },
    { link_ok: true, mode: 'ESTOP', state: 'NONE' })
  await expect(page.locator('.win.fault.show')).toBeVisible()
})
