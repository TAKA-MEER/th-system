// e2e/w0a-header-windows.spec.js — ブリーフ W0a の共通部品の縛り。
// Spec-webui §0 原則 4 / §1 / §4: 非常停止は最初に描き、中身の例外でも押せる。
// 窓は本文の中央で幕が本文（ヘッダ以外）を覆う。W-1/W-2 に隠すボタンが無い。
import { test, expect } from '@playwright/test'
import { gotoWithState, setTestFault, setTestState } from './helpers.js'

test('中身が例外で落ちても非常停止は表示され押せる（§0 原則 4）', async ({ page }) => {
  await page.addInitScript((s) => { window.__thTestState = s }, { mode: 'MANUAL', state: 'RUN' })
  await page.addInitScript(() => { window.__thTestThrow = true })
  await page.goto('/')
  await expect(page.getByText('画面を表示できませんでした')).toBeVisible()
  const estop = page.locator('#estopBtn')
  await expect(estop).toBeVisible()
  await expect(estop).toBeEnabled()
  await estop.click()
})

test('W-1 は本文の中央で、幕は本文を覆いヘッダは覆わない', async ({ page }) => {
  await gotoWithState(page, { mode: 'MANUAL', state: 'PAUSE', pause_reason: 'fault' })
  await setTestFault(page, { active: true, fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' })
  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()
  const body = await page.locator('#body').boundingBox()
  const w = await win.boundingBox()
  expect(w.x).toBeGreaterThanOrEqual(body.x - 1)
  expect(w.x + w.width).toBeLessThanOrEqual(body.x + body.width + 1)
  const bodyCx = body.x + body.width / 2, winCx = w.x + w.width / 2
  const bodyCy = body.y + body.height / 2, winCy = w.y + w.height / 2
  expect(Math.abs(winCx - bodyCx)).toBeLessThan(body.width / 4)
  expect(Math.abs(winCy - bodyCy)).toBeLessThan(body.height / 3)
  // ヘッダは幕より上（z 順）。非常停止は押せる。
  await expect(page.locator('#estopBtn')).toBeEnabled()
  // 隠すボタン・「はい／いいえ」は無い（D47・D48）。
  await expect(win.getByRole('button', { name: '非表示' })).toHaveCount(0)
  await expect(win.getByRole('button', { name: 'はい' })).toHaveCount(0)
  await expect(win.getByRole('button', { name: 'いいえ' })).toHaveCount(0)
})

test('場内だけ「速度制限」、場外は出さない（§1）', async ({ page }) => {
  await gotoWithState(page, { mode: 'PREP', state: 'MAPPING', zone: 'IN' })
  await expect(page.locator('#zonePill')).toContainText('速度制限')
  await setTestState(page, { zone: 'OUT' })
  // 場外では出さない（要素自体が無い）。
  await expect(page.locator('#zonePill')).toHaveCount(0)
})

test('自動ブレーキが切れている間だけ「自動ブレーキなし」（§1・D74）', async ({ page }) => {
  await gotoWithState(page, { mode: 'MANUAL', state: 'RUN', auto_brake: true })
  await expect(page.locator('#autoBrakeOff')).toHaveCount(0)
  await setTestState(page, { auto_brake: false })
  await expect(page.locator('#autoBrakeOff')).toContainText('自動ブレーキなし')
})
