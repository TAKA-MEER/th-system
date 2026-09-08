// e2e/onsite-ux2.spec.js — brief-onsite-ux UX-2-a / UX-2-b、brief-onsite-ux-fix UX-5
// UX-2-a: 「停止」ボタンの class（btn-stop）が他のどの kind とも異なること。
// UX-2-b: S-21 の手動ボタンは IDLE のとき非活性 + 理由バッジ（jog_denied）。
//         （サーバがこの keys を返さないことが dichotomy 証明で分かっているため
//         画面側でだけ判定する。UX-2-b の列挙テストは unit で担保済み。）
// UX-5:   S-20 / S-21 が 1280×720（scale 1）で「本文（#body）がスクロールを
//         要求しない」こと（元 UX-3 の #stage 矩形判定は「画面外にはみ出す
//         要素が無いか」しか見ておらず、区画内スクロールと本文スクロールを
//         区別できなかった。bodyOverflowPx() に置き換えた）。
import { expect, test } from '@playwright/test'
import {
  gotoScreen, gotoScreenWithOnsite, unlockOnsiteMap, unlockOnsiteVenueMap,
} from './helpers.js'

// ── UX-2-a: 「停止」の形は他と異なる ──────────────────────────────────────────

const IDLE_STATE = { mode: 'IDLE', state: 'IDLE', map_update: false }
const PREP_STATE = { mode: 'PREP', state: 'MAPPING' }

test('UX-2-a: S-21 の「停止」ボタンは btn-stop クラスを持ち、他の kind と異なる', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S21', IDLE_STATE, { pins: [], targets: [] })
  const stopBtn = page.locator('#s21 .op-stop')
  await expect(stopBtn).toHaveClass(/btn-stop/)
  const otherBtns = page.locator('#s21 .btn-advance, #s21 .btn-register, #s21 .btn-save-op, #s21 .btn-manual-op, #s21 .btn-finish')
  const count = await otherBtns.count()
  for (let i = 0; i < count; i++) {
    const cls = await otherBtns.nth(i).getAttribute('class') ?? ''
    expect(cls).not.toContain('btn-stop')
  }
})

test('UX-2-b: S-21 の手動ボタンは IDLE で disabled + 理由バッジ（jog_denied）', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S21', IDLE_STATE, { pins: [], targets: [] })
  // mock idling の attrs: IDLE.jog='denied'
  const manualBtn = page.locator('#s21 .op-manual')
  await expect(manualBtn).toBeVisible()
  await expect(manualBtn).toBeDisabled()
  const badge = page.locator('[data-testid="reason-manual"]')
  await expect(badge).toBeVisible()
  await expect(badge).toHaveText('このモードでは手動操作できません')
})

// ── UX-5: 「操作者がページをスクロールしなくてよい」を直接測る ──────────────
// UX-4（brief-onsite-ux-fix）で #stage 矩形判定に是正したが、その判定は
// 「画面外にはみ出す要素が無いか」であって「本文がスクロールを要求するか」
// ではなかった。#body が唯一のページスクロール領域（theme.css の
// #body{overflow-y:auto}）なので、中身が client 高さに収まっていれば本文
// スクロールは要らない、を直接測る。.lst-scroll / .tabpane のような意図的な
// 区画内スクロールは許す（区画自身が #body の中に収まっていればよい）。
async function bodyOverflowPx(page) {
  return page.evaluate(() => {
    const body = document.querySelector('#body')
    if (!body) return -1
    return body.scrollHeight - body.clientHeight
  })
}

const PINS6 = [
  { id: 'p1', name: '配電盤1', kind: 'PANEL', pose: { position: { x: 1, y: 1 } } },
  { id: 'p2', name: '配電盤2', kind: 'PANEL', pose: { position: { x: 2, y: 2 } } },
  { id: 'p3', name: '配電盤3', kind: 'PANEL', pose: { position: { x: 3, y: 3 } } },
  { id: 'p4', name: '配電盤4', kind: 'PANEL', pose: { position: { x: 4, y: 4 } } },
  { id: 'p5', name: '配電盤5', kind: 'PANEL', pose: { position: { x: 5, y: 5 } } },
  { id: 'p6', name: '配電盤6', kind: 'PANEL', pose: { position: { x: 6, y: 6 } } },
]
const TARGETS4 = [
  { position: { x: 0.3, y: 0, z: 0 }, score: 0.9, id: 'c1' },
  { position: { x: -0.3, y: 0, z: 0 }, score: 0.85, id: 'c2' },
  { position: { x: 0.9, y: 0, z: 0 }, score: 0.7, id: 'c3' },
  { position: { x: -0.9, y: 0, z: 0 }, score: 0.6, id: 'c4' },
]

// S-20: 空シード（デフォルト）で本文スクロール不要
test('UX-5: S-20（空シード）は 1280×720 で本文スクロールしない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S20', PREP_STATE, { pins: [], targets: [] })
  expect(await bodyOverflowPx(page)).toBeLessThanOrEqual(1)
})

// S-20: 中身多めシード（ピン 6 件 + 候補 4 件）で本文スクロール不要
test('UX-5: S-20（ピン6・候補4）は 1280×720 で本文スクロールしない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S20', PREP_STATE, { pins: PINS6, targets: TARGETS4 })
  // ピン一覧タブを開く
  await page.locator('[data-testid="s20-subtab-pins"]').click()
  expect(await bodyOverflowPx(page)).toBeLessThanOrEqual(1)
})

// S-21: 空シード・homeDeclared false で本文スクロール不要
test('UX-5: S-21（空シード・未宣言）は 1280×720 で本文スクロールしない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S21', IDLE_STATE, { pins: [], targets: [], homeDeclared: false })
  expect(await bodyOverflowPx(page)).toBeLessThanOrEqual(1)
})

// S-21: 中身多めシード（ピン 6 件・候補 4 件・宣言済み）で本文スクロール不要
test('UX-5: S-21（ピン6・候補4・宣言済み）は 1280×720 で本文スクロールしない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S21', IDLE_STATE, {
    pins: PINS6,
    targets: TARGETS4,
    homeDeclared: true,
  })
  expect(await bodyOverflowPx(page)).toBeLessThanOrEqual(1)
})

// S-21: 呼び寄せ・退避待ち（SUMMON/WAIT_CLEAR）+ ピン6・候補4 で本文スクロール不要
test('UX-5: S-21（SUMMON/WAIT_CLEAR・ピン6・候補4）は 1280×720 で本文スクロールしない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S21', { mode: 'SUMMON', state: 'WAIT_CLEAR' }, {
    pins: PINS6,
    targets: TARGETS4,
    homeDeclared: true,
    waitClear: { distance_m: 1.2, remaining_sec: 8, satisfied: false, verdict: 'WAITING' },
  })
  await page.locator('[data-testid="s21-subtab-summon"]').click()
  expect(await bodyOverflowPx(page)).toBeLessThanOrEqual(1)
})

// ── F-7: 手動操作パネル（W-6）が地図を隠さない（brief-onsite-ux2） ───────────
// S-20/S-21 の左列は地図タブなので、既定の左下配置だと浮いた W-6 が地図に
// 被る（実機指摘）。案A（W-6 を右列の幅に収めて右下へ寄せる）を採った。
// S-11/S-13/S-14 は無変更（e2e/w6-does-not-move-body.spec.js が別途担保）。
function overlaps(a, b) {
  return !(a.x + a.width <= b.x || b.x + b.width <= a.x
    || a.y + a.height <= b.y || b.y + b.height <= a.y)
}

test('F-7: S-20 で W-6 を開いても地図と重ならない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S20', PREP_STATE, { pins: [], targets: [], mappingActive: true })
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)
  await page.locator('#s20 .op-manual').click()
  await page.locator('#jogWin.show').waitFor()

  const mapBox = await page.getByTestId('s20-map').boundingBox()
  const jogBox = await page.locator('#jogWin').boundingBox()
  expect(overlaps(mapBox, jogBox), 'W-6 が S-20 の地図と重なっている').toBe(false)
})

test('F-7: S-21 で W-6 を開いても地図と重ならない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(
    page, 'S21', { mode: 'PANEL_NAV', state: 'NAV' },
    { pins: [], targets: [], openVenueMap: { success: true, message: '' } },
  )
  await page.locator('#s21').waitFor()
  await unlockOnsiteVenueMap(page)
  await page.locator('#s21 .op-manual').click()
  await page.locator('#jogWin.show').waitFor()

  const mapBox = await page.getByTestId('s21-map').boundingBox()
  const jogBox = await page.locator('#jogWin').boundingBox()
  expect(overlaps(mapBox, jogBox), 'W-6 が S-21 の地図と重なっている').toBe(false)
})