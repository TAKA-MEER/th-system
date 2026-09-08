// e2e/onsite-ux2.spec.js — brief-onsite-ux UX-2-a / UX-2-b / UX-3
// UX-2-a: 「停止」ボタンの class（btn-stop）が他のどの kind とも異なること。
// UX-2-b: S-21 の手動ボタンは IDLE のとき非活性 + 理由バッジ（jog_denied）。
//         （サーバがこの keys を返さないことが dichotomy 証明で分かっているため
//         画面側でだけ判定する。UX-2-b の列挙テストは unit で担保済み。）
// UX-3:   S-20 / S-21 が 1280×720（scale 1）で overflow しないこと。
import { expect, test } from '@playwright/test'
import { gotoScreen, gotoScreenWithOnsite } from './helpers.js'

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

// ── UX-3: overflow なし ─────────────────────────────────────────────────────
// brief の algorithms.js overflowingElements を、Playwright evaluate で
// ブラウザ内で実行して検証する（== [] が正）。

async function overflowingElements(page, { root = '#stage', items = '> * > *' } = {}) {
  // brief のアルゴリズムと同一。items は rootEl から相対（:scope を付与して
  // querySelectorAll がスコープ内で評価されるようにする）。
  return page.evaluate(({ root, items }) => {
    const rootEl = document.querySelector(root)
    if (!rootEl) return []
    return [...rootEl.querySelectorAll(`:scope ${items}`)].filter(
      (el) => el.scrollTop > 0 || el.scrollWidth > el.clientWidth,
    )
  }, { root, items })
}

// S-20: 空シード（デフォルト）で overflow なし
test('UX-3: S-20（空シード）は 1280×720 で overflow しない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S20', PREP_STATE, { pins: [], targets: [] })
  const overflowing = await overflowingElements(page)
  expect(overflowing).toEqual([])
})

// S-20: 中身多めシード（ピン 6 件 + 対象 2 件 + 対象未選択）で overflow なし
test('UX-3: S-20（中身多め）は 1280×720 で overflow しない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S20', PREP_STATE, {
    pins: [
      { id: 'p1', name: '配電盤1', kind: 'PANEL', pose: { position: { x: 1, y: 1 } } },
      { id: 'p2', name: '配電盤2', kind: 'PANEL', pose: { position: { x: 2, y: 2 } } },
      { id: 'p3', name: '配電盤3', kind: 'PANEL', pose: { position: { x: 3, y: 3 } } },
      { id: 'p4', name: '配電盤4', kind: 'PANEL', pose: { position: { x: 4, y: 4 } } },
      { id: 'p5', name: '配電盤5', kind: 'PANEL', pose: { position: { x: 5, y: 5 } } },
      { id: 'p6', name: '配電盤6', kind: 'PANEL', pose: { position: { x: 6, y: 6 } } },
    ],
    targets: [
      { position: { x: 0.3, y: 0, z: 0 }, score: 0.9, id: 'c1' },
      { position: { x: -0.3, y: 0, z: 0 }, score: 0.85, id: 'c2' },
    ],
  })
  // ピン一覧タブを開く
  await page.locator('[data-testid="s20-subtab-pins"]').click()
  const overflowing = await overflowingElements(page)
  expect(overflowing).toEqual([])
})

// S-21: 空シードで overflow なし
test('UX-3: S-21（空シード）は 1280×720 で overflow しない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S21', IDLE_STATE, { pins: [], targets: [] })
  const overflowing = await overflowingElements(page)
  expect(overflowing).toEqual([])
})

// S-21: 中身多めシード（ピン 6 件 + 配電盤前タブ + 対象 2 件）で overflow なし
test('UX-3: S-21（中身多め）は 1280×720 で overflow しない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithOnsite(page, 'S21', IDLE_STATE, {
    pins: [
      { id: 'p1', name: '配電盤1', kind: 'PANEL', pose: { position: { x: 1, y: 1 } } },
      { id: 'p2', name: '配電盤2', kind: 'PANEL', pose: { position: { x: 2, y: 2 } } },
      { id: 'p3', name: '配電盤3', kind: 'PANEL', pose: { position: { x: 3, y: 3 } } },
      { id: 'p4', name: '配電盤4', kind: 'PANEL', pose: { position: { x: 4, y: 4 } } },
      { id: 'p5', name: '配電盤5', kind: 'PANEL', pose: { position: { x: 5, y: 5 } } },
      { id: 'p6', name: '配電盤6', kind: 'PANEL', pose: { position: { x: 6, y: 6 } } },
    ],
    targets: [
      { position: { x: 0.3, y: 0, z: 0 }, score: 0.9, id: 'c1' },
      { position: { x: -0.3, y: 0, z: 0 }, score: 0.85, id: 'c2' },
    ],
    homeDeclared: true,
  })
  const overflowing = await overflowingElements(page)
  expect(overflowing).toEqual([])
})