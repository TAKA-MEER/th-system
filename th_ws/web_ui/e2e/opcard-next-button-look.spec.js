// Spec-webui.md §3.3.1: 次に押す「停止」は赤（next-stop）、次に押す「走行」は白い太枠（next-run）。
// 押せるボタンは背景透明の枠のみで描かない。無効は押せるボタンと形（破線枠・面なし）で区別する。
import { test, expect } from '@playwright/test'
import { gotoWithState, setTestStale } from './helpers.js'

const card = '[data-testid="opcard-harness"] .opsgrid'
const bgOf = (page, sel) =>
  page.locator(`${card} ${sel}`).evaluate((e) => getComputedStyle(e).backgroundColor)

test('青が走行（NAV 相当）のとき、停止に next-stop・走行が青', async ({ page }) => {
  await gotoWithState(page, { mode: 'HOME_NAV', state: 'NAV' })
  await expect(page.locator(`${card} .op-run`)).toHaveClass(/tone-current/)
  await expect(page.locator(`${card} .op-stop`)).toHaveClass(/tone-next-stop/)
  await expect(page.locator(`${card} .tone-next-stop`)).toHaveCount(1)
  await expect(page.locator(`${card} .tone-next-run`)).toHaveCount(0)
})

test('青が停止（PAUSE 相当）のとき、走行に next-run・停止が青', async ({ page }) => {
  await gotoWithState(page, { mode: 'HOME_NAV', state: 'PAUSE' })
  await expect(page.locator(`${card} .op-stop`)).toHaveClass(/tone-current/)
  await expect(page.locator(`${card} .op-run`)).toHaveClass(/tone-next-run/)
  await expect(page.locator(`${card} .tone-next-run`)).toHaveCount(1)
  await expect(page.locator(`${card} .tone-next-stop`)).toHaveCount(0)
})

test('押せるボタンは背景が透明でない／無効ボタンは面が無く破線枠', async ({ page }) => {
  await gotoWithState(page, { mode: 'HOME_NAV', state: 'PAUSE' })
  for (const sel of ['.op-stop', '.op-run', '.op-save', '.op-manual']) {
    expect(await bgOf(page, sel), `${sel} の背景が透明`).not.toBe('rgba(0, 0, 0, 0)')
  }
  await setTestStale(page, true) // ハーネスは stale で全ボタンが無効になる
  // 青（いまの状態）の停止は無効でも青のまま。次に押す走行と手動で確かめる
  for (const sel of ['.op-run', '.op-manual']) {
    await expect(page.locator(`${card} ${sel}`)).toBeDisabled()
    expect(await bgOf(page, sel), `${sel} の無効が面を持つ`).toBe('rgba(0, 0, 0, 0)')
    const style = await page.locator(`${card} ${sel}`).evaluate((e) => getComputedStyle(e).borderTopStyle)
    expect(style).toBe('dashed')
  }
})
