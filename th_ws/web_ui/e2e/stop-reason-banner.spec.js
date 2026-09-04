// WS-9R (2026-09-04): 止まっている理由を画面に出す。ただし出しすぎない。
//
// 実機フィードバック（1回目）「謎の一時停止。画面をスクロールすると復帰する」。
// 在席確認が speed_limit=stop に倒していたが、フォルトでも一時停止でもないので
// 画面に何も出ず、操作者に理由が分からなかった。
//
// 実機フィードバック（2回目）「赤い帯が基本的に常に出ていて邪魔。一部表示を
// 隠してしまっている」。走らせていない間は速度指令が無いのが normal で、
// リミッタは常に ZERO_STALE を返す。それを理由として出すと待機中ずっと帯が
// 出たままになる。→ 機体が走るはずのときだけ出す。
//
// 帯は本文を押し下げてはいけない（操作者の指の下でボタンが動く）。
// e2e/header-geometry-stable.spec.js が本文の開始位置を見張っている。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('待機中は帯を出さない（実機「常に出ていて邪魔」）', async ({ page }) => {
  await gotoScreen(page, 'S01', {
    mode: 'IDLE', state: 'NONE', zone: 'NA', speed_limit: 'stop',
  })
  await expect(page.getByTestId('stop-banner')).toHaveCount(0)
})

test('再生の準備中（READY）も帯を出さない', async ({ page }) => {
  await gotoScreen(page, 'S14', {
    mode: 'REPLAY', state: 'READY', zone: 'NA', speed_limit: 'stop',
  })
  await expect(page.getByTestId('stop-banner')).toHaveCount(0)
})

test('走行中に在席が確認できないと理由が出る', async ({ page }) => {
  await gotoScreen(page, 'S14', {
    mode: 'REPLAY', state: 'RUN', zone: 'NA', speed_limit: 'stop',
  })
  const banner = page.getByTestId('stop-banner')
  await expect(banner).toBeVisible()
  await expect(banner).toContainText('在席が確認できない')
})

test('走行中で問題が無ければ帯を出さない', async ({ page }) => {
  await gotoScreen(page, 'S14', {
    mode: 'REPLAY', state: 'RUN', zone: 'OUT', speed_limit: 'v_max',
  })
  await expect(page.getByTestId('stop-banner')).toHaveCount(0)
})

test('非常停止中は W-1 の窓が説明するので帯を二重に出さない', async ({ page }) => {
  await gotoScreen(page, 'S01', {
    mode: 'ESTOP', state: 'NONE', zone: 'NA', speed_limit: 'stop',
  })
  await expect(page.getByTestId('stop-banner')).toHaveCount(0)
})

test('帯は本文を押し下げない（指の下でボタンが動かない）', async ({ page }) => {
  await gotoScreen(page, 'S14', {
    mode: 'REPLAY', state: 'RUN', zone: 'OUT', speed_limit: 'v_max',
  })
  const before = await page.evaluate(
    () => document.getElementById('body').getBoundingClientRect().top)

  await page.evaluate(() => {
    window.__thSetTestState?.({
      mode: 'REPLAY', state: 'RUN', zone: 'NA', speed_limit: 'stop',
    })
  })
  await expect(page.getByTestId('stop-banner')).toBeVisible()

  const after = await page.evaluate(
    () => document.getElementById('body').getBoundingClientRect().top)
  expect(after).toBe(before)
})

test('帯は 1 行に収まり、隠す面積を最小にする', async ({ page }) => {
  await gotoScreen(page, 'S14', {
    mode: 'REPLAY', state: 'RUN', zone: 'NA', speed_limit: 'stop',
  })
  const banner = page.getByTestId('stop-banner')
  await expect(banner).toBeVisible()
  const box = await banner.boundingBox()
  const app = await page.evaluate(
    () => document.getElementById('app').getBoundingClientRect().height)
  // 本文の高さに対して十分小さいこと（実機「一部表示を隠してしまう」）。
  expect(box.height).toBeLessThan(app * 0.12)

  // 文言が長くなっても折り返さず、1 行に切り詰めること。幅の広い画面では
  // 短い文なら折り返さないので、寸法だけ見ても指定の有無を区別できない。
  const style = await page.evaluate(() => {
    const el = document.querySelector('[data-testid="stop-banner"]')
    const cs = getComputedStyle(el)
    return { ws: cs.whiteSpace, ov: cs.overflow, te: cs.textOverflow }
  })
  expect(style.ws, '折り返しを止めていない').toBe('nowrap')
  expect(style.te, '切り詰めの指定が無い').toBe('ellipsis')
  expect(style.ov, 'はみ出しを隠していない').not.toBe('visible')
})
